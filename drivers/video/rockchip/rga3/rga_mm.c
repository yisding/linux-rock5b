// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Cerf Yu <cerf.yu@rock-chips.com>
 */

#include <linux/atomic.h>
#include <linux/dma-buf.h>
#include <linux/iosys-map.h>
#include <linux/limits.h>
#include <linux/overflow.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>

#include "rga.h"
#include "rga_job.h"
#include "rga_mm.h"
#include "rga_dma_buf.h"
#include "rga_common.h"
#include "rga_iommu.h"
#include "rga_hw_config.h"
#include "rga_debugger.h"

/*
 * RGA3 fetches a window base address on a 16-byte granularity (the low 4 bits
 * of the *_BASE registers are ignored by the hardware), so an IOMMU-mapped base
 * must be 16-byte aligned to read/write the intended bytes.
 */
#define RGA_IOMMU_ADDR_ALIGN 16
#define RGA2_STAGE_MAX_SIZE		SZ_64M
#define RGA2_STAGE_SESSION_MAX_SIZE	SZ_128M
#define RGA2_STAGE_GLOBAL_MAX_SIZE	SZ_256M

struct rga_rga2_stage {
	struct list_head node;
	struct dma_buf *origin;
	struct page **pages;
	unsigned int page_count;
	void *vaddr;
	struct sg_table *sgt;
	struct rga_dma_buffer mapping;
	struct rga_session *session;
	size_t size;
	unsigned int users;
	bool copy_back;
	bool budget_charged;
};

static atomic64_t rga2_stage_attempt_count = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_success_count = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_failure_count = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_reuse_count = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_active_count = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_active_bytes = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_peak_bytes = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_copy_in_bytes = ATOMIC64_INIT(0);
static atomic64_t rga2_stage_copy_out_bytes = ATOMIC64_INIT(0);

static void rga2_stage_update_peak(s64 active_bytes)
{
	s64 peak = atomic64_read(&rga2_stage_peak_bytes);

	while (active_bytes > peak) {
		s64 old = atomic64_cmpxchg(&rga2_stage_peak_bytes, peak,
					   active_bytes);

		if (old == peak)
			break;
		peak = old;
	}
}

void rga_mm_rga2_stage_show(struct seq_file *m)
{
	seq_printf(m, "job_max_bytes: %u\n", RGA2_STAGE_MAX_SIZE);
	seq_printf(m, "session_max_bytes: %u\n",
		   RGA2_STAGE_SESSION_MAX_SIZE);
	seq_printf(m, "global_max_bytes: %u\n",
		   RGA2_STAGE_GLOBAL_MAX_SIZE);
	seq_printf(m, "attempt_count: %lld\n",
		   atomic64_read(&rga2_stage_attempt_count));
	seq_printf(m, "success_count: %lld\n",
		   atomic64_read(&rga2_stage_success_count));
	seq_printf(m, "failure_count: %lld\n",
		   atomic64_read(&rga2_stage_failure_count));
	seq_printf(m, "reuse_count: %lld\n",
		   atomic64_read(&rga2_stage_reuse_count));
	seq_printf(m, "active_count: %lld\n",
		   atomic64_read(&rga2_stage_active_count));
	seq_printf(m, "active_bytes: %lld\n",
		   atomic64_read(&rga2_stage_active_bytes));
	seq_printf(m, "peak_bytes: %lld\n",
		   atomic64_read(&rga2_stage_peak_bytes));
	seq_printf(m, "copy_in_bytes: %lld\n",
		   atomic64_read(&rga2_stage_copy_in_bytes));
	seq_printf(m, "copy_out_bytes: %lld\n",
		   atomic64_read(&rga2_stage_copy_out_bytes));
}

u64 rga_mm_rga2_stage_counter(enum rga2_stage_counter counter)
{
	switch (counter) {
	case RGA2_STAGE_ATTEMPT:
		return atomic64_read(&rga2_stage_attempt_count);
	case RGA2_STAGE_SUCCESS:
		return atomic64_read(&rga2_stage_success_count);
	case RGA2_STAGE_FAILURE:
		return atomic64_read(&rga2_stage_failure_count);
	case RGA2_STAGE_REUSE:
		return atomic64_read(&rga2_stage_reuse_count);
	case RGA2_STAGE_ACTIVE:
		return atomic64_read(&rga2_stage_active_count);
	case RGA2_STAGE_ACTIVE_BYTES:
		return atomic64_read(&rga2_stage_active_bytes);
	case RGA2_STAGE_PEAK_BYTES:
		return atomic64_read(&rga2_stage_peak_bytes);
	case RGA2_STAGE_COPY_IN_BYTES:
		return atomic64_read(&rga2_stage_copy_in_bytes);
	case RGA2_STAGE_COPY_OUT_BYTES:
		return atomic64_read(&rga2_stage_copy_out_bytes);
	default:
		return 0;
	}
}

struct rga_shadow_node {
	int page_idx;
	struct page *orig_page;
	struct page *shadow_page;
	size_t offset;
	size_t len;
	struct list_head node;
};

static void rga_shadow_list_init(struct rga_virt_addr *virt_addr)
{
	INIT_LIST_HEAD(&virt_addr->shadow_list);
}

static void rga_shadow_free_node(struct rga_virt_addr *virt_addr,
				 struct rga_shadow_node *shadow)
{
	virt_addr->pages[shadow->page_idx] = shadow->orig_page;
	__free_page(shadow->shadow_page);
	list_del(&shadow->node);
	kfree(shadow);
}

static int rga_shadow_add_node(struct rga_virt_addr *virt_addr,
			       int page_idx, size_t offset, size_t len)
{
	struct rga_shadow_node *shadow;

	shadow = kzalloc(sizeof(*shadow), GFP_KERNEL);
	if (!shadow) {
		rga_err("shadow_page alloc node failed\n");
		return -ENOMEM;
	}

	shadow->shadow_page = alloc_page(GFP_KERNEL | GFP_DMA32);
	if (!shadow->shadow_page) {
		rga_err("shadow_page alloc page failed\n");
		kfree(shadow);
		return -ENOMEM;
	}

	shadow->page_idx = page_idx;
	shadow->orig_page = virt_addr->pages[page_idx];
	shadow->offset = offset;
	shadow->len = len;

	virt_addr->pages[page_idx] = shadow->shadow_page;
	list_add_tail(&shadow->node, &virt_addr->shadow_list);

	return 0;
}

static void rga_shadow_release(struct rga_virt_addr *virt_addr)
{
	struct rga_shadow_node *shadow, *tmp;

	if (!virt_addr || list_empty(&virt_addr->shadow_list))
		return;

	list_for_each_entry_safe(shadow, tmp, &virt_addr->shadow_list, node)
		rga_shadow_free_node(virt_addr, shadow);
}

static int rga_shadow_setup(struct rga_virt_addr *virt_addr)
{
	int ret;
	int head_idx;
	int tail_idx;
	size_t head_len;
	size_t tail_len;
	size_t total;
	size_t head_offset;
	size_t cache_align = dma_get_cache_alignment();
	bool need_head = false;
	bool need_tail = false;

	if (!virt_addr || virt_addr->page_count <= 0 || virt_addr->size == 0)
		return 0;

	head_idx = 0;
	tail_idx = virt_addr->page_count - 1;
	total = virt_addr->size;
	head_offset = virt_addr->offset;

	head_len = min_t(size_t, PAGE_SIZE - head_offset, total);
	tail_len = (virt_addr->offset + total) - (tail_idx * PAGE_SIZE);

	need_head = !IS_ALIGNED(head_offset, cache_align);
	need_tail = !IS_ALIGNED(virt_addr->offset + total, cache_align) ||
		    tail_len < (2 * ARCH_DMA_MINALIGN);

	if (tail_idx == head_idx) {
		if (need_head || need_tail) {
			ret = rga_shadow_add_node(virt_addr, head_idx, head_offset, total);
			if (ret)
				goto err_release;
		}
	} else {
		if (need_head) {
			ret = rga_shadow_add_node(virt_addr, head_idx, head_offset, head_len);
			if (ret)
				goto err_release;
		}

		if (need_tail) {
			ret = rga_shadow_add_node(virt_addr, tail_idx, 0, tail_len);
			if (ret)
				goto err_release;
		}
	}

	virt_addr->shadow_head = need_head;
	virt_addr->shadow_tail = need_tail;

	return 0;

err_release:
	rga_shadow_release(virt_addr);
	return ret;
}

static inline bool rga_shadow_active(struct rga_virt_addr *virt_addr)
{
	return virt_addr && !list_empty(&virt_addr->shadow_list);
}

static void rga_shadow_sync_data(struct rga_virt_addr *virt_addr, bool to_shadow)
{
	struct rga_shadow_node *shadow;

	list_for_each_entry(shadow, &virt_addr->shadow_list, node) {
		void *orig_vaddr;
		void *shadow_vaddr;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
		orig_vaddr = kmap_local_page(shadow->orig_page);
		shadow_vaddr = kmap_local_page(shadow->shadow_page);
#else
		orig_vaddr = kmap_atomic(shadow->orig_page);
		shadow_vaddr = kmap_atomic(shadow->shadow_page);
#endif

		if (to_shadow) {
			memcpy(shadow_vaddr + shadow->offset,
			       orig_vaddr + shadow->offset,
			       shadow->len);
		} else {
			memcpy(orig_vaddr + shadow->offset,
			       shadow_vaddr + shadow->offset,
			       shadow->len);
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
		kunmap_local(shadow_vaddr);
		kunmap_local(orig_vaddr);
#else
		kunmap_atomic(shadow_vaddr);
		kunmap_atomic(orig_vaddr);
#endif
	}
}

static inline void rga_shadow_copy_to_shadow(struct rga_virt_addr *virt_addr)
{
	rga_shadow_sync_data(virt_addr, true);
}

static inline void rga_shadow_copy_from_shadow(struct rga_virt_addr *virt_addr)
{
	rga_shadow_sync_data(virt_addr, false);
}

static void rga_current_mm_read_lock(struct mm_struct *mm)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	mmap_read_lock(mm);
#else
	down_read(&mm->mmap_sem);
#endif
}

static void rga_current_mm_read_unlock(struct mm_struct *mm)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	mmap_read_unlock(mm);
#else
	up_read(&mm->mmap_sem);
#endif
}

static int rga_get_user_pages_from_vma(struct page **pages, unsigned long Memory,
				       uint32_t pageCount, struct mm_struct *current_mm)
{
	int ret = 0;
	int i;
	struct vm_area_struct *vma;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
	struct follow_pfnmap_args args;
#else
	spinlock_t *ptl;
	pte_t *pte;
	pgd_t *pgd;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	p4d_t *p4d;
#endif
	pud_t *pud;
	pmd_t *pmd;
	unsigned long pfn;
#endif

	for (i = 0; i < pageCount; i++) {
		vma = find_vma(current_mm, (Memory + i) << PAGE_SHIFT);
		if (!vma) {
			rga_err("page[%d] failed to get vma\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
		/*
		 * __pte_offset_map_lock() is no longer exported to modules on
		 * 6.12+; follow_pfnmap_start() performs the whole page-table
		 * walk under the proper lock and is GPL-exported.
		 */
		args.vma = vma;
		args.address = (Memory + i) << PAGE_SHIFT;
		if (follow_pfnmap_start(&args)) {
			rga_err("page[%d] failed to get pfnmap\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}

		/*
		 * follow_pfnmap_start() is reached precisely for VM_PFNMAP /
		 * VM_IO ranges -- device MMIO, reserved and no-map memory --
		 * where a PFN frequently has no struct page behind it and
		 * pfn_to_page() yields a pointer into a vmemmap hole. That
		 * pointer would go on to rga_alloc_sgt() and be DMA-mapped and
		 * cache-synced. pfn_valid() alone is not enough (it is true for
		 * sparse-memory holes and no-map ranges), so use the same
		 * stricter test the physical-import path already applies.
		 */
		if (!pfn_valid(args.pfn) ||
		    !virt_addr_valid(phys_to_virt(PFN_PHYS(args.pfn)))) {
			rga_err("page[%d] pfn %#lx is not mappable memory\n",
				i, (unsigned long)args.pfn);
			follow_pfnmap_end(&args);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}

		pages[i] = pfn_to_page(args.pfn);
		follow_pfnmap_end(&args);
#else
		pgd = pgd_offset(current_mm, (Memory + i) << PAGE_SHIFT);
		if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd))) {
			rga_err("page[%d] failed to get pgd\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
		/*
		 * In the four-level page table,
		 * it will do nothing and return pgd.
		 */
		p4d = p4d_offset(pgd, (Memory + i) << PAGE_SHIFT);
		if (p4d_none(*p4d) || unlikely(p4d_bad(*p4d))) {
			rga_err("page[%d] failed to get p4d\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}

		pud = pud_offset(p4d, (Memory + i) << PAGE_SHIFT);
#else
		pud = pud_offset(pgd, (Memory + i) << PAGE_SHIFT);
#endif

		if (pud_none(*pud) || unlikely(pud_bad(*pud))) {
			rga_err("page[%d] failed to get pud\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}
		pmd = pmd_offset(pud, (Memory + i) << PAGE_SHIFT);
		if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd))) {
			rga_err("page[%d] failed to get pmd\n", i);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}
		pte = pte_offset_map_lock(current_mm, pmd,
					  (Memory + i) << PAGE_SHIFT, &ptl);
		if (pte_none(*pte)) {
			rga_err("page[%d] failed to get pte\n", i);
			pte_unmap_unlock(pte, ptl);
			ret = RGA_OUT_OF_RESOURCES;
			break;
		}

		pfn = pte_pfn(*pte);
		pages[i] = pfn_to_page(pfn);
		pte_unmap_unlock(pte, ptl);
#endif
	}

	if (ret == RGA_OUT_OF_RESOURCES && i > 0)
		rga_err("Only get buffer %d byte from vma, but current image required %d byte",
			(int)(i * PAGE_SIZE), (int)(pageCount * PAGE_SIZE));

	return ret;
}

static int rga_get_user_pages(struct page **pages, unsigned long Memory,
			      uint32_t pageCount, int writeFlag,
			      struct mm_struct *current_mm)
{
	uint32_t i;
	int32_t ret = 0;
	int32_t result;

	rga_current_mm_read_lock(current_mm);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 168) && \
    LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	result = get_user_pages(current, current_mm, Memory << PAGE_SHIFT,
				pageCount, writeFlag ? FOLL_WRITE : 0,
				pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
	result = get_user_pages(current, current_mm, Memory << PAGE_SHIFT,
				pageCount, writeFlag ? FOLL_WRITE : 0, 0, pages, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	result = get_user_pages_remote(current, current_mm,
				       Memory << PAGE_SHIFT,
				       pageCount, writeFlag ? FOLL_WRITE : 0, pages, NULL, NULL);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 5, 0)
	result = get_user_pages_remote(current_mm, Memory << PAGE_SHIFT,
				       pageCount, writeFlag ? FOLL_WRITE : 0, pages, NULL, NULL);
#else
	result = get_user_pages_remote(current_mm, Memory << PAGE_SHIFT,
				       pageCount, writeFlag ? FOLL_WRITE : 0, pages, NULL);
#endif

	if (result > 0 && result >= pageCount) {
		ret = result;
	} else {
		if (result > 0)
			for (i = 0; i < result; i++)
				put_page(pages[i]);

		ret = rga_get_user_pages_from_vma(pages, Memory, pageCount, current_mm);
		if (ret < 0 && result > 0) {
			rga_err("Only get buffer %d byte from user pages, but current image required %d byte\n",
				(int)(result * PAGE_SIZE), (int)(pageCount * PAGE_SIZE));
		}
	}

	rga_current_mm_read_unlock(current_mm);

	return ret;
}

static int rga_get_phys_addr_pages(struct page **pages, phys_addr_t phys_addr, uint32_t page_count)
{
	u32 i;
	phys_addr_t addr;

	addr = phys_addr;
	for (i = 0; i < page_count; i++) {
		/*
		 * pfn_valid() only proves that a struct page exists.  It can be
		 * true for sparse-memory holes and no-map ranges which cannot be
		 * passed to dma_map_sg().
		 */
		if (!virt_addr_valid(phys_to_virt(addr)))
			return -EINVAL;

		pages[i] = phys_to_page(addr);
		if (i + 1 < page_count &&
		    check_add_overflow(addr, (phys_addr_t)PAGE_SIZE, &addr))
			return -EOVERFLOW;
	}

	return 0;
}

static void rga_free_sgt(struct sg_table **sgt_ptr)
{
	if (sgt_ptr == NULL || *sgt_ptr == NULL)
		return;

	sg_free_table(*sgt_ptr);
	kfree(*sgt_ptr);
	*sgt_ptr = NULL;
}

static unsigned int rga_dma_max_segment_size(struct device *dev)
{
	size_t max_segment = dma_max_mapping_size(dev);

	if (!max_segment || max_segment > UINT_MAX)
		return UINT_MAX;

	return max_segment;
}

static struct sg_table *rga_alloc_sgt_segment(struct page **pages,
					      int page_count, size_t offset,
					      size_t size,
					      unsigned int max_segment,
					      gfp_t gfp_mask)
{
	int ret;
	struct sg_table *sgt = NULL;

	sgt = kzalloc(sizeof(*sgt), gfp_mask);
	if (sgt == NULL) {
		rga_err("%s alloc sgt error!\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* get sg form pages. */
	ret = sg_alloc_table_from_pages_segment(sgt, pages, page_count,
						offset, size, max_segment,
						gfp_mask);
	if (ret) {
		rga_err("sg_alloc_table_from_pages_segment failed");
		goto out_free_sgt;
	}

	return sgt;

out_free_sgt:
	kfree(sgt);

	return ERR_PTR(ret);
}

static struct sg_table *rga_alloc_sgt(struct page **pages,
				      int page_count, size_t offset,
				      size_t size, gfp_t gfp_mask)
{
	return rga_alloc_sgt_segment(pages, page_count, offset, size,
				     UINT_MAX, gfp_mask);
}

static void rga_free_virt_addr(struct rga_virt_addr **virt_addr_p)
{
	int i;
	struct rga_virt_addr *virt_addr = NULL;

	if (virt_addr_p == NULL)
		return;

	virt_addr = *virt_addr_p;
	if (virt_addr == NULL)
		return;

	rga_shadow_release(virt_addr);

	for (i = 0; i < virt_addr->result; i++) {
		if (virt_addr->writable)
			set_page_dirty_lock(virt_addr->pages[i]);
		put_page(virt_addr->pages[i]);
	}

	free_pages((unsigned long)virt_addr->pages, virt_addr->pages_order);
	kfree(virt_addr);
	*virt_addr_p = NULL;
}

static int rga_alloc_virt_addr(struct rga_virt_addr **virt_addr_p,
			       uint64_t viraddr,
			       struct rga_memory_parm *memory_parm,
			       int write_flag,
			       struct mm_struct *mm)
{
	int i;
	int ret;
	int result = 0;
	int order;
	unsigned int count;
	int img_size;
	size_t offset;
	struct page **pages = NULL;
	struct rga_virt_addr *virt_addr = NULL;

	if (memory_parm->size)
		img_size = memory_parm->size;
	else
		img_size = rga_image_size_cal(memory_parm->width,
					      memory_parm->height,
					      memory_parm->format,
					      NULL, NULL, NULL);
	/*
	 * rga_image_size_cal() returns a negative errno for an unsupported
	 * format, and memory_parm->size is an unvalidated u32 that can land
	 * negative in this int. Either way the value ends up in the unsigned
	 * long virt_addr->size and is later used as a memcpy()/cache-sync
	 * length. The !count guard below does not catch it: img_size + offset
	 * promotes to size_t and wraps back to a plausible page count for any
	 * page offset above 14. Reject it here, as the dma-buf and phys-addr
	 * import paths already do.
	 */
	if (img_size <= 0) {
		rga_err("failed to calculating buffer size! img_size = %d\n",
			img_size);
		rga_dump_memory_parm(memory_parm);
		return img_size == 0 ? -EINVAL : img_size;
	}

	offset = viraddr & (~PAGE_MASK);
	count = RGA_GET_PAGE_COUNT(img_size + offset);
	if (!count) {
		rga_err("failed to calculating buffer size! img_size = %d, count = %d, offset = %ld\n",
			img_size, count, (unsigned long)offset);
		rga_dump_memory_parm(memory_parm);
		return -EFAULT;
	}

	/* alloc pages and page_table */
	pages = (struct page **)rga_get_free_pages(GFP_KERNEL,
		&order, count * sizeof(struct page *));
	if (pages == NULL) {
		rga_err("%s can not alloc pages for viraddr pages\n", __func__);
		return -ENOMEM;
	}

	/* get pages from virtual address. */
	ret = rga_get_user_pages(pages, viraddr >> PAGE_SHIFT, count,
				 write_flag, mm);
	if (ret < 0) {
		rga_err("failed to get pages from virtual adrees: 0x%lx\n",
		       (unsigned long)viraddr);
		ret = -EINVAL;
		goto out_free_pages;
	} else if (ret > 0) {
		/* For put pages */
		result = ret;
	}

	*virt_addr_p = kzalloc(sizeof(struct rga_virt_addr), GFP_KERNEL);
	if (*virt_addr_p == NULL) {
		rga_err("%s alloc virt_addr error!\n", __func__);
		ret = -ENOMEM;
		goto out_put_and_free_pages;
	}
	virt_addr = *virt_addr_p;

	virt_addr->addr = viraddr;
	virt_addr->pages = pages;
	virt_addr->pages_order = order;
	virt_addr->page_count = count;
	virt_addr->size = img_size;
	virt_addr->offset = offset;
	virt_addr->writable = write_flag;
	virt_addr->result = result;

	rga_shadow_list_init(virt_addr);

	ret = rga_shadow_setup(virt_addr);
	if (ret < 0) {
		rga_err("shadow_setup failed, va = 0x%lx ret = %d\n",
			(unsigned long)virt_addr->addr, ret);
		goto out_free_virt_addr;
	}

	return 0;

out_free_virt_addr:
	kfree(virt_addr);
	*virt_addr_p = NULL;
out_put_and_free_pages:
	for (i = 0; i < result; i++)
		put_page(pages[i]);
out_free_pages:
	free_pages((unsigned long)pages, order);

	return ret;
}

static inline bool rga_mm_check_memory_limit(struct rga_scheduler_t *scheduler, int mm_flag)
{
	if (!scheduler)
		return false;

	/*
	 * The RGA2 MMU consumes 32-bit page addresses. Over-4G memory is
	 * still usable when it goes through the MMU page tables backed by a
	 * DMA mapping of the 32-bit RGA2 device (the DMA API places or
	 * swiotlb-bounces it below 4G); only memory the hardware must
	 * address directly — physically contiguous buffers bypass the MMU —
	 * has a hard under-4G requirement.
	 */
	if (scheduler->data->mmu == RGA_MMU &&
	    (mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS) &&
	    !(mm_flag & RGA_MEM_UNDER_4G)) {
		rga_err("%s unsupported contiguous memory larger than 4G!\n",
		       rga_get_mmu_type_str(scheduler->data->mmu));
		return false;
	}

	return true;
}

/* If it is within 0~4G, return 1 (true). */
static int rga_mm_check_range_sgt(struct sg_table *sgt)
{
	int i;
	struct scatterlist *sg;
	phys_addr_t s_phys = 0;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		s_phys = sg_phys(sg);
		if ((s_phys > 0xffffffff) || (s_phys + sg->length > 0xffffffff))
			return 0;
	}

	return 1;
}

static inline bool rga_mm_check_contiguous_sgt(struct sg_table *sgt)
{
	if (sgt->orig_nents == 1)
		return true;

	return false;
}

static void rga_mm_unmap_dma_buffer(struct rga_internal_buffer *internal_buffer)
{
	if (rga_mm_is_invalid_dma_buffer(internal_buffer->dma_buffer))
		return;

	rga_dma_unmap_buf(internal_buffer->dma_buffer);

	if (internal_buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS &&
	    internal_buffer->phys_addr > 0)
		internal_buffer->phys_addr = 0;

	kfree(internal_buffer->dma_buffer);
	internal_buffer->dma_buffer = NULL;
}

static int
rga_mm_map_external_dma_buffer(struct rga_external_buffer *external_buffer,
			       struct rga_dma_buffer *buffer,
			       struct rga_scheduler_t *scheduler,
			       struct device *map_dev)
{
	switch (external_buffer->type) {
	case RGA_DMA_BUFFER:
		if (scheduler->data->mmu == RGA_MMU)
			return rga_dma_map_fd_pages((int)external_buffer->memory,
						    buffer, DMA_BIDIRECTIONAL,
						    map_dev);

		return rga_dma_map_fd((int)external_buffer->memory, buffer,
				      DMA_BIDIRECTIONAL, map_dev);
	case RGA_DMA_BUFFER_PTR:
		if (scheduler->data->mmu == RGA_MMU)
			return rga_dma_map_buf_pages((struct dma_buf *)
					u64_to_user_ptr(external_buffer->memory),
					buffer, DMA_BIDIRECTIONAL, map_dev);

		return rga_dma_map_buf((struct dma_buf *)
				u64_to_user_ptr(external_buffer->memory),
				buffer, DMA_BIDIRECTIONAL, map_dev);
	default:
		return -EFAULT;
	}
}

static int rga_mm_map_dma_buffer(struct rga_external_buffer *external_buffer,
				 struct rga_internal_buffer *internal_buffer,
				 struct rga_job *job)
{
	int ret;
	int ex_buffer_size;
	bool rga2_dma_incompatible = false;
	uint32_t mm_flag = 0;
	phys_addr_t phys_addr = 0;
	struct rga_dma_buffer *buffer;
	struct device *map_dev;
	struct device *fallback_dev;
	struct rga_scheduler_t *fallback_scheduler;
	struct rga_scheduler_t *scheduler;

	scheduler = job ? job->scheduler :
		    rga_drvdata->scheduler[rga_drvdata->map_scheduler_index];
	if (scheduler == NULL) {
		rga_err("Invalid scheduler device!\n");
		return -EINVAL;
	}

	if (external_buffer->memory_parm.size)
		ex_buffer_size = external_buffer->memory_parm.size;
	else
		ex_buffer_size = rga_image_size_cal(external_buffer->memory_parm.width,
						    external_buffer->memory_parm.height,
						    external_buffer->memory_parm.format,
						    NULL, NULL, NULL);
	if (ex_buffer_size <= 0) {
		rga_err("failed to calculating buffer size!\n");
		rga_dump_memory_parm(&external_buffer->memory_parm);
		return ex_buffer_size == 0 ? -EINVAL : ex_buffer_size;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL) {
		rga_err("%s alloc internal_buffer error!\n", __func__);
		return  -ENOMEM;
	}

	/*
	 * dma-buf api needs to use default_domain of main dev,
	 * and not IOMMU for devices without iommu_info ptr.
	 */
	map_dev = scheduler->iommu_info ? scheduler->iommu_info->default_dev : scheduler->dev;
	ret = rga_mm_map_external_dma_buffer(external_buffer, buffer,
					     scheduler, map_dev);
	if (ret == -EIO && scheduler->data->mmu == RGA_MMU) {
		/*
		 * The exporter can reject a large high-memory SG entry before
		 * SWIOTLB can map it for RGA2. Keep a normal attachment alive on
		 * the IOMMU-backed map core so the per-job staging path can copy
		 * the DMA-BUF without repeating the doomed RGA2 attachment.
		 */
		fallback_scheduler =
			rga_drvdata->scheduler[rga_drvdata->map_scheduler_index];
		fallback_dev = fallback_scheduler && fallback_scheduler->iommu_info ?
			fallback_scheduler->iommu_info->default_dev :
			fallback_scheduler ? fallback_scheduler->dev : NULL;
		if (fallback_scheduler && fallback_dev && fallback_dev != map_dev) {
			ret = rga_mm_map_external_dma_buffer(external_buffer, buffer,
							     fallback_scheduler,
							     fallback_dev);
			if (!ret)
				rga2_dma_incompatible = true;
		}
	}
	if (ret < 0) {
		rga_err("%s core[%d] map dma buffer error!\n",
			__func__, scheduler->core);
		goto free_buffer;
	}

	if (buffer->size < ex_buffer_size) {
		rga_err("Only get buffer %ld byte from %s = 0x%lx, but current image required %d byte\n",
			buffer->size, rga_get_memory_type_str(external_buffer->type),
			(unsigned long)external_buffer->memory, ex_buffer_size);
		rga_dump_memory_parm(&external_buffer->memory_parm);
		ret = -EINVAL;
		goto unmap_buffer;
	}

	if (scheduler->data->mmu == RGA_IOMMU)
		buffer->iova = buffer->dma_addr;

	if (rga_mm_check_range_sgt(buffer->sgt))
		mm_flag |= RGA_MEM_UNDER_4G;

	/*
	 * If it's physically contiguous, then the RGA_MMU can
	 * directly use the physical address.
	 */
	if (rga_mm_check_contiguous_sgt(buffer->sgt)) {
		phys_addr = sg_phys(buffer->sgt->sgl);
		if (phys_addr == 0) {
			rga_err("%s get physical address error!", __func__);
			ret = -EFAULT;
			goto unmap_buffer;
		}

		mm_flag |= RGA_MEM_PHYSICAL_CONTIGUOUS;
	}

	/* A high DMA-BUF selected on RGA2 must use its MMU or staging. */
	if (scheduler->data->mmu == RGA_MMU &&
	    !(mm_flag & RGA_MEM_UNDER_4G)) {
		mm_flag &= ~RGA_MEM_PHYSICAL_CONTIGUOUS;
		phys_addr = 0;
	}

	if (!rga_mm_check_memory_limit(scheduler, mm_flag)) {
		rga_err("scheduler core[%d] unsupported mm_flag[0x%x]!\n",
			scheduler->core, mm_flag);
		ret = -EINVAL;
		goto unmap_buffer;
	}

	internal_buffer->dma_buffer = buffer;
	internal_buffer->mm_flag = mm_flag;
	internal_buffer->phys_addr = phys_addr ? phys_addr : 0;
	internal_buffer->size = buffer->size - buffer->offset;
	internal_buffer->scheduler = scheduler;
	internal_buffer->rga2_dma_incompatible = rga2_dma_incompatible;

	return 0;

unmap_buffer:
	rga_dma_unmap_buf(buffer);

free_buffer:
	kfree(buffer);

	return ret;
}

static void rga_mm_unmap_virt_addr(struct rga_internal_buffer *internal_buffer)
{
	WARN_ON(internal_buffer->dma_buffer == NULL || internal_buffer->virt_addr == NULL);

	if (rga_mm_is_invalid_dma_buffer(internal_buffer->dma_buffer))
		return;

	rga_dma_unmap_sgt(internal_buffer->dma_buffer);

	if (internal_buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS &&
	    internal_buffer->phys_addr > 0)
		internal_buffer->phys_addr = 0;

	rga_free_sgt(&internal_buffer->dma_buffer->sgt);

	kfree(internal_buffer->dma_buffer);
	internal_buffer->dma_buffer = NULL;

	rga_free_virt_addr(&internal_buffer->virt_addr);

	mmput(internal_buffer->current_mm);
	mmdrop(internal_buffer->current_mm);
	internal_buffer->current_mm = NULL;
}

static int rga_mm_map_virt_addr(struct rga_external_buffer *external_buffer,
				struct rga_internal_buffer *internal_buffer,
				struct rga_job *job, int write_flag)
{
	int ret;
	uint32_t mm_flag = 0;
	size_t real_offset = 0;
	size_t map_offset = 0;
	size_t map_size = 0;
	phys_addr_t phys_addr = 0;
	struct sg_table *sgt;
	struct rga_virt_addr *virt_addr;
	struct rga_dma_buffer *buffer;
	struct device *map_dev;
	struct rga_scheduler_t *scheduler;

	scheduler = job ? job->scheduler :
		    rga_drvdata->scheduler[rga_drvdata->map_scheduler_index];
	if (scheduler == NULL) {
		rga_err("Invalid scheduler device!\n");
		return -EINVAL;
	}

	internal_buffer->current_mm = job ? job->mm : current->mm;
	if (internal_buffer->current_mm == NULL) {
		rga_err("%s, cannot get current mm!\n", __func__);
		return -EFAULT;
	}
	mmgrab(internal_buffer->current_mm);
	mmget(internal_buffer->current_mm);

	ret = rga_alloc_virt_addr(&virt_addr,
				  external_buffer->memory,
				  &internal_buffer->memory_parm,
				  write_flag, internal_buffer->current_mm);
	if (ret < 0) {
		rga_err("Can not alloc rga_virt_addr from 0x%lx\n",
		       (unsigned long)external_buffer->memory);
		goto put_current_mm;
	}

	/*
	 *   When shadow_page active, construct an sg_table with an
	 * offset of 0 and full page coverage or end-of-page coverage
	 * so that each sg entry is aligned with the page. This prevents
	 * dma_map_sg from falling back to the swiotlb bounce buffer
	 * due to a small, non-cacheline-aligned head or tail fragment.
	 *
	 * The actual data start is recovered later via iova + real_offset.
	 */
	if (rga_shadow_active(virt_addr)) {
		if (virt_addr->shadow_head) {
			real_offset = virt_addr->offset;
			map_offset = 0;
			map_size = (size_t)virt_addr->page_count << PAGE_SHIFT;
		} else {
			real_offset = 0;
			map_offset = virt_addr->offset;
			map_size =
				((size_t)virt_addr->page_count << PAGE_SHIFT) - virt_addr->offset;
		}
	} else {
		real_offset = 0;
		map_offset = virt_addr->offset;
		map_size = virt_addr->size;
	}

	if (scheduler->data->mmu == RGA_MMU) {
		/*
		 * RGA2 may bounce high pages through SWIOTLB, whose per-entry
		 * mapping ceiling is smaller than a merged USERPTR run.
		 */
		sgt = rga_alloc_sgt_segment(virt_addr->pages,
					    virt_addr->page_count,
					    map_offset, map_size,
					    rga_dma_max_segment_size(scheduler->dev),
					    GFP_KERNEL);
	} else {
		sgt = rga_alloc_sgt(virt_addr->pages, virt_addr->page_count,
				    map_offset, map_size, GFP_KERNEL);
	}
	if (IS_ERR(sgt)) {
		rga_err("alloc sgt error!\n");
		ret = PTR_ERR(sgt);
		goto free_virt_addr;
	}

	if (rga_mm_check_range_sgt(sgt))
		mm_flag |= RGA_MEM_UNDER_4G;

	if (rga_mm_check_contiguous_sgt(sgt)) {
		phys_addr = sg_phys(sgt->sgl);
		if (phys_addr == 0) {
			rga_err("%s get physical address error!", __func__);
			ret = -EFAULT;
			goto free_sgt;
		}

		phys_addr += real_offset;

		mm_flag |= RGA_MEM_PHYSICAL_CONTIGUOUS;
	} else if (scheduler->data->mmu == RGA_NONE_MMU) {
		rga_err("Current %s[%d] cannot support physically discontinuous virtual address!\n",
			rga_get_mmu_type_str(scheduler->data->mmu), scheduler->data->mmu);
		ret = -EOPNOTSUPP;
		goto free_sgt;
	}

	/*
	 * Some userspace virtual addresses do not have an
	 * interface for flushing the cache, so it is mandatory
	 * to flush the cache when the virtual address is used.
	 */
	mm_flag |= RGA_MEM_FORCE_FLUSH_CACHE;

	if (!rga_mm_check_memory_limit(scheduler, mm_flag)) {
		rga_err("scheduler core[%d] unsupported mm_flag[0x%x]!\n",
			scheduler->core, mm_flag);
		ret = -EINVAL;
		goto free_sgt;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL) {
		rga_err("%s alloc internal dma_buffer error!\n", __func__);
		ret =  -ENOMEM;
		goto free_sgt;
	}

	/*
	 * dma-buf api needs to use default_domain of main dev,
	 * and not IOMMU for devices without iommu_info ptr.
	 */
	map_dev = scheduler->iommu_info ? scheduler->iommu_info->default_dev : scheduler->dev;
	if (scheduler->data->mmu == RGA_MMU)
		/* Page-granular consumer: multi-segment mappings are fine. */
		ret = rga_dma_map_sgt_pages(sgt, buffer, DMA_BIDIRECTIONAL,
					    map_dev);
	else
		ret = rga_dma_map_sgt(sgt, buffer, DMA_BIDIRECTIONAL, map_dev);
	if (ret < 0) {
		rga_err("%s core[%d] rga map sgt failed! va = 0x%lx, orig_nents = %d\n",
			__func__, scheduler->core,
			(unsigned long)virt_addr->addr, sgt->orig_nents);
		goto free_dma_buffer;
	}

	if (scheduler->data->mmu == RGA_IOMMU)
		buffer->iova = buffer->dma_addr;

	buffer->offset = real_offset;

	internal_buffer->virt_addr = virt_addr;
	internal_buffer->dma_buffer = buffer;
	internal_buffer->mm_flag = mm_flag;
	internal_buffer->phys_addr = phys_addr;
	internal_buffer->size = virt_addr->size;
	internal_buffer->scheduler = scheduler;

	return 0;

free_dma_buffer:
	kfree(buffer);
free_sgt:
	rga_free_sgt(&sgt);
free_virt_addr:
	rga_free_virt_addr(&virt_addr);
put_current_mm:
	mmput(internal_buffer->current_mm);
	mmdrop(internal_buffer->current_mm);
	internal_buffer->current_mm = NULL;

	return ret;
}

static void rga_mm_unmap_phys_addr(struct rga_internal_buffer *internal_buffer)
{
	if (internal_buffer->dma_buffer != NULL) {
		rga_dma_unmap_sgt(internal_buffer->dma_buffer);
		rga_free_sgt(&internal_buffer->dma_buffer->sgt);
		kfree(internal_buffer->dma_buffer);
		internal_buffer->dma_buffer = NULL;
	}

	internal_buffer->phys_addr = 0;
	internal_buffer->size = 0;
}

static int rga_mm_map_phys_addr(struct rga_external_buffer *external_buffer,
				struct rga_internal_buffer *internal_buffer,
				struct rga_job *job)
{
	int ret;
	int buffer_size;
	size_t offset;
	uint32_t mm_flag = 0;
	uint32_t page_count;
	phys_addr_t phys_addr_end;
	phys_addr_t phys_addr, phys_addr_aligned;
	struct page **pages = NULL;
	struct sg_table *sgt = NULL;
	struct rga_dma_buffer *buffer = NULL;
	struct device *map_dev;
	struct rga_scheduler_t *scheduler;

	scheduler = job ? job->scheduler :
		    rga_drvdata->scheduler[rga_drvdata->map_scheduler_index];
	if (scheduler == NULL) {
		rga_err("Invalid scheduler device!\n");
		return -EINVAL;
	}

	if (external_buffer->memory_parm.size)
		buffer_size = external_buffer->memory_parm.size;
	else
		buffer_size = rga_image_size_cal(external_buffer->memory_parm.width,
						 external_buffer->memory_parm.height,
						 external_buffer->memory_parm.format,
						 NULL, NULL, NULL);
	if (buffer_size <= 0) {
		rga_err("failed to calculating buffer size!\n");
		rga_dump_memory_parm(&external_buffer->memory_parm);
		return buffer_size == 0 ? -EINVAL : buffer_size;
	}

	phys_addr = external_buffer->memory;
	if (check_add_overflow(phys_addr, (phys_addr_t)buffer_size - 1,
			       &phys_addr_end)) {
		rga_err("physical address range overflows: address = 0x%llx, size = %d\n",
			(unsigned long long)phys_addr, buffer_size);
		return -EOVERFLOW;
	}

	mm_flag |= RGA_MEM_PHYSICAL_CONTIGUOUS;
	if (phys_addr <= U32_MAX && phys_addr_end <= U32_MAX)
		mm_flag |= RGA_MEM_UNDER_4G;

	if (!rga_mm_check_memory_limit(scheduler, mm_flag)) {
		rga_err("scheduler core[%d] unsupported mm_flag[0x%x]!\n",
			scheduler->core, mm_flag);
		return -EINVAL;
	}

	if (scheduler->data->mmu == RGA_IOMMU) {
		phys_addr_aligned = phys_addr & PAGE_MASK;
		offset = phys_addr & (~PAGE_MASK);
		page_count = RGA_GET_PAGE_COUNT(buffer_size + offset);

		pages = vzalloc(sizeof(struct page *) * page_count);
		if (pages == NULL) {
			rga_err("%s can not alloc pages for phys_addr pages\n", __func__);
			return -ENOMEM;
		}

		ret = rga_get_phys_addr_pages(pages, phys_addr_aligned, page_count);
		if (ret < 0) {
			rga_err("failed to get pages from physical address: 0x%llx\n",
				(unsigned long long)phys_addr);
			goto free_pages;
		}

		sgt = rga_alloc_sgt(pages, page_count, offset, buffer_size, GFP_KERNEL);
		if (IS_ERR(sgt)) {
			rga_err("failed to alloc sgt\n");
			ret = PTR_ERR(sgt);
			goto free_pages;
		}

		buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
		if (buffer == NULL) {
			rga_err("%s alloc internal dma buffer error!\n", __func__);
			ret =  -ENOMEM;
			goto free_sgt;
		}

		/*
		 * dma-buf api needs to use default_domain of main dev,
		 * and not IOMMU for devices without iommu_info ptr.
		 */
		map_dev = scheduler->iommu_info ?
			scheduler->iommu_info->default_dev : scheduler->dev;
		ret = rga_dma_map_sgt(sgt, buffer, DMA_BIDIRECTIONAL, map_dev);
		if (ret < 0) {
			rga_err("%s core[%d] map phys_addr error!, phys_addr = 0x%llx\n",
				 __func__, scheduler->core,
				(unsigned long long)phys_addr);
			goto free_dma_buffer;
		}

		buffer->iova = buffer->dma_addr;
		vfree(pages);
	}

	internal_buffer->dma_buffer = buffer;
	internal_buffer->mm_flag = mm_flag;
	internal_buffer->phys_addr = phys_addr;
	internal_buffer->size = buffer ? buffer->size : buffer_size;
	internal_buffer->scheduler = scheduler;

	return 0;

free_dma_buffer:
	kfree(buffer);
free_sgt:
	rga_free_sgt(&sgt);
free_pages:
	vfree(pages);

	return ret;
}

static int rga_mm_unmap_buffer(struct rga_internal_buffer *internal_buffer)
{
	switch (internal_buffer->type) {
	case RGA_DMA_BUFFER:
	case RGA_DMA_BUFFER_PTR:
		rga_mm_unmap_dma_buffer(internal_buffer);
		break;
	case RGA_VIRTUAL_ADDRESS:
		rga_mm_unmap_virt_addr(internal_buffer);
		break;
	case RGA_PHYSICAL_ADDRESS:
		rga_mm_unmap_phys_addr(internal_buffer);
		break;
	default:
		rga_err("Illegal external buffer!\n");
		return -EFAULT;
	}

	return 0;
}

static int rga_mm_map_buffer(struct rga_external_buffer *external_buffer,
			     struct rga_internal_buffer *internal_buffer,
			     struct rga_job *job, int write_flag)
{
	int ret;

	memcpy(&internal_buffer->memory_parm, &external_buffer->memory_parm,
	       sizeof(internal_buffer->memory_parm));

	switch (external_buffer->type) {
	case RGA_DMA_BUFFER:
	case RGA_DMA_BUFFER_PTR:
		internal_buffer->type = external_buffer->type;

		ret = rga_mm_map_dma_buffer(external_buffer, internal_buffer, job);
		if (ret < 0)
			return ret;

		internal_buffer->mm_flag |= RGA_MEM_NEED_USE_IOMMU;
		break;
	case RGA_VIRTUAL_ADDRESS:
		internal_buffer->type = RGA_VIRTUAL_ADDRESS;

		ret = rga_mm_map_virt_addr(external_buffer, internal_buffer, job, write_flag);
		if (ret < 0)
			return ret;

		internal_buffer->mm_flag |= RGA_MEM_NEED_USE_IOMMU;
		break;
	case RGA_PHYSICAL_ADDRESS:
		internal_buffer->type = RGA_PHYSICAL_ADDRESS;

		ret = rga_mm_map_phys_addr(external_buffer, internal_buffer, job);
		if (ret < 0)
			return ret;

		internal_buffer->mm_flag |= RGA_MEM_NEED_USE_IOMMU;
		break;
	default:
		if (job)
			rga_job_err(job, "Illegal external buffer!\n");
		else
			rga_err("Illegal external buffer!\n");

		return -EFAULT;
	}

	return 0;
}

static void rga_mm_kref_release_buffer(struct kref *ref)
{
	struct rga_internal_buffer *internal_buffer;
	struct rga_mm *mm = rga_drvdata->mm;

	internal_buffer = container_of(ref, struct rga_internal_buffer, refcount);
	idr_remove(&mm->memory_idr, internal_buffer->handle);
	mm->buffer_count--;
	mutex_unlock(&mm->lock);

	WARN_ON_ONCE(!list_empty(&internal_buffer->import_list));
	rga_mm_unmap_buffer(internal_buffer);
	kfree(internal_buffer);

	mutex_lock(&mm->lock);
}

/* Force release the current internal_buffer from the IDR. */
static void rga_mm_force_releaser_buffer(struct rga_internal_buffer *buffer)
{
	struct rga_mm *mm = rga_drvdata->mm;
	struct rga_buffer_import *import, *next;

	WARN_ON(!mutex_is_locked(&mm->lock));

	idr_remove(&mm->memory_idr, buffer->handle);
	mm->buffer_count--;
	list_for_each_entry_safe(import, next, &buffer->import_list, node) {
		list_del(&import->node);
		kfree(import);
	}

	rga_mm_unmap_buffer(buffer);
	kfree(buffer);
}

/*
 * Called at driver close to release the memory's handle references.
 */
static int rga_mm_buffer_destroy_for_idr(int id, void *ptr, void *data)
{
	struct rga_internal_buffer *internal_buffer = ptr;

	rga_mm_force_releaser_buffer(internal_buffer);

	return 0;
}

static struct rga_internal_buffer *
rga_mm_lookup_external(struct rga_mm *mm_session,
		       struct rga_external_buffer *external_buffer,
		       struct mm_struct *current_mm)
{
	int id;
	struct dma_buf *dma_buf = NULL;
	struct rga_internal_buffer *temp_buffer = NULL;
	struct rga_internal_buffer *output_buffer = NULL;

	WARN_ON(!mutex_is_locked(&mm_session->lock));

	switch (external_buffer->type) {
	case RGA_DMA_BUFFER:
		dma_buf = dma_buf_get((int)external_buffer->memory);
		if (IS_ERR(dma_buf))
			return (struct rga_internal_buffer *)dma_buf;

		idr_for_each_entry(&mm_session->memory_idr, temp_buffer, id) {
			if (temp_buffer->dma_buffer == NULL)
				continue;

			if (temp_buffer->dma_buffer[0].dma_buf == dma_buf) {
				output_buffer = temp_buffer;
				break;
			}
		}

		dma_buf_put(dma_buf);
		break;
	case RGA_VIRTUAL_ADDRESS:
		idr_for_each_entry(&mm_session->memory_idr, temp_buffer, id) {
			if (temp_buffer->virt_addr == NULL)
				continue;

			if (temp_buffer->virt_addr->addr == external_buffer->memory) {
				if (temp_buffer->current_mm == current_mm) {
					output_buffer = temp_buffer;
					break;
				}

				continue;
			}
		}

		break;
	case RGA_PHYSICAL_ADDRESS:
		idr_for_each_entry(&mm_session->memory_idr, temp_buffer, id) {
			/*
			 * Match only against buffers of the same kind. Every
			 * non-contiguous import leaves phys_addr at 0, so
			 * without this a lookup for physical address 0 returned
			 * the first unrelated buffer in the global idr -- and
			 * the caller took a reference on someone else's memory.
			 */
			if (temp_buffer->type != RGA_PHYSICAL_ADDRESS)
				continue;

			if (temp_buffer->phys_addr == external_buffer->memory) {
				output_buffer = temp_buffer;
				break;
			}
		}

		break;
	case RGA_DMA_BUFFER_PTR:
		idr_for_each_entry(&mm_session->memory_idr, temp_buffer, id) {
			/*
			 * Both dma-buf types resolve to the same struct dma_buf
			 * and are compared pointer-to-pointer below, so an fd
			 * import and a PTR import of one buffer must still
			 * de-dup onto each other -- otherwise every MPI frame
			 * re-attaches and re-maps a buffer userspace already
			 * holds by fd. Only the address-keyed arms need a
			 * strict type match.
			 */
			if (temp_buffer->type != RGA_DMA_BUFFER_PTR &&
			    temp_buffer->type != RGA_DMA_BUFFER)
				continue;

			if (temp_buffer->dma_buffer == NULL)
				continue;

			if ((unsigned long)temp_buffer->dma_buffer[0].dma_buf ==
			    external_buffer->memory) {
				output_buffer = temp_buffer;
				break;
			}
		}

		break;

	default:
		rga_err("Illegal external buffer!\n");
		return NULL;
	}

	return output_buffer;
}

/* Compare a completed mapping without resolving a userspace fd a second time. */
static struct rga_internal_buffer *
rga_mm_lookup_mapped(struct rga_mm *mm, struct rga_internal_buffer *mapped)
{
	struct rga_internal_buffer *buffer;
	int id;

	WARN_ON(!mutex_is_locked(&mm->lock));

	idr_for_each_entry(&mm->memory_idr, buffer, id) {
		switch (mapped->type) {
		case RGA_DMA_BUFFER:
		case RGA_DMA_BUFFER_PTR:
			if ((buffer->type == RGA_DMA_BUFFER ||
			     buffer->type == RGA_DMA_BUFFER_PTR) &&
			    buffer->dma_buffer && mapped->dma_buffer &&
			    buffer->dma_buffer->dma_buf ==
				mapped->dma_buffer->dma_buf)
				return buffer;
			break;
		case RGA_VIRTUAL_ADDRESS:
			if (buffer->type == RGA_VIRTUAL_ADDRESS &&
			    buffer->virt_addr && mapped->virt_addr &&
			    buffer->current_mm == mapped->current_mm &&
			    buffer->virt_addr->addr == mapped->virt_addr->addr)
				return buffer;
			break;
		case RGA_PHYSICAL_ADDRESS:
			if (buffer->type == RGA_PHYSICAL_ADDRESS &&
			    buffer->phys_addr == mapped->phys_addr)
				return buffer;
			break;
		default:
			return NULL;
		}
	}

	return NULL;
}

struct rga_internal_buffer *rga_mm_lookup_handle(struct rga_mm *mm_session, uint32_t handle)
{
	struct rga_internal_buffer *output_buffer;

	WARN_ON(!mutex_is_locked(&mm_session->lock));

	output_buffer = idr_find(&mm_session->memory_idr, handle);

	return output_buffer;
}

static struct rga_buffer_import *
rga_mm_lookup_import(struct rga_internal_buffer *buffer,
		     struct rga_session *session)
{
	struct rga_buffer_import *import;

	list_for_each_entry(import, &buffer->import_list, node)
		if (import->session == session)
			return import;

	return NULL;
}

static bool rga_mm_handle_authorized(struct rga_internal_buffer *buffer,
				     struct rga_session *session)
{
	return session && rga_mm_lookup_import(buffer, session);
}

static int rga_mm_record_import(struct rga_internal_buffer *buffer,
				struct rga_session *session, bool take_ref)
{
	struct rga_buffer_import *import;

	if (!session)
		return -EINVAL;

	import = rga_mm_lookup_import(buffer, session);
	if (import) {
		if (import->count == UINT_MAX)
			return -EOVERFLOW;
		import->count++;
	} else {
		import = kzalloc(sizeof(*import), GFP_KERNEL);
		if (!import)
			return -ENOMEM;

		import->session = session;
		import->count = 1;
		list_add_tail(&import->node, &buffer->import_list);
		if (!buffer->session)
			buffer->session = session;
	}

	if (take_ref)
		kref_get(&buffer->refcount);

	return 0;
}

static void rga_mm_refresh_diagnostic_owner(struct rga_internal_buffer *buffer)
{
	struct rga_buffer_import *import;

	if (list_empty(&buffer->import_list)) {
		buffer->session = NULL;
		return;
	}

	import = list_first_entry(&buffer->import_list,
				  struct rga_buffer_import, node);
	buffer->session = import->session;
}

int rga_mm_lookup_flag(struct rga_mm *mm_session, uint64_t handle)
{
	struct rga_internal_buffer *output_buffer;

	output_buffer = rga_mm_lookup_handle(mm_session, handle);
	if (output_buffer == NULL) {
		rga_err("This handle[%ld] is illegal.\n", (unsigned long)handle);
		return -EINVAL;
	}

	return output_buffer->mm_flag;
}

int rga_mm_lookup_rga2_support(struct rga_mm *mm_session, uint64_t handle,
			       struct rga_session *session)
{
	struct rga_internal_buffer *buffer;

	buffer = rga_mm_lookup_handle(mm_session, handle);
	if (buffer == NULL) {
		rga_err("This handle[%ld] is illegal.\n", (unsigned long)handle);
		return -EINVAL;
	}
	if (!rga_mm_handle_authorized(buffer, session)) {
		rga_err("This session did not import handle[%ld].\n",
			(unsigned long)handle);
		return -EPERM;
	}

	if (buffer->mm_flag & RGA_MEM_UNDER_4G)
		return RGA2_BUFFER_DIRECT;

	/*
	 * A high DMA-BUF is stageable even when its exporter table is one
	 * physically contiguous entry. Raw high physical memory is not: only
	 * the exporter can authorize CPU access and lifetime for a copy.
	 */
	if (buffer->type == RGA_DMA_BUFFER ||
	    buffer->type == RGA_DMA_BUFFER_PTR)
		return buffer->dma_buffer && buffer->dma_buffer->dma_buf ?
		       RGA2_BUFFER_STAGEABLE : RGA2_BUFFER_UNSUPPORTED;

	/* Over-4G contiguous non-DMA-BUF memory bypasses the RGA2 MMU. */
	if (buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS)
		return RGA2_BUFFER_UNSUPPORTED;

	/*
	 * Over-4G memory is servable through the RGA2 MMU when a transient
	 * per-job DMA mapping of the 32-bit RGA2 device can bounce it below
	 * 4G: dma-buf imports re-attach, virtual imports re-map their pages.
	 */
	switch (buffer->type) {
	case RGA_DMA_BUFFER:
	case RGA_DMA_BUFFER_PTR:
		return RGA2_BUFFER_STAGEABLE;
	case RGA_VIRTUAL_ADDRESS:
		return buffer->virt_addr && buffer->virt_addr->pages ?
		       RGA2_BUFFER_DIRECT : RGA2_BUFFER_UNSUPPORTED;
	default:
		return RGA2_BUFFER_UNSUPPORTED;
	}
}

dma_addr_t rga_mm_lookup_iova(struct rga_internal_buffer *buffer)
{
	if (rga_mm_is_invalid_dma_buffer(buffer->dma_buffer))
		return DMA_MAPPING_ERROR;

	return buffer->dma_buffer->iova + buffer->dma_buffer->offset;
}

struct sg_table *rga_mm_lookup_sgt(struct rga_internal_buffer *buffer)
{
	if (rga_mm_is_invalid_dma_buffer(buffer->dma_buffer))
		return NULL;

	return buffer->dma_buffer->sgt;
}

static void rga_mm_dump_shadow_page(struct rga_internal_buffer *dump_buffer)
{
	struct rga_virt_addr *virt_addr;
	struct rga_shadow_node *shadow;

	virt_addr = dump_buffer->virt_addr;

	rga_buf_log(dump_buffer, "shadow_page active, head = %d, tail = %d\n",
			virt_addr->shadow_head, virt_addr->shadow_tail);

	list_for_each_entry(shadow, &virt_addr->shadow_list, node)
		rga_buf_log(dump_buffer,
			"shadow_page[%d], offset = %zu, len = %zu\n",
			shadow->page_idx, shadow->offset, shadow->len);
}

void rga_mm_dump_buffer(struct rga_internal_buffer *dump_buffer)
{
	rga_buf_log(dump_buffer, "type = %s, refcount = %d mm_flag = 0x%x, size = %ld\n",
		rga_get_memory_type_str(dump_buffer->type),
		kref_read(&dump_buffer->refcount),
		dump_buffer->mm_flag,
		dump_buffer->size);

	switch (dump_buffer->type) {
	case RGA_DMA_BUFFER:
	case RGA_DMA_BUFFER_PTR:
		if (rga_mm_is_invalid_dma_buffer(dump_buffer->dma_buffer))
			break;

		rga_buf_log(dump_buffer, "dma_buf = %p\n",
			dump_buffer->dma_buffer->dma_buf);
		rga_buf_log(dump_buffer, "iova = 0x%lx, dma_addr = 0x%lx, offset = 0x%lx, sgt = %p, size = %ld, map_core = 0x%x\n",
			(unsigned long)dump_buffer->dma_buffer->iova,
			(unsigned long)dump_buffer->dma_buffer->dma_addr,
			(unsigned long)dump_buffer->dma_buffer->offset,
			dump_buffer->dma_buffer->sgt,
			dump_buffer->dma_buffer->size,
			dump_buffer->scheduler->core);

		if (dump_buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS)
			rga_log("is contiguous, pa = 0x%lx\n",
				(unsigned long)dump_buffer->phys_addr);
		break;
	case RGA_VIRTUAL_ADDRESS:
		if (dump_buffer->virt_addr == NULL)
			break;

		rga_buf_log(dump_buffer, "va = 0x%lx, pages = %p, size = %ld\n",
			(unsigned long)dump_buffer->virt_addr->addr,
			dump_buffer->virt_addr->pages,
			dump_buffer->virt_addr->size);

		if (rga_mm_is_invalid_dma_buffer(dump_buffer->dma_buffer))
			break;

		rga_buf_log(dump_buffer, "iova = 0x%lx, dma_addr = 0x%lx, offset = 0x%lx, sgt = %p, size = %ld, map_core = 0x%x\n",
			(unsigned long)dump_buffer->dma_buffer->iova,
			(unsigned long)dump_buffer->dma_buffer->dma_addr,
			(unsigned long)dump_buffer->dma_buffer->offset,
			dump_buffer->dma_buffer->sgt,
			dump_buffer->dma_buffer->size,
			dump_buffer->scheduler->core);

		if (dump_buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS)
			rga_buf_log(dump_buffer, "is contiguous, pa = 0x%lx\n",
				(unsigned long)dump_buffer->phys_addr);

		if (rga_shadow_active(dump_buffer->virt_addr))
			rga_mm_dump_shadow_page(dump_buffer);

		break;
	case RGA_PHYSICAL_ADDRESS:
		rga_buf_log(dump_buffer, "pa = 0x%lx\n", (unsigned long)dump_buffer->phys_addr);

		if (rga_mm_is_invalid_dma_buffer(dump_buffer->dma_buffer))
			break;

		rga_buf_log(dump_buffer, "iova = 0x%lx, dma_addr = 0x%lx, offset = 0x%lx, size = %ld, map_core = 0x%x\n",
			(unsigned long)dump_buffer->dma_buffer->iova,
			(unsigned long)dump_buffer->dma_buffer->dma_addr,
			(unsigned long)dump_buffer->dma_buffer->offset,
			dump_buffer->dma_buffer->size,
			dump_buffer->scheduler->core);
		break;
	default:
		rga_buf_err(dump_buffer, "Illegal buffer! type= %d\n", dump_buffer->type);
		break;
	}
}

void rga_mm_dump_info(struct rga_mm *mm_session)
{
	int id;
	struct rga_internal_buffer *dump_buffer;

	WARN_ON(!mutex_is_locked(&mm_session->lock));

	rga_log("rga mm info:\n");

	rga_log("buffer count = %d\n", mm_session->buffer_count);
	rga_log("===============================================================\n");

	idr_for_each_entry(&mm_session->memory_idr, dump_buffer, id) {
		rga_mm_dump_buffer(dump_buffer);

		rga_log("---------------------------------------------------------------\n");
	}
}

static bool rga_mm_is_need_mmu(struct rga_job *job, struct rga_internal_buffer *buffer)
{
	if (buffer == NULL || job == NULL || job->scheduler == NULL)
		return false;

	/* RK_IOMMU no need to configure enable or not in the driver. */
	if (job->scheduler->data->mmu == RGA_IOMMU)
		return false;

	/*
	 * High DMA-BUFs that look physically contiguous still need the RGA2
	 * MMU: their direct address is outside the 32-bit aperture and the
	 * per-job path may replace them with DMA32 staging pages.
	 */
	if (buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS) {
		if (!(buffer->mm_flag & RGA_MEM_UNDER_4G) &&
		    (buffer->type == RGA_DMA_BUFFER ||
		     buffer->type == RGA_DMA_BUFFER_PTR))
			return true;

		return false;
	}

	return buffer->mm_flag & RGA_MEM_NEED_USE_IOMMU;
}

static int rga_mm_set_mmu_flag(struct rga_job *job,
			       struct rga_job_task_buffers *buffers,
			       struct rga_req *req)
{
	struct rga_mmu_t *mmu_info;
	int src_mmu_en;
	int src1_mmu_en;
	int dst_mmu_en;
	int els_mmu_en;

	src_mmu_en = rga_mm_is_need_mmu(job, buffers->src_buffer.addr);
	src1_mmu_en = rga_mm_is_need_mmu(job, buffers->src1_buffer.addr);
	dst_mmu_en = rga_mm_is_need_mmu(job, buffers->dst_buffer.addr);
	els_mmu_en = rga_mm_is_need_mmu(job, buffers->els_buffer.addr);

	mmu_info = &req->mmu_info;
	memset(mmu_info, 0x0, sizeof(*mmu_info));
	if (src_mmu_en)
		mmu_info->mmu_flag |= (0x1 << 8);
	if (src1_mmu_en)
		mmu_info->mmu_flag |= (0x1 << 9);
	if (dst_mmu_en)
		mmu_info->mmu_flag |= (0x1 << 10);
	if (els_mmu_en)
		mmu_info->mmu_flag |= (0x1 << 11);

	if (mmu_info->mmu_flag & (0xf << 8)) {
		mmu_info->mmu_flag |= 1;
		mmu_info->mmu_flag |= 1 << 31;
		mmu_info->mmu_en  = 1;
	}

	return 0;
}

static int rga_mm_emit_page_table_run(u64 address, u64 length,
				      uint32_t *page_table,
				      uint32_t page_count,
				      uint32_t *mapped_count)
{
	u64 base = address & PAGE_MASK;
	u64 span;
	u64 pages;
	u64 i;

	if (!length || check_add_overflow((u64)offset_in_page(address),
					 length, &span))
		return -EINVAL;
	pages = DIV_ROUND_UP_ULL(span, PAGE_SIZE);

	for (i = 0; i < pages && *mapped_count < page_count; i++) {
		u64 pte;

		if (check_add_overflow(base, i << PAGE_SHIFT, &pte))
			return -EOVERFLOW;
		/* RGA2 PTEs contain full 32-bit page addresses. */
		if (pte > SZ_4G - PAGE_SIZE)
			return -EOPNOTSUPP;
		page_table[(*mapped_count)++] = (uint32_t)pte;
	}

	return 0;
}

static int rga_mm_sgt_to_page_table(struct sg_table *sg,
				    uint32_t *page_table,
				    int32_t pageCount,
				    int32_t use_dma_address)
{
	struct scatterlist *sgl;
	u64 run_address = 0;
	u64 run_length = 0;
	u32 mapped_count = 0;
	unsigned int i;
	int ret;

	if (!sg || !sg->sgl || !page_table || pageCount <= 0)
		return -EINVAL;

#define RGA_MM_ACCUMULATE_ENTRY(_address, _length) do { \
		u64 __address = (_address); \
		u64 __length = (_length); \
		u64 __run_end; \
		if (!__length || check_add_overflow(__address, __length, &__run_end)) \
			return -EINVAL; \
		if (!run_length) { \
			run_address = __address; \
			run_length = __length; \
		} else { \
			u64 __previous_end; \
			if (check_add_overflow(run_address, run_length, &__previous_end)) \
				return -EOVERFLOW; \
			if (__address == __previous_end) { \
				if (check_add_overflow(run_length, __length, &run_length)) \
					return -EOVERFLOW; \
			} else { \
				ret = rga_mm_emit_page_table_run(run_address, run_length, \
							       page_table, pageCount, \
							       &mapped_count); \
				if (ret || mapped_count == pageCount) \
					return ret; \
				if (!IS_ALIGNED(__previous_end, PAGE_SIZE) || \
				    !IS_ALIGNED(__address, PAGE_SIZE)) \
					return -EOPNOTSUPP; \
				run_address = __address; \
				run_length = __length; \
			} \
		} \
	} while (0)

	if (use_dma_address) {
		for_each_sgtable_dma_sg(sg, sgl, i)
			RGA_MM_ACCUMULATE_ENTRY(sg_dma_address(sgl),
						 sg_dma_len(sgl));
	} else {
		for_each_sg(sg->sgl, sgl, sg->orig_nents, i)
			RGA_MM_ACCUMULATE_ENTRY(sg_phys(sgl), sgl->length);
	}

#undef RGA_MM_ACCUMULATE_ENTRY

	if (run_length) {
		ret = rga_mm_emit_page_table_run(run_address, run_length,
					       page_table, pageCount,
					       &mapped_count);
		if (ret)
			return ret;
	}

	if (mapped_count != pageCount)
		return -EINVAL;

	return 0;
}

static bool rga_mm_buffer_uses_dma_address(struct rga_job *job,
					   struct rga_internal_buffer *buffer)
{
	/*
	 * A buffer mapped directly against the executing RGA2 device (the
	 * per-job mapping path) already carries device-usable DMA addresses:
	 * below-4G pages map 1:1 and over-4G pages are swiotlb-bounced by
	 * the 32-bit DMA mask.
	 */
	return buffer->dma_buffer != NULL &&
	       !buffer->dma_buffer->iommu_mapped &&
	       buffer->dma_buffer->map_dev == job->scheduler->dev;
}

static int rga2_stage_copy_from_origin(struct rga_rga2_stage *stage)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	int end_ret;
	int ret;

	ret = dma_buf_begin_cpu_access(stage->origin, DMA_BIDIRECTIONAL);
	if (ret)
		return ret;

	ret = dma_buf_vmap_unlocked(stage->origin, &map);
	if (!ret) {
		iosys_map_memcpy_from(stage->vaddr, &map, 0, stage->size);
		dma_buf_vunmap_unlocked(stage->origin, &map);
	}

	end_ret = dma_buf_end_cpu_access(stage->origin, DMA_BIDIRECTIONAL);
	if (!ret)
		ret = end_ret;
	if (ret)
		return ret;

	dma_sync_sg_for_device(stage->mapping.map_dev, stage->sgt->sgl,
			       stage->sgt->orig_nents, DMA_BIDIRECTIONAL);
	atomic64_add(stage->size, &rga2_stage_copy_in_bytes);

	return 0;
}

static int rga2_stage_copy_to_origin(struct rga_rga2_stage *stage)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	int end_ret;
	int ret;

	dma_sync_sg_for_cpu(stage->mapping.map_dev, stage->sgt->sgl,
			    stage->sgt->orig_nents, DMA_BIDIRECTIONAL);

	ret = dma_buf_begin_cpu_access(stage->origin, DMA_BIDIRECTIONAL);
	if (ret)
		return ret;

	ret = dma_buf_vmap_unlocked(stage->origin, &map);
	if (!ret) {
		iosys_map_memcpy_to(&map, 0, stage->vaddr, stage->size);
		dma_buf_vunmap_unlocked(stage->origin, &map);
	}

	end_ret = dma_buf_end_cpu_access(stage->origin, DMA_BIDIRECTIONAL);
	if (!ret)
		ret = end_ret;
	if (!ret)
		atomic64_add(stage->size, &rga2_stage_copy_out_bytes);

	return ret;
}

static int rga2_stage_reserve(atomic64_t *counter, size_t size, s64 limit)
{
	s64 old = atomic64_read(counter);

	for (;;) {
		if (old < 0 || size > limit - old)
			return -EDQUOT;
		if (atomic64_try_cmpxchg(counter, &old, old + size))
			return 0;
	}
}

static int rga2_stage_charge(struct rga_rga2_stage *stage,
			     struct rga_session *session)
{
	int ret;

	ret = rga2_stage_reserve(&session->rga2_stage_active_bytes,
				 stage->size, RGA2_STAGE_SESSION_MAX_SIZE);
	if (ret)
		return ret;

	ret = rga2_stage_reserve(&rga2_stage_active_bytes, stage->size,
				 RGA2_STAGE_GLOBAL_MAX_SIZE);
	if (ret) {
		atomic64_sub(stage->size, &session->rga2_stage_active_bytes);
		return ret;
	}

	stage->session = session;
	stage->budget_charged = true;
	rga2_stage_update_peak(atomic64_read(&rga2_stage_active_bytes));

	return 0;
}

static void rga2_stage_uncharge(struct rga_rga2_stage *stage)
{
	if (!stage->budget_charged)
		return;

	atomic64_sub(stage->size, &rga2_stage_active_bytes);
	atomic64_sub(stage->size, &stage->session->rga2_stage_active_bytes);
	stage->budget_charged = false;
}

static void rga2_stage_free(struct rga_rga2_stage *stage)
{
	unsigned int i;

	rga2_stage_uncharge(stage);

	if (stage->mapping.map_dev)
		rga_dma_unmap_sgt(&stage->mapping);
	if (stage->sgt)
		rga_free_sgt(&stage->sgt);
	if (stage->vaddr)
		vunmap(stage->vaddr);
	if (stage->pages) {
		for (i = 0; i < stage->page_count; i++)
			if (stage->pages[i])
				__free_page(stage->pages[i]);
		kvfree(stage->pages);
	}
	if (stage->origin)
		dma_buf_put(stage->origin);
	kfree(stage);
}

static struct rga_rga2_stage *
rga2_stage_get(struct rga_job *job, struct rga_internal_buffer *buffer)
{
	struct rga_rga2_stage *stage;
	struct dma_buf *origin;
	size_t staged_bytes = 0;
	unsigned int i;
	int ret;

	if (!buffer->dma_buffer || !buffer->dma_buffer->dma_buf)
		return ERR_PTR(-EOPNOTSUPP);

	origin = buffer->dma_buffer->dma_buf;
	atomic64_inc(&rga2_stage_attempt_count);
	list_for_each_entry(stage, &job->rga2_stage_list, node) {
		if (stage->origin == origin) {
			stage->users++;
			atomic64_inc(&rga2_stage_reuse_count);
			return stage;
		}
		if (check_add_overflow(staged_bytes, stage->size,
				       &staged_bytes)) {
			ret = -EOVERFLOW;
			goto err_count;
		}
	}

	if (!origin->size || staged_bytes > RGA2_STAGE_MAX_SIZE ||
	    origin->size > RGA2_STAGE_MAX_SIZE - staged_bytes) {
		ret = -E2BIG;
		goto err_count;
	}

	stage = kzalloc(sizeof(*stage), GFP_KERNEL);
	if (!stage) {
		ret = -ENOMEM;
		goto err_count;
	}

	INIT_LIST_HEAD(&stage->node);
	stage->origin = origin;
	get_dma_buf(origin);
	stage->size = origin->size;
	ret = rga2_stage_charge(stage, job->session);
	if (ret)
		goto err_free;

	stage->page_count = DIV_ROUND_UP(stage->size, PAGE_SIZE);
	stage->pages = kvcalloc(stage->page_count, sizeof(*stage->pages),
				GFP_KERNEL);
	if (!stage->pages) {
		ret = -ENOMEM;
		goto err_free;
	}

	for (i = 0; i < stage->page_count; i++) {
		stage->pages[i] = alloc_page(GFP_KERNEL | GFP_DMA32 |
					     __GFP_NOWARN | __GFP_NORETRY);
		if (!stage->pages[i]) {
			ret = -ENOMEM;
			goto err_free;
		}
	}

	stage->vaddr = vmap(stage->pages, stage->page_count, VM_MAP,
			    PAGE_KERNEL);
	if (!stage->vaddr) {
		ret = -ENOMEM;
		goto err_free;
	}

	/* One-page source entries make the staging map independent of SWIOTLB. */
	stage->sgt = rga_alloc_sgt_segment(stage->pages, stage->page_count, 0,
					   stage->size, PAGE_SIZE,
					   GFP_KERNEL);
	if (IS_ERR(stage->sgt)) {
		ret = PTR_ERR(stage->sgt);
		stage->sgt = NULL;
		goto err_free;
	}

	ret = rga_dma_map_sgt_pages(stage->sgt, &stage->mapping,
				    DMA_BIDIRECTIONAL, job->scheduler->dev);
	if (ret)
		goto err_free;

	ret = rga2_stage_copy_from_origin(stage);
	if (ret)
		goto err_free;

	stage->users = 1;
	list_add_tail(&stage->node, &job->rga2_stage_list);
	atomic64_inc(&rga2_stage_success_count);
	atomic64_inc(&rga2_stage_active_count);

	return stage;

err_free:
	rga2_stage_free(stage);
err_count:
	atomic64_inc(&rga2_stage_failure_count);
	return ERR_PTR(ret);
}

static bool rga2_stage_job_succeeded(struct rga_job *job)
{
	return job->ret == 0 &&
	       test_bit(RGA_JOB_STATE_FINISH, &job->state) &&
	       !test_bit(RGA_JOB_STATE_INTR_ERR, &job->state) &&
	       job->finished_count == job->task_count;
}

static void rga2_stage_put(struct rga_job *job,
			   struct rga_rga2_stage *stage,
			   enum dma_data_direction dir)
{
	int ret = 0;

	if (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL)
		stage->copy_back = true;

	if (--stage->users)
		return;

	list_del_init(&stage->node);
	if (stage->copy_back && rga2_stage_job_succeeded(job)) {
		ret = rga2_stage_copy_to_origin(stage);
		if (ret) {
			rga_job_err(job,
				    "RGA2: DMA32 staging copy-back failed (%d)\n",
				    ret);
			job->ret = ret;
			atomic64_inc(&rga2_stage_failure_count);
		}
	}

	atomic64_dec(&rga2_stage_active_count);
	rga2_stage_free(stage);
}

static void rga_mm_put_rga2_bounce(struct rga_job *job,
				   struct rga_job_buffer *job_buf,
				   enum dma_data_direction dir)
{
	int i;

	for (i = 0; i < job_buf->rga2_bounce_count; i++) {
		struct rga_dma_buffer *bounce = job_buf->rga2_bounce[i];
		struct rga_internal_buffer *origin = job_buf->rga2_bounce_origin[i];

		if (bounce->attach) {
			rga_dma_unmap_buf(bounce);
		} else {
			rga_dma_unmap_sgt(bounce);
			rga_free_sgt(&bounce->sgt);
		}

		/*
		 * The bounce copy-back dirties the origin pages through the
		 * kernel mapping; clean them to the point of coherency via
		 * the origin's persistent mapping so that a later cache
		 * invalidate (exporter sync, FORCE_FLUSH put) cannot discard
		 * the copied-back data. This applies regardless of how the
		 * persistent mapping is made (the default map core is
		 * IOMMU-backed and its sync_for_cpu still invalidates); only
		 * read-only bounces have nothing copied back. A raw sync is
		 * used on purpose: the shadow-page logic of
		 * rga_mm_sync_dma_sg_for_device() must not run here, it
		 * would overwrite device output.
		 */
		if (bounce->dir != DMA_TO_DEVICE &&
		    origin != NULL && origin->dma_buffer != NULL &&
		    origin->dma_buffer->sgt != NULL &&
		    origin->dma_buffer->map_dev != NULL)
			dma_sync_sg_for_device(origin->dma_buffer->map_dev,
					       origin->dma_buffer->sgt->sgl,
					       origin->dma_buffer->sgt->orig_nents,
					       DMA_TO_DEVICE);

		kfree(bounce);
		job_buf->rga2_bounce[i] = NULL;
		job_buf->rga2_bounce_origin[i] = NULL;
	}
	job_buf->rga2_bounce_count = 0;

	for (i = 0; i < job_buf->rga2_stage_count; i++) {
		rga2_stage_put(job, job_buf->rga2_stage[i], dir);
		job_buf->rga2_stage[i] = NULL;
	}
	job_buf->rga2_stage_count = 0;
}

/*
 * Return the sg table an RGA2 page table for @buffer should be built from,
 * and whether to consume sg_dma_address() (device-usable, possibly bounced
 * below 4G) or sg_phys(). Over-4G buffers that are only mapped for another
 * core get a transient per-job mapping against the RGA2 device here.
 */
static struct sg_table *rga_mm_get_rga2_sgt(struct rga_job *job,
					    struct rga_job_buffer *job_buf,
					    struct rga_internal_buffer *buffer,
					    enum dma_data_direction dir,
					    int32_t *use_dma_address)
{
	struct rga_dma_buffer *bounce;
	struct rga_rga2_stage *stage;
	struct sg_table *sgt;
	int ret;

	*use_dma_address = false;

	if (buffer == NULL)
		return NULL;

	/*
	 * High DMA-BUFs always use the job-shared stage. A successful SWIOTLB
	 * mapping is still private to one channel, so aliases in a sequential
	 * job would otherwise operate on independent snapshots and copy back
	 * stale data in unmap order.
	 */
	if (!(buffer->mm_flag & RGA_MEM_UNDER_4G) &&
	    (buffer->type == RGA_DMA_BUFFER ||
	     buffer->type == RGA_DMA_BUFFER_PTR))
		goto stage_dma_buf;

	if (rga_mm_buffer_uses_dma_address(job, buffer)) {
		*use_dma_address = true;
		return rga_mm_lookup_sgt(buffer);
	}

	if (buffer->mm_flag & RGA_MEM_UNDER_4G)
		return rga_mm_lookup_sgt(buffer);
	if (job_buf->rga2_bounce_count >= ARRAY_SIZE(job_buf->rga2_bounce))
		return ERR_PTR(-EOPNOTSUPP);

	bounce = kzalloc(sizeof(*bounce), GFP_KERNEL);
	if (bounce == NULL)
		return ERR_PTR(-ENOMEM);

	/*
	 * The channel get-side @dir is DMA_TO_DEVICE for every channel
	 * (including the write channel, whose put side uses
	 * DMA_FROM_DEVICE), so it must not choose the bounce mapping
	 * direction: swiotlb only copies a bounce back to the origin on
	 * unmap for DMA_FROM_DEVICE/DMA_BIDIRECTIONAL, and a
	 * DMA_TO_DEVICE-mapped write channel would silently discard the
	 * device output. Map bidirectionally, like the persistent
	 * mappings; the map-time forward copy also preserves
	 * destination pixels the job does not write.
	 */
	switch (buffer->type) {
	case RGA_DMA_BUFFER:
	case RGA_DMA_BUFFER_PTR:
		if (buffer->dma_buffer == NULL ||
		    buffer->dma_buffer->dma_buf == NULL) {
			ret = -EOPNOTSUPP;
			break;
		}

		ret = rga_dma_map_buf_pages(buffer->dma_buffer->dma_buf,
					    bounce, DMA_BIDIRECTIONAL,
					    job->scheduler->dev);
		break;
	case RGA_VIRTUAL_ADDRESS:
		if (buffer->virt_addr == NULL ||
		    buffer->virt_addr->pages == NULL) {
			ret = -EOPNOTSUPP;
			break;
		}

		sgt = rga_alloc_sgt_segment(buffer->virt_addr->pages,
					    buffer->virt_addr->page_count, 0,
					    (size_t)buffer->virt_addr->page_count << PAGE_SHIFT,
					    rga_dma_max_segment_size(job->scheduler->dev),
					    GFP_KERNEL);
		if (IS_ERR(sgt)) {
			ret = PTR_ERR(sgt);
			break;
		}

		ret = rga_dma_map_sgt_pages(sgt, bounce, DMA_BIDIRECTIONAL,
					    job->scheduler->dev);
		if (ret < 0)
			rga_free_sgt(&sgt);
		break;
	default:
		/* Physical addresses cannot be remapped below 4G. */
		ret = -EOPNOTSUPP;
		break;
	}

	if (ret < 0) {
		kfree(bounce);
		rga_job_err(job,
			    "RGA2: can not map over-4G buffer below 4G (%d); use below-4G (e.g. CMA/DMA32) buffers\n",
			    ret);
		return ERR_PTR(-EOPNOTSUPP);
	}

	job_buf->rga2_bounce[job_buf->rga2_bounce_count] = bounce;
	job_buf->rga2_bounce_origin[job_buf->rga2_bounce_count] = buffer;
	job_buf->rga2_bounce_count++;

	*use_dma_address = true;
	return bounce->sgt;

stage_dma_buf:
	if (job_buf->rga2_stage_count >= ARRAY_SIZE(job_buf->rga2_stage))
		return ERR_PTR(-EOPNOTSUPP);

	stage = rga2_stage_get(job, buffer);
	if (IS_ERR(stage)) {
		ret = PTR_ERR(stage);
		rga_job_err(job,
			    "RGA2: DMA-BUF needs DMA32 staging but staging failed (%d)\n",
			    ret);
		return ERR_PTR(ret);
	}

	job_buf->rga2_stage[job_buf->rga2_stage_count++] = stage;
	*use_dma_address = true;

	return stage->sgt;
}

static int rga_mm_set_mmu_base(struct rga_job *job,
			       struct rga_img_info_t *img,
			       struct rga_job_buffer *job_buf,
			       enum dma_data_direction dir)
{
	int ret;
	int yrgb_count = 0;
	int uv_count = 0;
	int v_count = 0;
	int page_count = 0;
	int order = 0;
	int32_t use_dma_address = false;
	uint32_t *page_table = NULL;
	struct sg_table *sgt = NULL;

	int img_size, yrgb_size, uv_size, v_size;
	int img_offset = 0;
	int yrgb_offset = 0;
	int uv_offset = 0;
	int v_offset = 0;

	img_size = rga_image_size_cal(img->vir_w, img->vir_h, img->format,
				      &yrgb_size, &uv_size, &v_size);
	if (img_size <= 0) {
		rga_job_err(job, "Image size cal error! width = %d, height = %d, format = %s\n",
			img->vir_w, img->vir_h, rga_get_format_name(img->format));
		return -EINVAL;
	}

	/* using third-address */
	if (job_buf->uv_addr) {
		if (job_buf->y_addr && job_buf->y_addr->virt_addr != NULL)
			yrgb_offset = job_buf->y_addr->virt_addr->offset;
		if (job_buf->uv_addr && job_buf->uv_addr->virt_addr != NULL)
			uv_offset = job_buf->uv_addr->virt_addr->offset;
		if (job_buf->v_addr && job_buf->v_addr->virt_addr != NULL)
			v_offset = job_buf->v_addr->virt_addr->offset;

		yrgb_count = RGA_GET_PAGE_COUNT(yrgb_size + yrgb_offset);
		uv_count = RGA_GET_PAGE_COUNT(uv_size + uv_offset);
		v_count = RGA_GET_PAGE_COUNT(v_size + v_offset);
		page_count = yrgb_count + uv_count + v_count;

		if (page_count <= 0) {
			rga_job_err(job, "page count cal error! yrba = %d, uv = %d, v = %d\n",
				yrgb_count, uv_count, v_count);
			return -EFAULT;
		}

		/*
		 * Plane counts come from the format, while plane buffers are
		 * populated only for non-zero handles. Reject missing required
		 * planes before building an incomplete RGA MMU page table.
		 */
		if ((yrgb_count && !job_buf->y_addr) ||
		    (uv_count && !job_buf->uv_addr) ||
		    (v_count && !job_buf->v_addr)) {
			rga_job_err(job,
				    "multi-plane format missing a plane buffer\n");
			return -EINVAL;
		}

		if (job->flags & RGA_JOB_USE_HANDLE) {
			page_table = (uint32_t *)rga_get_free_pages(GFP_KERNEL | GFP_DMA32,
				&order, page_count * sizeof(uint32_t *));
			if (page_table == NULL) {
				rga_job_err(job, "%s can not alloc pages for page_table, order = %d\n",
					__func__, order);
				return -ENOMEM;
			}
		} else {
			mutex_lock(&rga_drvdata->lock);

			page_table = rga_mmu_buf_get(rga_drvdata->mmu_base, page_count);
			if (page_table == NULL) {
				rga_err("mmu_buf get error!\n");
				mutex_unlock(&rga_drvdata->lock);
				return -EFAULT;
			}

			mutex_unlock(&rga_drvdata->lock);
		}

		if (job_buf->y_addr) {
			sgt = rga_mm_get_rga2_sgt(job, job_buf, job_buf->y_addr,
						  dir, &use_dma_address);
			if (IS_ERR_OR_NULL(sgt)) {
				rga_job_err(job, "rga2 cannot get sgt from internal buffer!\n");
				ret = sgt ? PTR_ERR(sgt) : -EINVAL;
				goto err_free_page_table;
			}
			ret = rga_mm_sgt_to_page_table(sgt, page_table, yrgb_count,
						       use_dma_address);
			if (ret < 0) {
				rga_job_err(job, "rga2 page table reject: %d\n", ret);
				goto err_free_page_table;
			}
		}

		if (job_buf->uv_addr) {
			sgt = rga_mm_get_rga2_sgt(job, job_buf, job_buf->uv_addr,
						  dir, &use_dma_address);
			if (IS_ERR_OR_NULL(sgt)) {
				rga_job_err(job, "rga2 cannot get sgt from internal buffer!\n");
				ret = sgt ? PTR_ERR(sgt) : -EINVAL;
				goto err_free_page_table;
			}
			ret = rga_mm_sgt_to_page_table(sgt, page_table + yrgb_count,
						       uv_count, use_dma_address);
			if (ret < 0) {
				rga_job_err(job, "rga2 page table reject: %d\n", ret);
				goto err_free_page_table;
			}
		}

		if (job_buf->v_addr) {
			sgt = rga_mm_get_rga2_sgt(job, job_buf, job_buf->v_addr,
						  dir, &use_dma_address);
			if (IS_ERR_OR_NULL(sgt)) {
				rga_job_err(job, "rga2 cannot get sgt from internal buffer!\n");
				ret = sgt ? PTR_ERR(sgt) : -EINVAL;
				goto err_free_page_table;
			}
			ret = rga_mm_sgt_to_page_table(sgt,
						       page_table + yrgb_count + uv_count,
						       v_count, use_dma_address);
			if (ret < 0) {
				rga_job_err(job, "rga2 page table reject: %d\n", ret);
				goto err_free_page_table;
			}
		}

		img->yrgb_addr = yrgb_offset;
		img->uv_addr = (yrgb_count << PAGE_SHIFT) + uv_offset;
		img->v_addr = ((yrgb_count + uv_count) << PAGE_SHIFT) + v_offset;
	} else {
		if (job_buf->addr->virt_addr != NULL)
			img_offset = job_buf->addr->virt_addr->offset;

		page_count = RGA_GET_PAGE_COUNT(img_size + img_offset);
		if (page_count < 0) {
			rga_job_err(job, "page count cal error! yrba = %d, uv = %d, v = %d\n",
				yrgb_count, uv_count, v_count);
			return -EFAULT;
		}

		if (job->flags & RGA_JOB_USE_HANDLE) {
			page_table = (uint32_t *)rga_get_free_pages(GFP_KERNEL | GFP_DMA32,
				&order, page_count * sizeof(uint32_t *));
			if (page_table == NULL) {
				rga_job_err(job, "%s can not alloc pages for page_table, order = %d\n",
					__func__, order);
				return -ENOMEM;
			}
		} else {
			mutex_lock(&rga_drvdata->lock);

			page_table = rga_mmu_buf_get(rga_drvdata->mmu_base, page_count);
			if (page_table == NULL) {
				rga_job_err(job, "mmu_buf get error!\n");
				mutex_unlock(&rga_drvdata->lock);
				return -EFAULT;
			}

			mutex_unlock(&rga_drvdata->lock);
		}

		sgt = rga_mm_get_rga2_sgt(job, job_buf, job_buf->addr, dir,
					  &use_dma_address);
		if (IS_ERR_OR_NULL(sgt)) {
			rga_job_err(job, "rga2 cannot get sgt from internal buffer!\n");
			ret = sgt ? PTR_ERR(sgt) : -EINVAL;
			goto err_free_page_table;
		}
		ret = rga_mm_sgt_to_page_table(sgt, page_table, page_count,
					       use_dma_address);
		if (ret < 0) {
			rga_job_err(job, "rga2 page table reject: %d\n", ret);
			goto err_free_page_table;
		}

		img->yrgb_addr = img_offset;
		rga_convert_addr(img, false);
	}

	/*
	 * The hardware reads the CPU-filled table through DMA, so give the
	 * table a proper streaming mapping of the RGA2 device: per-job
	 * handle tables are mapped here (the mapping itself publishes the
	 * entries) and unmapped at put; ring windows borrow the persistent
	 * mapping made at bind time and are synced before each job.
	 */
	if (job->flags & RGA_JOB_USE_HANDLE) {
		job_buf->page_table_dma =
			dma_map_single(job->scheduler->dev, page_table,
				       page_count * sizeof(*page_table),
				       DMA_TO_DEVICE);
		if (dma_mapping_error(job->scheduler->dev, job_buf->page_table_dma)) {
			rga_job_err(job, "can not DMA-map page_table for RGA2\n");
			ret = -EFAULT;
			goto err_free_page_table;
		}
		job_buf->page_table_dev = job->scheduler->dev;
		job_buf->page_table_mapped = true;
	} else {
		job_buf->page_table_dma = rga_drvdata->mmu_base->dma_addr +
			(page_table - rga_drvdata->mmu_base->buf_virtual) *
			sizeof(*page_table);
		job_buf->page_table_dev = rga_drvdata->mmu_base->map_dev;
		job_buf->page_table_mapped = false;
	}

	job_buf->page_table = page_table;
	job_buf->order = order;
	job_buf->page_count = page_count;

	return 0;

err_free_page_table:
	rga_mm_put_rga2_bounce(job, job_buf, DMA_NONE);
	if (job->flags & RGA_JOB_USE_HANDLE)
		free_pages((unsigned long)page_table, order);
	return ret;
}

static int rga_mm_sync_dma_sg_for_device(struct rga_internal_buffer *buffer,
					 struct rga_job *job,
					 enum dma_data_direction dir)
{
	struct sg_table *sgt;
	struct rga_scheduler_t *scheduler;
	ktime_t timestamp = ktime_get();
	bool has_shadow = rga_shadow_active(buffer->virt_addr);

	scheduler = buffer->scheduler;
	if (scheduler == NULL) {
		rga_job_err(job, "%s(%d), failed to get scheduler, core = 0x%x\n",
			__func__, __LINE__, job->core);
		return -EFAULT;
	}

	if (has_shadow && (dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL))
		rga_shadow_copy_to_shadow(buffer->virt_addr);

	if (buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS) {
		if (scheduler->data->mmu == RGA_IOMMU) {
			dma_addr_t iova = rga_mm_lookup_iova(buffer);

			if (dma_mapping_error(buffer->dma_buffer->map_dev, iova)) {
				rga_job_err(job, "invalid iova for dma-buffer with IOMMU device!\n");
				return -EFAULT;
			}

			dma_sync_single_for_device(buffer->dma_buffer->map_dev,
				iova, buffer->size, dir);
		} else {
			dma_sync_single_for_device(scheduler->dev,
				buffer->phys_addr, buffer->size, dir);
		}
	} else {
		sgt = rga_mm_lookup_sgt(buffer);
		if (sgt == NULL) {
			rga_job_err(job, "%s(%d), failed to get sgt, core = 0x%x\n",
				__func__, __LINE__, job->core);
			return -EINVAL;
		}

		if (rga_mm_is_invalid_dma_buffer(buffer->dma_buffer)) {
			rga_job_err(job, "invalid dma-buffer with IOMMU device!\n");
			return -EFAULT;
		}

		dma_sync_sg_for_device(buffer->dma_buffer->map_dev, sgt->sgl,
				       sgt->orig_nents, dir);
	}

	if (DEBUGGER_EN(TIME))
		rga_job_log(job, "handle[%d], %s, flush CPU cache for device cost %lld us\n",
			buffer->handle, rga_get_dma_data_direction_str(dir),
			ktime_us_delta(ktime_get(), timestamp));

	return 0;
}

static int rga_mm_sync_dma_sg_for_cpu(struct rga_internal_buffer *buffer,
				      struct rga_job *job,
				      enum dma_data_direction dir)
{
	struct sg_table *sgt;
	struct rga_scheduler_t *scheduler;
	ktime_t timestamp = ktime_get();
	bool has_shadow = rga_shadow_active(buffer->virt_addr);

	scheduler = buffer->scheduler;
	if (scheduler == NULL) {
		rga_job_err(job, "%s(%d), failed to get scheduler, core = 0x%x\n",
			__func__, __LINE__, job->core);
		return -EFAULT;
	}

	if (buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS) {
		if (scheduler->data->mmu == RGA_IOMMU) {
			dma_addr_t iova = rga_mm_lookup_iova(buffer);

			if (dma_mapping_error(buffer->dma_buffer->map_dev, iova)) {
				rga_job_err(job, "invalid iova for dma-buffer with IOMMU device!\n");
				return -EFAULT;
			}

			dma_sync_single_for_cpu(buffer->dma_buffer->map_dev,
				iova, buffer->size, dir);
		} else {
			dma_sync_single_for_cpu(scheduler->dev,
				buffer->phys_addr, buffer->size, dir);
		}
	} else {
		sgt = rga_mm_lookup_sgt(buffer);
		if (sgt == NULL) {
			rga_job_err(job, "%s(%d), failed to get sgt, core = 0x%x\n",
				__func__, __LINE__, job->core);
			return -EINVAL;
		}

		if (rga_mm_is_invalid_dma_buffer(buffer->dma_buffer)) {
			rga_job_err(job, "invalid dma-buffer with IOMMU device!\n");
			return -EFAULT;
		}

		dma_sync_sg_for_cpu(buffer->dma_buffer->map_dev, sgt->sgl,
				    sgt->orig_nents, dir);
	}

	if (has_shadow && (dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL))
		rga_shadow_copy_from_shadow(buffer->virt_addr);

	if (DEBUGGER_EN(TIME))
		rga_job_log(job, "handle[%d], %s, flush CPU cache for CPU cost %lld us\n",
			buffer->handle, rga_get_dma_data_direction_str(dir),
			ktime_us_delta(ktime_get(), timestamp));

	return 0;
}

static int rga_mm_get_buffer_info(struct rga_job *job,
				  struct rga_internal_buffer *internal_buffer,
				  uint64_t *channel_addr)
{
	uint64_t addr;

	switch (job->scheduler->data->mmu) {
	case RGA_IOMMU:
		if (rga_mm_is_invalid_dma_buffer(internal_buffer->dma_buffer)) {
			rga_job_err(job,
				"core[%d] handle[%d] lookup buffer_type[0x%x] iova error!\n",
				job->core, internal_buffer->handle, internal_buffer->type);
			return -EINVAL;
		}

		addr = rga_mm_lookup_iova(internal_buffer);
		if (dma_mapping_error(internal_buffer->dma_buffer->map_dev, (dma_addr_t)addr)) {
			rga_job_err(job, "invalid iova for dma-buffer with IOMMU device!\n");
			return -EFAULT;
		}

		/*
		 * The RGA3 window base is fetched on a 16-byte granularity: the
		 * low 4 bits of yrgb_addr are dropped by the hardware. The
		 * scattered-userptr path carries the original sub-page byte
		 * offset in the base (iova + real_offset) and, for a shadow_page
		 * head, leaves the bytes before real_offset zeroed -- so a base
		 * whose offset is not 16-byte aligned makes RGA read the zeroed
		 * head and silently return all-zero pixels (see
		 * findings/2026-07-23-rga-scattered-userptr-unaligned-src-zero-output).
		 * Sub-16 alignment cannot be expressed as a whole-pixel window
		 * offset for every format, so reject it loudly here instead of
		 * corrupting data silently. Aligned userptr and dma-buf imports
		 * always land on a >=page-aligned base and are unaffected.
		 */
		if (!IS_ALIGNED(addr, RGA_IOMMU_ADDR_ALIGN)) {
			rga_job_err(job,
				"core[%d] handle[%d] iova 0x%llx not %d-byte aligned; unaligned scattered userptr is unsupported\n",
				job->core, internal_buffer->handle,
				(unsigned long long)addr, RGA_IOMMU_ADDR_ALIGN);
			return -EINVAL;
		}

		break;
	case RGA_MMU:
	default:
		if (internal_buffer->mm_flag & RGA_MEM_PHYSICAL_CONTIGUOUS) {
			addr = internal_buffer->phys_addr;
			break;
		}

		switch (internal_buffer->type) {
		case RGA_DMA_BUFFER:
		case RGA_DMA_BUFFER_PTR:
			addr = 0;
			break;
		case RGA_VIRTUAL_ADDRESS:
			addr = internal_buffer->virt_addr->addr;
			break;
		case RGA_PHYSICAL_ADDRESS:
			addr = internal_buffer->phys_addr;
			break;
		default:
			rga_job_err(job, "Illegal external buffer!\n");
			return -EFAULT;
		}
		break;
	}

	*channel_addr = addr;

	return 0;
}

static int rga_mm_get_buffer(struct rga_mm *mm,
			     struct rga_job *job,
			     uint64_t handle,
			     uint64_t *channel_addr,
			     struct rga_internal_buffer **buf,
			     int require_size,
			     enum dma_data_direction dir)
{
	int ret = 0;
	struct rga_internal_buffer *internal_buffer = NULL;

	if (handle == 0) {
		rga_job_err(job, "No buffer handle can be used!\n");
		return -EFAULT;
	}

	mutex_lock(&mm->lock);
	*buf = rga_mm_lookup_handle(mm, handle);
	if (*buf == NULL) {
		rga_job_err(job, "This handle[%ld] is illegal.\n", (unsigned long)handle);

		mutex_unlock(&mm->lock);
		return -EFAULT;
	}
	if (!rga_mm_handle_authorized(*buf, job->session)) {
		rga_job_err(job,
			    "session did not import handle[%ld]\n",
			    (unsigned long)handle);
		*buf = NULL;
		mutex_unlock(&mm->lock);
		return -EPERM;
	}

	internal_buffer = *buf;
	kref_get(&internal_buffer->refcount);

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "handle[%d] get info:\n", (int)handle);
		rga_mm_dump_buffer(internal_buffer);
	}

	mutex_unlock(&mm->lock);

	ret = rga_mm_get_buffer_info(job, internal_buffer, channel_addr);
	if (ret < 0) {
		rga_job_err(job, "handle[%ld] failed to get internal buffer info!\n",
			(unsigned long)handle);
		goto put_internal_buffer;
	}

	if (internal_buffer->size < require_size) {
		ret = -EINVAL;
		rga_job_err(job, "Only get buffer %ld byte from handle[%ld], but current required %d byte\n",
		       internal_buffer->size, (unsigned long)handle, require_size);

		goto put_internal_buffer;
	}

	if (internal_buffer->mm_flag & RGA_MEM_FORCE_FLUSH_CACHE) {
		/*
		 * Some userspace virtual addresses do not have an
		 * interface for flushing the cache, so it is mandatory
		 * to flush the cache when the virtual address is used.
		 */
		ret = rga_mm_sync_dma_sg_for_device(internal_buffer, job, dir);
		if (ret < 0) {
			rga_job_err(job, "sync sgt for device error!\n");
			goto put_internal_buffer;
		}
	}

	return 0;

put_internal_buffer:
	mutex_lock(&mm->lock);
	kref_put(&internal_buffer->refcount, rga_mm_kref_release_buffer);
	mutex_unlock(&mm->lock);

	*buf = NULL;
	return ret;

}

static void rga_mm_put_buffer(struct rga_mm *mm,
			      struct rga_job *job,
			      struct rga_internal_buffer *internal_buffer,
			      enum dma_data_direction dir)
{
	if (internal_buffer->mm_flag & RGA_MEM_FORCE_FLUSH_CACHE && dir != DMA_NONE)
		if (rga_mm_sync_dma_sg_for_cpu(internal_buffer, job, dir))
			rga_job_err(job, "sync sgt for cpu error!\n");

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "handle[%d] put info:\n", (int)internal_buffer->handle);
		rga_mm_dump_buffer(internal_buffer);
	}

	mutex_lock(&mm->lock);
	kref_put(&internal_buffer->refcount, rga_mm_kref_release_buffer);
	mutex_unlock(&mm->lock);
}

static void rga_mm_put_channel_handle_info(struct rga_mm *mm,
					   struct rga_job *job,
					   struct rga_job_buffer *job_buf,
					   enum dma_data_direction dir)
{
	/*
	 * Release transient RGA2 bounce mappings first: their unmap copies
	 * bounced data back and the post-clean below needs the origin
	 * buffers still mapped and not yet invalidated.
	 */
	rga_mm_put_rga2_bounce(job, job_buf, dir);

	if (job_buf->y_addr) {
		rga_mm_put_buffer(mm, job, job_buf->y_addr, dir);
		job_buf->y_addr = NULL;
	}
	if (job_buf->uv_addr) {
		rga_mm_put_buffer(mm, job, job_buf->uv_addr, dir);
		job_buf->uv_addr = NULL;
	}
	if (job_buf->v_addr) {
		rga_mm_put_buffer(mm, job, job_buf->v_addr, dir);
		job_buf->v_addr = NULL;
	}

	if (job_buf->page_table) {
		if (job_buf->page_table_mapped) {
			dma_unmap_single(job_buf->page_table_dev,
					 job_buf->page_table_dma,
					 job_buf->page_count *
					 sizeof(*job_buf->page_table),
					 DMA_TO_DEVICE);
			job_buf->page_table_mapped = false;
		}
		free_pages((unsigned long)job_buf->page_table, job_buf->order);
		job_buf->page_table = NULL;
	}
}

static int rga_mm_get_channel_handle_info(struct rga_mm *mm,
					  struct rga_job *job,
					  struct rga_img_info_t *img,
					  struct rga_job_buffer *job_buf,
					  enum dma_data_direction dir)
{
	int ret = 0;
	int handle = 0;
	int img_size, yrgb_size, uv_size, v_size;

	img_size = rga_image_size_cal(img->vir_w, img->vir_h, img->format,
				      &yrgb_size, &uv_size, &v_size);
	if (img_size <= 0) {
		rga_job_err(job, "Image size cal error! width = %d, height = %d, format = %s\n",
			img->vir_w, img->vir_h, rga_get_format_name(img->format));
		return -EINVAL;
	}

	/* using third-address */
	if (img->uv_addr > 0) {
		handle = img->yrgb_addr;
		if (handle > 0) {
			ret = rga_mm_get_buffer(mm, job, handle, &img->yrgb_addr,
						&job_buf->y_addr, yrgb_size, dir);
			if (ret < 0) {
				rga_job_err(job, "handle[%d] Can't get y/rgb address info!\n",
					handle);
				return ret;
			}
		}

		handle = img->uv_addr;
		if (handle > 0) {
			ret = rga_mm_get_buffer(mm, job, handle, &img->uv_addr,
						&job_buf->uv_addr, uv_size, dir);
			if (ret < 0) {
				rga_job_err(job, "handle[%d] Can't get uv address info!\n", handle);
				return ret;
			}
		}

		handle = img->v_addr;
		if (handle > 0) {
			ret = rga_mm_get_buffer(mm, job, handle, &img->v_addr,
						&job_buf->v_addr, v_size, dir);
			if (ret < 0) {
				rga_job_err(job, "handle[%d] Can't get uv address info!\n", handle);
				return ret;
			}
		}
	} else {
		handle = img->yrgb_addr;
		if (handle > 0) {
			ret = rga_mm_get_buffer(mm, job, handle, &img->yrgb_addr,
						&job_buf->addr, img_size, dir);
			if (ret < 0) {
				rga_job_err(job, "handle[%d] Can't get y/rgb address info!\n",
					handle);
				return ret;
			}
		}

		rga_convert_addr(img, false);
	}

	if (job->scheduler->data->mmu == RGA_MMU &&
	    rga_mm_is_need_mmu(job, job_buf->addr)) {
		ret = rga_mm_set_mmu_base(job, img, job_buf, dir);
		if (ret < 0) {
			rga_job_err(job, "Can't set RGA2 MMU_BASE from handle!\n");

			rga_mm_put_channel_handle_info(mm, job, job_buf, dir);
			return ret;
		}
	}

	return 0;
}

static void rga_mm_put_handle_info(struct rga_job *job,
				   struct rga_job_task_buffers *buffers,
				   struct rga_req *req);

static int rga_mm_get_handle_info(struct rga_job *job,
				  struct rga_job_task_buffers *buffers,
				  struct rga_req *req)
{
	int ret = 0;
	struct rga_mm *mm = NULL;
	enum dma_data_direction dir;

	mm = rga_drvdata->mm;

	switch (req->render_mode) {
	case BITBLT_MODE:
	case COLOR_PALETTE_MODE:
		if (unlikely(req->src.yrgb_addr <= 0)) {
			rga_job_err(job, "render_mode[0x%x] src0 channel handle[%ld] must is valid!",
				req->render_mode, (unsigned long)req->src.yrgb_addr);
			return -EINVAL;
		}

		if (unlikely(req->dst.yrgb_addr <= 0)) {
			rga_job_err(job, "render_mode[0x%x] dst channel handle[%ld] must is valid!",
				req->render_mode, (unsigned long)req->dst.yrgb_addr);
			return -EINVAL;
		}

		if (req->bsfilter_flag) {
			if (unlikely(req->pat.yrgb_addr <= 0)) {
				rga_job_err(job, "render_mode[0x%x] src1/pat channel handle[%ld] must is valid!",
					req->render_mode, (unsigned long)req->pat.yrgb_addr);
				return -EINVAL;
			}
		}

		break;
	case COLOR_FILL_MODE:
		if (unlikely(req->dst.yrgb_addr <= 0)) {
			rga_job_err(job, "render_mode[0x%x] dst channel handle[%ld] must is valid!",
				req->render_mode, (unsigned long)req->dst.yrgb_addr);
			return -EINVAL;
		}

		break;

	case UPDATE_PALETTE_TABLE_MODE:
	case UPDATE_PATTEN_BUF_MODE:
		if (unlikely(req->pat.yrgb_addr <= 0)) {
			rga_job_err(job, "render_mode[0x%x] lut/pat channel handle[%ld] must is valid!",
				req->render_mode, (unsigned long)req->pat.yrgb_addr);
			return -EINVAL;
		}

		break;
	default:
		rga_job_err(job, "%s, unknown render mode!\n", __func__);
		break;
	}

	if (likely(req->src.yrgb_addr > 0)) {
		ret = rga_mm_get_channel_handle_info(mm, job, &req->src,
						     &buffers->src_buffer,
						     DMA_TO_DEVICE);
		if (ret < 0) {
			rga_job_err(job, "Can't get src buffer info from handle!\n");
			goto err_put_handle_info;
		}
	}

	if (likely(req->dst.yrgb_addr > 0)) {
		ret = rga_mm_get_channel_handle_info(mm, job, &req->dst,
						     &buffers->dst_buffer,
						     DMA_TO_DEVICE);
		if (ret < 0) {
			rga_job_err(job, "Can't get dst buffer info from handle!\n");
			goto err_put_handle_info;
		}
	}

	if (likely(req->pat.yrgb_addr > 0)) {
		if (req->render_mode != UPDATE_PALETTE_TABLE_MODE) {
			if (req->bsfilter_flag)
				dir = DMA_BIDIRECTIONAL;
			else
				dir = DMA_TO_DEVICE;

			ret = rga_mm_get_channel_handle_info(mm, job, &req->pat,
							     &buffers->src1_buffer,
							     dir);
		} else {
			ret = rga_mm_get_channel_handle_info(mm, job, &req->pat,
							     &buffers->els_buffer,
							     DMA_BIDIRECTIONAL);
		}
		if (ret < 0) {
			rga_job_err(job, "Can't get pat buffer info from handle!\n");
			goto err_put_handle_info;
		}
	}

	rga_mm_set_mmu_flag(job, buffers, req);

	return 0;

err_put_handle_info:
	rga_mm_put_handle_info(job, buffers, req);
	return ret;
}

static void rga_mm_put_handle_info(struct rga_job *job,
				   struct rga_job_task_buffers *buffers,
				   struct rga_req *req)
{
	struct rga_mm *mm = rga_drvdata->mm;

	rga_mm_put_channel_handle_info(mm, job, &buffers->src_buffer, DMA_NONE);
	rga_mm_put_channel_handle_info(mm, job, &buffers->dst_buffer, DMA_FROM_DEVICE);
	rga_mm_put_channel_handle_info(mm, job, &buffers->src1_buffer, DMA_NONE);
	rga_mm_put_channel_handle_info(mm, job, &buffers->els_buffer, DMA_NONE);
}

static void rga_mm_put_channel_external_buffer(struct rga_job_buffer *job_buffer)
{
	if (job_buffer->ex_addr->type == RGA_DMA_BUFFER_PTR)
		dma_buf_put((struct dma_buf *)(unsigned long)job_buffer->ex_addr->memory);

	kfree(job_buffer->ex_addr);
	job_buffer->ex_addr = NULL;
}

static int rga_mm_get_channel_external_buffer(int mmu_flag,
					      struct rga_img_info_t *img_info,
					      struct rga_job_buffer *job_buffer)
{
	struct dma_buf *dma_buf = NULL;
	struct rga_external_buffer *external_buffer = NULL;

	/* Default unsupported multi-planar format */
	external_buffer = kzalloc(sizeof(*external_buffer), GFP_KERNEL);
	if (external_buffer == NULL) {
		rga_err("Cannot alloc job_buffer!\n");
		return -ENOMEM;
	}

	if (img_info->yrgb_addr) {
		dma_buf = dma_buf_get(img_info->yrgb_addr);
		if (IS_ERR(dma_buf)) {
			rga_err("%s dma_buf_get fail fd[%lu]\n",
				__func__, (unsigned long)img_info->yrgb_addr);
			kfree(external_buffer);
			return -EINVAL;
		}

		external_buffer->memory = (unsigned long)dma_buf;
		external_buffer->type = RGA_DMA_BUFFER_PTR;
	} else if (mmu_flag && img_info->uv_addr) {
		external_buffer->memory = (uint64_t)img_info->uv_addr;
		external_buffer->type = RGA_VIRTUAL_ADDRESS;
	} else if (img_info->uv_addr) {
		external_buffer->memory = (uint64_t)img_info->uv_addr;
		external_buffer->type = RGA_PHYSICAL_ADDRESS;
	} else {
		kfree(external_buffer);
		return -EINVAL;
	}

	external_buffer->memory_parm.width = img_info->vir_w;
	external_buffer->memory_parm.height = img_info->vir_h;
	external_buffer->memory_parm.format = img_info->format;

	job_buffer->ex_addr = external_buffer;

	return 0;
}

static void rga_mm_put_external_buffer(struct rga_job *job, struct rga_job_task_buffers *buffers)
{
	if (buffers->src_buffer.ex_addr)
		rga_mm_put_channel_external_buffer(&buffers->src_buffer);
	if (buffers->src1_buffer.ex_addr)
		rga_mm_put_channel_external_buffer(&buffers->src1_buffer);
	if (buffers->dst_buffer.ex_addr)
		rga_mm_put_channel_external_buffer(&buffers->dst_buffer);
	if (buffers->els_buffer.ex_addr)
		rga_mm_put_channel_external_buffer(&buffers->els_buffer);
}

static int rga_mm_get_external_buffer(struct rga_job *job,
				      struct rga_job_task_buffers *buffers,
				      struct rga_req *req)
{
	int ret = -EINVAL;
	int mmu_flag;

	struct rga_img_info_t *src0 = NULL;
	struct rga_img_info_t *src1 = NULL;
	struct rga_img_info_t *dst = NULL;
	struct rga_img_info_t *els = NULL;

	if (req->render_mode != COLOR_FILL_MODE)
		src0 = &req->src;

	if (req->render_mode != UPDATE_PALETTE_TABLE_MODE)
		src1 = req->bsfilter_flag ?
		       &req->pat : NULL;
	else
		els = &req->pat;

	dst = &req->dst;

	if (likely(src0)) {
		mmu_flag = ((req->mmu_info.mmu_flag >> 8) & 1);
		ret = rga_mm_get_channel_external_buffer(mmu_flag, src0, &buffers->src_buffer);
		if (ret < 0) {
			rga_job_err(job, "Cannot get src0 channel buffer!\n");
			return ret;
		}
	}

	if (likely(dst)) {
		mmu_flag = ((req->mmu_info.mmu_flag >> 10) & 1);
		ret = rga_mm_get_channel_external_buffer(mmu_flag, dst, &buffers->dst_buffer);
		if (ret < 0) {
			rga_job_err(job, "Cannot get dst channel buffer!\n");
			goto error_put_buffer;
		}
	}

	if (src1) {
		mmu_flag = ((req->mmu_info.mmu_flag >> 9) & 1);
		ret = rga_mm_get_channel_external_buffer(mmu_flag, src1, &buffers->src1_buffer);
		if (ret < 0) {
			rga_job_err(job, "Cannot get src1 channel buffer!\n");
			goto error_put_buffer;
		}
	}

	if (els) {
		mmu_flag = ((req->mmu_info.mmu_flag >> 11) & 1);
		ret = rga_mm_get_channel_external_buffer(mmu_flag, els, &buffers->els_buffer);
		if (ret < 0) {
			rga_job_err(job, "Cannot get els channel buffer!\n");
			goto error_put_buffer;
		}
	}

	return 0;
error_put_buffer:
	rga_mm_put_external_buffer(job, buffers);
	return ret;
}

static void rga_mm_unmap_channel_job_buffer(struct rga_job *job,
					    struct rga_job_buffer *job_buffer,
					    enum dma_data_direction dir)
{
	rga_mm_put_rga2_bounce(job, job_buffer, dir);

	if (job_buffer->addr->mm_flag & RGA_MEM_FORCE_FLUSH_CACHE && dir != DMA_NONE)
		if (rga_mm_sync_dma_sg_for_cpu(job_buffer->addr, job, dir))
			rga_job_err(job, "sync sgt for cpu error!\n");

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "unmap buffer:\n");
		rga_mm_dump_buffer(job_buffer->addr);
	}

	rga_mm_unmap_buffer(job_buffer->addr);
	kfree(job_buffer->addr);

	job_buffer->page_table = NULL;
}

static int rga_mm_map_channel_job_buffer(struct rga_job *job,
					 struct rga_img_info_t *img,
					 struct rga_job_buffer *job_buffer,
					 enum dma_data_direction dir,
					 int write_flag)
{
	int ret;
	struct rga_internal_buffer *buffer = NULL;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL) {
		rga_job_err(job, "%s alloc internal_buffer error!\n", __func__);
		return -ENOMEM;
	}

	ret = rga_mm_map_buffer(job_buffer->ex_addr, buffer, job, write_flag);
	if (ret < 0) {
		rga_job_err(job, "job buffer map failed!\n");
		goto error_free_buffer;
	}

	buffer->session = job->session;

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "map buffer:\n");
		rga_mm_dump_buffer(buffer);
	}

	ret = rga_mm_get_buffer_info(job, buffer, &img->yrgb_addr);
	if (ret < 0) {
		rga_job_err(job, "Failed to get internal buffer info!\n");
		goto error_unmap_buffer;
	}

	if (buffer->mm_flag & RGA_MEM_FORCE_FLUSH_CACHE) {
		ret = rga_mm_sync_dma_sg_for_device(buffer, job, dir);
		if (ret < 0) {
			rga_job_err(job, "sync sgt for device error!\n");
			goto error_unmap_buffer;
		}
	}

	rga_convert_addr(img, false);

	job_buffer->addr = buffer;

	if (job->scheduler->data->mmu == RGA_MMU &&
	    rga_mm_is_need_mmu(job, job_buffer->addr)) {
		ret = rga_mm_set_mmu_base(job, img, job_buffer, dir);
		if (ret < 0) {
			rga_job_err(job, "Can't set RGA2 MMU_BASE!\n");
			job_buffer->addr = NULL;
			goto error_unmap_buffer;
		}
	}

	return 0;

error_unmap_buffer:
	rga_mm_unmap_buffer(buffer);
error_free_buffer:
	kfree(buffer);

	return ret;
}

static void rga_mm_unmap_buffer_info(struct rga_job *job, struct rga_job_task_buffers *buffers)
{
	if (buffers->src_buffer.addr)
		rga_mm_unmap_channel_job_buffer(job, &buffers->src_buffer, DMA_NONE);
	if (buffers->dst_buffer.addr)
		rga_mm_unmap_channel_job_buffer(job, &buffers->dst_buffer, DMA_FROM_DEVICE);
	if (buffers->src1_buffer.addr)
		rga_mm_unmap_channel_job_buffer(job, &buffers->src1_buffer, DMA_NONE);
	if (buffers->els_buffer.addr)
		rga_mm_unmap_channel_job_buffer(job, &buffers->els_buffer, DMA_NONE);
	rga_mm_put_external_buffer(job, buffers);
}

static int rga_mm_map_buffer_info(struct rga_job *job,
				  struct rga_job_task_buffers *buffers,
				  struct rga_req *req)
{
	int ret = 0;
	enum dma_data_direction dir;

	ret = rga_mm_get_external_buffer(job, buffers, req);
	if (ret < 0) {
		rga_job_err(job, "failed to get external buffer from job_cmd!\n");
		return ret;
	}

	if (likely(buffers->src_buffer.ex_addr)) {
		ret = rga_mm_map_channel_job_buffer(job, &req->src,
						    &buffers->src_buffer,
						    DMA_TO_DEVICE, false);
		if (ret < 0) {
			rga_job_err(job, "src channel map job buffer failed!");
			goto error_unmap_buffer;
		}
	}

	if (likely(buffers->dst_buffer.ex_addr)) {
		ret = rga_mm_map_channel_job_buffer(job, &req->dst,
						    &buffers->dst_buffer,
						    DMA_TO_DEVICE, true);
		if (ret < 0) {
			rga_job_err(job, "dst channel map job buffer failed!");
			goto error_unmap_buffer;
		}
	}

	if (buffers->src1_buffer.ex_addr) {
		if (req->bsfilter_flag)
			dir = DMA_BIDIRECTIONAL;
		else
			dir = DMA_TO_DEVICE;

		ret = rga_mm_map_channel_job_buffer(job, &req->pat,
						    &buffers->src1_buffer,
						    dir, false);
		if (ret < 0) {
			rga_job_err(job, "src1 channel map job buffer failed!");
			goto error_unmap_buffer;
		}
	}

	if (buffers->els_buffer.ex_addr) {
		ret = rga_mm_map_channel_job_buffer(job, &req->pat,
						    &buffers->els_buffer,
						    DMA_BIDIRECTIONAL, false);
		if (ret < 0) {
			rga_job_err(job, "els channel map job buffer failed!");
			goto error_unmap_buffer;
		}
	}

	rga_mm_set_mmu_flag(job, buffers, req);
	return 0;

error_unmap_buffer:
	rga_mm_unmap_buffer_info(job, buffers);

	return ret;
}

static void rga_mm_free_channel_fake_buffer(struct rga_job *job,
					    struct rga_job_buffer *job_buffer,
					    enum dma_data_direction dir)
{
	struct rga_internal_buffer *buffer = job_buffer->addr;

	if (rga_mm_is_invalid_dma_buffer(buffer->dma_buffer))
		return;

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "free fake-buffer dump info:\n");
		rga_mm_dump_buffer(buffer);
	}

	rga_dma_free(buffer->dma_buffer);
	kfree(buffer);
	job_buffer->addr = NULL;
}

static int rga_mm_alloc_channel_fake_buffer(struct rga_job *job,
					    struct rga_img_info_t *img,
					    struct rga_job_buffer *job_buffer,
					    enum dma_data_direction dir)
{
	int ret;
	int size;
	uint32_t mm_flag;
	uint64_t phys_addr;
	struct rga_internal_buffer *buffer;
	struct rga_dma_buffer *dma_buf;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (buffer == NULL) {
		rga_job_err(job, "%s alloc internal_buffer error!\n", __func__);
		return -ENOMEM;
	}

	size = rga_image_size_cal(img->vir_w, img->vir_h, img->format,
				  NULL, NULL, NULL);
	dma_buf = rga_dma_alloc_coherent(job->scheduler, size);
	if (dma_buf == NULL) {
		ret = -ENOMEM;
		rga_job_err(job, "%s failed to alloc dma_buf.\n", __func__);
		goto error_free_buffer;
	}

	mm_flag = RGA_MEM_PHYSICAL_CONTIGUOUS | RGA_MEM_UNDER_4G;
	if (job->scheduler->data->mmu != RGA_IOMMU) {
		mm_flag |= RGA_MEM_NEED_USE_IOMMU;
		phys_addr = 0;
	} else {
		phys_addr = dma_buf->dma_addr;
	}

	if (!rga_mm_check_memory_limit(job->scheduler, mm_flag)) {
		rga_job_err(job, "%s scheduler core[%d] unsupported mm_flag[0x%x]!\n",
		       __func__, job->scheduler->core, mm_flag);
		ret = -EINVAL;
		goto error_free_dma_buf;
	}

	buffer->type = RGA_DMA_BUFFER_PTR;
	buffer->size = dma_buf->size - dma_buf->offset;
	buffer->mm_flag = mm_flag;
	buffer->dma_buffer = dma_buf;
	buffer->phys_addr = phys_addr;

	buffer->memory_parm.width = img->vir_w;
	buffer->memory_parm.height = img->vir_h;
	buffer->memory_parm.format = img->format;
	buffer->memory_parm.size = size;

	ret = rga_mm_get_buffer_info(job, buffer, &img->yrgb_addr);
	if (ret < 0)
		goto error_free_dma_buf;

	rga_convert_addr(img, false);

	job_buffer->addr = buffer;

	if (job->scheduler->data->mmu == RGA_MMU &&
	    rga_mm_is_need_mmu(job, job_buffer->addr)) {
		ret = rga_mm_set_mmu_base(job, img, job_buffer, dir);
		if (ret < 0) {
			job_buffer->addr = NULL;
			goto error_free_dma_buf;
		}
	}

	if (DEBUGGER_EN(MM)) {
		rga_job_log(job, "alloc fake-buffer dump info:\n");
		rga_mm_dump_buffer(buffer);
	}

	return 0;

error_free_dma_buf:
	rga_dma_free(dma_buf);

error_free_buffer:
	kfree(buffer);

	return ret;
}

static void rga_mm_free_fake_buffer(struct rga_job *job, struct rga_job_task_buffers *buffers)
{
	if (buffers->src_buffer.addr)
		rga_mm_free_channel_fake_buffer(job, &buffers->src_buffer, DMA_NONE);
	if (buffers->dst_buffer.addr)
		rga_mm_free_channel_fake_buffer(job, &buffers->dst_buffer, DMA_FROM_DEVICE);
	if (buffers->src1_buffer.addr)
		rga_mm_free_channel_fake_buffer(job, &buffers->src1_buffer, DMA_NONE);
	if (buffers->els_buffer.addr)
		rga_mm_free_channel_fake_buffer(job, &buffers->els_buffer, DMA_NONE);
}

static int rga_mm_alloc_fake_buffer(struct rga_job *job,
				    struct rga_job_task_buffers *buffers,
				    struct rga_req *req)
{
	int ret = 0;
	enum dma_data_direction dir;

	if (req->src.yrgb_addr != 0 || req->src.uv_addr != 0) {
		ret = rga_mm_alloc_channel_fake_buffer(job, &req->src,
						       &buffers->src_buffer, DMA_TO_DEVICE);
		if (ret < 0) {
			rga_job_err(job, "%s src channel map job buffer failed!", __func__);
			goto error_free_fake_buffer;
		}
	}

	if (req->dst.yrgb_addr != 0 || req->dst.uv_addr != 0) {
		ret = rga_mm_alloc_channel_fake_buffer(job, &req->dst,
						       &buffers->dst_buffer, DMA_TO_DEVICE);
		if (ret < 0) {
			rga_job_err(job, "%s dst channel map job buffer failed!", __func__);
			goto error_free_fake_buffer;
		}
	}

	if (req->render_mode != UPDATE_PALETTE_TABLE_MODE &&
	    (req->pat.yrgb_addr != 0 || req->pat.uv_addr != 0)) {
		if (req->bsfilter_flag)
			dir = DMA_BIDIRECTIONAL;
		else
			dir = DMA_TO_DEVICE;

		ret = rga_mm_alloc_channel_fake_buffer(job, &req->pat,
						       &buffers->src1_buffer, dir);
		if (ret < 0) {
			rga_job_err(job, "%s src1 channel map job buffer failed!", __func__);
			goto error_free_fake_buffer;
		}
	} else if (req->pat.yrgb_addr != 0 || req->pat.uv_addr != 0) {
		ret = rga_mm_alloc_channel_fake_buffer(job, &req->pat,
						       &buffers->els_buffer, DMA_TO_DEVICE);
		if (ret < 0) {
			rga_job_err(job, "%s els channel map job buffer failed!", __func__);
			goto error_free_fake_buffer;
		}
	}

	rga_mm_set_mmu_flag(job, buffers, req);

	return 0;

error_free_fake_buffer:
	rga_mm_free_fake_buffer(job, buffers);

	return ret;
}

static int rga_mm_map_task_info(struct rga_job *job,
				struct rga_job_task_buffers *buffers,
				struct rga_req *req)
{
	int ret;
	ktime_t timestamp = ktime_get();

	if (job->flags & RGA_JOB_DEBUG_FAKE_BUFFER) {
		ret = rga_mm_alloc_fake_buffer(job, buffers, req);
		if (ret < 0)
			return ret;

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "alloc fake buffer cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));

		job->flags &= ~RGA_JOB_USE_HANDLE;
		job->flags |= RGA_JOB_DEBUG_FAKE_BUFFER;

		return 0;
	}

	if (job->flags & RGA_JOB_USE_HANDLE) {
		ret = rga_mm_get_handle_info(job, buffers, req);
		if (ret < 0) {
			rga_job_err(job, "failed to get buffer from handle\n");
			return ret;
		}

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "get buffer_handle info cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));
	} else {
		ret = rga_mm_map_buffer_info(job, buffers, req);
		if (ret < 0) {
			rga_job_err(job, "failed to map buffer\n");
			return ret;
		}

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "map buffer cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));
	}

	return 0;
}

static void rga_mm_unmap_task_info(struct rga_job *job,
				   struct rga_job_task_buffers *buffers,
				   struct rga_req *req)
{
	ktime_t timestamp = ktime_get();

	if (job->flags & RGA_JOB_DEBUG_FAKE_BUFFER) {
		rga_mm_free_fake_buffer(job, buffers);

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "free fake buffer cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));

		return;
	}

	if (job->flags & RGA_JOB_USE_HANDLE) {
		rga_mm_put_handle_info(job, buffers, req);

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "put buffer_handle info cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));
	} else {
		rga_mm_unmap_buffer_info(job, buffers);

		if (DEBUGGER_EN(TIME))
			rga_job_log(job, "unmap buffer cost %lld us\n",
				ktime_us_delta(ktime_get(), timestamp));
	}
}

int rga_mm_map_job_info(struct rga_job *job)
{
	int ret = 0;
	int i;
	struct rga_job_task_buffers *task_buffers;
	struct rga_req *req;

	for (i = 0; i < job->task_count; i++) {
		req = &job->task_list[i];
		task_buffers = &job->task_buffers[i];

		ret = rga_mm_map_task_info(job, task_buffers, req);
		if (ret < 0) {
			rga_job_err(job, "task[%d] failed to map job info\n", i);
			goto err_unmap_job_info;
		}
	}

	return 0;

err_unmap_job_info:
	for (; i > 0; i--) {
		task_buffers = &job->task_buffers[i - 1];
		req = &job->task_list[i - 1];

		rga_mm_unmap_task_info(job, task_buffers, req);
	}

	return ret;
}

void rga_mm_unmap_job_info(struct rga_job *job)
{
	int i;
	struct rga_job_task_buffers *task_buffers;
	struct rga_req *req;

	for (i = 0; i < job->task_count; i++) {
		task_buffers = &job->task_buffers[i];
		req = &job->task_list[i];

		rga_mm_unmap_task_info(job, task_buffers, req);
	}
}

/*
 * rga_mm_import_buffer - Importing external buffer into the RGA driver
 *
 * @external_buffer: [in] Parameters of external buffer
 * @session:         [in] Session of the current process
 *
 * returns:
 * if return value > 0, the buffer import is successful and is the generated
 * buffer-handle, negative error code on failure.
 */
int rga_mm_import_buffer(struct rga_external_buffer *external_buffer,
			 struct rga_session *session)
{
	int ret = 0, new_id;
	struct rga_mm *mm;
	struct rga_internal_buffer *internal_buffer;
	struct rga_internal_buffer *existing;
	u32 handle;
	ktime_t timestamp = ktime_get();

	mm = rga_drvdata->mm;
	if (mm == NULL) {
		rga_err("rga mm is null!\n");
		return -EFAULT;
	}

	mutex_lock(&mm->lock);

	/* first, Check whether to rga_mm */
	internal_buffer = rga_mm_lookup_external(mm, external_buffer, current->mm);
	if (IS_ERR(internal_buffer)) {
		ret = PTR_ERR(internal_buffer);
		mutex_unlock(&mm->lock);
		return ret;
	}
	if (internal_buffer) {
		ret = rga_mm_record_import(internal_buffer, session, true);
		if (ret) {
			mutex_unlock(&mm->lock);
			return ret;
		}
		handle = internal_buffer->handle;

		if (DEBUGGER_EN(MM)) {
			rga_buf_log(internal_buffer, "import existing buffer:\n");
			rga_mm_dump_buffer(internal_buffer);
		}
		mutex_unlock(&mm->lock);

		return handle;
	}

	mutex_unlock(&mm->lock);

	/* finally, map and cached external_buffer in rga_mm */
	internal_buffer = kzalloc(sizeof(struct rga_internal_buffer), GFP_KERNEL);
	if (internal_buffer == NULL) {
		rga_err("%s alloc internal_buffer error!\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&internal_buffer->import_list);
	ret = rga_mm_map_buffer(external_buffer, internal_buffer, NULL, true);
	if (ret < 0)
		goto FREE_INTERNAL_BUFFER;

	kref_init(&internal_buffer->refcount);

	mutex_lock(&mm->lock);
	/* Another importer may have published this object while mapping slept. */
	existing = rga_mm_lookup_mapped(mm, internal_buffer);
	if (existing) {
		ret = rga_mm_record_import(existing, session, true);
		handle = existing->handle;
		mutex_unlock(&mm->lock);
		rga_mm_unmap_buffer(internal_buffer);
		kfree(internal_buffer);
		if (ret)
			return ret;
		return handle;
	}

	ret = rga_mm_record_import(internal_buffer, session, false);
	if (ret) {
		mutex_unlock(&mm->lock);
		goto UNMAP_INTERNAL_BUFFER;
	}

	/*
	 * Get the user-visible handle using idr. Preload and perform
	 * allocation under our spinlock.
	 */
	idr_preload(GFP_KERNEL);
	new_id = idr_alloc_cyclic(&mm->memory_idr, internal_buffer, 1, 0, GFP_NOWAIT);
	idr_preload_end();
	if (new_id < 0) {
		rga_err("internal_buffer alloc id failed!\n");
		ret = new_id;

		while (!list_empty(&internal_buffer->import_list)) {
			struct rga_buffer_import *import;

			import = list_first_entry(&internal_buffer->import_list,
						  struct rga_buffer_import, node);
			list_del(&import->node);
			kfree(import);
		}
		mutex_unlock(&mm->lock);
		goto UNMAP_INTERNAL_BUFFER;
	}

	internal_buffer->handle = new_id;
	handle = new_id;
	mm->buffer_count++;

	if (DEBUGGER_EN(MM)) {
		rga_buf_log(internal_buffer, "import buffer:\n");
		rga_mm_dump_buffer(internal_buffer);
	}
	if (DEBUGGER_EN(TIME))
		rga_buf_log(internal_buffer, "import buffer cost %lld us\n",
			ktime_us_delta(ktime_get(), timestamp));

	mutex_unlock(&mm->lock);

	return handle;

UNMAP_INTERNAL_BUFFER:
	rga_mm_unmap_buffer(internal_buffer);
FREE_INTERNAL_BUFFER:
	kfree(internal_buffer);

	return ret;
}

int rga_mm_release_buffer(uint32_t handle, struct rga_session *session)
{
	struct rga_mm *mm;
	struct rga_internal_buffer *internal_buffer;
	struct rga_buffer_import *import;
	ktime_t timestamp = ktime_get();

	mm = rga_drvdata->mm;
	if (mm == NULL) {
		rga_err("rga mm is null!\n");
		return -EFAULT;
	}

	mutex_lock(&mm->lock);

	/* Find the buffer that has been imported */
	internal_buffer = rga_mm_lookup_handle(mm, handle);
	if (IS_ERR_OR_NULL(internal_buffer)) {
		rga_err("This is not a buffer that has been imported, handle = %d\n", (int)handle);

		mutex_unlock(&mm->lock);
		return -ENOENT;
	}

	import = rga_mm_lookup_import(internal_buffer, session);
	if (!import) {
		rga_err("handle[%d] was not imported by this session\n",
			(int)handle);

		mutex_unlock(&mm->lock);
		return -EPERM;
	}

	if (DEBUGGER_EN(MM)) {
		rga_buf_log(internal_buffer, "release buffer:\n");
		rga_mm_dump_buffer(internal_buffer);
	}

	if (--import->count == 0) {
		list_del(&import->node);
		kfree(import);
		rga_mm_refresh_diagnostic_owner(internal_buffer);
	}

	kref_put(&internal_buffer->refcount, rga_mm_kref_release_buffer);

	if (DEBUGGER_EN(TIME))
		rga_log("handle[%d]: release buffer cost %lld us\n",
			handle, ktime_us_delta(ktime_get(), timestamp));

	mutex_unlock(&mm->lock);

	return 0;
}

int rga_mm_session_release_buffer(struct rga_session *session)
{
	int i;
	struct rga_mm *mm;
	struct rga_internal_buffer *buffer;
	struct rga_buffer_import *import;

	mm = rga_drvdata->mm;
	if (mm == NULL) {
		rga_err("rga mm is null!\n");
		return -EFAULT;
	}

	for (;;) {
		u32 n = 0;

		mutex_lock(&mm->lock);
		buffer = NULL;
		idr_for_each_entry(&mm->memory_idr, buffer, i) {
			import = rga_mm_lookup_import(buffer, session);
			if (import)
				break;
			buffer = NULL;
		}
		if (!buffer) {
			mutex_unlock(&mm->lock);
			break;
		}

		n = import->count;
		list_del(&import->node);
		kfree(import);
		rga_mm_refresh_diagnostic_owner(buffer);
		while (n--)
			kref_put(&buffer->refcount, rga_mm_kref_release_buffer);
		mutex_unlock(&mm->lock);
	}

	return 0;
}

int rga_mm_init(struct rga_mm **mm_session)
{
	struct rga_mm *mm = NULL;

	*mm_session = kzalloc(sizeof(struct rga_mm), GFP_KERNEL);
	if (*mm_session == NULL) {
		pr_err("can not kzalloc for rga buffer mm_session\n");
		return -ENOMEM;
	}

	mm = *mm_session;

	mutex_init(&mm->lock);
	idr_init_base(&mm->memory_idr, 1);

	return 0;
}

int rga_mm_remove(struct rga_mm **mm_session)
{
	struct rga_mm *mm = *mm_session;

	mutex_lock(&mm->lock);

	idr_for_each(&mm->memory_idr, &rga_mm_buffer_destroy_for_idr, mm);
	idr_destroy(&mm->memory_idr);

	mutex_unlock(&mm->lock);

	kfree(*mm_session);
	*mm_session = NULL;

	return 0;
}
