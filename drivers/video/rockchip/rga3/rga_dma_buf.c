// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Huang Lee <Putin.li@rock-chips.com>
 */

#include <linux/iommu.h>
#include <linux/overflow.h>

#include "rga_dma_buf.h"
#include "rga.h"
#include "rga_common.h"
#include "rga_job.h"
#include "rga_debugger.h"

static int rga_dma_check_iova_span(dma_addr_t dma_addr, size_t size,
				   const char *source, bool log_errors)
{
	u64 dma_end;

	if (!size) {
		if (log_errors)
			rga_err("reject %s DMA mapping: zero length segment, iova = %pad\n",
				source, &dma_addr);
		return -EINVAL;
	}

	if (check_add_overflow((u64)dma_addr, (u64)size - 1, &dma_end) ||
	    dma_addr > U32_MAX || dma_end > U32_MAX) {
		if (log_errors)
			rga_err("reject %s DMA mapping: 32-bit IOVA span overflow, iova = %pad, size = %zu, end = 0x%llx\n",
				source, &dma_addr, size, dma_end);
		return -EOVERFLOW;
	}

	return 0;
}

static int rga_dma_check_iova_contract(struct sg_table *sgt,
				       const char *source, bool log_errors)
{
	if (!sgt || !sgt->sgl) {
		if (log_errors)
			rga_err("reject %s DMA mapping: empty sg table\n",
				source);
		return -EINVAL;
	}

	if (sgt->nents != 1) {
		if (log_errors)
			rga_err("reject %s DMA mapping: expected one DMA segment, got %u, orig_nents = %u\n",
				source, sgt->nents, sgt->orig_nents);
		return -EOPNOTSUPP;
	}

	return rga_dma_check_iova_span(sg_dma_address(sgt->sgl),
				       sg_dma_len(sgt->sgl), source,
				       log_errors);
}

static void rga_dma_reset_sgt_dma_state(struct sg_table *sgt)
{
	struct scatterlist *sg;
	unsigned int i;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		sg_dma_address(sg) = DMA_MAPPING_ERROR;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		sg_dma_len(sg) = 0;
#endif
#ifdef CONFIG_NEED_SG_DMA_FLAGS
		sg->dma_flags &= ~(SG_DMA_BUS_ADDRESS | SG_DMA_SWIOTLB);
#endif
	}

	sgt->nents = sgt->orig_nents;
}

static int rga_dma_iommu_prot(struct device *dev,
			      enum dma_data_direction dir)
{
	int prot = dev_is_dma_coherent(dev) ? IOMMU_CACHE : 0;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

static struct iova_domain *rga_dma_iommu_iovad(struct iommu_domain *domain)
{
	return iommu_dma_get_iova_domain(domain);
}

static int rga_dma_alloc_iommu_iova(struct iommu_domain *domain,
				    struct device *dev, size_t size,
				    dma_addr_t *iova_out)
{
	struct iova_domain *iovad;
	unsigned long shift;
	unsigned long iova_len;
	unsigned long iova;
	u64 dma_limit;

	iovad = rga_dma_iommu_iovad(domain);
	if (!iovad)
		return -EOPNOTSUPP;

	/*
	 * Route B exposes one byte-contiguous RGA span. Larger IOVA granules can
	 * force padding between non-contiguous user pages, so fail closed.
	 */
	if (iovad->granule > PAGE_SIZE)
		return -EOPNOTSUPP;

	if (iova_align(iovad, size) != size)
		return -EINVAL;

	shift = iova_shift(iovad);
	iova_len = size >> shift;
	if (!iova_len)
		return -EINVAL;

	dma_limit = dma_get_mask(dev);
	if (dev->bus_dma_limit)
		dma_limit = min_t(u64, dma_limit, dev->bus_dma_limit);
	dma_limit = min_t(u64, dma_limit, RGA_IOMMU_DMA_LIMIT);
	if (domain->geometry.force_aperture)
		dma_limit = min_t(u64, dma_limit,
				  domain->geometry.aperture_end);

	iova = alloc_iova_fast(iovad, iova_len, dma_limit >> shift, true);
	if (!iova)
		return -ENOMEM;

	*iova_out = (dma_addr_t)iova << shift;

	return 0;
}

static void rga_dma_free_iommu_iova(struct iommu_domain *domain,
				    dma_addr_t iova, size_t size)
{
	struct iova_domain *iovad = rga_dma_iommu_iovad(domain);

	if (!iovad)
		return;

	free_iova_fast(iovad, iova_pfn(iovad, iova),
		       size >> iova_shift(iovad));
}

static int rga_dma_alloc_aligned_sgt(struct sg_table *sgt,
				     struct sg_table *aligned_sgt,
				     size_t *data_size, size_t *map_size)
{
	struct scatterlist *src;
	struct scatterlist *dst;
	size_t data = 0;
	size_t map = 0;
	int i;
	int ret;

	if (!sgt || !sgt->sgl || !sgt->orig_nents)
		return -EINVAL;

	ret = sg_alloc_table(aligned_sgt, sgt->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	dst = aligned_sgt->sgl;
	for_each_sg(sgt->sgl, src, sgt->orig_nents, i) {
		phys_addr_t phys = sg_phys(src);
		phys_addr_t start = ALIGN_DOWN(phys, PAGE_SIZE);
		u64 end = (u64)phys + src->length;
		u64 aligned_end = ALIGN(end, PAGE_SIZE);
		size_t len;

		if (!src->length || end < phys || aligned_end < end) {
			ret = -EINVAL;
			goto err_free_table;
		}

		len = aligned_end - start;
		if (len > UINT_MAX ||
		    check_add_overflow(data, (size_t)src->length, &data) ||
		    check_add_overflow(map, len, &map)) {
			ret = -EOVERFLOW;
			goto err_free_table;
		}

		sg_set_page(dst, phys_to_page(start), len, 0);
		dst = sg_next(dst);
	}

	if (!data || !map) {
		ret = -EINVAL;
		goto err_free_table;
	}

	*data_size = data;
	*map_size = map;

	return 0;

err_free_table:
	sg_free_table(aligned_sgt);
	return ret;
}

static int rga_dma_map_sgt_iommu(struct sg_table *sgt,
				 struct rga_dma_buffer *buffer,
				 enum dma_data_direction dir,
				 struct device *map_dev)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(map_dev);
	struct sg_table aligned_sgt;
	size_t data_size;
	size_t map_size;
	size_t buffer_size;
	size_t offset;
	dma_addr_t iova;
	ssize_t mapped;
	int prot;
	int ret;

	if (!domain || !(domain->type & __IOMMU_DOMAIN_PAGING))
		return -EOPNOTSUPP;

	memset(&aligned_sgt, 0, sizeof(aligned_sgt));
	ret = rga_dma_alloc_aligned_sgt(sgt, &aligned_sgt, &data_size,
					&map_size);
	if (ret)
		return ret;

	offset = sgt->sgl->offset;
	if (check_add_overflow(offset, data_size, &buffer_size)) {
		ret = -EOVERFLOW;
		goto err_free_aligned_sgt;
	}

	ret = rga_dma_alloc_iommu_iova(domain, map_dev, map_size, &iova);
	if (ret)
		goto err_free_aligned_sgt;

	prot = rga_dma_iommu_prot(map_dev, dir);
	if (!prot) {
		ret = -EINVAL;
		goto err_free_iova;
	}

	mapped = iommu_map_sg(domain, iova, aligned_sgt.sgl,
			      aligned_sgt.orig_nents, prot, GFP_KERNEL);
	if (mapped < 0) {
		ret = mapped;
		goto err_free_iova;
	}
	if ((size_t)mapped < map_size) {
		if (mapped)
			iommu_unmap(domain, iova, mapped);
		ret = -EIO;
		goto err_free_iova;
	}

	ret = rga_dma_check_iova_span(iova + offset, data_size,
				      "driver-owned IOMMU", true);
	if (ret)
		goto err_unmap_iova;

	sg_free_table(&aligned_sgt);

	buffer->sgt = sgt;
	buffer->domain = domain;
	buffer->dir = dir;
	buffer->iova = iova;
	buffer->dma_addr = iova;
	buffer->iova_size = map_size;
	buffer->size = buffer_size;
	buffer->offset = offset;
	buffer->map_dev = map_dev;
	buffer->iommu_mapped = true;

	return 0;

err_unmap_iova:
	iommu_unmap(domain, iova, map_size);
err_free_iova:
	rga_dma_free_iommu_iova(domain, iova, map_size);
err_free_aligned_sgt:
	sg_free_table(&aligned_sgt);
	return ret;
}

static void rga_dma_unmap_sgt_iommu(struct rga_dma_buffer *buffer)
{
	size_t unmapped;

	if (!buffer->domain || !buffer->iova_size)
		return;

	unmapped = iommu_unmap(buffer->domain, buffer->iova,
			       buffer->iova_size);
	if (unmapped != buffer->iova_size)
		rga_err("driver-owned IOMMU unmap short: iova = %pad, size = %zu, unmapped = %zu\n",
			&buffer->iova, buffer->iova_size, unmapped);

	rga_dma_free_iommu_iova(buffer->domain, buffer->iova,
				buffer->iova_size);
}

static int rga_dma_set_buffer_mapping(struct sg_table *sgt,
				      struct rga_dma_buffer *buffer,
				      enum dma_data_direction dir,
				      struct device *map_dev,
				      const char *source)
{
	int ret;

	ret = rga_dma_check_iova_contract(sgt, source, true);
	if (ret)
		return ret;

	buffer->sgt = sgt;
	buffer->dma_addr = sg_dma_address(sgt->sgl);
	buffer->dir = dir;
	buffer->size = sg_dma_len(sgt->sgl);
	buffer->map_dev = map_dev;
	buffer->offset = 0;
	buffer->iova_size = 0;
	buffer->iommu_mapped = false;

	return 0;
}

int rga_virtual_memory_check(void *vaddr, u32 w, u32 h, u32 format, int fd)
{
	int bits = 32;
	int temp_data = 0;
	void *one_line = NULL;

	bits = rga_get_format_bits(format);
	if (bits < 0)
		return -1;

	one_line = kzalloc(w * 4, GFP_KERNEL);
	if (!one_line) {
		rga_err("kzalloc fail %s[%d]\n", __func__, __LINE__);
		return 0;
	}

	temp_data = w * (h - 1) * bits >> 3;
	if (fd > 0) {
		rga_log("vaddr is%p, bits is %d, fd check\n", vaddr, bits);
		memcpy(one_line, (char *)vaddr + temp_data, w * bits >> 3);
		rga_log("fd check ok\n");
	} else {
		rga_log("vir addr memory check.\n");
		memcpy((void *)((char *)vaddr + temp_data), one_line,
			 w * bits >> 3);
		rga_log("vir addr check ok.\n");
	}

	kfree(one_line);
	return 0;
}

int rga_dma_memory_check(struct rga_dma_buffer *rga_dma_buffer, struct rga_img_info_t *img)
{
	int ret = 0;
	void *vaddr;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	struct iosys_map map;
#endif
	struct dma_buf *dma_buf;

	dma_buf = rga_dma_buffer->dma_buf;

	if (!IS_ERR_OR_NULL(dma_buf)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		ret = dma_buf_vmap_unlocked(dma_buf, &map);
		vaddr = ret ? NULL : map.vaddr;
#else
		vaddr = dma_buf_vmap(dma_buf);
#endif
		if (vaddr) {
			ret = rga_virtual_memory_check(vaddr, img->vir_w,
				img->vir_h, img->format, img->yrgb_addr);
		} else {
			rga_err("can't vmap the dma buffer!\n");
			return -EINVAL;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		dma_buf_vunmap_unlocked(dma_buf, &map);
#else
		dma_buf_vunmap(dma_buf, vaddr);
#endif
	}

	return ret;
}

int rga_dma_map_sgt(struct sg_table *sgt, struct rga_dma_buffer *buffer,
		    enum dma_data_direction dir, struct device *map_dev)
{
	int ret = 0;

	ret = dma_map_sg(map_dev, sgt->sgl, sgt->orig_nents, dir);
	if (ret <= 0) {
		rga_err("dma_map_sg failed! ret = %d\n", ret);
		return ret < 0 ? ret : -EINVAL;
	}
	sgt->nents = ret;

	ret = rga_dma_check_iova_contract(sgt, "sg_table", false);
	if (ret) {
		dma_unmap_sg(map_dev, sgt->sgl, sgt->orig_nents, dir);
		rga_dma_reset_sgt_dma_state(sgt);

		if (ret == -EOPNOTSUPP || ret == -EOVERFLOW) {
			ret = rga_dma_map_sgt_iommu(sgt, buffer, dir,
						    map_dev);
			if (!ret)
				return 0;
		}

		return ret;
	}

	ret = rga_dma_set_buffer_mapping(sgt, buffer, dir, map_dev,
					 "sg_table");
	if (ret) {
		dma_unmap_sg(map_dev, sgt->sgl, sgt->orig_nents, dir);
		return ret;
	}

	return 0;
}

void rga_dma_unmap_sgt(struct rga_dma_buffer *buffer)
{
	if (!buffer->sgt)
		return;

	if (buffer->iommu_mapped) {
		rga_dma_unmap_sgt_iommu(buffer);
		return;
	}

	dma_unmap_sg(buffer->map_dev,
		     buffer->sgt->sgl,
		     buffer->sgt->orig_nents,
		     buffer->dir);
}

int rga_dma_map_buf(struct dma_buf *dma_buf, struct rga_dma_buffer *rga_dma_buffer,
		    enum dma_data_direction dir, struct device *map_dev)
{
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *sgt = NULL;
	int ret = 0;

	if (dma_buf != NULL) {
		get_dma_buf(dma_buf);
	} else {
		rga_err("dma_buf is invalid[%p]\n", dma_buf);
		return -EINVAL;
	}

	attach = dma_buf_attach(dma_buf, map_dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		rga_err("Failed to attach dma_buf, ret[%d]\n", ret);
		goto err_get_attach;
	}

	sgt = dma_buf_map_attachment_unlocked(attach, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		rga_err("Failed to map attachment, ret[%d]\n", ret);
		goto err_get_sgt;
	}

	ret = rga_dma_set_buffer_mapping(sgt, rga_dma_buffer, dir, map_dev, "dma_buf");
	if (ret)
		goto err_map_attachment;

	rga_dma_buffer->dma_buf = dma_buf;
	rga_dma_buffer->attach = attach;

	return ret;

err_map_attachment:
	dma_buf_unmap_attachment_unlocked(attach, sgt, dir);
err_get_sgt:
	if (attach)
		dma_buf_detach(dma_buf, attach);
err_get_attach:
	if (dma_buf)
		dma_buf_put(dma_buf);

	return ret;
}

int rga_dma_map_fd(int fd, struct rga_dma_buffer *rga_dma_buffer,
		   enum dma_data_direction dir, struct device *map_dev)
{
	struct dma_buf *dma_buf = NULL;
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *sgt = NULL;
	int ret = 0;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf)) {
		ret = PTR_ERR(dma_buf);
		rga_err("Fail to get dma_buf from fd[%d], ret[%d]\n", fd, ret);
		return ret;
	}

	attach = dma_buf_attach(dma_buf, map_dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		rga_err("Failed to attach dma_buf, ret[%d]\n", ret);
		goto err_get_attach;
	}

	sgt = dma_buf_map_attachment_unlocked(attach, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		rga_err("Failed to map attachment, ret[%d]\n", ret);
		goto err_get_sgt;
	}

	ret = rga_dma_set_buffer_mapping(sgt, rga_dma_buffer, dir, map_dev, "dma_buf_fd");
	if (ret)
		goto err_map_attachment;

	rga_dma_buffer->dma_buf = dma_buf;
	rga_dma_buffer->attach = attach;

	return ret;

err_map_attachment:
	dma_buf_unmap_attachment_unlocked(attach, sgt, dir);
err_get_sgt:
	if (attach)
		dma_buf_detach(dma_buf, attach);
err_get_attach:
	if (dma_buf)
		dma_buf_put(dma_buf);

	return ret;
}

void rga_dma_unmap_buf(struct rga_dma_buffer *rga_dma_buffer)
{
	if (rga_dma_buffer->attach && rga_dma_buffer->sgt)
		dma_buf_unmap_attachment_unlocked(rga_dma_buffer->attach,
						  rga_dma_buffer->sgt,
						  rga_dma_buffer->dir);

	if (rga_dma_buffer->attach) {
		dma_buf_detach(rga_dma_buffer->dma_buf, rga_dma_buffer->attach);
		dma_buf_put(rga_dma_buffer->dma_buf);
	}
}

int rga_dma_free(struct rga_dma_buffer *buffer)
{
	if (buffer == NULL) {
		rga_err("rga_dma_buffer is NULL.\n");
		return -EINVAL;
	}

	dma_free_coherent(buffer->map_dev, buffer->size, buffer->vaddr, buffer->dma_addr);
	buffer->vaddr = NULL;
	buffer->dma_addr = 0;
	buffer->iova = 0;
	buffer->size = 0;
	buffer->map_dev = NULL;

	kfree(buffer);

	return 0;
}

struct rga_dma_buffer *rga_dma_alloc_coherent(struct rga_scheduler_t *scheduler,
					      int size)
{
	size_t align_size;
	dma_addr_t dma_addr;
	struct  rga_dma_buffer *buffer;
	struct device *map_dev;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	align_size = PAGE_ALIGN(size);
	map_dev = scheduler->iommu_info ? scheduler->iommu_info->default_dev : scheduler->dev;
	buffer->vaddr = dma_alloc_coherent(map_dev, align_size, &dma_addr, GFP_KERNEL);
	if (!buffer->vaddr)
		goto fail_dma_alloc;

	buffer->size = align_size;
	buffer->dma_addr = dma_addr;
	buffer->map_dev = map_dev;
	if (scheduler->data->mmu == RGA_IOMMU)
		buffer->iova = buffer->dma_addr;

	return buffer;

fail_dma_alloc:
	kfree(buffer);

	return NULL;
}

struct rga_dma_buf_pool *rga_dma_buf_pool_init(struct rga_scheduler_t *scheduler, int block_size)
{
	int ret;
	struct rga_dma_buf_pool *pool;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool) {
		rga_err("Failed to allocate memory for rga_dma_buf_pool.\n");
		return ERR_PTR(-ENOMEM);
	}

#ifdef CONFIG_ROCKCHIP_RGA_GENPOOL
	block_size = ALIGN(block_size, scheduler->data->byte_stride_align);

	pool->dma_buf = rga_dma_alloc_coherent(scheduler,
					       block_size * CONFIG_ROCKCHIP_RGA_CMD_BUF_COUNT);
	if (pool->dma_buf == NULL) {
		rga_err("Failed to allocate coherent memory for dma_buf_pool.\n");
		ret = -ENOMEM;
		goto err_free_pool;
	}

	pool->pool = gen_pool_create(ilog2(block_size), -1);
	if (!pool->pool) {
		rga_err("Failed to create memory pool.\n");
		ret = -ENOMEM;
		goto err_free_dma_buf;
	}

	ret = gen_pool_add_virt(pool->pool, (unsigned long)pool->dma_buf->vaddr,
				pool->dma_buf->dma_addr, pool->dma_buf->size, -1);
	if (ret < 0) {
		rga_err("Failed to add memory to gen_pool.\n");
		goto err_destroy_pool;
	}
#else
	pool->pool = dma_pool_create("rga_cmd_buf_pool",
				     scheduler->iommu_info ? scheduler->iommu_info->default_dev :
							     scheduler->dev,
				     block_size, scheduler->data->byte_stride_align, 0);
	if (!pool->pool) {
		rga_err("Failed to create dma pool.\n");
		ret = -ENOMEM;
		goto err_free_pool;
	}
#endif

	pool->scheduler = scheduler;
	pool->block_size = block_size;

	return pool;

#ifdef CONFIG_ROCKCHIP_RGA_GENPOOL
err_destroy_pool:
	gen_pool_destroy(pool->pool);
	pool->pool = NULL;

err_free_dma_buf:
	rga_dma_free(pool->dma_buf);
	pool->dma_buf = NULL;
#endif

err_free_pool:
	kfree(pool);

	return ERR_PTR(ret);
}

void rga_dma_buf_pool_destroy(struct rga_dma_buf_pool *pool)
{
	if (!pool)
		return;

	if (pool->pool) {
#ifdef CONFIG_ROCKCHIP_RGA_GENPOOL
		gen_pool_destroy(pool->pool);
#else
		dma_pool_destroy(pool->pool);
#endif
		pool->pool = NULL;
	}

	if (pool->dma_buf) {
		rga_dma_free(pool->dma_buf);
		pool->dma_buf = NULL;
	}

	kfree(pool);
}

struct rga_dma_buffer *rga_dma_buf_pool_alloc(struct rga_dma_buf_pool *pool)
{
	struct rga_dma_buffer *buffer;

	if (!pool || !pool->pool) {
		rga_err("rga_dma_buf_pool is not initialized.\n");
		return NULL;
	}

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		rga_err("Failed to allocate memory for rga_dma_buffer.\n");
		return NULL;
	}

#ifdef CONFIG_ROCKCHIP_RGA_GENPOOL
	buffer->vaddr = gen_pool_dma_zalloc(pool->pool, pool->block_size, &buffer->dma_addr);
#else
	buffer->vaddr = dma_pool_zalloc(pool->pool, GFP_KERNEL, &buffer->dma_addr);
#endif
	if (!buffer->vaddr) {
		rga_err("Failed to allocate memory from gen_pool.\n");
		kfree(buffer);
		return NULL;
	}

	buffer->size = pool->block_size;
	buffer->map_dev = pool->scheduler->dev;
	if (pool->scheduler->data->mmu == RGA_IOMMU)
		buffer->iova = buffer->dma_addr;

	return buffer;
}

int rga_dma_buf_pool_free(struct rga_dma_buf_pool *pool, struct rga_dma_buffer *buffer)
{
	if (!pool || !pool->pool || !buffer || !buffer->vaddr) {
		rga_err("Invalid pool or buffer.\n");
		return -EINVAL;
	}

#ifdef CONFIG_ROCKCHIP_RGA_GENPOOL
	gen_pool_free(pool->pool, (unsigned long)buffer->vaddr, buffer->size);
#else
	dma_pool_free(pool->pool, buffer->vaddr, buffer->dma_addr);
#endif
	buffer->vaddr = NULL;
	buffer->dma_addr = 0;
	buffer->iova = 0;

	kfree(buffer);

	return 0;
}
