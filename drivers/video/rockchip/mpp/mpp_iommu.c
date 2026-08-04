// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2019 Fuzhou Rockchip Electronics Co., Ltd
 *
 * author:
 *	Alpha Lin, alpha.lin@rock-chips.com
 *	Randy Li, randy.li@rock-chips.com
 *	Ding Wei, leo.ding@rock-chips.com
 *
 */
#include <linux/delay.h>
/* 6.18: <linux/dma-buf-cache.h> does not exist upstream; CONFIG_DMABUF_CACHE
 * folds to 0 so the cache path is dead. Use the standard dma-buf header.
 */
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/kref.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

#ifdef CONFIG_ARM_DMA_USE_IOMMU
#include <asm/dma-iommu.h>
#endif
#include <soc/rockchip/rockchip_iommu.h>
#include <soc/rockchip/vsi_iommu.h>

#include "mpp_debug.h"
#include "mpp_iommu.h"
#include "mpp_common.h"

/*
 * Caller must hold dma->list_mutex, and must keep holding it for as long as it
 * uses the returned pointer: no reference is taken, so dropping the lock lets
 * the slot be released and recycled for a different dma_buf underneath it.
 */
static struct mpp_dma_buffer *
mpp_dma_find_buffer_locked(struct mpp_dma_session *dma, struct dma_buf *dmabuf)
{
	struct mpp_dma_buffer *out = NULL;
	struct mpp_dma_buffer *buffer = NULL, *n;
	int find = 0;

	list_for_each_entry_safe(buffer, n,
				 &dma->used_list, link) {
		/*
		 * fd may dup several and point the same dambuf.
		 * thus, here should be distinguish with the dmabuf.
		 */
		if (buffer->dmabuf == dmabuf) {
			out = buffer;
			find = 1;
			list_move_tail(&buffer->link, &buffer->dma->used_list);
			break;
		}
	}
	if (!find) {
		list_for_each_entry_safe(buffer, n,
					&dma->static_list, link) {
			if (buffer->dmabuf == dmabuf) {
				out = buffer;
				list_move_tail(&buffer->link, &buffer->dma->static_list);
				break;
			}
		}
	}

	return out;
}

struct mpp_dma_buffer *
mpp_dma_find_buffer_fd(struct mpp_dma_session *dma, int fd)
{
	struct dma_buf *dmabuf;
	struct mpp_dma_buffer *out = NULL;
	struct mpp_dma_buffer *buffer = NULL, *n;
	int find = 0;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return NULL;

	mutex_lock(&dma->list_mutex);
	list_for_each_entry_safe(buffer, n,
				 &dma->used_list, link) {
		/*
		 * fd may dup several and point the same dambuf.
		 * thus, here should be distinguish with the dmabuf.
		 */
		if (buffer->dmabuf == dmabuf) {
			out = buffer;
			find = 1;
			list_move_tail(&buffer->link, &buffer->dma->used_list);
			break;
		}
	}
	if (!find) {
		list_for_each_entry_safe(buffer, n,
					&dma->static_list, link) {
			/*
			 * fd may dup several and point the same dambuf.
			 * thus, here should be distinguish with the dmabuf.
			 */
			if (buffer->dmabuf == dmabuf) {
				out = buffer;
				list_move_tail(&buffer->link, &buffer->dma->static_list);
				break;
			}
		}
	}

	mutex_unlock(&dma->list_mutex);
	dma_buf_put(dmabuf);

	return out;
}

static int mpp_dma_check_iova_contract(struct mpp_dma_session *dma, int fd,
				       struct sg_table *sgt)
{
	dma_addr_t dma_addr;
	unsigned int dma_len;
	u64 dma_end;

	if (!sgt || !sgt->sgl) {
		dev_err(dma->dev, "reject fd %d DMA mapping: empty sg table\n", fd);
		return -EINVAL;
	}

	if (sgt->nents != 1) {
		dev_err(dma->dev,
			"reject fd %d DMA mapping: expected one DMA segment, got %u, orig_nents = %u\n",
			fd, sgt->nents, sgt->orig_nents);
		return -EOPNOTSUPP;
	}

	dma_addr = sg_dma_address(sgt->sgl);
	dma_len = sg_dma_len(sgt->sgl);
	if (!dma_len) {
		dev_err(dma->dev,
			"reject fd %d DMA mapping: zero length segment, iova = %pad\n",
			fd, &dma_addr);
		return -EINVAL;
	}

	dma_end = (u64)dma_addr + dma_len - 1;
	if (dma_addr > DMA_BIT_MASK(32) || dma_end > DMA_BIT_MASK(32)) {
		dev_err(dma->dev,
			"reject fd %d DMA mapping: 32-bit IOVA span overflow, iova = %pad, size = %u, end = 0x%llx\n",
			fd, &dma_addr, dma_len, dma_end);
		return -EOVERFLOW;
	}

	return 0;
}

/* Release the buffer from the current list */
static void mpp_dma_release_buffer(struct kref *ref)
{
	struct mpp_dma_buffer *buffer =
		container_of(ref, struct mpp_dma_buffer, ref);
	struct dma_buf_attachment *attach = buffer->attach;
	struct dma_buf *dmabuf = buffer->dmabuf;
	struct sg_table *sgt = buffer->sgt;

	buffer->dma->buffer_count--;
	list_move_tail(&buffer->link, &buffer->dma->unused_list);

	/*
	 * Clear the pointers before dropping what they point at. dma_buf_detach()
	 * frees the attachment's scatterlist, so publishing NULL afterwards left
	 * a window in which mpp_dma_buf_sync() -- which runs from the task worker
	 * and the IRQ completion path -- could walk a freed sg table.
	 */
	buffer->dmabuf = NULL;
	buffer->attach = NULL;
	buffer->sgt = NULL;

	/* 6.18: locked variant now asserts dma_resv held; use _unlocked */
	dma_buf_unmap_attachment_unlocked(attach, sgt, buffer->dir);
	dma_buf_detach(dmabuf, attach);
	dma_buf_put(dmabuf);
	buffer->dma = NULL;
	buffer->copy_sgt = NULL;
	buffer->iova = 0;
	buffer->size = 0;
	buffer->vaddr = NULL;
	buffer->static_cnt = 0;
	buffer->last_used = 0;
}

/* Remove the oldest buffer when count more than the setting */
static int
mpp_dma_remove_extra_buffer(struct mpp_dma_session *dma)
{
	struct mpp_dma_buffer *n;
	struct mpp_dma_buffer *removable = NULL, *buffer = NULL;

	if (dma->buffer_count > dma->max_buffers) {
		mutex_lock(&dma->list_mutex);
		list_for_each_entry_safe(buffer, n,
					 &dma->used_list,
					 link) {
			if (kref_read(&buffer->ref) == 1) {
				removable = buffer;
				break;
			}
		}
		if (removable)
			kref_put(&removable->ref, mpp_dma_release_buffer);
		mutex_unlock(&dma->list_mutex);
	}

	return 0;
}

int mpp_dma_release(struct mpp_dma_session *dma,
		    struct mpp_dma_buffer *buffer)
{
	if (!dma)
		return -EINVAL;

	mutex_lock(&dma->list_mutex);
	kref_put(&buffer->ref, mpp_dma_release_buffer);
	mutex_unlock(&dma->list_mutex);

	return 0;
}

int mpp_dma_release_fd(struct mpp_dma_session *dma, int fd)
{
	struct device *dev;
	struct dma_buf *dmabuf;
	struct mpp_dma_buffer *buffer = NULL;

	/*
	 * A session that never issued MPP_CMD_INIT_CLIENT_TYPE has no DMA
	 * session (session->dma stays NULL). Callers must not dereference it:
	 * fail closed instead of oopsing on dma->dev.
	 */
	if (!dma)
		return -EINVAL;

	dev = dma->dev;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		dev_err_ratelimited(dev, "can not get dma_buf for fd %d\n", fd);

		return -EINVAL;
	}

	/*
	 * Look up, check and put under one hold of list_mutex. Splitting them
	 * around mpp_dma_find_buffer_fd()'s own lock would leave a window in
	 * which the slot is released and recycled for a different dma_buf,
	 * and this would then spend the new owner's count.
	 */
	mutex_lock(&dma->list_mutex);
	buffer = mpp_dma_find_buffer_locked(dma, dmabuf);
	if (!buffer) {
		mutex_unlock(&dma->list_mutex);
		dma_buf_put(dmabuf);
		dev_err_ratelimited(dev, "can not find %d buffer in list\n", fd);

		return -EINVAL;
	}

	/*
	 * Only give back a reference this ioctl handed out. Without the count
	 * this is an unauthenticated kref_put: MPP_CMD_RELEASE_FD takes an
	 * array of up to MPP_MAX_REG_TRANS_NUM fds and never checked that the
	 * caller owned any of them, so repeating one fd drove a buffer that an
	 * in-flight task still referenced to zero -- unmapping its IOVA under
	 * live DMA and then recycling the slot under the task's stale pointer.
	 */
	if (!buffer->static_cnt) {
		mutex_unlock(&dma->list_mutex);
		dma_buf_put(dmabuf);
		dev_err_ratelimited(dev, "fd %d was not imported by this session\n",
				    fd);

		return -EINVAL;
	}
	buffer->static_cnt--;
	kref_put(&buffer->ref, mpp_dma_release_buffer);
	mutex_unlock(&dma->list_mutex);
	dma_buf_put(dmabuf);

	return 0;
}

struct mpp_dma_buffer *
mpp_dma_alloc(struct device *dev, size_t size)
{
	size_t align_size;
	dma_addr_t iova;
	struct  mpp_dma_buffer *buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	align_size = PAGE_ALIGN(size);
	buffer->vaddr = dma_alloc_coherent(dev, align_size, &iova, GFP_KERNEL);
	if (!buffer->vaddr)
		goto fail_dma_alloc;

	buffer->size = align_size;
	buffer->iova = iova;
	buffer->dev = dev;

	return buffer;
fail_dma_alloc:
	kfree(buffer);
	return NULL;
}

int mpp_dma_free(struct mpp_dma_buffer *buffer)
{
	dma_free_coherent(buffer->dev, buffer->size,
			buffer->vaddr, buffer->iova);
	buffer->vaddr = NULL;
	buffer->iova = 0;
	buffer->size = 0;
	buffer->dev = NULL;
	kfree(buffer);

	return 0;
}

struct mpp_dma_buffer *mpp_dma_import_fd(struct mpp_iommu_info *iommu_info,
					 struct mpp_dma_session *dma,
					 int fd, int static_use)
{
	int ret = 0;
	struct sg_table *sgt;
	struct dma_buf *dmabuf;
	struct mpp_dma_buffer *buffer;
	struct dma_buf_attachment *attach;

	if (!dma) {
		mpp_err("dma session is null\n");
		return ERR_PTR(-EINVAL);
	}

	/* remove the oldest before add buffer */
	if (!IS_ENABLED(CONFIG_DMABUF_CACHE))
		mpp_dma_remove_extra_buffer(dma);

	/* Check whether in dma session */
	buffer = mpp_dma_find_buffer_fd(dma, fd);
	if (!IS_ERR_OR_NULL(buffer)) {
		if (kref_get_unless_zero(&buffer->ref)) {
			if (static_use) {
				mutex_lock(&dma->list_mutex);
				buffer->static_cnt++;
				mutex_unlock(&dma->list_mutex);
			}
			return buffer;
		}
		dev_dbg(dma->dev, "missing the fd %d\n", fd);
	}

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		mpp_err("dma_buf_get fd %d failed(%d)\n", fd, ret);
		return ERR_PTR(ret);
	}
	/* A new DMA buffer */
	mutex_lock(&dma->list_mutex);
	buffer = list_first_entry_or_null(&dma->unused_list,
					   struct mpp_dma_buffer,
					   link);
	if (!buffer) {
		ret = -ENOMEM;
		mutex_unlock(&dma->list_mutex);
		goto fail;
	}
	list_del_init(&buffer->link);
	mutex_unlock(&dma->list_mutex);

	buffer->dmabuf = dmabuf;
	buffer->dir = DMA_BIDIRECTIONAL;

	attach = dma_buf_attach(buffer->dmabuf, dma->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		mpp_err("dma_buf_attach fd %d failed(%d)\n", fd, ret);
		goto fail_attach;
	}

	/* 6.18: locked variant now asserts dma_resv held; use _unlocked */
	sgt = dma_buf_map_attachment_unlocked(attach, buffer->dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		mpp_err("dma_buf_map_attachment fd %d failed(%d)\n", fd, ret);
		goto fail_map;
	}
	ret = mpp_dma_check_iova_contract(dma, fd, sgt);
	if (ret)
		goto fail_map_attachment;

	buffer->iova = sg_dma_address(sgt->sgl);
	buffer->size = sg_dma_len(sgt->sgl);
	buffer->attach = attach;
	buffer->sgt = sgt;
	buffer->dma = dma;

	kref_init(&buffer->ref);

	if (!static_use && !IS_ENABLED(CONFIG_DMABUF_CACHE))
		/* Increase the reference for used outside the buffer pool */
		kref_get(&buffer->ref);

	mutex_lock(&dma->list_mutex);
	dma->buffer_count++;
	if (static_use) {
		buffer->static_cnt = 1;
		list_add_tail(&buffer->link, &dma->static_list);
	} else {
		buffer->static_cnt = 0;
		list_add_tail(&buffer->link, &dma->used_list);
	}
	mutex_unlock(&dma->list_mutex);

	return buffer;

fail_map_attachment:
	dma_buf_unmap_attachment_unlocked(attach, sgt, buffer->dir);
fail_map:
	dma_buf_detach(buffer->dmabuf, attach);
fail_attach:
	mutex_lock(&dma->list_mutex);
	list_add_tail(&buffer->link, &dma->unused_list);
	mutex_unlock(&dma->list_mutex);
fail:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

int mpp_dma_unmap_kernel(struct mpp_dma_session *dma,
			 struct mpp_dma_buffer *buffer)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(buffer->vaddr);
	struct dma_buf *dmabuf = buffer->dmabuf;

	if (IS_ERR_OR_NULL(map.vaddr) ||
	    IS_ERR_OR_NULL(dmabuf))
		return -EINVAL;

	/* 6.18: locked variant now asserts dma_resv held; use _unlocked */
	dma_buf_vunmap_unlocked(dmabuf, &map);
	buffer->vaddr = NULL;

	dma_buf_end_cpu_access(dmabuf, DMA_FROM_DEVICE);

	return 0;
}

int mpp_dma_map_kernel(struct mpp_dma_session *dma,
		       struct mpp_dma_buffer *buffer)
{
	int ret;
	struct iosys_map map;
	struct dma_buf *dmabuf = buffer->dmabuf;

	if (IS_ERR_OR_NULL(dmabuf))
		return -EINVAL;

	ret = dma_buf_begin_cpu_access(dmabuf, DMA_FROM_DEVICE);
	if (ret) {
		dev_dbg(dma->dev, "can't access the dma buffer\n");
		goto failed_access;
	}

	/* 6.18: locked variant now asserts dma_resv held; use _unlocked */
	ret = dma_buf_vmap_unlocked(dmabuf, &map);
	if (ret) {
		dev_dbg(dma->dev, "can't vmap the dma buffer\n");
		goto failed_vmap;
	}

	buffer->vaddr = map.vaddr;

	return 0;

failed_vmap:
	dma_buf_end_cpu_access(dmabuf, DMA_FROM_DEVICE);
failed_access:

	return ret;
}

int mpp_dma_session_destroy(struct mpp_dma_session *dma)
{
	struct mpp_dma_buffer *n, *buffer = NULL;

	if (!dma)
		return -EINVAL;

	mutex_lock(&dma->list_mutex);
	list_for_each_entry_safe(buffer, n,
				 &dma->used_list,
				 link) {
		kref_put(&buffer->ref, mpp_dma_release_buffer);
	}
	list_for_each_entry_safe(buffer, n,
				 &dma->static_list,
				 link) {
		kref_put(&buffer->ref, mpp_dma_release_buffer);
	}
	mutex_unlock(&dma->list_mutex);

	kfree(dma);

	return 0;
}

struct mpp_dma_session *
mpp_dma_session_create(struct device *dev, u32 max_buffers)
{
	int i;
	struct mpp_dma_session *dma = NULL;
	struct mpp_dma_buffer *buffer = NULL;

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return NULL;

	mutex_init(&dma->list_mutex);
	INIT_LIST_HEAD(&dma->unused_list);
	INIT_LIST_HEAD(&dma->used_list);
	INIT_LIST_HEAD(&dma->static_list);

	if (max_buffers > MPP_SESSION_MAX_BUFFERS) {
		mpp_debug(DEBUG_IOCTL, "session_max_buffer %d must less than %d\n",
			  max_buffers, MPP_SESSION_MAX_BUFFERS);
		dma->max_buffers = MPP_SESSION_MAX_BUFFERS;
	} else {
		dma->max_buffers = max_buffers;
	}

	for (i = 0; i < ARRAY_SIZE(dma->dma_bufs); i++) {
		buffer = &dma->dma_bufs[i];
		buffer->dma = dma;
		INIT_LIST_HEAD(&buffer->link);
		list_add_tail(&buffer->link, &dma->unused_list);
	}
	dma->dev = dev;

	return dma;
}

/*
 * begin cpu access => for_cpu = true
 * end cpu access => for_cpu = false
 */
void mpp_dma_buf_sync(struct mpp_dma_buffer *buffer, u32 offset, u32 length,
		      enum dma_data_direction dir, bool for_cpu)
{
	struct device *dev;
	struct sg_table *sgt;
	struct scatterlist *sg;
	dma_addr_t sg_dma_addr;
	unsigned int len = 0;
	int i;

	/*
	 * A buffer released concurrently has these cleared before its backing
	 * is dropped, so catch the already-released case rather than walking a
	 * freed sg table.
	 */
	if (!buffer || !buffer->dma)
		return;
	sgt = buffer->sgt;
	if (!sgt || !sgt->sgl)
		return;

	dev = buffer->dma->dev;
	sg = sgt->sgl;
	sg_dma_addr = sg_dma_address(sg);

	for_each_sgtable_sg(sgt, sg, i) {
		unsigned int sg_offset, sg_left, size = 0;

		len += sg->length;
		if (len <= offset) {
			sg_dma_addr += sg->length;
			continue;
		}

		sg_left = len - offset;
		sg_offset = sg->length - sg_left;

		size = (length < sg_left) ? length : sg_left;

		if (for_cpu)
			dma_sync_single_range_for_cpu(dev, sg_dma_addr,
						      sg_offset, size, dir);
		else
			dma_sync_single_range_for_device(dev, sg_dma_addr,
							 sg_offset, size, dir);

		offset += size;
		length -= size;
		sg_dma_addr += sg->length;

		if (length == 0)
			break;
	}
}

int mpp_iommu_detach(struct mpp_iommu_info *info)
{
	if (!info)
		return 0;

	iommu_detach_group(info->domain, info->group);
	return 0;
}

int mpp_iommu_attach(struct mpp_iommu_info *info)
{
	struct mpp_iommu_info *last_info;

	if (!info)
		return 0;

	/* if device changed, detach last first */
	last_info = info->queue->last_iommu_info;
	if (info->shared && last_info && last_info->shared
	    && (info->dev != last_info->dev)) {
		iommu_detach_group(last_info->domain, last_info->group);
	}
	info->queue->last_iommu_info = info;

	if (info->domain == iommu_get_domain_for_dev(info->dev))
		return 0;

	return iommu_attach_group(info->domain, info->group);
}

static int mpp_iommu_attach_current_domain(struct mpp_iommu_info *info)
{
	if (!info)
		return 0;

	if (info->domain == iommu_get_domain_for_dev(info->dev))
		return 0;

	return iommu_attach_group(info->domain, info->group);
}

static int mpp_iommu_check_iova_span(dma_addr_t iova, size_t size, u64 *end)
{
	u64 dma_end;

	if (!size)
		return -EINVAL;

	if (check_add_overflow((u64)iova, (u64)size - 1, &dma_end))
		return -EOVERFLOW;

	if (iova > DMA_BIT_MASK(32) || dma_end > DMA_BIT_MASK(32))
		return -EOVERFLOW;

	if (end)
		*end = dma_end;

	return 0;
}

/*
 * Initialize a CCU cluster's shared domain from its main core.
 *
 * The owner is the service-visible core (rkvdec2 core 0, rkvenc2 main_core);
 * its default DMA domain becomes the cluster's shared IOVA space and its
 * rw_sem becomes the cluster-wide map/unmap/reset lock. Secondary cores are
 * joined later with mpp_iommu_shared_domain_bind().
 */
int mpp_iommu_shared_domain_init(struct mpp_iommu_shared_domain *shared,
				 struct mpp_iommu_info *owner)
{
	if (!shared || !owner)
		return -EINVAL;

	if (!owner->domain || !owner->rw_sem)
		return -ENODEV;

	shared->owner = owner;
	shared->domain = owner->domain;
	shared->rw_sem = owner->rw_sem;
	shared->nr_windows = 0;
	spin_lock_init(&shared->window_lock);

	return 0;
}

/*
 * Join a secondary core to the cluster shared domain.
 *
 * Point the secondary's iommu_info at the shared domain and rw_sem, then
 * attach its IOMMU group to that domain so map/unmap/flush on the shared
 * domain reach this core too. On attach failure the borrowed fields are
 * rolled back, so the caller can fail the core's probe cleanly and the
 * secondary keeps its own default domain.
 *
 * This centralizes the field swap rkvdec2/rkvenc2 previously open-coded in
 * their attach paths and, unlike the old decoder path, always shares the
 * rw_sem so the whole cluster serializes through one lock. Binding the owner
 * is a no-op: it already sits on the shared domain by construction.
 */
int mpp_iommu_shared_domain_bind(struct mpp_iommu_shared_domain *shared,
				 struct mpp_iommu_info *info)
{
	struct iommu_domain *old_domain;
	struct rw_semaphore *old_rw_sem;
	int ret;

	if (!shared || !info)
		return -EINVAL;

	if (!shared->domain || !shared->rw_sem)
		return -ENODEV;

	if (info == shared->owner)
		return 0;

	if (info->shared_domain == shared)
		return 0;

	if (info->shared_domain)
		return -EBUSY;

	old_domain = info->domain;
	old_rw_sem = info->rw_sem;
	info->domain = shared->domain;
	info->rw_sem = shared->rw_sem;

	ret = mpp_iommu_attach_current_domain(info);
	if (ret) {
		info->domain = old_domain;
		info->rw_sem = old_rw_sem;
		return ret;
	}

	info->shared_domain = shared;

	return 0;
}

int mpp_iommu_shared_domain_unbind(struct mpp_iommu_shared_domain *shared,
				   struct mpp_iommu_info *info)
{
	struct iommu_domain *cur;
	int ret;

	if (!shared || !info)
		return 0;

	if (info == shared->owner)
		return 0;

	if (info->shared_domain != shared)
		return 0;

	iommu_detach_group(shared->domain, info->group);

	info->domain = info->default_domain;
	info->rw_sem = info->default_rw_sem;
	info->shared_domain = NULL;

	cur = iommu_get_domain_for_dev(info->dev);
	if (cur == info->domain)
		return 0;

	ret = iommu_attach_group(info->domain, info->group);
	if (ret) {
		dev_err(info->dev, "failed to restore default IOMMU domain: %d\n", ret);
		info->domain = shared->domain;
		info->rw_sem = shared->rw_sem;
		info->shared_domain = shared;
		mpp_iommu_attach_current_domain(info);
		return ret;
	}

	return 0;
}

/*
 * Diagnostic: verify a CCU-bound core is actually attached to the shared
 * domain. This is the single place for shared-domain assertions -- reset,
 * refresh and empty-domain restore paths must always leave a secondary core
 * on the cluster domain, never back on its own default domain. Returns true
 * when the core's current domain is the shared domain.
 *
 * This is a warn-once audit hook, not correctness proof of attachment: do not
 * gate mapping/dispatch on its result.
 */
bool mpp_iommu_shared_domain_verify(struct mpp_iommu_shared_domain *shared,
				    struct mpp_iommu_info *info)
{
	struct iommu_domain *cur;

	if (!shared || !info || !shared->domain)
		return false;

	cur = iommu_get_domain_for_dev(info->dev);
	if (cur != shared->domain) {
		dev_warn_once(info->dev,
			      "iommu: core not on CCU shared domain (cur %p want %p)\n",
			      cur, shared->domain);
		return false;
	}

	return true;
}

/*
 * Record a fixed RCB/SRAM IOVA window for a CCU cluster and reject an overlap.
 *
 * Every core in a cluster maps its own RCB/SRAM window into the one shared
 * domain, each at a distinct fixed IOVA from its "rockchip,rcb-iova" DT
 * property. Two cores whose windows overlap would corrupt each other's fixed
 * buffers in the shared IOVA space, so refuse the second map and say so loudly.
 * Windows are tracked per cluster; the array is cleared when the owner detaches.
 */
int mpp_iommu_shared_domain_reserve_window(struct mpp_iommu_shared_domain *shared,
					   dma_addr_t iova, size_t size,
					   struct device *dev)
{
	unsigned long flags;
	u64 end;
	unsigned int i;
	int ret;

	if (!shared)
		return -EINVAL;

	ret = mpp_iommu_check_iova_span(iova, size, &end);
	if (ret) {
		dev_err(dev,
			"invalid CCU fixed IOVA window %pad+%#zx: %d\n",
			&iova, size, ret);
		return ret;
	}

	spin_lock_irqsave(&shared->window_lock, flags);
	for (i = 0; i < shared->nr_windows; i++) {
		dma_addr_t a0 = shared->windows[i].iova;
		size_t asz = shared->windows[i].size;
		u64 a_end = (u64)a0 + asz - 1;

		if ((u64)iova <= a_end && (u64)a0 <= end) {
			spin_unlock_irqrestore(&shared->window_lock, flags);
			dev_err(dev,
				"CCU fixed IOVA window %pad+%#zx overlaps existing %pad+%#zx\n",
				&iova, size, &a0, asz);
			return -EADDRINUSE;
		}
	}

	if (shared->nr_windows >= MPP_IOMMU_MAX_FIXED_WINDOWS) {
		spin_unlock_irqrestore(&shared->window_lock, flags);
		dev_err(dev, "CCU fixed IOVA window table full; rejecting %pad\n",
			&iova);
		return -ENOSPC;
	}

	shared->windows[shared->nr_windows].iova = iova;
	shared->windows[shared->nr_windows].size = size;
	shared->nr_windows++;
	spin_unlock_irqrestore(&shared->window_lock, flags);

	return 0;
}

void mpp_iommu_shared_domain_unreserve_window(struct mpp_iommu_shared_domain *shared,
					      dma_addr_t iova, size_t size)
{
	unsigned long flags;
	unsigned int i;

	if (!shared || !size)
		return;

	spin_lock_irqsave(&shared->window_lock, flags);
	for (i = 0; i < shared->nr_windows; i++) {
		if (shared->windows[i].iova != iova ||
		    shared->windows[i].size != size)
			continue;

		shared->nr_windows--;
		if (i != shared->nr_windows)
			shared->windows[i] = shared->windows[shared->nr_windows];
		memset(&shared->windows[shared->nr_windows], 0,
		       sizeof(shared->windows[shared->nr_windows]));
		break;
	}
	spin_unlock_irqrestore(&shared->window_lock, flags);
}

static int mpp_iommu_handle(struct iommu_domain *iommu,
			    struct device *iommu_dev,
			    unsigned long iova,
			    int status, void *arg)
{
	struct mpp_dev *mpp = (struct mpp_dev *)arg;
	unsigned long flags;

	/*
	 * Mask iommu irq, in order for iommu not repeatedly trigger pagefault.
	 * Until the pagefault task finish by hw timeout.
	 */
	if (mpp) {
		rockchip_iommu_mask_irq(mpp->dev);
		vsi_iommu_mask_irq(mpp->dev);
	}

	dev_err(iommu_dev, "fault addr 0x%08lx status %x arg %p\n",
		iova, status, arg);

	if (!mpp) {
		dev_err(iommu_dev, "pagefault without device to handle\n");
		return 0;
	}

	spin_lock_irqsave(&mpp->queue->running_lock, flags);
	if (mpp->cur_task)
		mpp_task_dump_mem_region(mpp, mpp->cur_task);
	spin_unlock_irqrestore(&mpp->queue->running_lock, flags);

	if (mpp->dev_ops && mpp->dev_ops->dump_dev)
		mpp->dev_ops->dump_dev(mpp);
	else
		mpp_task_dump_hw_reg(mpp);

	return 0;
}

struct mpp_iommu_info *
mpp_iommu_probe(struct device *dev)
{
	int ret = 0;
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct mpp_iommu_info *info = NULL;
	struct iommu_domain *domain = NULL;
	struct iommu_group *group = NULL;
#ifdef CONFIG_ARM_DMA_USE_IOMMU
	struct dma_iommu_mapping *mapping;
#endif
	np = of_parse_phandle(dev->of_node, "iommus", 0);
	if (!np || !of_device_is_available(np)) {
		mpp_err("failed to get device node\n");
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev) {
		mpp_err("failed to get platform device\n");
		return ERR_PTR(-ENODEV);
	}

	group = iommu_group_get(dev);
	if (!group) {
		ret = -EINVAL;
		goto err_put_pdev;
	}

	/*
	 * On arm32-arch, group->default_domain should be NULL,
	 * domain store in mapping created by arm32-arch.
	 * we re-attach domain here
	 */
#ifdef CONFIG_ARM_DMA_USE_IOMMU
	if (!iommu_group_default_domain(group)) {
		mapping = to_dma_iommu_mapping(dev);
		WARN_ON(!mapping);
		domain = mapping->domain;
	}
#endif
	if (!domain) {
		domain = iommu_get_domain_for_dev(dev);
		if (!domain) {
			ret = -EINVAL;
			goto err_put_group;
		}
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_put_group;
	}

	init_rwsem(&info->rw_sem_self);
	info->rw_sem = &info->rw_sem_self;
	info->default_rw_sem = &info->rw_sem_self;
	spin_lock_init(&info->dev_lock);
	info->dev = dev;
	info->pdev = pdev;
	info->group = group;
	info->domain = domain;
	info->default_domain = domain;
	info->dev_active = NULL;
	info->irq = platform_get_irq(pdev, 0);
	info->got_irq = (info->irq < 0) ? false : true;

	/* get shared flag, if true detach */
	if (IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU)) {
		struct platform_driver *drv = to_platform_driver(dev->driver);

		info->shared = drv->driver_managed_dma;
		if (info->shared)
			iommu_detach_group(info->domain, info->group);
	}

	return info;

err_put_group:
	if (group)
		iommu_group_put(group);
err_put_pdev:
	if (pdev)
		platform_device_put(pdev);

	return ERR_PTR(ret);
}

static void mpp_iommu_clear_fault_handler(struct mpp_iommu_info *info)
{
	if (!info)
		return;

	if (info->rockchip_fault_handler) {
		rockchip_iommu_set_fault_handler(info->dev, NULL, NULL);
		info->rockchip_fault_handler = false;
	}

	if (info->vsi_fault_handler) {
		vsi_iommu_set_fault_handler(info->dev, NULL, NULL);
		info->vsi_fault_handler = false;
	}

	if (info->generic_fault_handler && info->domain &&
	    info->domain->cookie_type == IOMMU_COOKIE_FAULT_HANDLER) {
		info->domain->handler = NULL;
		info->domain->handler_token = NULL;
		info->domain->cookie_type = IOMMU_COOKIE_NONE;
		info->generic_fault_handler = false;
	}
}

void mpp_iommu_quiesce_fault_handler(struct mpp_iommu_info *info)
{
	if (!info)
		return;

	mpp_iommu_clear_fault_handler(info);
	/*
	 * Callback removal waits on the provider registration lock.  Also wait
	 * for the rest of a provider IRQ before codec teardown releases state
	 * that the IRQ path itself can still access.
	 */
	rockchip_iommu_sync_fault_handler(info->dev);
	vsi_iommu_sync_fault_handler(info->dev);
}

int mpp_iommu_remove(struct mpp_iommu_info *info)
{
	if (!info)
		return 0;

	/* if iommu shared, ensure current device's domain, then remove correctly */
	if (info->shared)
		mpp_iommu_attach(info);

	mpp_iommu_quiesce_fault_handler(info);
	iommu_group_put(info->group);
	platform_device_put(info->pdev);

	return 0;
}

int mpp_iommu_refresh(struct mpp_iommu_info *info, struct device *dev)
{
	int ret;

	if (!info)
		return 0;

	/* disable iommu */
	ret = rockchip_iommu_disable(dev);
	if (ret == -ENODEV) {
		ret = vsi_iommu_refresh(dev);
		if (ret == -ENODEV)
			return mpp_iommu_flush_tlb(info);
		return ret;
	}
	if (ret)
		return ret;
	/* re-enable iommu */
	return rockchip_iommu_enable(dev);
}

int mpp_iommu_flush_tlb(struct mpp_iommu_info *info)
{
	if (!info)
		return 0;

	if (info->domain && info->domain->ops)
		iommu_flush_iotlb_all(info->domain);

	return 0;
}

int mpp_iommu_dev_activate(struct mpp_iommu_info *info, struct mpp_dev *dev)
{
	unsigned long flags;
	int ret = 0;

	if (!info)
		return 0;

	spin_lock_irqsave(&info->dev_lock, flags);

	if (info->dev_active || !dev) {
		dev_err(info->dev, "can not activate %s -> %s\n",
			info->dev_active ? dev_name(info->dev_active->dev) : NULL,
			dev ? dev_name(dev->dev) : NULL);
		ret = -EINVAL;
	} else {
		iommu_fault_handler_t handler;

		/*
		 * MPP uses normal DMA domains, so the generic fault-handler cookie path
		 * cannot be used for Rockchip IOMMUs. The provider hook handles that
		 * path and we keep the generic fallback for any cookie-less domain.
		 */
		handler = dev->fault_handler ? dev->fault_handler : mpp_iommu_handle;
		ret = rockchip_iommu_set_fault_handler(info->dev,
						       handler, dev);
		if (!ret) {
			info->rockchip_fault_handler = true;
		} else if (ret == -ENODEV) {
			ret = vsi_iommu_set_fault_handler(info->dev,
							  handler, dev);
			if (!ret)
				info->vsi_fault_handler = true;
		}
		if (ret == -ENODEV) {
			if (info->domain &&
			    info->domain->cookie_type == IOMMU_COOKIE_NONE) {
				iommu_set_fault_handler(info->domain,
							handler, dev);
				info->generic_fault_handler = true;
				ret = 0;
			}
		}

		if (!ret) {
			info->dev_active = dev;
			dev_dbg(info->dev, "activate -> %p %s\n", dev, dev_name(dev->dev));
		}
	}

	spin_unlock_irqrestore(&info->dev_lock, flags);

	return ret;
}

int mpp_iommu_dev_deactivate(struct mpp_iommu_info *info, struct mpp_dev *dev)
{
	unsigned long flags;

	if (!info)
		return 0;

	spin_lock_irqsave(&info->dev_lock, flags);

	if (info->dev_active != dev) {
		dev_err(info->dev, "can not deactivate %s when %s activated\n",
			dev_name(dev->dev),
			info->dev_active ? dev_name(info->dev_active->dev) : NULL);
		spin_unlock_irqrestore(&info->dev_lock, flags);
		return -EBUSY;
	}

	dev_dbg(info->dev, "deactivate %p\n", info->dev_active);
	mpp_iommu_clear_fault_handler(info);
	info->dev_active = NULL;
	spin_unlock_irqrestore(&info->dev_lock, flags);

	return 0;
}

int mpp_iommu_reserve_iova(struct mpp_iommu_info *info, dma_addr_t iova, size_t size)
{
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	unsigned long pfn_lo, pfn_hi;
	u64 end;
	int ret;

	if (!info)
		return -EINVAL;

	ret = mpp_iommu_check_iova_span(iova, size, &end);
	if (ret) {
		dev_err(info->dev, "invalid reserved IOVA span %pad+%#zx: %d\n",
			&iova, size, ret);
		return ret;
	}

	domain = info->domain;
	/*
	 * iova_cookie is one arm of a union discriminated by domain->cookie_type,
	 * so a non-NULL test does not prove it is an iommu_dma_cookie. Use the
	 * accessor that checks the discriminator, as RGA already does.
	 */
	iovad = iommu_dma_get_iova_domain(domain);
	if (!iovad)
		return -EINVAL;

	/* iova will be freed automatically by put_iova_domain() */
	pfn_lo = iova_pfn(iovad, iova);
	pfn_hi = iova_pfn(iovad, end);
	if (!reserve_iova_exclusive(iovad, pfn_lo, pfn_hi))
		return -EBUSY;

	return 0;

}

void mpp_iommu_unreserve_iova(struct mpp_iommu_info *info, dma_addr_t iova, size_t size)
{
	struct iommu_domain *domain;
	struct iova_domain *iovad;

	if (!info || !size)
		return;

	domain = info->domain;
	iovad = iommu_dma_get_iova_domain(domain);
	if (!iovad)
		return;

	free_iova(iovad, iova_pfn(iovad, iova));
}
