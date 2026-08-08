/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Copyright (c) 2019 Fuzhou Rockchip Electronics Co., Ltd
 *
 * author:
 *	Alpha Lin, alpha.lin@rock-chips.com
 *	Randy Li, randy.li@rock-chips.com
 *	Ding Wei, leo.ding@rock-chips.com
 *
 */
#ifndef __ROCKCHIP_MPP_IOMMU_H__
#define __ROCKCHIP_MPP_IOMMU_H__

#include <linux/iommu.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iova.h>
#include <linux/spinlock.h>
#include <linux/stddef.h>

struct mpp_dma_buffer {
	/* link to dma session buffer list */
	struct list_head link;

	/* dma session belong */
	struct mpp_dma_session *dma;
	/* DMABUF information */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct sg_table *copy_sgt;
	enum dma_data_direction dir;

	dma_addr_t iova;
	unsigned long size;
	void *vaddr;

	struct kref ref;
	/*
	 * References handed to userspace by MPP_CMD_TRANS_FD_TO_IOVA, guarded
	 * by dma->list_mutex. MPP_CMD_RELEASE_FD may only give back what it
	 * took: without this it is an unauthenticated kref_put, and repeating
	 * one fd in its array drops references belonging to in-flight tasks.
	 */
	u32 static_cnt;
	ktime_t last_used;
	/* alloc by device */
	struct device *dev;
};

#define MPP_SESSION_MAX_BUFFERS		60

struct mpp_dma_session {
	/* the buffer used in session */
	struct list_head unused_list;
	struct list_head used_list;
	/*
	 * For those buffer import by ioctl MPP_CMD_TRANS_FD_TO_IOVA,
	 * move to static_list instead of used_list and don't increase extra kref,
	 * so that it will release when user space call ioctl MPP_CMD_RELEASE_FD.
	 */
	struct list_head static_list;
	struct mpp_dma_buffer dma_bufs[MPP_SESSION_MAX_BUFFERS];
	/* the mutex for the above buffer list */
	struct mutex list_mutex;
	/* the max buffer num for the buffer list */
	u32 max_buffers;
	/* the count for the buffer list */
	int buffer_count;

	struct device *dev;
};

struct mpp_rk_iommu {
	struct list_head link;
	u32 grf_val;
	int mmu_num;
	u32 base_addr[2];
	void __iomem *bases[2];
	u32 dte_addr;
	u32 is_paged;
};

struct mpp_dev;
struct mpp_iommu_shared_domain;

struct mpp_iommu_info {
	struct rw_semaphore *rw_sem;
	struct rw_semaphore rw_sem_self;
	struct rw_semaphore *default_rw_sem;

	struct device *dev;
	struct platform_device *pdev;
	struct iommu_domain *domain;
	struct iommu_domain *default_domain;
	struct iommu_group *group;
	struct mpp_rk_iommu *iommu;
	iommu_fault_handler_t hdl;
	struct mpp_iommu_shared_domain *shared_domain;
	bool rockchip_fault_handler;
	bool vsi_fault_handler;
	bool generic_fault_handler;

	spinlock_t dev_lock;
	struct mpp_dev *dev_active;
	/* reservation held from task prepare through task deactivate/abort */
	struct mpp_dev *task_admission_owner;

	int irq;
	int got_irq;
	/* flag for mark iommu whether shared */
	bool shared;
	struct mpp_taskqueue *queue;
};

/* fixed RCB/SRAM windows tracked per CCU cluster (one per core + slack) */
#define MPP_IOMMU_MAX_FIXED_WINDOWS	8

/*
 * CCU cluster shared IOMMU domain.
 *
 * A multicore CCU cluster (rkvdec2 or rkvenc2) shares one IOVA address space
 * across all of its hardware cores. That space is the main/service-visible
 * core's default DMA domain -- the domain the DMA API (VB2, dma_buf import,
 * mpp_dma_session_create()) already populates -- so every buffer mapped for
 * the cluster is visible to whichever core a task is dispatched to.
 *
 * Each participating core's IOMMU is attached to that one domain; the Rockchip
 * provider keeps a per-domain IOMMU list (struct rk_iommu_domain::iommus), so a
 * single iommu_map()/iommu_unmap()/flush on the shared domain reaches every
 * attached core.
 *
 * This object makes that ownership explicit and auditable: codec drivers join
 * their secondary cores through it instead of open-coding domain/rw_sem swaps
 * on struct mpp_iommu_info. The encoder and decoder clusters own separate
 * shared domains and must never share one with each other, with RGA, or with
 * the AV1/VSI path.
 */
struct mpp_iommu_shared_domain {
	/* main/service-visible core that owns the shared domain */
	struct mpp_iommu_info *owner;
	/* shared IOVA space == owner->domain (owner's default DMA domain) */
	struct iommu_domain *domain;
	/* cluster-wide map/unmap/reset serialization == owner->rw_sem */
	struct rw_semaphore *rw_sem;
	/*
	 * Fixed RCB/SRAM IOVA windows mapped into this cluster domain. Each core
	 * maps its own SRAM window at a distinct fixed IOVA (Rock 5B decoder:
	 * core 0 0xFFF00000, core 1 0xFFE00000), so tracking them lets us reject
	 * an accidental overlap inside the one shared domain.
	 */
	struct {
		dma_addr_t iova;
		size_t size;
	} windows[MPP_IOMMU_MAX_FIXED_WINDOWS];
	unsigned int nr_windows;
	/* protects fixed-window tracking */
	spinlock_t window_lock;
};

struct mpp_dma_session *
mpp_dma_session_create(struct device *dev, u32 max_buffers);
int mpp_dma_session_destroy(struct mpp_dma_session *dma);

struct mpp_dma_buffer *
mpp_dma_alloc(struct device *dev, size_t size);
int mpp_dma_free(struct mpp_dma_buffer *buffer);

struct mpp_dma_buffer *
mpp_dma_import_fd(struct mpp_iommu_info *iommu_info,
		  struct mpp_dma_session *dma, int fd, int static_use);
int mpp_dma_release(struct mpp_dma_session *dma,
		    struct mpp_dma_buffer *buffer);
int mpp_dma_release_fd(struct mpp_dma_session *dma, int fd);

int mpp_dma_unmap_kernel(struct mpp_dma_session *dma,
			 struct mpp_dma_buffer *buffer);
int mpp_dma_map_kernel(struct mpp_dma_session *dma,
		       struct mpp_dma_buffer *buffer);
struct mpp_dma_buffer *mpp_dma_find_buffer_fd(struct mpp_dma_session *dma, int fd);
void mpp_dma_buf_sync(struct mpp_dma_buffer *buffer, u32 offset, u32 length,
		      enum dma_data_direction dir, bool for_cpu);

struct mpp_iommu_info *
mpp_iommu_probe(struct device *dev);
void mpp_iommu_quiesce_fault_handler(struct mpp_iommu_info *info);
int mpp_iommu_remove(struct mpp_iommu_info *info);

int mpp_iommu_attach(struct mpp_iommu_info *info);
int mpp_iommu_detach(struct mpp_iommu_info *info);

int mpp_iommu_shared_domain_init(struct mpp_iommu_shared_domain *shared,
				 struct mpp_iommu_info *owner);
int mpp_iommu_shared_domain_bind(struct mpp_iommu_shared_domain *shared,
				 struct mpp_iommu_info *info);
int mpp_iommu_shared_domain_unbind(struct mpp_iommu_shared_domain *shared,
				   struct mpp_iommu_info *info);
bool mpp_iommu_shared_domain_verify(struct mpp_iommu_shared_domain *shared,
				    struct mpp_iommu_info *info);
int mpp_iommu_shared_domain_reserve_window(struct mpp_iommu_shared_domain *shared,
					   dma_addr_t iova, size_t size,
					   struct device *dev);
void mpp_iommu_shared_domain_unreserve_window(struct mpp_iommu_shared_domain *shared,
					      dma_addr_t iova, size_t size);

int mpp_iommu_refresh(struct mpp_iommu_info *info, struct device *dev);
int mpp_iommu_flush_tlb(struct mpp_iommu_info *info);

int mpp_iommu_dev_activate(struct mpp_iommu_info *info, struct mpp_dev *dev);
int mpp_iommu_dev_activate_task(struct mpp_iommu_info *info,
				struct mpp_dev *dev);
int mpp_iommu_dev_deactivate(struct mpp_iommu_info *info, struct mpp_dev *dev);
int mpp_iommu_dev_prepare_task(struct mpp_iommu_info *info,
			       struct mpp_dev *dev);
int mpp_iommu_dev_commit_task(struct mpp_iommu_info *info,
			      struct mpp_dev *dev);
int mpp_iommu_dev_abort_task(struct mpp_iommu_info *info,
			     struct mpp_dev *dev);
int mpp_iommu_reserve_iova(struct mpp_iommu_info *info, dma_addr_t iova, size_t size);
void mpp_iommu_unreserve_iova(struct mpp_iommu_info *info, dma_addr_t iova, size_t size);

static inline int mpp_iommu_down_read(struct mpp_iommu_info *info)
{
	if (info)
		down_read(info->rw_sem);

	return 0;
}

static inline int mpp_iommu_up_read(struct mpp_iommu_info *info)
{
	if (info)
		up_read(info->rw_sem);

	return 0;
}

static inline int mpp_iommu_down_write(struct mpp_iommu_info *info)
{
	if (info)
		down_write(info->rw_sem);

	return 0;
}

static inline int mpp_iommu_up_write(struct mpp_iommu_info *info)
{
	if (info)
		up_write(info->rw_sem);

	return 0;
}

static inline void mpp_iommu_enable_irq(struct mpp_iommu_info *info)
{
	if (info && info->got_irq)
		enable_irq(info->irq);
}

static inline void mpp_iommu_disable_irq(struct mpp_iommu_info *info)
{
	if (info && info->got_irq)
		disable_irq(info->irq);
}

#endif
