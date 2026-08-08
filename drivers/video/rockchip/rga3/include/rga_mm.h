/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author:
 *  Cerf Yu <cerf.yu@rock-chips.com>
 */

#ifndef __LINUX_RKRGA_MM_H_
#define __LINUX_RKRGA_MM_H_

#include "rga_drv.h"

struct seq_file;

enum rga_mm_flag {
	/* It will identify whether the buffer is within 0 ~ 4G. */
	RGA_MEM_UNDER_4G		= 1 << 0,
	/* Logo enable IOMMU */
	RGA_MEM_NEED_USE_IOMMU		= 1 << 1,
	/* Flag this is a physical contiguous memory. */
	RGA_MEM_PHYSICAL_CONTIGUOUS	= 1 << 2,
	/* need force flush cache */
	RGA_MEM_FORCE_FLUSH_CACHE	= 1 << 3,
};

enum rga2_buffer_support {
	RGA2_BUFFER_UNSUPPORTED = 0,
	RGA2_BUFFER_DIRECT = 1,
	RGA2_BUFFER_STAGEABLE = 2,
};

enum rga2_stage_counter {
	RGA2_STAGE_ATTEMPT,
	RGA2_STAGE_SUCCESS,
	RGA2_STAGE_FAILURE,
	RGA2_STAGE_REUSE,
	RGA2_STAGE_ACTIVE,
	RGA2_STAGE_ACTIVE_BYTES,
	RGA2_STAGE_PEAK_BYTES,
	RGA2_STAGE_COPY_IN_BYTES,
	RGA2_STAGE_COPY_OUT_BYTES,
};

struct rga_mm {
	struct mutex lock;

	/*
	 * @memory_idr:
	 *
	 * Mapping of memory object handles to object pointers. Used by the GEM
	 * subsystem. Protected by @memory_lock.
	 */
	struct idr memory_idr;

	/* the count of buffer in the cached_list */
	int buffer_count;
};

static inline bool rga_mm_is_invalid_dma_buffer(struct rga_dma_buffer *buffer)
{
	if (buffer == NULL)
		return true;

	return buffer->map_dev == NULL ? true : false;
}

struct rga_internal_buffer *rga_mm_lookup_handle(struct rga_mm *mm_session, uint32_t handle);
int rga_mm_lookup_flag(struct rga_mm *mm_session, uint64_t handle);
int rga_mm_lookup_rga2_support(struct rga_mm *mm_session, uint64_t handle);
dma_addr_t rga_mm_lookup_iova(struct rga_internal_buffer *buffer);
struct sg_table *rga_mm_lookup_sgt(struct rga_internal_buffer *buffer);

void rga_mm_dump_buffer(struct rga_internal_buffer *dump_buffer);
void rga_mm_dump_info(struct rga_mm *session);
void rga_mm_rga2_stage_show(struct seq_file *m);
u64 rga_mm_rga2_stage_counter(enum rga2_stage_counter counter);

int rga_mm_map_job_info(struct rga_job *job);
void rga_mm_unmap_job_info(struct rga_job *job);

int rga_mm_import_buffer(struct rga_external_buffer *external_buffer,
			 struct rga_session *session);
int rga_mm_release_buffer(uint32_t handle, struct rga_session *session);
int rga_mm_session_release_buffer(struct rga_session *session);

int rga_mm_init(struct rga_mm **session);
int rga_mm_remove(struct rga_mm **session);

#endif
