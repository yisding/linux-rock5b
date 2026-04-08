// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Rockchip Electronics Co., Ltd.
 *
 * Author: Huang Lee <Putin.li@rock-chips.com>
 */

#include "rga_iommu.h"
#include "rga_dma_buf.h"
#include "rga_mm.h"
#include "rga_job.h"
#include "rga_common.h"
#include "rga_hw_config.h"

#include <soc/rockchip/rockchip_iommu.h>

int rga_user_memory_check(struct page **pages, u32 w, u32 h, u32 format, int flag)
{
	int bits;
	void *vaddr = NULL;
	int taipage_num;
	int taidata_num;
	int *tai_vaddr = NULL;

	bits = rga_get_format_bits(format);
	if (bits < 0)
		return -1;

	taipage_num = w * h * bits / 8 / (1024 * 4);
	taidata_num = w * h * bits / 8 % (1024 * 4);
	if (taidata_num == 0) {
		vaddr = kmap(pages[taipage_num - 1]);
		tai_vaddr = (int *)vaddr + 1023;
	} else {
		vaddr = kmap(pages[taipage_num]);
		tai_vaddr = (int *)vaddr + taidata_num / 4 - 1;
	}

	if (flag == 1) {
		rga_log("src user memory check\n");
		rga_log("tai data is %d\n", *tai_vaddr);
	} else {
		rga_log("dst user memory check\n");
		rga_log("tai data is %d\n", *tai_vaddr);
	}

	if (taidata_num == 0)
		kunmap(pages[taipage_num - 1]);
	else
		kunmap(pages[taipage_num]);

	return 0;
}

int rga_set_mmu_base(struct rga_job *job,
		     struct rga_job_task_buffers *task_buffers,
		     struct rga2_req *req)
{
	if (task_buffers->src_buffer.page_table) {
		rga_dma_sync_flush_range(task_buffers->src_buffer.page_table,
					 (task_buffers->src_buffer.page_table +
					  task_buffers->src_buffer.page_count),
					 job->scheduler);
		req->mmu_info.src0_base_addr = virt_to_phys(task_buffers->src_buffer.page_table);
	}

	if (task_buffers->src1_buffer.page_table) {
		rga_dma_sync_flush_range(task_buffers->src1_buffer.page_table,
					 (task_buffers->src1_buffer.page_table +
					  task_buffers->src1_buffer.page_count),
					 job->scheduler);
		req->mmu_info.src1_base_addr = virt_to_phys(task_buffers->src1_buffer.page_table);
	}

	if (task_buffers->dst_buffer.page_table) {
		rga_dma_sync_flush_range(task_buffers->dst_buffer.page_table,
					 (task_buffers->dst_buffer.page_table +
					  task_buffers->dst_buffer.page_count),
					 job->scheduler);
		req->mmu_info.dst_base_addr = virt_to_phys(task_buffers->dst_buffer.page_table);

		if (((req->alpha_rop_flag & 1) == 1) && (req->bitblt_mode == 0)) {
			req->mmu_info.src1_base_addr = req->mmu_info.dst_base_addr;
			req->mmu_info.src1_mmu_flag = req->mmu_info.dst_mmu_flag;
		}
	}

	if (task_buffers->els_buffer.page_table) {
		rga_dma_sync_flush_range(task_buffers->els_buffer.page_table,
					 (task_buffers->els_buffer.page_table +
					  task_buffers->els_buffer.page_count),
					 job->scheduler);
		req->mmu_info.els_base_addr = virt_to_phys(task_buffers->els_buffer.page_table);
	}

	return 0;
}

static int rga_mmu_buf_get_try(struct rga_mmu_base *t, uint32_t size)
{
	int ret = 0;

	if ((t->back - t->front) > t->size) {
		if (t->front + size > t->back - t->size) {
			rga_log("front %d, back %d dsize %d size %d",
				t->front, t->back, t->size, size);
			ret = -ENOMEM;
			goto out;
		}
	} else {
		if ((t->front + size) > t->back) {
			rga_log("front %d, back %d dsize %d size %d",
				t->front, t->back, t->size, size);
			ret = -ENOMEM;
			goto out;
		}

		if (t->front + size > t->size) {
			if (size > (t->back - t->size)) {
				rga_log("front %d, back %d dsize %d size %d",
					t->front, t->back, t->size, size);
				ret = -ENOMEM;
				goto out;
			}
			t->front = 0;
		}
	}
out:
	return ret;
}

unsigned int *rga_mmu_buf_get(struct rga_mmu_base *mmu_base, uint32_t size)
{
	int ret;
	unsigned int *buf = NULL;

	WARN_ON(!mutex_is_locked(&rga_drvdata->lock));

	size = ALIGN(size, 16);

	ret = rga_mmu_buf_get_try(mmu_base, size);
	if (ret < 0) {
		rga_err("Get MMU mem failed\n");
		return NULL;
	}

	buf = mmu_base->buf_virtual + mmu_base->front;

	mmu_base->front += size;

	if (mmu_base->back + size > 2 * mmu_base->size)
		mmu_base->back = size + mmu_base->size;
	else
		mmu_base->back += size;

	return buf;
}

struct rga_mmu_base *rga_mmu_base_init(size_t size)
{
	int order = 0;
	struct rga_mmu_base *mmu_base;

	mmu_base = kzalloc(sizeof(*mmu_base), GFP_KERNEL);
	if (mmu_base == NULL) {
		pr_err("Cannot alloc mmu_base!\n");
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * malloc pre scale mid buf mmu table:
	 * size * channel_num * address_size
	 */
	mmu_base->buf_virtual = (uint32_t *)rga_get_free_pages(GFP_KERNEL | GFP_DMA32,
		&order, size * 3 * sizeof(*mmu_base->buf_virtual));
	if (mmu_base->buf_virtual == NULL) {
		pr_err("Can not alloc pages for mmu_page_table\n");
		goto err_free_mmu_base;
	}
	mmu_base->buf_order = order;

	mmu_base->pages = (struct page **)rga_get_free_pages(GFP_KERNEL | GFP_DMA32,
		&order, size * sizeof(*mmu_base->pages));
	if (mmu_base->pages == NULL) {
		pr_err("Can not alloc pages for mmu_base->pages\n");
		goto err_free_buf_virtual;
	}
	mmu_base->pages_order = order;

	mmu_base->front = 0;
	mmu_base->back = RGA2_PHY_PAGE_SIZE * 3;
	mmu_base->size = RGA2_PHY_PAGE_SIZE * 3;

	return mmu_base;

err_free_buf_virtual:
	free_pages((unsigned long)mmu_base->buf_virtual, mmu_base->buf_order);
	mmu_base->buf_order = 0;

err_free_mmu_base:
	kfree(mmu_base);

	return ERR_PTR(-ENOMEM);
}

void rga_mmu_base_free(struct rga_mmu_base **mmu_base)
{
	struct rga_mmu_base *base = *mmu_base;

	if (base->buf_virtual != NULL) {
		free_pages((unsigned long)base->buf_virtual, base->buf_order);
		base->buf_virtual = NULL;
		base->buf_order = 0;
	}

	if (base->pages != NULL) {
		free_pages((unsigned long)base->pages, base->pages_order);
		base->pages = NULL;
		base->pages_order = 0;
	}

	kfree(base);
	*mmu_base = NULL;
}

static int rga_iommu_intr_fault_handler(struct iommu_domain *iommu, struct device *iommu_dev,
					unsigned long iova, int status, void *arg)
{
	struct rga_scheduler_t *scheduler = (struct rga_scheduler_t *)arg;
	struct rga_job *job = scheduler->running_job;

	if (job == NULL)
		return 0;

	rga_err("IOMMU intr fault, IOVA[0x%lx], STATUS[0x%x]\n", iova, status);
	if (scheduler->ops->irq)
		scheduler->ops->irq(scheduler);

	/* iommu interrupts on rga2 do not affect rga2 itself. */
	if (!test_bit(RGA_JOB_STATE_INTR_ERR, &job->state)) {
		set_bit(RGA_JOB_STATE_INTR_ERR, &job->state);
		scheduler->ops->soft_reset(scheduler);
	}

	if (status & (ROCKCHIP_IOMMU_FAULT_BUS_ERROR | RGA_IOMMU_IRQ_BUS_ERROR)) {
		rga_err("RGA IOMMU: bus error! Please check if the memory is invalid or has been freed.\n");
		job->ret = -EACCES;
	} else if (status == IOMMU_FAULT_WRITE) {
		rga_err("RGA IOMMU: write fault! Please check the memory size.\n");
		job->ret = -EACCES;
	} else {
		rga_err("RGA IOMMU: read fault! Please check the memory size.\n");
		job->ret = -EACCES;
	}

	return 0;
}

static int rga_iommu_set_fault_handler(struct rga_scheduler_t *scheduler)
{
	struct rga_iommu_info *info = scheduler->iommu_info;
	int ret;

	if (!info)
		return 0;

	ret = rockchip_iommu_set_fault_handler(info->dev,
					       rga_iommu_intr_fault_handler,
					       scheduler);
	if (!ret) {
		info->rockchip_fault_handler = true;
		return 0;
	}
	if (ret != -ENODEV)
		return ret;

	if (info->domain &&
	    info->domain->cookie_type == IOMMU_COOKIE_FAULT_HANDLER &&
	    info->domain->handler == rga_iommu_intr_fault_handler)
		return 0;

	if (info->domain &&
	    info->domain->cookie_type == IOMMU_COOKIE_NONE) {
		iommu_set_fault_handler(info->domain,
					rga_iommu_intr_fault_handler,
					scheduler);
		info->generic_fault_handler = true;
		return 0;
	}

	dev_err(info->dev, "failed to install RGA IOMMU fault handler\n");

	return ret;
}

static void rga_iommu_clear_fault_handler(struct rga_iommu_info *info)
{
	if (!info)
		return;

	if (info->rockchip_fault_handler) {
		rockchip_iommu_set_fault_handler(info->dev, NULL, NULL);
		info->rockchip_fault_handler = false;
	}

	if (info->generic_fault_handler && info->domain &&
	    info->domain->cookie_type == IOMMU_COOKIE_FAULT_HANDLER &&
	    info->domain->handler == rga_iommu_intr_fault_handler) {
		info->domain->handler = NULL;
		info->domain->handler_token = NULL;
		info->domain->cookie_type = IOMMU_COOKIE_NONE;
		info->generic_fault_handler = false;
	}
}

int rga_iommu_detach(struct rga_iommu_info *info)
{
	if (!info)
		return 0;

	iommu_detach_group(info->domain, info->group);
	return 0;
}

int rga_iommu_attach(struct rga_iommu_info *info)
{
	if (!info)
		return 0;

	return iommu_attach_group(info->domain, info->group);
}

static void rga_iommu_unbind_shared_domain(struct rga_iommu_info *info)
{
	struct iommu_domain *cur;
	struct iommu_domain *shared_domain;
	struct device *shared_default_dev;
	bool was_shared;
	int ret;

	if (!info || !info->shared_domain)
		return;

	shared_domain = info->domain;
	shared_default_dev = info->default_dev;
	was_shared = info->shared_domain;

	rga_iommu_detach(info);
	info->domain = info->default_domain;
	info->default_dev = info->dev;
	info->shared_domain = false;

	cur = iommu_get_domain_for_dev(info->dev);
	if (cur == info->domain)
		return;

	ret = rga_iommu_attach(info);
	if (ret) {
		dev_err(info->dev, "failed to restore default RGA IOMMU domain: %d\n",
			ret);
		info->domain = shared_domain;
		info->default_dev = shared_default_dev;
		info->shared_domain = was_shared;
		ret = rga_iommu_attach(info);
		if (ret)
			dev_err(info->dev, "failed to reattach shared RGA IOMMU domain: %d\n",
				ret);
	}
}

struct rga_iommu_info *rga_iommu_probe(struct device *dev)
{
	int ret = 0;
	struct rga_iommu_info *info = NULL;
	struct iommu_domain *domain = NULL;
	struct iommu_group *group = NULL;

	group = iommu_group_get(dev);
	if (!group)
		return ERR_PTR(-EINVAL);

	domain = iommu_get_domain_for_dev(dev);
	if (!domain) {
		ret = -EINVAL;
		goto err_put_group;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_put_group;
	}

	info->dev = dev;
	info->default_dev = info->dev;
	info->group = group;
	info->domain = domain;
	info->default_domain = domain;

	return info;

err_put_group:
	if (group)
		iommu_group_put(group);

	return ERR_PTR(ret);
}

int rga_iommu_remove(struct rga_iommu_info *info)
{
	if (!info)
		return 0;

	iommu_group_put(info->group);

	return 0;
}

int rga_iommu_bind(void)
{
	int i;
	int ret;
	struct rga_scheduler_t *scheduler = NULL;
	struct rga_iommu_info *main_iommu = NULL;
	int main_iommu_index = -1;
	int main_mmu_index = -1;
	int another_index = -1;

	for (i = 0; i < rga_drvdata->num_of_scheduler; i++) {
		scheduler = rga_drvdata->scheduler[i];

		switch (scheduler->data->mmu) {
		case RGA_IOMMU:
			if (scheduler->iommu_info == NULL)
				continue;

			if (main_iommu == NULL) {
				main_iommu = scheduler->iommu_info;
				main_iommu_index = i;
				ret = rga_iommu_set_fault_handler(scheduler);
				if (ret)
					goto err_unbind;
			} else {
				struct rga_iommu_info *info = scheduler->iommu_info;
				struct device *old_default_dev = info->default_dev;
				struct iommu_domain *old_domain = info->domain;

				info->domain = main_iommu->domain;
				info->default_dev = main_iommu->default_dev;
				ret = rga_iommu_attach(info);
				if (ret) {
					dev_err(scheduler->dev,
						"failed to attach shared RGA IOMMU domain: %d\n",
						ret);
					info->domain = old_domain;
					info->default_dev = old_default_dev;
					goto err_unbind;
				}
				info->shared_domain = true;

				ret = rga_iommu_set_fault_handler(scheduler);
				if (ret)
					goto err_unbind;
			}

			break;

		case RGA_MMU:
			if (rga_drvdata->mmu_base != NULL)
				continue;

			rga_drvdata->mmu_base = rga_mmu_base_init(RGA2_PHY_PAGE_SIZE);
			if (IS_ERR(rga_drvdata->mmu_base)) {
				dev_err(scheduler->dev, "rga mmu base init failed!\n");
				ret = PTR_ERR(rga_drvdata->mmu_base);
				rga_drvdata->mmu_base = NULL;

				goto err_unbind;
			}

			main_mmu_index = i;

			break;
		default:
			if (another_index != RGA_NONE_CORE)
				another_index = i;

			break;
		}
	}

	/*
	 * priority order: iommu > mmu > another
	 *   The scheduler core with IOMMU will be used preferentially as the
	 * default memory-mapped core. This ensures that all cores can obtain
	 * the required memory data when they are equipped with different
	 * versions of cores.
	 */
	if (main_iommu_index >= 0) {
		rga_drvdata->map_scheduler_index = main_iommu_index;
	} else if (main_mmu_index >= 0) {
		rga_drvdata->map_scheduler_index = main_mmu_index;
	} else if (another_index >= 0) {
		rga_drvdata->map_scheduler_index = another_index;
	} else {
		rga_drvdata->map_scheduler_index = -1;
		pr_err("%s, binding map scheduler failed!\n", __func__);
		ret = -EFAULT;
		goto err_unbind;
	}

	pr_info("IOMMU binding successfully, default mapping core[0x%x]\n",
		rga_drvdata->scheduler[rga_drvdata->map_scheduler_index]->core);

	return 0;

err_unbind:
	rga_iommu_unbind();

	return ret;
}

void rga_iommu_unbind(void)
{
	int i;

	for (i = 0; i < rga_drvdata->num_of_scheduler; i++)
		if (rga_drvdata->scheduler[i]->iommu_info) {
			rga_iommu_clear_fault_handler(rga_drvdata->scheduler[i]->iommu_info);
			rga_iommu_unbind_shared_domain(rga_drvdata->scheduler[i]->iommu_info);
		}

	if (rga_drvdata->mmu_base)
		rga_mmu_base_free(&rga_drvdata->mmu_base);

	rga_drvdata->map_scheduler_index = -1;
}
