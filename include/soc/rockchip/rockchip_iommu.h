/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip IOMMU helpers used by Rockchip media drivers.
 */
#ifndef __SOC_ROCKCHIP_IOMMU_H
#define __SOC_ROCKCHIP_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_ENABLED(CONFIG_ROCKCHIP_IOMMU)
int rockchip_iommu_enable(struct device *dev);
int rockchip_iommu_disable(struct device *dev);
bool rockchip_iommu_is_enabled(struct device *dev);
int rockchip_iommu_force_reset(struct device *dev);
void rockchip_iommu_mask_irq(struct device *dev);
void rockchip_iommu_unmask_irq(struct device *dev);
int rockchip_pagefault_done(struct device *dev);
int rockchip_iommu_set_fault_handler(struct device *dev,
				     iommu_fault_handler_t handler, void *token);
#else
static inline int rockchip_iommu_enable(struct device *dev)
{
	return -ENODEV;
}

static inline int rockchip_iommu_disable(struct device *dev)
{
	return -ENODEV;
}

static inline bool rockchip_iommu_is_enabled(struct device *dev)
{
	return false;
}

static inline int rockchip_iommu_force_reset(struct device *dev)
{
	return -ENODEV;
}

static inline void rockchip_iommu_mask_irq(struct device *dev)
{
}

static inline void rockchip_iommu_unmask_irq(struct device *dev)
{
}

static inline int rockchip_pagefault_done(struct device *dev)
{
	return -ENODEV;
}

static inline int rockchip_iommu_set_fault_handler(struct device *dev,
					  iommu_fault_handler_t handler,
					  void *token)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_ROCKCHIP_IOMMU_H */
