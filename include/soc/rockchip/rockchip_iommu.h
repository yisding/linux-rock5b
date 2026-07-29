/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_ROCKCHIP_IOMMU_H
#define __SOC_ROCKCHIP_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_REACHABLE(CONFIG_ROCKCHIP_IOMMU)
/*
 * Atomic-safe; passing NULL unregisters without waiting. Teardown paths that
 * free the token must call rockchip_iommu_sync_fault_handler() afterwards.
 */
int rockchip_iommu_set_fault_handler(struct device *dev,
				     iommu_fault_handler_t handler, void *token);
/* Waits (sleeps) until no in-flight provider IRQ callback can be running. */
int rockchip_iommu_sync_fault_handler(struct device *dev);
#else
static inline int rockchip_iommu_set_fault_handler(struct device *dev,
						   iommu_fault_handler_t handler,
						   void *token)
{
	return -ENODEV;
}

static inline int rockchip_iommu_sync_fault_handler(struct device *dev)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_ROCKCHIP_IOMMU_H */
