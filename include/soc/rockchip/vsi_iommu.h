/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_ROCKCHIP_VSI_IOMMU_H
#define __SOC_ROCKCHIP_VSI_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_REACHABLE(CONFIG_VSI_IOMMU)
/* Re-enables translation and the masked fault source, then flushes the TLB. */
int vsi_iommu_refresh(struct device *dev);
void vsi_iommu_mask_irq(struct device *dev);
/*
 * Atomic-safe; passing NULL unregisters without waiting. Teardown paths that
 * free the token must call vsi_iommu_sync_fault_handler() afterwards.
 */
int vsi_iommu_set_fault_handler(struct device *dev,
				iommu_fault_handler_t handler, void *token);
/* Waits (sleeps) until no in-flight provider IRQ callback can be running. */
int vsi_iommu_sync_fault_handler(struct device *dev);
#else
static inline int vsi_iommu_refresh(struct device *dev)
{
	return -ENODEV;
}

static inline void vsi_iommu_mask_irq(struct device *dev)
{
}

static inline int vsi_iommu_set_fault_handler(struct device *dev,
					      iommu_fault_handler_t handler,
					      void *token)
{
	return -ENODEV;
}

static inline int vsi_iommu_sync_fault_handler(struct device *dev)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_ROCKCHIP_VSI_IOMMU_H */
