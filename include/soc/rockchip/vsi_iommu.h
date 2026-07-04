/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_ROCKCHIP_VSI_IOMMU_H
#define __SOC_ROCKCHIP_VSI_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_ENABLED(CONFIG_VSI_IOMMU)
int vsi_iommu_refresh(struct device *dev);
void vsi_iommu_mask_irq(struct device *dev);
int vsi_iommu_set_fault_handler(struct device *dev,
					iommu_fault_handler_t handler, void *token);
#else
static inline int vsi_iommu_refresh(struct device *dev)
{
	return -ENODEV;
}

static inline void vsi_iommu_mask_irq(struct device *dev)
{
}

static inline int vsi_iommu_set_fault_handler(struct device *dev,
				     iommu_fault_handler_t handler, void *token)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_ROCKCHIP_VSI_IOMMU_H */
