/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_ROCKCHIP_IOMMU_H
#define __SOC_ROCKCHIP_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_REACHABLE(CONFIG_ROCKCHIP_IOMMU)
/* Passing NULL unregisters and waits for in-flight provider IRQ callbacks. */
int rockchip_iommu_set_fault_handler(struct device *dev,
				     iommu_fault_handler_t handler, void *token);
#else
static inline int rockchip_iommu_set_fault_handler(struct device *dev,
						   iommu_fault_handler_t handler,
						   void *token)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_ROCKCHIP_IOMMU_H */
