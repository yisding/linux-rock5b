/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_ROCKCHIP_VSI_IOMMU_H
#define __SOC_ROCKCHIP_VSI_IOMMU_H

#include <linux/errno.h>
#include <linux/iommu.h>

struct device;

#if IS_REACHABLE(CONFIG_VSI_IOMMU)
/* Re-enables translation and the masked fault source, then flushes the TLB. */
int vsi_iommu_refresh(struct device *dev);
/*
 * Check for a provider fault before publishing a new DMA generation.
 * Returns -EBUSY while a captured fault still owns refresh. May invoke the
 * installed fault handler synchronously before returning.
 */
int vsi_iommu_prepare_dma(struct device *dev);
/*
 * Serialize the final provider fault check with one DMA doorbell.  A
 * successful reserve holds admission ownership, provider IRQ disablement,
 * and a runtime-PM reference. The same task must call release_dma() exactly
 * once; release may invoke the installed fault handler synchronously after
 * dropping admission ownership.
 */
int vsi_iommu_reserve_dma(struct device *dev);
void vsi_iommu_release_dma(struct device *dev);
/* Clear a retired fault reservation without re-enabling the fault source. */
void vsi_iommu_clear_fault(struct device *dev);
void vsi_iommu_mask_irq(struct device *dev);
/*
 * Installing a handler may synchronously replay a captured fault. Unregister
 * with set_fault_handler(NULL), then call sync_fault_handler() to prevent new
 * callbacks and wait (sleeping) for callbacks from every delivery path.
 */
int vsi_iommu_set_fault_handler(struct device *dev,
				iommu_fault_handler_t handler, void *token);
int vsi_iommu_sync_fault_handler(struct device *dev);
#else
static inline int vsi_iommu_refresh(struct device *dev)
{
	return -ENODEV;
}

static inline int vsi_iommu_prepare_dma(struct device *dev)
{
	return -ENODEV;
}

static inline int vsi_iommu_reserve_dma(struct device *dev)
{
	return -ENODEV;
}

static inline void vsi_iommu_release_dma(struct device *dev)
{
}

static inline void vsi_iommu_clear_fault(struct device *dev)
{
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
