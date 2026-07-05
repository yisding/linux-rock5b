// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2025 Collabora Ltd.
 *
 * IOMMU API for Verisilicon
 *
 * Module Authors:	Yandong Lin <yandong.lin@rock-chips.com>
 *			Simon Xue <xxm@rock-chips.com>
 *			Benjamin Gaignard <benjamin.gaignard@collabora.com>
 *
 * This hardware block is using a 2 pages tables allocation structure.
 * That make very similar to Rockhip iommu hardware blocks but it has
 * it own driver because the registers offset and configuration bits
 * are completely different. An additional reason is that this hardware
 * has been developed by Verisilicon to be used by their hardware video
 * decoders and not for a general purpose like Rockchip iommus.
 */

#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <soc/rockchip/vsi_iommu.h>

#include "iommu-pages.h"

struct vsi_iommu {
	struct device *dev;
	void __iomem *regs;
	struct clk_bulk_data *clocks;
	int num_clocks;
	struct iommu_device iommu;
	struct list_head node; /* entry in vsi_iommu_domain.iommus */
	struct iommu_domain *domain; /* domain to which iommu is attached */
	spinlock_t lock; /* lock to protect vsi_iommu fields */
	spinlock_t fault_lock; /* lock to protect fault-handler fields */
	iommu_fault_handler_t fault_handler;
	void *fault_handler_token;
	int irq;
	bool enable;
};

struct vsi_iommu_domain {
	struct list_head iommus;
	struct device *dev;
	u32 *dt;
	dma_addr_t dt_dma;
	struct iommu_domain domain;
	u64 *pta;
	dma_addr_t pta_dma;
	spinlock_t lock; /* lock to protect vsi_iommu_domain fields */
};

static struct iommu_domain vsi_identity_domain;

static const struct iommu_ops vsi_iommu_ops;

#define NUM_DT_ENTRIES	1024
#define NUM_PT_ENTRIES	1024

#define SPAGE_SIZE	BIT(12)

/* vsi iommu regs address */
#define VSI_MMU_CONFIG1_BASE			0x1ac
#define VSI_MMU_AHB_EXCEPTION_BASE		0x380
#define VSI_MMU_AHB_CONTROL_BASE		0x388
#define VSI_MMU_AHB_TLB_ARRAY_BASE_L_BASE	0x38C

/* MMU register offsets */
#define VSI_MMU_FLUSH_BASE		0x184
#define VSI_MMU_BIT_FLUSH		BIT(4)

#define VSI_MMU_PAGE_FAULT_ADDR		0x380
#define VSI_MMU_STATUS_BASE		0x384	/* IRQ status */

#define VSI_MMU_BIT_ENABLE		BIT(0)

#define VSI_MMU_OUT_OF_BOUND		BIT(28)
/* Irq mask */
#define VSI_MMU_IRQ_MASK		0x7

#define VSI_DTE_PT_ADDRESS_MASK		0xffffffc0
#define VSI_DTE_PT_VALID		BIT(0)

#define VSI_PAGE_DESC_LO_MASK		0xfffff000
#define VSI_PAGE_DESC_HI_MASK		GENMASK_ULL(39, 32)
#define VSI_PAGE_DESC_HI_SHIFT		(32 - 4)

static inline phys_addr_t vsi_dte_pt_address(u32 dte)
{
	return (phys_addr_t)dte & VSI_DTE_PT_ADDRESS_MASK;
}

static inline u32 vsi_mk_dte(u32 dte)
{
	return (phys_addr_t)dte | VSI_DTE_PT_VALID;
}

#define VSI_PTE_PAGE_WRITABLE		BIT(2)
#define VSI_PTE_PAGE_VALID		BIT(0)

static inline phys_addr_t vsi_pte_page_address(u64 pte)
{
	return ((pte << VSI_PAGE_DESC_HI_SHIFT) & VSI_PAGE_DESC_HI_MASK) |
	       (pte & VSI_PAGE_DESC_LO_MASK);
}

static u32 vsi_mk_pte(phys_addr_t page, int prot)
{
	u32 flags = 0;

	flags |= (prot & IOMMU_WRITE) ? VSI_PTE_PAGE_WRITABLE : 0;

	page = (page & VSI_PAGE_DESC_LO_MASK) |
	       ((page & VSI_PAGE_DESC_HI_MASK) >> VSI_PAGE_DESC_HI_SHIFT);

	return page | flags | VSI_PTE_PAGE_VALID;
}

#define VSI_DTE_PT_VALID	BIT(0)

static inline bool vsi_dte_is_pt_valid(u32 dte)
{
	return dte & VSI_DTE_PT_VALID;
}

static inline bool vsi_pte_is_page_valid(u32 pte)
{
	return pte & VSI_PTE_PAGE_VALID;
}

static u32 vsi_mk_pte_invalid(u32 pte)
{
	return pte & ~VSI_PTE_PAGE_VALID;
}

#define VSI_MASTER_TLB_MASK	GENMASK_ULL(31, 10)
/* mode 0 : 4k */
#define VSI_PTA_4K_MODE	0

static u64 vsi_mk_pta(dma_addr_t dt_dma)
{
	u64 val = (dt_dma & VSI_MASTER_TLB_MASK) | VSI_PTA_4K_MODE;

	return val;
}

static struct vsi_iommu_domain *to_vsi_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct vsi_iommu_domain, domain);
}

static inline void vsi_table_flush(struct vsi_iommu_domain *vsi_domain, dma_addr_t dma,
				   unsigned int count)
{
	size_t size = count * sizeof(u32); /* count of u32 entry */

	dma_sync_single_for_device(vsi_domain->dev, dma, size, DMA_TO_DEVICE);
}

#define VSI_IOVA_DTE_MASK	0xffc00000
#define VSI_IOVA_DTE_SHIFT	22
#define VSI_IOVA_PTE_MASK	0x003ff000
#define VSI_IOVA_PTE_SHIFT	12
#define VSI_IOVA_PAGE_MASK	0x00000fff
#define VSI_IOVA_PAGE_SHIFT	0

static u32 vsi_iova_dte_index(u32 iova)
{
	return (iova & VSI_IOVA_DTE_MASK) >> VSI_IOVA_DTE_SHIFT;
}

static u32 vsi_iova_pte_index(u32 iova)
{
	return (iova & VSI_IOVA_PTE_MASK) >> VSI_IOVA_PTE_SHIFT;
}

static u32 vsi_iova_page_offset(u32 iova)
{
	return (iova & VSI_IOVA_PAGE_MASK) >> VSI_IOVA_PAGE_SHIFT;
}

static int vsi_iommu_call_fault_handler(struct vsi_iommu *iommu,
					struct iommu_domain *domain,
					dma_addr_t iova, int flags)
{
	iommu_fault_handler_t handler;
	unsigned long irq_flags;
	void *token;

	spin_lock_irqsave(&iommu->fault_lock, irq_flags);
	handler = iommu->fault_handler;
	token = iommu->fault_handler_token;
	spin_unlock_irqrestore(&iommu->fault_lock, irq_flags);

	if (!handler || !domain || domain == &vsi_identity_domain)
		return -EOPNOTSUPP;

	return handler(domain, iommu->dev, iova, flags, token);
}

static void vsi_iommu_mask_irq_locked(struct vsi_iommu *iommu)
{
	writel(0, iommu->regs + VSI_MMU_AHB_EXCEPTION_BASE);
}

static int vsi_iommu_fault_flags(u32 status)
{
	/* The VSI status bits are provider-specific; do not pass them as IOMMU flags. */
	return IOMMU_FAULT_READ;
}

static irqreturn_t vsi_iommu_irq(int irq, void *dev_id)
{
	struct vsi_iommu *iommu = dev_id;
	struct iommu_domain *domain = NULL;
	unsigned long flags;
	dma_addr_t iova = 0;
	u32 status;
	bool fault = false;
	int ret;

	ret = pm_runtime_get_if_in_use(iommu->dev);
	if (ret <= 0)
		return IRQ_NONE;

	spin_lock_irqsave(&iommu->lock, flags);

	status = readl(iommu->regs + VSI_MMU_STATUS_BASE);
	if (status & VSI_MMU_IRQ_MASK) {
		dev_err(iommu->dev, "unexpected int_status=%08x\n", status);
		iova = readl(iommu->regs + VSI_MMU_PAGE_FAULT_ADDR);
		domain = iommu->domain;
		fault = true;
		vsi_iommu_mask_irq_locked(iommu);
	}
	writel(0, iommu->regs + VSI_MMU_STATUS_BASE);

	spin_unlock_irqrestore(&iommu->lock, flags);

	if (fault) {
		int fault_flags = vsi_iommu_fault_flags(status);
		int fault_ret;

		fault_ret = vsi_iommu_call_fault_handler(iommu, domain, iova, fault_flags);
		if (fault_ret) {
			if (domain && domain != &vsi_identity_domain)
				report_iommu_fault(domain, iommu->dev, iova, fault_flags);
			else
				dev_err(iommu->dev, "fault without active paging domain\n");
		}
	}
	pm_runtime_put_autosuspend(iommu->dev);

	return IRQ_HANDLED;
}

static struct vsi_iommu *vsi_iommu_get_from_dev(struct device *dev,
					       struct device **provider_dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct device *iommu_dev;
	struct vsi_iommu *iommu;

	if (!fwspec || !fwspec->iommu_fwnode)
		return ERR_PTR(-ENODEV);

	iommu_dev = bus_find_device_by_fwnode(&platform_bus_type,
						       fwspec->iommu_fwnode);
	if (!iommu_dev)
		return ERR_PTR(-EPROBE_DEFER);

	iommu = dev_get_drvdata(iommu_dev);
	if (!iommu) {
		put_device(iommu_dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	*provider_dev = iommu_dev;
	return iommu;
}

static struct iommu_domain *vsi_iommu_domain_alloc_paging(struct device *dev)
{
	struct vsi_iommu *iommu = dev_iommu_priv_get(dev);
	struct vsi_iommu_domain *vsi_domain;

	vsi_domain = kzalloc(sizeof(*vsi_domain), GFP_KERNEL);
	if (!vsi_domain)
		return NULL;

	vsi_domain->dev = iommu->dev;
	spin_lock_init(&vsi_domain->lock);

	/*
	 * iommu use a 2 level pagetable.
	 * Each level1 (dt) and level2 (pt) table has 1024 4-byte entries.
	 * Allocate one 4 KiB page for each table.
	 */
	vsi_domain->dt = iommu_alloc_pages_sz(GFP_KERNEL | GFP_DMA32,
					      SPAGE_SIZE);
	if (!vsi_domain->dt)
		goto err_free_domain;

	vsi_domain->dt_dma = dma_map_single(vsi_domain->dev, vsi_domain->dt,
					    SPAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(vsi_domain->dev, vsi_domain->dt_dma)) {
		dev_err(dev, "DMA map error for DT\n");
		goto err_free_dt;
	}

	vsi_domain->pta = iommu_alloc_pages_sz(GFP_KERNEL | GFP_DMA32,
					       SPAGE_SIZE);
	if (!vsi_domain->pta)
		goto err_unmap_dt;

	vsi_domain->pta[0] = vsi_mk_pta(vsi_domain->dt_dma);
	vsi_domain->pta_dma = dma_map_single(vsi_domain->dev, vsi_domain->pta,
					     SPAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(vsi_domain->dev, vsi_domain->pta_dma)) {
		dev_err(dev, "DMA map error for PTA\n");
		goto err_free_pta;
	}

	INIT_LIST_HEAD(&vsi_domain->iommus);

	vsi_domain->domain.geometry.aperture_start = 0;
	vsi_domain->domain.geometry.aperture_end   = DMA_BIT_MASK(32);
	vsi_domain->domain.geometry.force_aperture = true;
	vsi_domain->domain.pgsize_bitmap	   = SZ_4K;

	return &vsi_domain->domain;

err_free_pta:
	iommu_free_pages(vsi_domain->pta);
err_unmap_dt:
	dma_unmap_single(vsi_domain->dev, vsi_domain->dt_dma,
			 SPAGE_SIZE, DMA_TO_DEVICE);
err_free_dt:
	iommu_free_pages(vsi_domain->dt);
err_free_domain:
	kfree(vsi_domain);

	return NULL;
}

static phys_addr_t vsi_iommu_iova_to_phys(struct iommu_domain *domain,
					  dma_addr_t iova)
{
	struct vsi_iommu_domain *vsi_domain = to_vsi_domain(domain);
	phys_addr_t pt_phys, phys = 0;
	unsigned long flags;
	u32 dte, pte;
	u32 *page_table;

	spin_lock_irqsave(&vsi_domain->lock, flags);
	dte = vsi_domain->dt[vsi_iova_dte_index(iova)];
	if (!vsi_dte_is_pt_valid(dte))
		goto unlock;

	pt_phys = vsi_dte_pt_address(dte);
	page_table = (u32 *)phys_to_virt(pt_phys);
	pte = page_table[vsi_iova_pte_index(iova)];
	if (!vsi_pte_is_page_valid(pte))
		goto unlock;

	phys = vsi_pte_page_address(pte) + vsi_iova_page_offset(iova);

unlock:
	spin_unlock_irqrestore(&vsi_domain->lock, flags);
	return phys;
}

static size_t vsi_iommu_unmap_iova(struct vsi_iommu_domain *vsi_domain,
				   u32 *pte_addr, dma_addr_t pte_dma,
				   size_t size, unsigned int pte_limit)
{
	unsigned int pte_count;
	unsigned int pte_total = min_t(unsigned int, size / SPAGE_SIZE, pte_limit);

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];

		if (!vsi_pte_is_page_valid(pte))
			break;

		pte_addr[pte_count] = vsi_mk_pte_invalid(pte);
	}

	if (pte_count)
		vsi_table_flush(vsi_domain, pte_dma, pte_count);

	return pte_count * SPAGE_SIZE;
}

static int vsi_iommu_map_iova(struct vsi_iommu_domain *vsi_domain, u32 *pte_addr,
			      dma_addr_t pte_dma, dma_addr_t iova,
			      phys_addr_t paddr, size_t size, int prot,
			      unsigned int pte_limit, size_t *mapped)
{
	unsigned int pte_count;
	unsigned int pte_total = min_t(unsigned int, size / SPAGE_SIZE, pte_limit);
	phys_addr_t page_phys;

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];

		if (vsi_pte_is_page_valid(pte))
			goto unwind;

		pte_addr[pte_count] = vsi_mk_pte(paddr, prot);

		paddr += SPAGE_SIZE;
	}

	if (pte_count)
		vsi_table_flush(vsi_domain, pte_dma, pte_count);

	*mapped = pte_count * SPAGE_SIZE;

	return 0;
unwind:
	vsi_iommu_unmap_iova(vsi_domain, pte_addr, pte_dma,
			     pte_count * SPAGE_SIZE, pte_limit);

	iova += pte_count * SPAGE_SIZE;
	page_phys = vsi_pte_page_address(pte_addr[pte_count]);
	dev_err(vsi_domain->dev,
		"iova %pad already mapped to %pa cannot remap to phys %pa prot %#x\n",
		&iova, &page_phys, &paddr, prot);

	return -EADDRINUSE;
}

static void vsi_iommu_flush_tlb_locked(struct vsi_iommu_domain *vsi_domain)
{
	struct vsi_iommu *iommu;

	list_for_each_entry(iommu, &vsi_domain->iommus, node) {
		int ret = pm_runtime_get_if_active(iommu->dev);

		if (WARN_ON_ONCE(ret < 0))
			continue;
		if (!ret)
			continue;

		spin_lock(&iommu->lock);

		if (iommu->enable) {
			writel(VSI_MMU_BIT_FLUSH, iommu->regs + VSI_MMU_FLUSH_BASE);
			writel(0, iommu->regs + VSI_MMU_FLUSH_BASE);
		}

		spin_unlock(&iommu->lock);

		pm_runtime_put_autosuspend(iommu->dev);
	}
}

static void vsi_iommu_flush_tlb(struct iommu_domain *domain)
{
	struct vsi_iommu_domain *vsi_domain;
	unsigned long flags;

	if (domain == &vsi_identity_domain)
		return;

	vsi_domain = to_vsi_domain(domain);

	spin_lock_irqsave(&vsi_domain->lock, flags);
	vsi_iommu_flush_tlb_locked(vsi_domain);
	spin_unlock_irqrestore(&vsi_domain->lock, flags);
}

static size_t vsi_iommu_unmap(struct iommu_domain *domain, unsigned long _iova,
			      size_t size, size_t count, struct iommu_iotlb_gather *gather)
{
	struct vsi_iommu_domain *vsi_domain = to_vsi_domain(domain);
	dma_addr_t pte_dma, iova = (dma_addr_t)_iova;
	size_t total_size = size * count;
	unsigned long flags;
	phys_addr_t pt_phys;
	u32 dte;
	u32 *pte_addr;
	size_t unmap_size = 0;

	spin_lock_irqsave(&vsi_domain->lock, flags);

	dte = vsi_domain->dt[vsi_iova_dte_index(iova)];
	/* Just return 0 if iova is unmapped */
	if (!vsi_dte_is_pt_valid(dte))
		goto unlock;

	pt_phys = vsi_dte_pt_address(dte);
	pte_addr = (u32 *)phys_to_virt(pt_phys) + vsi_iova_pte_index(iova);
	pte_dma = pt_phys + vsi_iova_pte_index(iova) * sizeof(u32);
	unmap_size = vsi_iommu_unmap_iova(vsi_domain, pte_addr, pte_dma, total_size,
					    NUM_PT_ENTRIES - vsi_iova_pte_index(iova));
	if (!unmap_size)
		goto unlock;

	vsi_iommu_flush_tlb_locked(vsi_domain);
unlock:
	spin_unlock_irqrestore(&vsi_domain->lock, flags);

	return unmap_size;
}

static u32 *vsi_dte_get_page_table(struct vsi_iommu_domain *vsi_domain,
				   dma_addr_t iova, gfp_t gfp)
{
	u32 *page_table, *dte_addr;
	u32 dte_index, dte;
	phys_addr_t pt_phys;
	dma_addr_t pt_dma;
	gfp_t flags;

	dte_index = vsi_iova_dte_index(iova);
	dte_addr = &vsi_domain->dt[dte_index];
	dte = *dte_addr;
	if (vsi_dte_is_pt_valid(dte))
		goto done;

	/* Do not allow to sleep while allocating the buffer */
	flags = (gfp & ~GFP_KERNEL) | GFP_ATOMIC | GFP_DMA32;
	page_table = iommu_alloc_pages_sz(flags, PAGE_SIZE);
	if (!page_table)
		return ERR_PTR(-ENOMEM);

	pt_dma = dma_map_single(vsi_domain->dev, page_table, PAGE_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(vsi_domain->dev, pt_dma)) {
		dev_err(vsi_domain->dev, "DMA mapping error while allocating page table\n");
		iommu_free_pages(page_table);
		return ERR_PTR(-ENOMEM);
	}

	dte = vsi_mk_dte(pt_dma);
	*dte_addr = dte;

	vsi_table_flush(vsi_domain,
			vsi_domain->dt_dma + dte_index * sizeof(u32), 1);
done:
	pt_phys = vsi_dte_pt_address(dte);
	return (u32 *)phys_to_virt(pt_phys);
}

static int vsi_iommu_map(struct iommu_domain *domain, unsigned long _iova,
			 phys_addr_t paddr, size_t size, size_t count,
			 int prot, gfp_t gfp, size_t *mapped)
{
	struct vsi_iommu_domain *vsi_domain = to_vsi_domain(domain);
	dma_addr_t pte_dma, iova = (dma_addr_t)_iova;
	size_t total_size = size * count;
	u32 *page_table, *pte_addr;
	u32 dte, pte_index;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&vsi_domain->lock, flags);

	page_table = vsi_dte_get_page_table(vsi_domain, iova, gfp);
	if (IS_ERR(page_table)) {
		spin_unlock_irqrestore(&vsi_domain->lock, flags);
		return PTR_ERR(page_table);
	}

	dte = vsi_domain->dt[vsi_iova_dte_index(iova)];
	pte_index = vsi_iova_pte_index(iova);
	pte_addr = &page_table[pte_index];
	pte_dma = vsi_dte_pt_address(dte) + pte_index * sizeof(u32);
	ret = vsi_iommu_map_iova(vsi_domain, pte_addr, pte_dma, iova,
				 paddr, total_size, prot, NUM_PT_ENTRIES - pte_index,
				 mapped);

	vsi_iommu_flush_tlb_locked(vsi_domain);

	spin_unlock_irqrestore(&vsi_domain->lock, flags);

	return ret;
}

static void vsi_iommu_disable(struct vsi_iommu *iommu)
{
	writel(0, iommu->regs + VSI_MMU_AHB_CONTROL_BASE);
	iommu->enable = false;
}

static int vsi_iommu_identity_attach(struct iommu_domain *domain,
				     struct device *dev)
{
	struct vsi_iommu *iommu = dev_iommu_priv_get(dev);
	struct iommu_domain *old_domain;
	unsigned long flags;
	int ret;

	old_domain = iommu->domain;
	if (old_domain == domain)
		return 0;

	if (old_domain && old_domain != &vsi_identity_domain) {
		struct vsi_iommu_domain *old_vsi_domain = to_vsi_domain(old_domain);

		spin_lock_irqsave(&old_vsi_domain->lock, flags);
		spin_lock(&iommu->lock);
		if (iommu->domain == old_domain) {
			list_del_init(&iommu->node);
			iommu->domain = domain;
		}
		spin_unlock(&iommu->lock);
		spin_unlock_irqrestore(&old_vsi_domain->lock, flags);
	} else {
		spin_lock_irqsave(&iommu->lock, flags);
		if (iommu->domain != domain)
			iommu->domain = domain;
		spin_unlock_irqrestore(&iommu->lock, flags);
	}

	ret = pm_runtime_get_if_active(iommu->dev);
	if (ret > 0) {
		spin_lock_irqsave(&iommu->lock, flags);
		if (iommu->domain == domain)
			vsi_iommu_disable(iommu);
		spin_unlock_irqrestore(&iommu->lock, flags);

		pm_runtime_put_autosuspend(iommu->dev);
	} else if (ret < 0) {
		dev_dbg(iommu->dev, "identity attach skipped active disable: %d\n", ret);
	}

	return 0;
}

static const struct iommu_domain_ops vsi_identity_ops = {
	.attach_dev = vsi_iommu_identity_attach,
};

static struct iommu_domain vsi_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &vsi_identity_ops,
};

static void vsi_iommu_enable(struct vsi_iommu *iommu, struct iommu_domain *domain)
{
	struct vsi_iommu_domain *vsi_domain;

	if (domain == &vsi_identity_domain)
		return;

	vsi_domain = to_vsi_domain(domain);

	writel(vsi_domain->pta_dma, iommu->regs + VSI_MMU_AHB_TLB_ARRAY_BASE_L_BASE);
	writel(VSI_MMU_OUT_OF_BOUND, iommu->regs + VSI_MMU_CONFIG1_BASE);
	writel(VSI_MMU_BIT_ENABLE, iommu->regs + VSI_MMU_AHB_EXCEPTION_BASE);
	writel(VSI_MMU_BIT_ENABLE, iommu->regs + VSI_MMU_AHB_CONTROL_BASE);
	iommu->enable = true;
}

static int vsi_iommu_attach_device(struct iommu_domain *domain,
				   struct device *dev)
{
	struct vsi_iommu *iommu = dev_iommu_priv_get(dev);
	struct vsi_iommu_domain *vsi_domain = to_vsi_domain(domain);
	struct iommu_domain *old_domain;
	unsigned long flags;
	int ret = 0;

	ret = pm_runtime_resume_and_get(iommu->dev);
	if (ret < 0)
		return ret;

	old_domain = iommu->domain;
	if (old_domain == domain)
		goto out_pm;

	if (old_domain && old_domain != &vsi_identity_domain) {
		struct vsi_iommu_domain *old_vsi_domain = to_vsi_domain(old_domain);

		spin_lock_irqsave(&old_vsi_domain->lock, flags);
		spin_lock(&iommu->lock);
		if (iommu->domain == old_domain) {
			vsi_iommu_disable(iommu);
			list_del_init(&iommu->node);
			iommu->domain = NULL;
		}
		spin_unlock(&iommu->lock);
		spin_unlock_irqrestore(&old_vsi_domain->lock, flags);
	}

	spin_lock_irqsave(&vsi_domain->lock, flags);
	spin_lock(&iommu->lock);

	vsi_iommu_enable(iommu, domain);
	writel(VSI_MMU_BIT_FLUSH, iommu->regs + VSI_MMU_FLUSH_BASE);
	writel(0, iommu->regs + VSI_MMU_FLUSH_BASE);

	list_del_init(&iommu->node);
	list_add_tail(&iommu->node, &vsi_domain->iommus);

	iommu->domain = domain;

	spin_unlock(&iommu->lock);
	spin_unlock_irqrestore(&vsi_domain->lock, flags);

out_pm:
	pm_runtime_put_autosuspend(iommu->dev);
	return ret;
}

static void vsi_iommu_domain_free(struct iommu_domain *domain)
{
	struct vsi_iommu_domain *vsi_domain = to_vsi_domain(domain);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&vsi_domain->lock, flags);

	WARN_ON(!list_empty(&vsi_domain->iommus));

	for (i = 0; i < NUM_DT_ENTRIES; i++) {
		u32 dte = vsi_domain->dt[i];

		if (vsi_dte_is_pt_valid(dte)) {
			phys_addr_t pt_phys = vsi_dte_pt_address(dte);
			u32 *page_table = phys_to_virt(pt_phys);

			dma_unmap_single(vsi_domain->dev, pt_phys,
					 SPAGE_SIZE, DMA_TO_DEVICE);
			iommu_free_pages(page_table);
		}
	}

	dma_unmap_single(vsi_domain->dev, vsi_domain->dt_dma,
			 SPAGE_SIZE, DMA_TO_DEVICE);
	iommu_free_pages(vsi_domain->dt);

	dma_unmap_single(vsi_domain->dev, vsi_domain->pta_dma,
			 SPAGE_SIZE, DMA_TO_DEVICE);
	iommu_free_pages(vsi_domain->pta);

	spin_unlock_irqrestore(&vsi_domain->lock, flags);

	kfree(vsi_domain);
}

static struct iommu_device *vsi_iommu_probe_device(struct device *dev)
{
	struct device *provider_dev;
	struct vsi_iommu *iommu = vsi_iommu_get_from_dev(dev, &provider_dev);
	struct device_link *link;

	if (IS_ERR(iommu))
		return ERR_CAST(iommu);

	link = device_link_add(dev, provider_dev,
			       DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
	if (!link)
		dev_err(dev, "Unable to link %s\n", dev_name(provider_dev));
	put_device(provider_dev);

	/*
	 * MPP dma-buf imports pass a single IOVA/size pair to the hardware.
	 * Allow the DMA API to merge mappings across the full 32-bit aperture.
	 */
	if (!dev->dma_parms)
		dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms),
					      GFP_KERNEL);
	if (!dev->dma_parms) {
		if (link)
			device_link_del(link);
		return ERR_PTR(-ENOMEM);
	}

	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));

	dev_iommu_priv_set(dev, iommu);
	return &iommu->iommu;
}

static void vsi_iommu_release_device(struct device *dev)
{
	struct vsi_iommu *iommu = dev_iommu_priv_get(dev);

	if (!iommu)
		return;

	device_link_remove(dev, iommu->dev);
}

static int vsi_iommu_of_xlate(struct device *dev, const struct of_phandle_args *args)
{
	if (args->args_count)
		return -EINVAL;

	return 0;
}

static const struct iommu_ops vsi_iommu_ops = {
	.identity_domain = &vsi_identity_domain,
	.release_domain = &vsi_identity_domain,
	.domain_alloc_paging = vsi_iommu_domain_alloc_paging,
	.of_xlate = vsi_iommu_of_xlate,
	.probe_device = vsi_iommu_probe_device,
	.release_device = vsi_iommu_release_device,
	.device_group = generic_single_device_group,
	.owner = THIS_MODULE,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev		= vsi_iommu_attach_device,
		.map_pages		= vsi_iommu_map,
		.unmap_pages		= vsi_iommu_unmap,
		.flush_iotlb_all	= vsi_iommu_flush_tlb,
		.iova_to_phys		= vsi_iommu_iova_to_phys,
		.free			= vsi_iommu_domain_free,
	}
};

static struct vsi_iommu *vsi_iommu_from_dev_checked(struct device *dev)
{
	if (!dev || !dev->iommu || !dev->iommu->iommu_dev ||
	    dev->iommu->iommu_dev->ops != &vsi_iommu_ops)
		return NULL;

	return dev_iommu_priv_get(dev);
}

int vsi_iommu_refresh(struct device *dev)
{
	struct vsi_iommu *iommu = vsi_iommu_from_dev_checked(dev);
	struct vsi_iommu_domain *vsi_domain;
	unsigned long flags;
	int ret;

	if (!iommu || !iommu->domain || iommu->domain == &vsi_identity_domain)
		return -ENODEV;

	ret = pm_runtime_resume_and_get(iommu->dev);
	if (ret < 0)
		return ret;

	vsi_domain = to_vsi_domain(iommu->domain);
	spin_lock_irqsave(&vsi_domain->lock, flags);
	spin_lock(&iommu->lock);
	vsi_iommu_disable(iommu);
	vsi_iommu_enable(iommu, iommu->domain);
	writel(VSI_MMU_BIT_FLUSH, iommu->regs + VSI_MMU_FLUSH_BASE);
	writel(0, iommu->regs + VSI_MMU_FLUSH_BASE);
	spin_unlock(&iommu->lock);
	spin_unlock_irqrestore(&vsi_domain->lock, flags);

	pm_runtime_put_autosuspend(iommu->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(vsi_iommu_refresh);

void vsi_iommu_mask_irq(struct device *dev)
{
	struct vsi_iommu *iommu = vsi_iommu_from_dev_checked(dev);
	unsigned long flags;

	if (!iommu)
		return;

	spin_lock_irqsave(&iommu->lock, flags);
	vsi_iommu_mask_irq_locked(iommu);
	spin_unlock_irqrestore(&iommu->lock, flags);
}
EXPORT_SYMBOL_GPL(vsi_iommu_mask_irq);

int vsi_iommu_set_fault_handler(struct device *dev,
					iommu_fault_handler_t handler, void *token)
{
	struct vsi_iommu *iommu = vsi_iommu_from_dev_checked(dev);
	unsigned long flags;

	if (!iommu)
		return -ENODEV;

	spin_lock_irqsave(&iommu->fault_lock, flags);
	iommu->fault_handler = handler;
	iommu->fault_handler_token = token;
	spin_unlock_irqrestore(&iommu->fault_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(vsi_iommu_set_fault_handler);

static const struct of_device_id vsi_iommu_dt_ids[] = {
	{
		.compatible = "verisilicon,iommu-1.2",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vsi_iommu_dt_ids);

static int vsi_iommu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vsi_iommu *iommu;
	int err;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	iommu->dev = dev;
	spin_lock_init(&iommu->lock);
	spin_lock_init(&iommu->fault_lock);
	INIT_LIST_HEAD(&iommu->node);
	iommu->domain = &vsi_identity_domain;

	iommu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(iommu->regs))
		return PTR_ERR(iommu->regs);

	iommu->num_clocks = devm_clk_bulk_get_all(dev, &iommu->clocks);
	if  (iommu->num_clocks < 0)
		return iommu->num_clocks;

	err = clk_bulk_prepare(iommu->num_clocks, iommu->clocks);
	if (err)
		return err;

	iommu->irq = platform_get_irq(pdev, 0);
	if (iommu->irq < 0) {
		err = iommu->irq;
		goto err_unprepare_clocks;
	}

	err = devm_request_irq(iommu->dev, iommu->irq, vsi_iommu_irq,
			       IRQF_SHARED, dev_name(dev), iommu);
	if (err)
		goto err_unprepare_clocks;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err)
		goto err_unprepare_clocks;
	platform_set_drvdata(pdev, iommu);

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	err = iommu_device_sysfs_add(&iommu->iommu, dev, NULL, "%s",
				     dev_name(dev));
	if (err)
		goto err_runtime_disable;

	err = iommu_device_register(&iommu->iommu, &vsi_iommu_ops, dev);
	if (err)
		goto err_remove_sysfs;

	return 0;

err_remove_sysfs:
	iommu_device_sysfs_remove(&iommu->iommu);
err_runtime_disable:
	pm_runtime_disable(dev);
err_unprepare_clocks:
	clk_bulk_unprepare(iommu->num_clocks, iommu->clocks);
	return err;
}

static void vsi_iommu_shutdown(struct platform_device *pdev)
{
	struct vsi_iommu *iommu = platform_get_drvdata(pdev);

	disable_irq(iommu->irq);
	pm_runtime_force_suspend(&pdev->dev);
}

static int __maybe_unused vsi_iommu_suspend(struct device *dev)
{
	struct vsi_iommu *iommu = dev_get_drvdata(dev);

	vsi_iommu_disable(iommu);

	clk_bulk_disable(iommu->num_clocks, iommu->clocks);

	return 0;
}

static int __maybe_unused vsi_iommu_resume(struct device *dev)
{
	struct vsi_iommu *iommu = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	ret = clk_bulk_enable(iommu->num_clocks, iommu->clocks);
	if (ret)
		return ret;

	if (iommu->domain && iommu->domain != &vsi_identity_domain) {
		struct vsi_iommu_domain *vsi_domain = to_vsi_domain(iommu->domain);

		spin_lock_irqsave(&vsi_domain->lock, flags);
		spin_lock(&iommu->lock);
		vsi_iommu_enable(iommu, iommu->domain);
		spin_unlock(&iommu->lock);
		spin_unlock_irqrestore(&vsi_domain->lock, flags);
	}

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(vsi_iommu_pm_ops,
				 vsi_iommu_suspend, vsi_iommu_resume,
				 NULL);

static struct platform_driver rockchip_vsi_iommu_driver = {
	.probe = vsi_iommu_probe,
	.shutdown = vsi_iommu_shutdown,
	.driver = {
		   .name = "vsi_iommu",
		   .of_match_table = vsi_iommu_dt_ids,
		   .pm = pm_ptr(&vsi_iommu_pm_ops),
		   .suppress_bind_attrs = true,
	},
};
module_platform_driver(rockchip_vsi_iommu_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Benjamin Gaignard <benjamin.gaignard@collabora.com>");
MODULE_DESCRIPTION("Verisilicon IOMMU driver");
