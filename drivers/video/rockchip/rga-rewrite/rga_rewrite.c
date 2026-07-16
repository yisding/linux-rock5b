// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Rockchip RGA compatibility rewrite.
 *
 * The driver owns /dev/rga and preserves the userspace ioctl surface used by
 * librga.  Buffer import/release, request lifetime, prepared job resource
 * ownership, and the first RK3588 RGA2/RGA3 execution paths are implemented.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-buf.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/iova.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#if IS_ENABLED(CONFIG_ROCKCHIP_RGA_REWRITE_KUNIT_TEST)
#include <linux/mman.h>
#include <kunit/test.h>
#endif
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/stddef.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <soc/rockchip/rockchip_iommu.h>

#ifndef kzalloc_obj
#define kzalloc_obj(obj, flags)	kzalloc(sizeof(obj), flags)
#endif

#define RK_RGA_REWRITE_VERSION		"rk3588-rga-rewrite-0.1"
#define RK_RGA2_CMD_REG_COUNT		32
#define RK_RGA3_CMD_REG_COUNT		48
#define RK_RGA_JOB_TIMEOUT_MS		1000
#define RK_RGA_RESET_TIMEOUT_US		1000
#define RK_RGA_IOMMU_DMA_LIMIT		(DMA_BIT_MASK(32) - SZ_512M)
#define RK_RGA_FULL_CSC_ENABLE		BIT(0)

#define RK_RGA2_SYS_CTRL	0x000
#define RK_RGA2_CMD_CTRL	0x004
#define RK_RGA2_CMD_BASE	0x008
#define RK_RGA2_STATUS1		0x00c
#define RK_RGA2_INT		0x010
#define RK_RGA2_STATUS2		0x01c
#define RK_RGA2_WORK_CNT	0x020
#define RK_RGA2_VERSION_NUM	0x028
#define RK_RGA2_MIN_REG_SIZE	0x090

#define RK_RGA2_VERSION_MAJOR	GENMASK(31, 24)
#define RK_RGA2_VERSION_MINOR	GENMASK(23, 20)

#define RK_RGA2_SYS_CTRL_AUTO_RST		BIT(5)
#define RK_RGA2_SYS_CTRL_CCLK_SRESET		BIT(4)
#define RK_RGA2_SYS_CTRL_ACLK_SRESET		BIT(3)
#define RK_RGA2_SYS_CTRL_RST_PROTECT		BIT(6)
#define RK_RGA2_SYS_CTRL_AUTO_CKG		BIT(2)
#define RK_RGA2_SYS_CTRL_CMD_MODE		BIT(1)
#define RK_RGA2_SYS_CTRL_CMD_OP_ST		BIT(0)
#define RK_RGA2_CMD_CTRL_CMD_LINE_ST		BIT(0)
#define RK_RGA2_INT_FBCIN_DEC_ERROR_CLEAR	BIT(24)
#define RK_RGA2_INT_FBCIN_DEC_ERROR_EN		BIT(23)
#define RK_RGA2_INT_FBCIN_DEC_ERROR		BIT(22)
#define RK_RGA2_INT_SCL_ERROR_CLEAR		BIT(19)
#define RK_RGA2_INT_SCL_ERROR_EN		BIT(18)
#define RK_RGA2_INT_SCL_ERROR_INTR		BIT(17)
#define RK_RGA2_INT_LINE_WR_CLEAR		BIT(16)
#define RK_RGA2_INT_LINE_RD_CLEAR		BIT(15)
#define RK_RGA2_INT_LINE_WR_EN			BIT(14)
#define RK_RGA2_INT_LINE_RD_EN			BIT(13)
#define RK_RGA2_INT_WRITE_CNT_FLAG		BIT(12)
#define RK_RGA2_INT_READ_CNT_FLAG		BIT(11)
#define RK_RGA2_INT_ALL_CMD_DONE_INT_EN		BIT(10)
#define RK_RGA2_INT_MMU_INT_EN			BIT(9)
#define RK_RGA2_INT_ERROR_INT_EN		BIT(8)
#define RK_RGA2_INT_NOW_CMD_DONE_INT_CLEAR	BIT(7)
#define RK_RGA2_INT_ALL_CMD_DONE_INT_CLEAR	BIT(6)
#define RK_RGA2_INT_MMU_INT_CLEAR		BIT(5)
#define RK_RGA2_INT_ERROR_INT_CLEAR		BIT(4)
#define RK_RGA2_INT_CUR_CMD_DONE_INT_FLAG	BIT(3)
#define RK_RGA2_INT_ALL_CMD_DONE_INT_FLAG	BIT(2)
#define RK_RGA2_INT_MMU_INT_FLAG		BIT(1)
#define RK_RGA2_INT_ERROR_INT_FLAG		BIT(0)
#define RK_RGA2_INT_DONE_MASK \
	(RK_RGA2_INT_CUR_CMD_DONE_INT_FLAG | RK_RGA2_INT_ALL_CMD_DONE_INT_FLAG)
#define RK_RGA2_INT_ERROR_MASK \
	(RK_RGA2_INT_MMU_INT_FLAG | RK_RGA2_INT_ERROR_INT_FLAG | \
	 RK_RGA2_INT_SCL_ERROR_INTR | RK_RGA2_INT_FBCIN_DEC_ERROR)
#define RK_RGA2_INT_LINE_MASK \
	(RK_RGA2_INT_READ_CNT_FLAG | RK_RGA2_INT_WRITE_CNT_FLAG)
#define RK_RGA2_INT_CLEAR_MASK \
	(RK_RGA2_INT_MMU_INT_CLEAR | RK_RGA2_INT_ERROR_INT_CLEAR | \
	 RK_RGA2_INT_SCL_ERROR_CLEAR | RK_RGA2_INT_FBCIN_DEC_ERROR_CLEAR | \
	 RK_RGA2_INT_ALL_CMD_DONE_INT_CLEAR | \
	 RK_RGA2_INT_NOW_CMD_DONE_INT_CLEAR | RK_RGA2_INT_LINE_RD_CLEAR | \
	 RK_RGA2_INT_LINE_WR_CLEAR)
#define RK_RGA2_INT_ENABLE_MASK \
	(RK_RGA2_INT_MMU_INT_EN | RK_RGA2_INT_ERROR_INT_EN | \
	 RK_RGA2_INT_SCL_ERROR_EN | RK_RGA2_INT_FBCIN_DEC_ERROR_EN | \
	 RK_RGA2_INT_ALL_CMD_DONE_INT_EN)
#define RK_RGA2_STATUS2_RPP_ERROR		BIT(2)
#define RK_RGA2_STATUS2_BUS_ERROR		BIT(1)
#define RK_RGA2_READ_LINE_CNT			0x030
#define RK_RGA2_WRITE_LINE_CNT			0x034
#define RK_RGA2_SYS_CTRL_HOLD_MODE_EN		BIT(9)
#define RK_RGA2_LINE_RD_THRESHOLD		GENMASK(12, 0)
#define RK_RGA2_LINE_WR_START			GENMASK(12, 0)
#define RK_RGA2_LINE_WR_STEP			GENMASK(28, 16)

#define RK_RGA2_MODE_CTRL_OFFSET		0x000
#define RK_RGA2_SRC_INFO_OFFSET		0x004
#define RK_RGA2_SRC_BASE0_OFFSET		0x008
#define RK_RGA2_SRC_BASE1_OFFSET		0x00c
#define RK_RGA2_SRC_BASE2_OFFSET		0x010
#define RK_RGA2_SRC_BASE3_OFFSET		0x014
#define RK_RGA2_SRC_VIR_INFO_OFFSET		0x018
#define RK_RGA2_SRC_ACT_INFO_OFFSET		0x01c
#define RK_RGA2_SRC_X_FACTOR_OFFSET		0x020
#define RK_RGA2_SRC_Y_FACTOR_OFFSET		0x024
#define RK_RGA2_SRC_BG_COLOR_OFFSET		0x028
#define RK_RGA2_GAUSS_COE_OFFSET		0x028
#define RK_RGA2_SRC_FG_COLOR_OFFSET		0x02c
#define RK_RGA2_SRC_TR_COLOR0_OFFSET		0x030
#define RK_RGA2_CF_GR_A_OFFSET			0x030
#define RK_RGA2_SRC_TR_COLOR1_OFFSET		0x034
#define RK_RGA2_CF_GR_B_OFFSET			0x034
#define RK_RGA2_DST_INFO_OFFSET		0x038
#define RK_RGA2_DST_BASE0_OFFSET		0x03c
#define RK_RGA2_DST_BASE1_OFFSET		0x040
#define RK_RGA2_DST_BASE2_OFFSET		0x044
#define RK_RGA2_DST_VIR_INFO_OFFSET		0x048
#define RK_RGA2_DST_ACT_INFO_OFFSET		0x04c
#define RK_RGA2_ALPHA_CTRL0_OFFSET		0x050
#define RK_RGA2_ALPHA_CTRL1_OFFSET		0x054
#define RK_RGA2_FADING_CTRL_OFFSET		0x058
#define RK_RGA2_PAT_CON_OFFSET			0x05c
#define RK_RGA2_ROP_CTRL0_OFFSET		0x060
#define RK_RGA2_ROP_CTRL1_OFFSET		0x064
#define RK_RGA2_CF_GR_G_OFFSET			0x060
#define RK_RGA2_CF_GR_R_OFFSET			0x064
#define RK_RGA2_DST_Y4MAP_LUT0_OFFSET		0x060
#define RK_RGA2_DST_Y4MAP_LUT1_OFFSET		0x064
#define RK_RGA2_DST_QUANTIZE_SCALE_OFFSET	0x060
#define RK_RGA2_DST_QUANTIZE_OFFSET_OFFSET	0x064
#define RK_RGA2_MASK_BASE_OFFSET		0x068
#define RK_RGA2_DST_CSC_00_OFFSET		0x060
#define RK_RGA2_DST_CSC_01_OFFSET		0x064
#define RK_RGA2_DST_CSC_02_OFFSET		0x068
#define RK_RGA2_DST_CSC_OFF0_OFFSET		0x06c
#define RK_RGA2_DST_CSC_10_OFFSET		0x070
#define RK_RGA2_DST_CSC_11_OFFSET		0x074
#define RK_RGA2_DST_CSC_12_OFFSET		0x078
#define RK_RGA2_DST_CSC_OFF1_OFFSET		0x07c
#define RK_RGA2_DST_CSC_20_OFFSET		0x080
#define RK_RGA2_DST_CSC_21_OFFSET		0x084
#define RK_RGA2_DST_CSC_22_OFFSET		0x088
#define RK_RGA2_DST_CSC_OFF2_OFFSET		0x08c

#define RK_RGA2_MODE_RENDER_MODE		GENMASK(2, 0)
#define RK_RGA2_MODE_BITBLT_MODE		BIT(3)
#define RK_RGA2_MODE_COLOR_FILL_MODE		BIT(4)
#define RK_RGA2_MODE_INTR_CF_E			BIT(7)
#define RK_RGA2_MODE_OSD_EN			BIT(8)
#define RK_RGA2_MODE_MOSAIC_EN			BIT(9)
#define RK_RGA2_MODE_SRC_GAUSS_EN		BIT(17)
#define RK_RGA2_MOSAIC_MODE_OFFSET		0x030
#define RK_RGA2_HW_RENDER_UPDATE_PALETTE	3

#define RK_RGA2_OSD_CTRL0_OFFSET		0x020
#define RK_RGA2_OSD_CTRL1_OFFSET		0x024
#define RK_RGA2_OSD_COLOR0_OFFSET		0x028
#define RK_RGA2_OSD_COLOR1_OFFSET		0x02c
#define RK_RGA2_OSD_LAST_FLAGS0_OFFSET		0x030
#define RK_RGA2_OSD_LAST_FLAGS1_OFFSET		0x034
#define RK_RGA2_OSD_INVERSION_CAL0_OFFSET	0x060
#define RK_RGA2_OSD_INVERSION_CAL1_OFFSET	0x064

#define RK_RGA2_OSD_CTRL0_MODE			GENMASK(1, 0)
#define RK_RGA2_OSD_CTRL0_DIRECTION		BIT(2)
#define RK_RGA2_OSD_CTRL0_WIDTH_MODE		BIT(3)
#define RK_RGA2_OSD_CTRL0_BLOCK_COUNT		GENMASK(8, 4)
#define RK_RGA2_OSD_CTRL0_FLAGS_INDEX		GENMASK(19, 10)
#define RK_RGA2_OSD_CTRL0_FIX_WIDTH		GENMASK(29, 20)

#define RK_RGA2_OSD_CTRL1_COLOR_MODE		BIT(0)
#define RK_RGA2_OSD_CTRL1_FLAGS_MODE		BIT(1)
#define RK_RGA2_OSD_CTRL1_DEFAULT_COLOR		BIT(2)
#define RK_RGA2_OSD_CTRL1_INVERT_MODE		BIT(3)
#define RK_RGA2_OSD_CTRL1_THRESH		GENMASK(11, 4)
#define RK_RGA2_OSD_CTRL1_INVERT_A		BIT(12)
#define RK_RGA2_OSD_CTRL1_INVERT_Y		BIT(13)
#define RK_RGA2_OSD_CTRL1_INVERT_C		BIT(14)
#define RK_RGA2_OSD_CTRL1_UNFIX_INDEX		GENMASK(19, 16)

#define RK_RGA2_SRC_FORMAT			GENMASK(3, 0)
#define RK_RGA2_SRC_RB_SWAP			BIT(4)
#define RK_RGA2_SRC_ALPHA_SWAP			BIT(5)
#define RK_RGA2_SRC_UV_SWAP			BIT(6)
#define RK_RGA2_SRC_CP_ENDIAN			BIT(7)
#define RK_RGA2_SRC_CSC_MODE			GENMASK(9, 8)
#define RK_RGA2_SRC_ROT_MODE			GENMASK(11, 10)
#define RK_RGA2_SRC_MIR_MODE			GENMASK(13, 12)
#define RK_RGA2_SRC_HSCL_MODE			GENMASK(15, 14)
#define RK_RGA2_SRC_VSCL_MODE			GENMASK(17, 16)
#define RK_RGA2_SRC_TRANS_MODE			BIT(18)
#define RK_RGA2_SRC_TRANS_ENABLE		GENMASK(22, 19)
#define RK_RGA2_SRC_SCL_FILTER			GENMASK(25, 24)
#define RK_RGA2_SRC_VSP_MODE_SEL		BIT(26)
#define RK_RGA2_SRC_YUV10_EN			BIT(27)
#define RK_RGA2_SRC_YUV10_ROUND_EN		BIT(28)
#define RK_RGA2_SRC_VSD_MODE_SEL		BIT(29)
#define RK_RGA2_SRC_HSP_MODE_SEL		BIT(30)
#define RK_RGA2_SRC_HSD_MODE_SEL		BIT(31)

#define RK_RGA2_DST_FORMAT			GENMASK(3, 0)
#define RK_RGA2_DST_RB_SWAP			BIT(4)
#define RK_RGA2_DST_ALPHA_SWAP			BIT(5)
#define RK_RGA2_DST_UV_SWAP			BIT(6)
#define RK_RGA2_DST_SRC1_FORMAT		GENMASK(9, 7)
#define RK_RGA2_DST_SRC1_RB_SWAP		BIT(10)
#define RK_RGA2_DST_SRC1_ALPHA_SWAP		BIT(11)
#define RK_RGA2_DST_DITHER_UP_EN		BIT(12)
#define RK_RGA2_DST_DITHER_DOWN_EN		BIT(13)
#define RK_RGA2_DST_DITHER_MODE		GENMASK(15, 14)
#define RK_RGA2_DST_CSC_MODE			GENMASK(17, 16)
#define RK_RGA2_DST_CSC_CLIP			BIT(18)
#define RK_RGA2_DST_FULL_CSC_EN		BIT(19)
#define RK_RGA2_DST_YUV400_EN			BIT(24)
#define RK_RGA2_DST_Y4_EN			BIT(25)
#define RK_RGA2_DST_NN_QUANTIZE_EN		BIT(26)
#define RK_RGA2_DST_SRC1_A1555_ALPHA_EN	BIT(28)

#define RK_RGA2_ALPHA_ROP_0			BIT(0)
#define RK_RGA2_ALPHA_ROP_SEL			BIT(1)
#define RK_RGA2_ALPHA_ROP_MODE			GENMASK(3, 2)
#define RK_RGA2_ALPHA_SRC_GLOBAL		GENMASK(11, 4)
#define RK_RGA2_ALPHA_DST_GLOBAL		GENMASK(19, 12)
#define RK_RGA2_ALPHA_ROP_ENDIAN		BIT(20)

#define RK_RGA2_ALPHA_CTRL1_DST_COLOR_M0	BIT(0)
#define RK_RGA2_ALPHA_CTRL1_SRC_COLOR_M0	BIT(1)
#define RK_RGA2_ALPHA_CTRL1_DST_FACTOR_M0	GENMASK(4, 2)
#define RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M0	GENMASK(7, 5)
#define RK_RGA2_ALPHA_CTRL1_DST_ALPHA_CAL_M0	BIT(8)
#define RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_CAL_M0	BIT(9)
#define RK_RGA2_ALPHA_CTRL1_DST_BLEND_M0	GENMASK(11, 10)
#define RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M0	GENMASK(13, 12)
#define RK_RGA2_ALPHA_CTRL1_DST_ALPHA_M0	BIT(14)
#define RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_M0	BIT(15)
#define RK_RGA2_ALPHA_CTRL1_DST_FACTOR_M1	GENMASK(18, 16)
#define RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M1	GENMASK(21, 19)
#define RK_RGA2_ALPHA_CTRL1_DST_ALPHA_CAL_M1	BIT(22)
#define RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_CAL_M1	BIT(23)
#define RK_RGA2_ALPHA_CTRL1_DST_BLEND_M1	GENMASK(25, 24)
#define RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M1	GENMASK(27, 26)
#define RK_RGA2_ALPHA_CTRL1_DST_ALPHA_M1	BIT(28)
#define RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_M1	BIT(29)

#define RK_RGA2_GAUSS_COE0			GENMASK(5, 0)
#define RK_RGA2_GAUSS_COE1			GENMASK(13, 8)
#define RK_RGA2_GAUSS_COE2			GENMASK(23, 16)

#define RK_RGA2_NN_QUANTIZE_MASK		GENMASK(9, 0)

#define RK_RGA2_ALPHA_FLAG_ENABLE		BIT(0)
#define RK_RGA2_ALPHA_FLAG_PD_ENABLE		BIT(3)
#define RK_RGA2_ALPHA_FLAG_CAL_MODE		BIT(4)
#define RK_RGA2_ALPHA_FLAG_DST_DITHER_DOWN	BIT(5)
#define RK_RGA2_ALPHA_FLAG_REAL_COLOR		BIT(9)

#define RK_RGA2_ALPHA_STRAIGHT			0
#define RK_RGA2_ALPHA_GLOBAL			0
#define RK_RGA2_ALPHA_PER_PIXEL		1
#define RK_RGA2_ALPHA_ZERO			0
#define RK_RGA2_ALPHA_ONE			1
#define RK_RGA2_ALPHA_OPPOSITE_INVERSE		3
#define RK_RGA2_ALPHA_PRE_MULTIPLIED		0
#define RK_RGA2_ALPHA_NO_PRE_MULTIPLIED	1

#define RK_RGA_ROP_AND				0x88
#define RK_RGA_ROP_OR				0xee
#define RK_RGA_ROP_NOT_DST			0x55
#define RK_RGA_ROP_NOT_SRC			0x33
#define RK_RGA_ROP_XOR				0xf6
#define RK_RGA_ROP_NOT_XOR			0xf9

#define RK_RGA2_SCALE_BYPASS			0
#define RK_RGA2_SCALE_DOWN			1
#define RK_RGA2_SCALE_UP			2
#define RK_RGA2_SCALE_FORCE_TILE		3
#define RK_RGA2_BILINEAR_PREC			12
#define RK_RGA2_INTERP_DEFAULT			0
#define RK_RGA2_INTERP_LINEAR			1
#define RK_RGA2_INTERP_BICUBIC			2
#define RK_RGA2_INTERP_AVERAGE			3

#define RK_RGA3_SYS_CTRL	0x000
#define RK_RGA3_CMD_CTRL	0x004
#define RK_RGA3_CMD_ADDR	0x008
#define RK_RGA3_VERSION_NUM	0x018
#define RK_RGA3_INT_EN		0x020
#define RK_RGA3_INT_RAW		0x024
#define RK_RGA3_INT_CLR		0x02c
#define RK_RGA3_RO_SRST		0x030
#define RK_RGA3_STATUS0		0x034
#define RK_RGA3_CMD_STATE	0x040
#define RK_RGA3_MIN_REG_SIZE	0x044

#define RK_RGA3_VERSION_MAJOR	GENMASK(31, 28)
#define RK_RGA3_VERSION_MINOR	GENMASK(27, 20)
#define RK_RGA_VERSION_REVISION	GENMASK(19, 0)

#define RK_RGA3_SYS_CTRL_CCLK_SRESET	BIT(4)
#define RK_RGA3_SYS_CTRL_ACLK_SRESET	BIT(3)
#define RK_RGA3_SYS_CTRL_CMD_MODE	BIT(1)
#define RK_RGA3_RO_SRST_RST_DONE	GENMASK(5, 0)
#define RK_RGA3_CMD_CTRL_LINE_START	BIT(0)
#define RK_RGA3_INT_WIN1_VOR_FIFO_REN_ERR	BIT(29)
#define RK_RGA3_INT_WIN1_VOR_FIFO_WEN_ERR	BIT(28)
#define RK_RGA3_INT_WIN1_HOR_FIFO_REN_ERR	BIT(27)
#define RK_RGA3_INT_WIN1_HOR_FIFO_WEN_ERR	BIT(26)
#define RK_RGA3_INT_WIN1_IN_FIFO_REB_ERR	BIT(25)
#define RK_RGA3_INT_WIN1_IN_FIFO_WEN_ERR	BIT(24)
#define RK_RGA3_INT_WIN0_VOR_FIFO_REN_ERR	BIT(21)
#define RK_RGA3_INT_WIN0_VOR_FIFO_WEN_ERR	BIT(20)
#define RK_RGA3_INT_WIN0_HOR_FIFO_REN_ERR	BIT(19)
#define RK_RGA3_INT_WIN0_HOR_FIFO_WEN_ERR	BIT(18)
#define RK_RGA3_INT_WIN0_IN_FIFO_REB_ERR	BIT(17)
#define RK_RGA3_INT_WIN0_IN_FIFO_WEN_ERR	BIT(16)
#define RK_RGA3_INT_RGA_MI_WR_BUS_ERR		BIT(15)
#define RK_RGA3_INT_RGA_MI_WR_IN_HERR		BIT(14)
#define RK_RGA3_INT_WIN1_V_ERR			BIT(11)
#define RK_RGA3_INT_WIN1_H_ERR			BIT(10)
#define RK_RGA3_INT_WIN1_FBCD_DEC_ERR		BIT(9)
#define RK_RGA3_INT_WIN1_RD_FRM_END		BIT(8)
#define RK_RGA3_INT_WIN0_V_ERR			BIT(7)
#define RK_RGA3_INT_WIN0_H_ERR			BIT(6)
#define RK_RGA3_INT_WIN0_FBCD_DEC_ERR		BIT(5)
#define RK_RGA3_INT_WIN0_RD_FRM_END		BIT(4)
#define RK_RGA3_INT_CMD_LINE_FINISH		BIT(3)
#define RK_RGA3_INT_RGA_MI_RD_BUS_ERR		BIT(2)
#define RK_RGA3_INT_RGA_MMU_INTR		BIT(1)
#define RK_RGA3_INT_FRM_DONE			BIT(0)
#define RK_RGA3_INT_DONE_MASK \
	(RK_RGA3_INT_FRM_DONE | RK_RGA3_INT_CMD_LINE_FINISH)
#define RK_RGA3_INT_ERROR_MASK \
	(RK_RGA3_INT_RGA_MMU_INTR | RK_RGA3_INT_RGA_MI_RD_BUS_ERR | \
	 RK_RGA3_INT_WIN0_FBCD_DEC_ERR | RK_RGA3_INT_WIN0_H_ERR | \
	 RK_RGA3_INT_WIN0_V_ERR | RK_RGA3_INT_WIN1_FBCD_DEC_ERR | \
	 RK_RGA3_INT_WIN1_H_ERR | RK_RGA3_INT_WIN1_V_ERR | \
	 RK_RGA3_INT_RGA_MI_WR_IN_HERR | RK_RGA3_INT_RGA_MI_WR_BUS_ERR | \
	 RK_RGA3_INT_WIN0_IN_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN0_IN_FIFO_REB_ERR | \
	 RK_RGA3_INT_WIN0_HOR_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN0_HOR_FIFO_REN_ERR | \
	 RK_RGA3_INT_WIN0_VOR_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN0_VOR_FIFO_REN_ERR | \
	 RK_RGA3_INT_WIN1_IN_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN1_IN_FIFO_REB_ERR | \
	 RK_RGA3_INT_WIN1_HOR_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN1_HOR_FIFO_REN_ERR | \
	 RK_RGA3_INT_WIN1_VOR_FIFO_WEN_ERR | \
	 RK_RGA3_INT_WIN1_VOR_FIFO_REN_ERR)

#define RK_RGA3_WIN0_RD_CTRL_OFFSET		0x000
#define RK_RGA3_WIN0_Y_BASE_OFFSET		0x010
#define RK_RGA3_WIN0_U_BASE_OFFSET		0x014
#define RK_RGA3_WIN0_V_BASE_OFFSET		0x018
#define RK_RGA3_WIN0_VIR_STRIDE_OFFSET		0x01c
#define RK_RGA3_WIN0_SRC_SIZE_OFFSET		0x024
#define RK_RGA3_WIN0_ACT_OFF_OFFSET		0x028
#define RK_RGA3_WIN0_ACT_SIZE_OFFSET		0x02c
#define RK_RGA3_WIN0_DST_SIZE_OFFSET		0x030
#define RK_RGA3_WIN0_SCL_FAC_OFFSET		0x034
#define RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET	0x038
#define RK_RGA3_WIN1_RD_CTRL_OFFSET		0x040
#define RK_RGA3_WIN1_Y_BASE_OFFSET		0x050
#define RK_RGA3_WIN1_U_BASE_OFFSET		0x054
#define RK_RGA3_WIN1_V_BASE_OFFSET		0x058
#define RK_RGA3_WIN1_VIR_STRIDE_OFFSET		0x05c
#define RK_RGA3_WIN1_SRC_SIZE_OFFSET		0x064
#define RK_RGA3_WIN1_ACT_OFF_OFFSET		0x068
#define RK_RGA3_WIN1_ACT_SIZE_OFFSET		0x06c
#define RK_RGA3_WIN1_DST_SIZE_OFFSET		0x070
#define RK_RGA3_WIN1_SCL_FAC_OFFSET		0x074
#define RK_RGA3_WIN1_UV_VIR_STRIDE_OFFSET	0x078
#define RK_RGA3_OVLP_CTRL_OFFSET		0x080
#define RK_RGA3_OVLP_OFF_OFFSET			0x084
#define RK_RGA3_OVLP_TOP_KEY_MIN_OFFSET		0x088
#define RK_RGA3_OVLP_TOP_KEY_MAX_OFFSET		0x08c
#define RK_RGA3_OVLP_TOP_CTRL_OFFSET		0x090
#define RK_RGA3_OVLP_BOT_CTRL_OFFSET		0x094
#define RK_RGA3_OVLP_TOP_ALPHA_OFFSET		0x098
#define RK_RGA3_OVLP_BOT_ALPHA_OFFSET		0x09c
#define RK_RGA3_WR_CTRL_OFFSET			0x0a0
#define RK_RGA3_WR_FBCE_CTRL_OFFSET		0x0a4
#define RK_RGA3_WR_VIR_STRIDE_OFFSET		0x0a8
#define RK_RGA3_WR_PL_VIR_STRIDE_OFFSET		0x0ac
#define RK_RGA3_WR_Y_BASE_OFFSET		0x0b0
#define RK_RGA3_WR_U_BASE_OFFSET		0x0b4
#define RK_RGA3_WR_V_BASE_OFFSET		0x0b8

#define RK_RGA3_WIN0_ENABLE			BIT(0)
#define RK_RGA3_WIN0_RD_MODE			GENMASK(2, 1)
#define RK_RGA3_WIN0_PIC_FORMAT			GENMASK(7, 4)
#define RK_RGA3_WIN0_RD_FORMAT			GENMASK(9, 8)
#define RK_RGA3_WIN0_YUV10_COMPACT		BIT(10)
#define RK_RGA3_WIN0_ENDIAN_MODE		BIT(11)
#define RK_RGA3_WIN0_PIX_SWAP			BIT(12)
#define RK_RGA3_WIN0_YC_SWAP			BIT(13)
#define RK_RGA3_WIN0_ROT			BIT(16)
#define RK_RGA3_WIN0_XMIRROR			BIT(17)
#define RK_RGA3_WIN0_YMIRROR			BIT(18)
#define RK_RGA3_WIN0_HOR_BY			BIT(20)
#define RK_RGA3_WIN0_HOR_UP			BIT(21)
#define RK_RGA3_WIN0_VER_BY			BIT(22)
#define RK_RGA3_WIN0_VER_UP			BIT(23)
#define RK_RGA3_WIN0_Y2R_EN			BIT(24)
#define RK_RGA3_WIN0_R2Y_EN			BIT(25)
#define RK_RGA3_WIN0_CSC_MODE			GENMASK(27, 26)

#define RK_RGA3_OVLP_MODE			GENMASK(1, 0)
#define RK_RGA3_OVLP_FIELD			BIT(2)
#define RK_RGA3_OVLP_TOP_ALPHA_EN		BIT(4)
#define RK_RGA3_OVLP_TOP_KEY_EN		GENMASK(19, 5)

#define RK_RGA3_ALPHA_COLOR_MODE		BIT(0)
#define RK_RGA3_ALPHA_MODE			BIT(1)
#define RK_RGA3_ALPHA_BLEND_MODE		GENMASK(3, 2)
#define RK_RGA3_ALPHA_CAL_MODE			BIT(4)
#define RK_RGA3_ALPHA_FACTOR			GENMASK(7, 5)
#define RK_RGA3_ALPHA_GLOBAL			GENMASK(23, 16)

#define RK_RGA3_ALPHA_GLOBAL_BLEND		0
#define RK_RGA3_ALPHA_PER_PIXEL			1
#define RK_RGA3_ALPHA_PER_PIXEL_GLOBAL		2
#define RK_RGA3_ALPHA_SATURATION		0
#define RK_RGA3_ALPHA_NO_SATURATION		1
#define RK_RGA3_ALPHA_PRE_MULTIPLIED		0
#define RK_RGA3_ALPHA_NO_PRE_MULTIPLIED		1
#define RK_RGA3_ALPHA_STRAIGHT			0
#define RK_RGA3_ALPHA_ZERO			0
#define RK_RGA3_ALPHA_ONE			1
#define RK_RGA3_ALPHA_OPPOSITE			2
#define RK_RGA3_ALPHA_OPPOSITE_INVERSE		3
#define RK_RGA3_ALPHA_SUPPORTED_FLAGS \
	(BIT(0) | BIT(3) | BIT(4) | BIT(9))

#define RK_RGA3_ROT_BIT_ROT_90			BIT(0)
#define RK_RGA3_ROT_BIT_X_MIRROR		BIT(1)
#define RK_RGA3_ROT_BIT_Y_MIRROR		BIT(2)

#define RK_RGA3_WR_MODE				GENMASK(1, 0)
#define RK_RGA3_WR_FBCE_SPARSE_EN		BIT(2)
#define RK_RGA3_WR_PIC_FORMAT			GENMASK(7, 4)
#define RK_RGA3_WR_FORMAT			GENMASK(9, 8)
#define RK_RGA3_WR_YUV10_COMPACT		BIT(10)
#define RK_RGA3_WR_ENDIAN_MODE			BIT(11)
#define RK_RGA3_WR_PIX_SWAP			BIT(12)
#define RK_RGA3_WR_OUTSTANDING_MAX		GENMASK(18, 13)
#define RK_RGA3_WR_YC_SWAP			BIT(20)

#define RK_RGA3_FACTOR_MAX			(2 << 15)
#define RK_RGA3_SIZE_MASK			0x1fff
#define RK_RGA_RASTER_MODE			BIT(0)
#define RK_RGA_FBC_MODE			BIT(1)
#define RK_RGA_TILE_MODE			BIT(2)
/*
 * RKFBC64x4 and AFBC32x8 are RGA2-Pro compressed modes. RK3588 is
 * documented as one RGA2-Enhance core plus two RGA3 cores, so the rewrite
 * recognizes these userspace mode values but rejects submit profiles that
 * require them.
 */
#define RK_RGA_RKFBC_MODE			BIT(4)
#define RK_RGA_AFBC32X8_MODE			BIT(5)
#define RK_RGA_10BIT_INCOMPACT			1

#define RGA_IOC_MAGIC			'r'
#define RGA_IOW(nr, type)		_IOW(RGA_IOC_MAGIC, nr, type)
#define RGA_IOR(nr, type)		_IOR(RGA_IOC_MAGIC, nr, type)
#define RGA_IOWR(nr, type)		_IOWR(RGA_IOC_MAGIC, nr, type)

#define RGA_BLIT_SYNC			0x5017
#define RGA_BLIT_ASYNC			0x5018
#define RGA_FLUSH			0x5019
#define RGA_GET_RESULT			0x501a
#define RGA_GET_VERSION			0x501b
#define RGA_CACHE_FLUSH			0x501c
#define RGA2_FLUSH			0x6019
#define RGA2_GET_RESULT		0x601a
#define RGA2_GET_VERSION		0x601b
#define RGA_IMPORT_DMA			0x601d
#define RGA_RELEASE_DMA			0x601e

#define RGA_TASK_NUM_MAX		256
#define RGA_BUFFER_POOL_SIZE_MAX	40
#define RGA_VERSION_SIZE		16
#define RGA_HW_SIZE			5

#define RK_RGA_CORE_RGA3_MASK		(BIT(0) | BIT(1))
#define RK_RGA_CORE_RGA2_MASK		(BIT(2) | BIT(3))
#define RK_RGA_CORE_MASK		(RK_RGA_CORE_RGA3_MASK | \
					 RK_RGA_CORE_RGA2_MASK)
#define RK_RGA_CORE_COUNTER_COUNT	4
#define RK_RGA_SCHED_PRIORITY_MAX	6

#define RK_RGA_BACKEND_QUEUED		1
#define RK_RGA_MMU_SRC0		BIT(8)
#define RK_RGA_MMU_SRC1		BIT(9)
#define RK_RGA_MMU_DST		BIT(10)
#define RK_RGA_MMU_ELSE		BIT(11)
#define RK_RGA_RELEASE_FENCE_ABORT_ERR	(-EFAULT)

#define DRIVER_MAJOR_VERISON		1
#define DRIVER_MINOR_VERSION		3
#define DRIVER_REVISION_VERSION		11
#define DRIVER_VERSION			"1.3.11"

enum rk_rga_hw_type {
	RK_RGA_HW_RGA2 = 2,
	RK_RGA_HW_RGA3 = 3,
};

#define RK_RGA_HW_TYPE_MASK_RGA2	BIT(RK_RGA_HW_RGA2)
#define RK_RGA_HW_TYPE_MASK_RGA3	BIT(RK_RGA_HW_RGA3)
#define RK_RGA_HW_TYPE_MASK_ALL		(RK_RGA_HW_TYPE_MASK_RGA2 | \
					 RK_RGA_HW_TYPE_MASK_RGA3)

enum rk_rga_memory_type {
	RGA_DMA_BUFFER = 0,
	RGA_VIRTUAL_ADDRESS,
	RGA_PHYSICAL_ADDRESS,
	RGA_DMA_BUFFER_PTR,
};

enum rk_rga_import_type {
	RK_RGA_IMPORT_DMABUF,
	RK_RGA_IMPORT_USERPTR,
};

enum rk_rga_render_mode {
	RK_RGA_RENDER_BITBLT = 0x0,
	RK_RGA_RENDER_COLOR_PALETTE = 0x1,
	RK_RGA_RENDER_COLOR_FILL = 0x2,
	RK_RGA_RENDER_UPDATE_PALETTE = 0x6,
	RK_RGA_RENDER_UPDATE_PATTERN = 0x7,
};

enum rk_rga_format {
	RK_RGA_FORMAT_RGBA_8888 = 0x0,
	RK_RGA_FORMAT_RGBX_8888 = 0x1,
	RK_RGA_FORMAT_RGB_888 = 0x2,
	RK_RGA_FORMAT_BGRA_8888 = 0x3,
	RK_RGA_FORMAT_RGB_565 = 0x4,
	RK_RGA_FORMAT_RGBA_5551 = 0x5,
	RK_RGA_FORMAT_RGBA_4444 = 0x6,
	RK_RGA_FORMAT_BGR_888 = 0x7,
	RK_RGA_FORMAT_YCBCR_422_SP = 0x8,
	RK_RGA_FORMAT_YCBCR_422_P = 0x9,
	RK_RGA_FORMAT_YCBCR_420_SP = 0xa,
	RK_RGA_FORMAT_YCBCR_420_P = 0xb,
	RK_RGA_FORMAT_YCRCB_422_SP = 0xc,
	RK_RGA_FORMAT_YCRCB_422_P = 0xd,
	RK_RGA_FORMAT_YCRCB_420_SP = 0xe,
	RK_RGA_FORMAT_YCRCB_420_P = 0xf,
	RK_RGA_FORMAT_BPP1 = 0x10,
	RK_RGA_FORMAT_BPP2 = 0x11,
	RK_RGA_FORMAT_BPP4 = 0x12,
	RK_RGA_FORMAT_BPP8 = 0x13,
	RK_RGA_FORMAT_Y4 = 0x14,
	RK_RGA_FORMAT_YCBCR_400 = 0x15,
	RK_RGA_FORMAT_BGRX_8888 = 0x16,
	RK_RGA_FORMAT_YVYU_422 = 0x18,
	RK_RGA_FORMAT_YVYU_420 = 0x19,
	RK_RGA_FORMAT_VYUY_422 = 0x1a,
	RK_RGA_FORMAT_VYUY_420 = 0x1b,
	RK_RGA_FORMAT_YUYV_422 = 0x1c,
	RK_RGA_FORMAT_YUYV_420 = 0x1d,
	RK_RGA_FORMAT_UYVY_422 = 0x1e,
	RK_RGA_FORMAT_UYVY_420 = 0x1f,
	RK_RGA_FORMAT_YCBCR_420_SP_10B = 0x20,
	RK_RGA_FORMAT_YCRCB_420_SP_10B = 0x21,
	RK_RGA_FORMAT_YCBCR_422_SP_10B = 0x22,
	RK_RGA_FORMAT_YCRCB_422_SP_10B = 0x23,
	RK_RGA_FORMAT_BGR_565 = 0x24,
	RK_RGA_FORMAT_BGRA_5551 = 0x25,
	RK_RGA_FORMAT_BGRA_4444 = 0x26,
	RK_RGA_FORMAT_ARGB_8888 = 0x28,
	RK_RGA_FORMAT_XRGB_8888 = 0x29,
	RK_RGA_FORMAT_ARGB_5551 = 0x2a,
	RK_RGA_FORMAT_ARGB_4444 = 0x2b,
	RK_RGA_FORMAT_ABGR_8888 = 0x2c,
	RK_RGA_FORMAT_XBGR_8888 = 0x2d,
	RK_RGA_FORMAT_ABGR_5551 = 0x2e,
	RK_RGA_FORMAT_ABGR_4444 = 0x2f,
	RK_RGA_FORMAT_RGBA_2BPP = 0x30,
	RK_RGA_FORMAT_A8 = 0x31,
	RK_RGA_FORMAT_YCBCR_444_SP = 0x32,
	RK_RGA_FORMAT_YCRCB_444_SP = 0x33,
	RK_RGA_FORMAT_Y8 = 0x34,
};

struct rga_version_t {
	__u32 major;
	__u32 minor;
	__u32 revision;
	__u8 str[RGA_VERSION_SIZE];
};

struct rga_hw_versions_t {
	struct rga_version_t version[RGA_HW_SIZE];
	__u32 size;
};

struct rga_memory_parm {
	__u32 width;
	__u32 height;
	__u32 format;
	__u32 size;
};

struct rga_external_buffer {
	__u64 memory;
	__u32 type;
	__u32 handle;
	struct rga_memory_parm memory_parm;
	__u8 reserve[252];
};

struct rga_buffer_pool {
	__u64 buffers_ptr;
	__u32 size;
};

struct rga_user_request {
	__u64 task_ptr;
	__u32 task_num;
	__u32 id;
	__u32 sync_mode;
	__u32 release_fence_fd;
	__u32 mpi_config_flags;
	__u32 acquire_fence_fd;
	__u8 reservr[120];
};

struct rga_color_fill_t {
	__s16 gr_x_a;
	__s16 gr_y_a;
	__s16 gr_x_b;
	__s16 gr_y_b;
	__s16 gr_x_g;
	__s16 gr_y_g;
	__s16 gr_x_r;
	__s16 gr_y_r;
};

struct rga_fading_t {
	__u8 b;
	__u8 g;
	__u8 r;
	__u8 res;
};

struct rga_mmu_t {
	__u8 mmu_en;
	__u64 base_addr;
	__u32 mmu_flag;
};

struct rga_rect_t {
	__u16 xmin;
	__u16 xmax;
	__u16 ymin;
	__u16 ymax;
};

struct rga_point_t {
	__u16 x;
	__u16 y;
};

struct rga_line_draw_t {
	struct rga_point_t start_point;
	struct rga_point_t end_point;
	__u32 color;
	__u32 flag;
	__u32 line_width;
};

struct rga_csc_coe {
	__s16 r_v;
	__s16 g_y;
	__s16 b_u;
	__s32 off;
};

struct rga_full_csc {
	__u8 flag;
	struct rga_csc_coe coe_y;
	struct rga_csc_coe coe_u;
	struct rga_csc_coe coe_v;
};

struct rga_csc_range {
	__u16 max;
	__u16 min;
};

struct rga_csc_clip {
	struct rga_csc_range y;
	struct rga_csc_range uv;
};

struct rga_mosaic_info {
	__u8 enable;
	__u8 mode;
};

struct rga_gauss_config {
	__u32 size;
	__u64 coe_ptr;
};

struct rga_osd_invert_factor {
	__u8 alpha_max;
	__u8 alpha_min;
	__u8 yg_max;
	__u8 yg_min;
	__u8 crb_max;
	__u8 crb_min;
};

struct rga_color {
	union {
		struct {
			__u8 red;
			__u8 green;
			__u8 blue;
			__u8 alpha;
		};
		__u32 value;
	};
};

struct rga_osd_bpp2 {
	__u8 ac_swap;
	__u8 endian_swap;
	struct rga_color color0;
	struct rga_color color1;
};

struct rga_osd_mode_ctrl {
	__u8 mode;
	__u8 direction_mode;
	__u8 width_mode;
	__u16 block_fix_width;
	__u8 block_num;
	__u16 flags_index;
	__u8 color_mode;
	__u8 invert_flags_mode;
	__u8 default_color_sel;
	__u8 invert_enable;
	__u8 invert_mode;
	__u8 invert_thresh;
	__u8 unfix_index;
};

struct rga_osd_info {
	__u8 enable;
	struct rga_osd_mode_ctrl mode_ctrl;
	struct rga_osd_invert_factor cal_factor;
	struct rga_osd_bpp2 bpp2_info;
	union {
		struct {
			__u32 last_flags0;
			__u32 last_flags1;
		};
		__u64 last_flags;
	};
	union {
		struct {
			__u32 cur_flags0;
			__u32 cur_flags1;
		};
		__u64 cur_flags;
	};
};

struct rga_pre_intr_info {
	__u8 enable;
	__u8 read_intr_en;
	__u8 write_intr_en;
	__u8 read_hold_en;
	__u32 read_threshold;
	__u32 write_start;
	__u32 write_step;
};

static u32 rk_rga2_pre_intr_read_line(const struct rga_pre_intr_info *intr)
{
	if (!intr->enable || !intr->read_intr_en)
		return 0;

	return FIELD_PREP(RK_RGA2_LINE_RD_THRESHOLD, intr->read_threshold);
}

static u32 rk_rga2_pre_intr_write_line(const struct rga_pre_intr_info *intr)
{
	if (!intr->enable || !intr->write_intr_en)
		return 0;

	return FIELD_PREP(RK_RGA2_LINE_WR_START, intr->write_start) |
	       FIELD_PREP(RK_RGA2_LINE_WR_STEP, intr->write_step);
}

static u32 rk_rga2_pre_intr_int_enable(const struct rga_pre_intr_info *intr)
{
	if (!intr->enable)
		return 0;

	return FIELD_PREP(RK_RGA2_INT_LINE_RD_EN, intr->read_intr_en) |
	       FIELD_PREP(RK_RGA2_INT_LINE_WR_EN, intr->write_intr_en);
}

static u32 rk_rga2_pre_intr_sys_ctrl(const struct rga_pre_intr_info *intr)
{
	if (!intr->enable || !intr->read_hold_en)
		return 0;

	return RK_RGA2_SYS_CTRL_HOLD_MODE_EN;
}

struct rga_img_info_t {
	__u64 yrgb_addr;
	__u64 uv_addr;
	__u64 v_addr;
	__u32 format;
	__u16 act_w;
	__u16 act_h;
	__u16 x_offset;
	__u16 y_offset;
	__u16 vir_w;
	__u16 vir_h;
	__u16 endian_mode;
	__u16 alpha_swap;
	__u16 rotate_mode;
	__u16 rd_mode;
	__u16 compact_mode;
	__u16 is_10b_endian;
	__u16 enable;
};

struct rga_feature {
	__u32 global_alpha_en:1;
	__u32 full_csc_clip_en:1;
	__u32 user_close_fence:1;
};

struct rga_interp {
	__u8 horiz:4;
	__u8 verti:4;
};

struct rga_rgba5551_alpha {
	__u16 flags;
	__u8 alpha0;
	__u8 alpha1;
};

struct rga_req {
	__u8 render_mode;
	struct rga_img_info_t src;
	struct rga_img_info_t dst;
	struct rga_img_info_t pat;
	__u64 rop_mask_addr;
	__u64 LUT_addr;
	struct rga_rect_t clip;
	__s32 sina;
	__s32 cosa;
	__u16 alpha_rop_flag;
	union {
		struct rga_interp interp;
		__u8 scale_mode;
	};
	__u32 color_key_max;
	__u32 color_key_min;
	__u32 fg_color;
	__u32 bg_color;
	struct rga_color_fill_t gr_color;
	struct rga_line_draw_t line_draw_info;
	struct rga_fading_t fading;
	__u8 PD_mode;
	__u8 alpha_global_value;
	__u16 rop_code;
	__u8 bsfilter_flag;
	__u8 palette_mode;
	__u8 yuv2rgb_mode;
	__u8 endian_mode;
	__u8 rotate_mode;
	__u8 color_fill_mode;
	struct rga_mmu_t mmu_info;
	__u8 alpha_rop_mode;
	__u8 src_trans_mode;
	__u8 dither_mode;
	struct rga_full_csc full_csc;
	__s32 in_fence_fd;
	__u8 core;
	__u8 priority;
	__s32 out_fence_fd;
	__u8 handle_flag;
	struct rga_mosaic_info mosaic_info;
	__u8 uvhds_mode;
	__u8 uvvds_mode;
	struct rga_osd_info osd_info;
	struct rga_pre_intr_info pre_intr_info;
	__u8 fg_global_alpha;
	__u8 bg_global_alpha;
	struct rga_feature feature;
	struct rga_csc_clip full_csc_clip;
	struct rga_rgba5551_alpha rgba5551_alpha;
	struct rga_gauss_config gauss_config;
	__u8 reservr[24];
};

#define RGA_IOC_GET_DRVIER_VERSION	RGA_IOR(0x1, struct rga_version_t)
#define RGA_IOC_GET_HW_VERSION		RGA_IOR(0x2, struct rga_hw_versions_t)
#define RGA_IOC_IMPORT_BUFFER		RGA_IOWR(0x3, struct rga_buffer_pool)
#define RGA_IOC_RELEASE_BUFFER		RGA_IOW(0x4, struct rga_buffer_pool)
#define RGA_IOC_REQUEST_CREATE		RGA_IOR(0x5, __u32)
#define RGA_IOC_REQUEST_SUBMIT		RGA_IOWR(0x6, struct rga_user_request)
#define RGA_IOC_REQUEST_CONFIG		RGA_IOWR(0x7, struct rga_user_request)
#define RGA_IOC_REQUEST_CANCEL		RGA_IOWR(0x8, __u32)

#define RGA_IMG_INFO_ABI_SIZE			56
#define RGA_EXTERNAL_BUFFER_ABI_SIZE		288
#define RGA_BUFFER_POOL_ABI_SIZE		16
#define RGA_USER_REQUEST_ABI_SIZE		152
#define RGA_REQ_ABI_SIZE			504
#define RGA_VERSION_ABI_SIZE			28
#define RGA_HW_VERSIONS_ABI_SIZE		144

static_assert(sizeof(struct rga_version_t) == RGA_VERSION_ABI_SIZE);
static_assert(sizeof(struct rga_hw_versions_t) ==
	      RGA_HW_VERSIONS_ABI_SIZE);
static_assert(offsetof(struct rga_hw_versions_t, size) == 140);
static_assert(sizeof(struct rga_img_info_t) == RGA_IMG_INFO_ABI_SIZE);
static_assert(offsetof(struct rga_img_info_t, compact_mode) == 48);
static_assert(sizeof(struct rga_external_buffer) ==
	      RGA_EXTERNAL_BUFFER_ABI_SIZE);
static_assert(offsetof(struct rga_external_buffer, memory_parm) == 16);
static_assert(sizeof(struct rga_buffer_pool) == RGA_BUFFER_POOL_ABI_SIZE);
static_assert(sizeof(struct rga_user_request) == RGA_USER_REQUEST_ABI_SIZE);
static_assert(offsetof(struct rga_user_request, release_fence_fd) == 20);
static_assert(offsetof(struct rga_user_request, acquire_fence_fd) == 28);
static_assert(sizeof(struct rga_req) == RGA_REQ_ABI_SIZE);
static_assert(offsetof(struct rga_req, src) == 8);
static_assert(offsetof(struct rga_req, mmu_info) == 280);
static_assert(offsetof(struct rga_req, in_fence_fd) == 348);
static_assert(offsetof(struct rga_req, out_fence_fd) == 356);
static_assert(offsetof(struct rga_req, handle_flag) == 360);
static_assert(_IOC_NR(RGA_IOC_GET_DRVIER_VERSION) == 0x1);
static_assert(_IOC_SIZE(RGA_IOC_GET_DRVIER_VERSION) ==
	      sizeof(struct rga_version_t));
static_assert(_IOC_NR(RGA_IOC_GET_HW_VERSION) == 0x2);
static_assert(_IOC_SIZE(RGA_IOC_GET_HW_VERSION) ==
	      sizeof(struct rga_hw_versions_t));
static_assert(_IOC_NR(RGA_IOC_IMPORT_BUFFER) == 0x3);
static_assert(_IOC_SIZE(RGA_IOC_IMPORT_BUFFER) ==
	      sizeof(struct rga_buffer_pool));
static_assert(_IOC_NR(RGA_IOC_RELEASE_BUFFER) == 0x4);
static_assert(_IOC_SIZE(RGA_IOC_RELEASE_BUFFER) ==
	      sizeof(struct rga_buffer_pool));
static_assert(_IOC_NR(RGA_IOC_REQUEST_CREATE) == 0x5);
static_assert(_IOC_SIZE(RGA_IOC_REQUEST_CREATE) == sizeof(__u32));
static_assert(_IOC_NR(RGA_IOC_REQUEST_SUBMIT) == 0x6);
static_assert(_IOC_SIZE(RGA_IOC_REQUEST_SUBMIT) ==
	      sizeof(struct rga_user_request));
static_assert(_IOC_NR(RGA_IOC_REQUEST_CONFIG) == 0x7);
static_assert(_IOC_SIZE(RGA_IOC_REQUEST_CONFIG) ==
	      sizeof(struct rga_user_request));
static_assert(_IOC_NR(RGA_IOC_REQUEST_CANCEL) == 0x8);
static_assert(_IOC_SIZE(RGA_IOC_REQUEST_CANCEL) == sizeof(__u32));
static_assert(RGA_BLIT_SYNC == 0x5017);
static_assert(RGA_BLIT_ASYNC == 0x5018);
static_assert(RGA_FLUSH == 0x5019);
static_assert(RGA_GET_RESULT == 0x501a);
static_assert(RGA_GET_VERSION == 0x501b);
static_assert(RGA_CACHE_FLUSH == 0x501c);
static_assert(RGA2_FLUSH == 0x6019);
static_assert(RGA2_GET_RESULT == 0x601a);
static_assert(RGA2_GET_VERSION == 0x601b);
static_assert(RGA_IMPORT_DMA == 0x601d);
static_assert(RGA_RELEASE_DMA == 0x601e);

struct rk_rga_import {
	refcount_t refs;
	enum rk_rga_import_type type;
	int fd;
	struct device *dev;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct page **pages;
	struct iommu_domain *domain;
	dma_addr_t iova;
	size_t iova_size;
	size_t size;
	unsigned int page_count;
	unsigned int pinned_pages;
	unsigned int page_offset;
	bool iommu_mapped;
	bool counted;
};

struct rk_rga_job_mapping {
	struct rk_rga_import *import;
	struct device *dev;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct iommu_domain *domain;
	dma_addr_t iova;
	size_t iova_size;
	unsigned int page_offset;
	bool userptr;
	bool iommu_mapped;
};

struct rk_rga_img_layout {
	size_t yrgb_size;
	size_t uv_size;
	size_t v_size;
	size_t total_size;
};

struct rk_rga_request {
	__u32 flags;
	__u32 task_count;
	__u32 sync_mode;
	__u32 mpi_config_flags;
	__s32 acquire_fence_fd;
	__s32 release_fence_fd;
	struct rga_req *tasks;
	struct rk_rga_import **imports;
	struct dma_fence **acquire_fences;
	u32 *gauss_coeffs;
	u32 import_count;
	u32 acquire_fence_count;
	bool configured;
};

struct rk_rga_fence_waiter {
	struct dma_fence_cb cb;
	struct rk_rga_job *job;
};

struct rk_rga_acquire_fd {
	int fd;
	bool kernel_close;
};

struct rk_rga_job {
	struct list_head node;
	struct list_head session_node;
	struct rk_rga_hw *hw;
	struct rk_rga_session *session;
	struct work_struct acquire_work;
	refcount_t refs;
	struct rga_req *tasks;
	struct rk_rga_import **imports;
	struct rk_rga_job_mapping *mappings;
	struct dma_fence **acquire_fences;
	u32 *gauss_coeffs;
	struct rk_rga_fence_waiter *acquire_waiters;
	struct dma_fence *release_fence;
	struct device *cmd_dev;
	void *cmd_vaddr;
	dma_addr_t cmd_dma;
	size_t cmd_size;
	wait_queue_head_t wait;
	atomic_t pending_acquire_count;
	atomic_t acquire_work_queued;
	u32 task_count;
	u32 current_task;
	u32 import_count;
	u32 mapping_count;
	u32 acquire_fence_count;
	u32 intr_status;
	u32 hw_status;
	u32 cmd_status;
	u32 work_cycle;
	u64 hw_start_ns;
	u64 hw_elapsed_ns;
	__u32 sync_mode;
	u8 priority;
	int release_fence_fd;
	int irq_result;
	int result;
	bool queued;
	bool session_linked;
	bool cmd_ready;
	bool irq_seen;
	bool waiting_acquire;
	bool done;
};

struct rk_rga_hw_match {
	enum rk_rga_hw_type type;
	const char *name;
	__u32 version_major;
	__u32 version_minor;
	__u32 version_revision;
	u32 min_reg_size;
};

struct rk_rga_hw {
	struct list_head node;
	struct list_head fault_node;
	struct device *dev;
	struct device_node *iommu_node;
	struct iommu_domain *iommu_domain;
	void __iomem *regs;
	struct clk_bulk_data *clks;
	struct reset_control *resets;
	int num_clks;
	int irq;
	int index;
	enum rk_rga_hw_type type;
	const struct rk_rga_hw_match *match;
	struct rga_version_t version;
	u32 core_mask;
	refcount_t refs;
	wait_queue_head_t idle;
	spinlock_t job_lock;
	struct mutex run_lock; /* serializes start, timeout, IRQ, and remove */
	struct delayed_work timeout_work;
	struct work_struct iommu_fault_work;
	struct list_head job_queue;
	struct rk_rga_job *active_job;
	struct rk_rga_job *timeout_job;
	u64 active_generation;
	u64 iommu_fault_generation;
	atomic_t irq_disable_depth;
	u32 queued_jobs;
	bool iommu_fault_handler_registered;
	bool irq_registered;
	bool recovery_failed;
	bool removing;
};

struct rk_rga_session {
	struct mutex lock;
	spinlock_t job_lock;
	wait_queue_head_t job_wait;
	struct idr imports;
	struct idr requests;
	struct list_head service_node;
	struct list_head jobs;
	u32 dispatching_jobs;
	bool closing;
	bool service_linked;
};

struct rk_rga_service {
	struct miscdevice miscdev;
	struct dentry *debugfs_root;
	struct mutex hw_lock;
	struct mutex session_lock;
	spinlock_t fault_lock; /* protects fault_hws in fault handler context */
	struct list_head hw_list;
	struct list_head sessions;
	struct list_head fault_hws;
	struct rga_hw_versions_t hw_versions;
	u32 hw_count;
	u32 core_select_seq;
	u64 fence_context;
	u32 fence_seqno;
	spinlock_t fence_lock;
	atomic_t ioctl_count;
	atomic_t import_count;
	atomic_t prepared_job_count;
	atomic_t release_fence_count;
	atomic_t completed_job_count;
	atomic_t scheduled_job_count;
	atomic_t scheduled_core_count[RK_RGA_CORE_COUNTER_COUNT];
	atomic_t dispatched_job_count;
	atomic_t dispatched_core_count[RK_RGA_CORE_COUNTER_COUNT];
	atomic_t started_job_count;
	atomic_t started_core_count[RK_RGA_CORE_COUNTER_COUNT];
	atomic64_t hw_total_ns;
	atomic64_t hw_max_ns;
	atomic64_t hw_total_core_ns[RK_RGA_CORE_COUNTER_COUNT];
	atomic64_t hw_max_core_ns[RK_RGA_CORE_COUNTER_COUNT];
	atomic_t cmd_alloc_count;
	atomic_t power_cycle_count;
	atomic_t irq_count;
	atomic_t irq_thread_count;
	atomic_t irq_error_count;
	atomic_t irq_spurious_count;
	atomic_t timeout_count;
	atomic_t iommu_fault_count;
	atomic_t iommu_refresh_count;
	atomic_t recovery_failure_count;
	atomic_t unsupported_count;
	atomic_t route_b_attempt_count;
	atomic_t route_b_ok_count;
	atomic_t route_b_active_count;
	bool route_b_force_remap;
};

static struct rk_rga_service rk_rga;

static int rk_rga_release(struct inode *inode, struct file *file);

static void rk_rga_session_init(struct rk_rga_session *session)
{
	mutex_init(&session->lock);
	spin_lock_init(&session->job_lock);
	init_waitqueue_head(&session->job_wait);
	idr_init(&session->imports);
	idr_init(&session->requests);
	INIT_LIST_HEAD(&session->service_node);
	INIT_LIST_HEAD(&session->jobs);
}

static void rk_rga_session_link(struct rk_rga_session *session)
{
	mutex_lock(&rk_rga.session_lock);
	if (!session->service_linked) {
		list_add_tail(&session->service_node, &rk_rga.sessions);
		session->service_linked = true;
	}
	mutex_unlock(&rk_rga.session_lock);
}

static void rk_rga_session_unlink(struct rk_rga_session *session)
{
	mutex_lock(&rk_rga.session_lock);
	if (session->service_linked) {
		list_del_init(&session->service_node);
		session->service_linked = false;
	}
	mutex_unlock(&rk_rga.session_lock);
}

static int rk_rga_core_counter_index(u32 core_mask)
{
	switch (core_mask) {
	case BIT(0):
		return 0;
	case BIT(1):
		return 1;
	case BIT(2):
		return 2;
	case BIT(3):
		return 3;
	default:
		return -EINVAL;
	}
}

static void rk_rga_count_core(atomic_t counters[RK_RGA_CORE_COUNTER_COUNT],
			      const struct rk_rga_hw *hw)
{
	int index = rk_rga_core_counter_index(hw->core_mask);

	if (index >= 0)
		atomic_inc(&counters[index]);
}

static void rk_rga_atomic64_max(atomic64_t *counter, u64 value)
{
	s64 old = atomic64_read(counter);

	while ((u64)old < value) {
		s64 prev = atomic64_cmpxchg(counter, old, value);

		if (prev == old)
			break;
		old = prev;
	}
}

static int rk_rga_debugfs_atomic64_get(void *data, u64 *val)
{
	*val = atomic64_read(data);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(rk_rga_debugfs_atomic64_fops,
			 rk_rga_debugfs_atomic64_get, NULL, "%llu\n");

static void rk_rga_debugfs_create_atomic64(const char *name, atomic64_t *value)
{
	debugfs_create_file(name, 0444, rk_rga.debugfs_root, value,
			    &rk_rga_debugfs_atomic64_fops);
}

static void rk_rga_count_core_ns(atomic64_t counters[RK_RGA_CORE_COUNTER_COUNT],
				 const struct rk_rga_hw *hw, u64 value,
				 bool max)
{
	int index;

	if (!hw)
		return;

	index = rk_rga_core_counter_index(hw->core_mask);
	if (index < 0)
		return;

	if (max)
		rk_rga_atomic64_max(&counters[index], value);
	else
		atomic64_add(value, &counters[index]);
}

static void rk_rga_job_note_hw_done(struct rk_rga_job *job)
{
	u64 elapsed;
	u64 start = job->hw_start_ns;

	if (!start)
		return;

	elapsed = ktime_get_ns() - start;
	job->hw_elapsed_ns += elapsed;
	job->hw_start_ns = 0;
	rk_rga_count_core_ns(rk_rga.hw_total_core_ns, job->hw, elapsed, false);
	rk_rga_count_core_ns(rk_rga.hw_max_core_ns, job->hw, elapsed, true);
}

static void rk_rga_job_record_hw_stats(struct rk_rga_job *job)
{
	u64 elapsed = job->hw_elapsed_ns;

	if (!elapsed)
		return;

	atomic64_add(elapsed, &rk_rga.hw_total_ns);
	rk_rga_atomic64_max(&rk_rga.hw_max_ns, elapsed);
	job->hw_elapsed_ns = 0;
}

static u32 rk_rga_core_distance(u32 core_mask, u32 start)
{
	int index = rk_rga_core_counter_index(core_mask);

	if (index < 0)
		return U32_MAX;

	return (index + RK_RGA_CORE_COUNTER_COUNT -
		(start % RK_RGA_CORE_COUNTER_COUNT)) %
	       RK_RGA_CORE_COUNTER_COUNT;
}

static u32 rk_rga_core_select_next(const struct rk_rga_hw *hw,
				   u32 current_seq)
{
	int index = rk_rga_core_counter_index(hw->core_mask);

	if (index < 0)
		return current_seq;

	return index + 1;
}

static void rk_rga_of_node_put(void *data)
{
	of_node_put(data);
}

static const char *rk_rga_fence_get_name(struct dma_fence *fence)
{
	return "rockchip-rga-rewrite";
}

static const struct dma_fence_ops rk_rga_fence_ops = {
	.get_driver_name = rk_rga_fence_get_name,
	.get_timeline_name = rk_rga_fence_get_name,
};

static struct dma_fence *rk_rga_fence_alloc(void)
{
	struct dma_fence *fence;
	unsigned long flags;
	u32 seqno;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	spin_lock_irqsave(&rk_rga.fence_lock, flags);
	seqno = ++rk_rga.fence_seqno;
	spin_unlock_irqrestore(&rk_rga.fence_lock, flags);

	dma_fence_init(fence, &rk_rga_fence_ops, &rk_rga.fence_lock,
		       rk_rga.fence_context, seqno);
	atomic_inc(&rk_rga.release_fence_count);

	return fence;
}

static void rk_rga_fence_signal(struct dma_fence *fence, int result)
{
	if (!fence || dma_fence_is_signaled(fence))
		return;

	if (result < 0)
		dma_fence_set_error(fence, result);
	dma_fence_signal(fence);
}

static int rk_rga_fence_create_fd(struct dma_fence *fence,
				  struct sync_file **sync_file_out)
{
	struct sync_file *sync_file;
	int fd;

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0)
		return fd;

	sync_file = sync_file_create(fence);
	if (!sync_file) {
		put_unused_fd(fd);
		return -ENOMEM;
	}

	*sync_file_out = sync_file;

	return fd;
}

static void rk_rga_fence_install_fd(int fd, struct sync_file *sync_file)
{
	fd_install(fd, sync_file->file);
}

static void rk_rga_fence_abort_fd(int fd, struct sync_file *sync_file)
{
	fput(sync_file->file);
	put_unused_fd(fd);
}

static void rk_rga_set_version(struct rga_version_t *version, __u32 major,
			       __u32 minor, __u32 revision)
{
	version->major = major;
	version->minor = minor;
	version->revision = revision;
	snprintf((char *)version->str, sizeof(version->str), "%x.%01x.%05x",
		 major, minor, revision);
}

static void rk_rga_add_hw_version(struct rga_hw_versions_t *versions,
				  __u32 major, __u32 minor, __u32 revision)
{
	if (versions->size >= RGA_HW_SIZE)
		return;

	rk_rga_set_version(&versions->version[versions->size],
			   major, minor, revision);
	versions->size++;
}

static int rk_rga_decode_hw_version(enum rk_rga_hw_type type, u32 raw,
				    struct rga_version_t *version)
{
	u32 major;
	u32 minor;

	switch (type) {
	case RK_RGA_HW_RGA2:
		major = FIELD_GET(RK_RGA2_VERSION_MAJOR, raw);
		minor = FIELD_GET(RK_RGA2_VERSION_MINOR, raw);
		break;
	case RK_RGA_HW_RGA3:
		major = FIELD_GET(RK_RGA3_VERSION_MAJOR, raw);
		minor = FIELD_GET(RK_RGA3_VERSION_MINOR, raw);
		break;
	default:
		return -EINVAL;
	}

	rk_rga_set_version(version, major, minor,
			   FIELD_GET(RK_RGA_VERSION_REVISION, raw));

	return 0;
}

static int rk_rga_validate_hw_version(const struct rk_rga_hw_match *match,
				      u32 raw,
				      struct rga_version_t *version)
{
	int ret;

	ret = rk_rga_decode_hw_version(match->type, raw, version);
	if (ret)
		return ret;

	if (version->major != match->version_major ||
	    version->minor != match->version_minor ||
	    version->revision != match->version_revision)
		return -ENODEV;

	return 0;
}

static void rk_rga_refresh_hw_versions_locked(void)
{
	struct rga_hw_versions_t versions = {};
	struct rk_rga_hw *hw;

	list_for_each_entry(hw, &rk_rga.hw_list, node)
		rk_rga_add_hw_version(&versions, hw->version.major,
				      hw->version.minor,
				      hw->version.revision);

	rk_rga.hw_versions = versions;
	rk_rga.hw_count = versions.size;
}

static void rk_rga_refresh_hw_versions(void)
{
	mutex_lock(&rk_rga.hw_lock);
	rk_rga_refresh_hw_versions_locked();
	mutex_unlock(&rk_rga.hw_lock);
}

static void rk_rga_hw_put(struct rk_rga_hw *hw)
{
	if (!hw)
		return;

	refcount_dec(&hw->refs);
	wake_up(&hw->idle);
}

static int rk_rga_hw_power_on_internal(struct rk_rga_hw *hw, bool count_cycle)
{
	int ret;

	ret = pm_runtime_resume_and_get(hw->dev);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(hw->num_clks, hw->clks);
	if (ret) {
		pm_runtime_put_sync(hw->dev);
		return ret;
	}

	if (count_cycle)
		atomic_inc(&rk_rga.power_cycle_count);

	return 0;
}

static int rk_rga_hw_power_on(struct rk_rga_hw *hw)
{
	return rk_rga_hw_power_on_internal(hw, true);
}

static void rk_rga_hw_power_off(struct rk_rga_hw *hw)
{
	clk_bulk_disable_unprepare(hw->num_clks, hw->clks);
	pm_runtime_put_sync(hw->dev);
}

static int rk_rga_hw_reset_controls(struct rk_rga_hw *hw)
{
	int ret;

	if (!hw->resets)
		return 0;

	ret = reset_control_assert(hw->resets);
	if (ret)
		return ret;

	udelay(1);

	return reset_control_deassert(hw->resets);
}

static bool rk_rga_hw_disable_irq(struct rk_rga_hw *hw)
{
	if (!hw->irq_registered || READ_ONCE(hw->recovery_failed))
		return false;

	atomic_inc(&hw->irq_disable_depth);
	disable_irq(hw->irq);
	return true;
}

static void rk_rga_hw_quarantine_irq(struct rk_rga_hw *hw)
{
	if (!hw->irq_registered)
		return;

	atomic_inc(&hw->irq_disable_depth);
	disable_irq_nosync(hw->irq);
}

static void rk_rga_hw_enable_irq(struct rk_rga_hw *hw, bool disabled)
{
	if (!disabled || READ_ONCE(hw->recovery_failed))
		return;

	atomic_dec(&hw->irq_disable_depth);
	enable_irq(hw->irq);
}

static void rk_rga_hw_restore_irq_depth(struct rk_rga_hw *hw)
{
	while (atomic_read(&hw->irq_disable_depth) > 0) {
		atomic_dec(&hw->irq_disable_depth);
		enable_irq(hw->irq);
	}
}

static void rk_rga_set_iommu_dma_limit(struct device *dev)
{
	if (!dev->bus_dma_limit || dev->bus_dma_limit > RK_RGA_IOMMU_DMA_LIMIT)
		dev->bus_dma_limit = RK_RGA_IOMMU_DMA_LIMIT;
}

static bool rk_rga_hw_accepting_jobs(const struct rk_rga_hw *hw)
{
	return !READ_ONCE(hw->removing) &&
	       !READ_ONCE(hw->recovery_failed);
}

static struct device *rk_rga_get_map_dev(void)
{
	struct rk_rga_hw *hw;
	struct device *dev = NULL;

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (!rk_rga_hw_accepting_jobs(hw))
			continue;
		if (hw->type == RK_RGA_HW_RGA3) {
			dev = get_device(hw->dev);
			break;
		}
	}
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (dev)
			break;
		if (!rk_rga_hw_accepting_jobs(hw))
			continue;
		dev = get_device(hw->dev);
		break;
	}
	mutex_unlock(&rk_rga.hw_lock);

	return dev;
}

static bool rk_rga_has_available_hw(void)
{
	struct rk_rga_hw *hw;
	bool available = false;

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (rk_rga_hw_accepting_jobs(hw)) {
			available = true;
			break;
		}
	}
	mutex_unlock(&rk_rga.hw_lock);

	return available;
}

static int rk_rga_check_iova_span(dma_addr_t iova, size_t size,
				  const char *source, bool log_errors)
{
	u64 end;

	if (!size)
		return -EINVAL;

	if (check_add_overflow((u64)iova, (u64)size - 1, &end) ||
	    iova > U32_MAX || end > U32_MAX) {
		if (log_errors)
			pr_err("reject %s DMA mapping: 32-bit IOVA span overflow, iova=%pad size=%zu end=%#llx\n",
			       source, &iova, size, end);
		return -EOVERFLOW;
	}

	return 0;
}

static int rk_rga_validate_clock_count(int count)
{
	if (count < 0)
		return count;

	return count ?: -EINVAL;
}

static int rk_rga_validate_mmio_size(u32 minimum, resource_size_t size)
{
	return size < minimum ? -EINVAL : 0;
}

static int rk_rga_check_dma_sgt(struct sg_table *sgt, const char *source,
				size_t required_size, dma_addr_t *iova_out,
				bool log_errors)
{
	if (!sgt || !sgt->sgl)
		return -EINVAL;
	if (sgt->nents != 1) {
		if (log_errors)
			pr_err("reject %s DMA mapping: expected one DMA segment, got %u, orig_nents=%u\n",
			       source, sgt->nents, sgt->orig_nents);
		return -EOPNOTSUPP;
	}

	if (!sg_dma_len(sgt->sgl))
		return -EINVAL;

	*iova_out = sg_dma_address(sgt->sgl);

	if (!required_size)
		required_size = sg_dma_len(sgt->sgl);

	if (sg_dma_len(sgt->sgl) < required_size) {
		if (log_errors)
			pr_err("reject %s DMA mapping: segment too small, len=%u required=%zu\n",
			       source, sg_dma_len(sgt->sgl), required_size);
		return -EINVAL;
	}

	return rk_rga_check_iova_span(*iova_out, required_size, source,
				      log_errors);
}

static void rk_rga_reset_sgt_dma_state(struct sg_table *sgt)
{
	struct scatterlist *sg;
	unsigned int i;

	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		sg_dma_address(sg) = DMA_MAPPING_ERROR;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		sg_dma_len(sg) = 0;
#endif
#ifdef CONFIG_NEED_SG_DMA_FLAGS
		sg->dma_flags &= ~(SG_DMA_BUS_ADDRESS | SG_DMA_SWIOTLB);
#endif
	}

	sgt->nents = sgt->orig_nents;
}

static int rk_rga_iommu_prot(struct device *dev, enum dma_data_direction dir)
{
	int prot = dev_is_dma_coherent(dev) ? IOMMU_CACHE : 0;

	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

static struct iova_domain *rk_rga_iommu_iovad(struct iommu_domain *domain)
{
	return iommu_dma_get_iova_domain(domain);
}

static int rk_rga_alloc_iommu_iova(struct iommu_domain *domain,
				   struct device *dev, size_t size,
				   dma_addr_t *iova_out)
{
	struct iova_domain *iovad;
	unsigned long shift;
	unsigned long iova_len;
	unsigned long iova;
	u64 dma_limit;

	iovad = rk_rga_iommu_iovad(domain);
	if (!iovad)
		return -EOPNOTSUPP;

	/*
	 * Route B exposes one byte-contiguous RGA span. Larger IOVA granules can
	 * force padding between non-contiguous user pages, so fail closed.
	 */
	if (iovad->granule > PAGE_SIZE)
		return -EOPNOTSUPP;

	if (iova_align(iovad, size) != size)
		return -EINVAL;

	shift = iova_shift(iovad);
	iova_len = size >> shift;
	if (!iova_len)
		return -EINVAL;

	dma_limit = dma_get_mask(dev);
	if (dev->bus_dma_limit)
		dma_limit = min_t(u64, dma_limit, dev->bus_dma_limit);
	dma_limit = min_t(u64, dma_limit, RK_RGA_IOMMU_DMA_LIMIT);
	if (domain->geometry.force_aperture)
		dma_limit = min_t(u64, dma_limit,
				  domain->geometry.aperture_end);

	iova = alloc_iova_fast(iovad, iova_len, dma_limit >> shift, true);
	if (!iova)
		return -ENOMEM;

	*iova_out = (dma_addr_t)iova << shift;

	return 0;
}

static void rk_rga_free_iommu_iova(struct iommu_domain *domain,
				   dma_addr_t iova, size_t size)
{
	struct iova_domain *iovad = rk_rga_iommu_iovad(domain);

	if (!iovad)
		return;

	free_iova_fast(iovad, iova_pfn(iovad, iova),
		       size >> iova_shift(iovad));
}

static int rk_rga_alloc_aligned_sgt(struct sg_table *sgt,
				    struct sg_table *aligned_sgt,
				    size_t *data_size, size_t *map_size)
{
	struct scatterlist *src;
	struct scatterlist *dst;
	size_t data = 0;
	size_t map = 0;
	int ret;
	int i;

	if (!sgt || !sgt->sgl || !sgt->orig_nents)
		return -EINVAL;

	ret = sg_alloc_table(aligned_sgt, sgt->orig_nents, GFP_KERNEL);
	if (ret)
		return ret;

	dst = aligned_sgt->sgl;
	for_each_sg(sgt->sgl, src, sgt->orig_nents, i) {
		phys_addr_t phys = sg_phys(src);
		phys_addr_t start = ALIGN_DOWN(phys, PAGE_SIZE);
		u64 end = (u64)phys + src->length;
		u64 aligned_end = ALIGN(end, PAGE_SIZE);
		size_t len;

		if (!src->length || end < phys || aligned_end < end) {
			ret = -EINVAL;
			goto err_free_table;
		}

		len = aligned_end - start;
		if (len > UINT_MAX ||
		    check_add_overflow(data, (size_t)src->length, &data) ||
		    check_add_overflow(map, len, &map)) {
			ret = -EOVERFLOW;
			goto err_free_table;
		}

		sg_set_page(dst, phys_to_page(start), len, 0);
		dst = sg_next(dst);
	}

	if (!data || !map) {
		ret = -EINVAL;
		goto err_free_table;
	}

	*data_size = data;
	*map_size = map;

	return 0;

err_free_table:
	sg_free_table(aligned_sgt);
	return ret;
}

static void rk_rga_unmap_userptr_iommu(struct iommu_domain *domain,
				       dma_addr_t iova, size_t iova_size,
				       unsigned int page_offset)
{
	dma_addr_t base = iova - page_offset;
	size_t unmapped;

	if (!domain || !iova_size)
		return;

	unmapped = iommu_unmap(domain, base, iova_size);
	if (unmapped != iova_size)
		pr_err("driver-owned IOMMU unmap short: iova=%pad size=%zu unmapped=%zu\n",
		       &base, iova_size, unmapped);

	rk_rga_free_iommu_iova(domain, base, iova_size);
	atomic_dec(&rk_rga.route_b_active_count);
}

static void rk_rga_unmap_userptr_sgt(struct device *dev, struct sg_table *sgt,
				     struct iommu_domain *domain,
				     dma_addr_t iova, size_t iova_size,
				     unsigned int page_offset,
				     bool iommu_mapped)
{
	if (!sgt)
		return;

	if (iommu_mapped)
		rk_rga_unmap_userptr_iommu(domain, iova, iova_size,
					   page_offset);
	else
		dma_unmap_sgtable(dev, sgt, DMA_BIDIRECTIONAL, 0);

	sg_free_table(sgt);
	kfree(sgt);
}

static void rk_rga_import_destroy(struct rk_rga_import *import)
{
	if (import->type == RK_RGA_IMPORT_DMABUF) {
		if (import->sgt)
			dma_buf_unmap_attachment(import->attach, import->sgt,
						 DMA_BIDIRECTIONAL);
		if (import->attach)
			dma_buf_detach(import->dmabuf, import->attach);
		if (import->dmabuf)
			dma_buf_put(import->dmabuf);
	} else if (import->type == RK_RGA_IMPORT_USERPTR) {
		rk_rga_unmap_userptr_sgt(import->dev, import->sgt,
					 import->domain, import->iova,
					 import->iova_size,
					 import->page_offset,
					 import->iommu_mapped);
		if (import->pages) {
			unpin_user_pages_dirty_lock(import->pages,
						    import->pinned_pages,
						    true);
			kfree(import->pages);
		}
	}
	if (import->dev)
		put_device(import->dev);
	if (import->counted)
		atomic_dec(&rk_rga.import_count);
	kfree(import);
}

static void rk_rga_import_put(struct rk_rga_import *import)
{
	if (refcount_dec_and_test(&import->refs))
		rk_rga_import_destroy(import);
}

static int rk_rga_import_dmabuf_fd(const struct rga_external_buffer *buffer,
				   int *fd)
{
	if (buffer->memory > INT_MAX)
		return -EINVAL;

	*fd = (int)buffer->memory;

	return 0;
}

static int rk_rga_import_dmabuf(struct rga_external_buffer *buffer,
				struct rk_rga_import **import_out);
static int rk_rga_import_userptr(struct rga_external_buffer *buffer,
				 struct rk_rga_import **import_out);

static void rk_rga_request_clear_imports(struct rk_rga_request *request)
{
	for (u32 i = 0; i < request->import_count; i++)
		rk_rga_import_put(request->imports[i]);

	kfree(request->imports);
	request->imports = NULL;
	request->import_count = 0;
}

static void rk_rga_put_fence_array(struct dma_fence **fences, u32 fence_count)
{
	for (u32 i = 0; i < fence_count; i++)
		dma_fence_put(fences[i]);
	kfree(fences);
}

static void rk_rga_request_clear_fences(struct rk_rga_request *request)
{
	rk_rga_put_fence_array(request->acquire_fences,
			       request->acquire_fence_count);
	request->acquire_fences = NULL;
	request->acquire_fence_count = 0;
}

static void rk_rga_request_clear_gauss(struct rk_rga_request *request)
{
	kfree(request->gauss_coeffs);
	request->gauss_coeffs = NULL;
}

static void rk_rga_request_free(void *ptr)
{
	struct rk_rga_request *request = ptr;

	rk_rga_request_clear_imports(request);
	rk_rga_request_clear_fences(request);
	rk_rga_request_clear_gauss(request);
	kfree(request->tasks);
	kfree(request);
}

static bool rk_rga_request_remove_free(struct rk_rga_session *session, __u32 id)
{
	struct rk_rga_request *request;

	mutex_lock(&session->lock);
	request = idr_remove(&session->requests, id);
	mutex_unlock(&session->lock);
	if (!request)
		return false;

	rk_rga_request_free(request);
	return true;
}

static int rk_rga_layout_size(size_t pixels, size_t multiplier,
			      size_t divisor, size_t *size)
{
	size_t bytes;

	if (check_mul_overflow(pixels, multiplier, &bytes))
		return -EOVERFLOW;

	*size = bytes / divisor;

	return 0;
}

static int rk_rga_bpp_layout_size(const struct rga_img_info_t *img,
				  u8 shift, size_t *size)
{
	size_t stride;

	stride = ALIGN((u32)img->vir_w >> shift, 4);
	if (!stride)
		return -EINVAL;
	if (check_mul_overflow(stride, (size_t)img->vir_h, size))
		return -EOVERFLOW;

	if (!*size)
		return -EINVAL;

	return 0;
}

static int rk_rga_fbc_strides(const struct rga_img_info_t *img,
			      u32 *header_stride, u32 *payload_stride)
{
	u32 aligned_w = ALIGN((u32)img->vir_w, 16);

	*header_stride = aligned_w >> 2;

	switch (img->format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_BGRX_8888:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_XRGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XBGR_8888:
		*payload_stride = aligned_w;
		return 0;
	case RK_RGA_FORMAT_RGB_888:
	case RK_RGA_FORMAT_BGR_888:
		*payload_stride = (aligned_w >> 2) * 3;
		return 0;
	case RK_RGA_FORMAT_RGB_565:
	case RK_RGA_FORMAT_BGR_565:
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
		*payload_stride = aligned_w >> 1;
		return 0;
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCRCB_420_SP:
		*payload_stride = (aligned_w >> 3) * 3;
		return 0;
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		*payload_stride = (aligned_w >> 3) * 5;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rk_rga_fbc_layout(const struct rga_img_info_t *img,
			     struct rk_rga_img_layout *layout)
{
	size_t header_words;
	size_t payload_words;
	size_t header_size;
	size_t payload_size;
	u32 header_stride;
	u32 payload_stride;
	u32 aligned_h = ALIGN((u32)img->vir_h, 16);
	int ret;

	ret = rk_rga_fbc_strides(img, &header_stride, &payload_stride);
	if (ret)
		return ret;

	if (check_mul_overflow((size_t)header_stride, (size_t)aligned_h,
			       &header_words))
		return -EOVERFLOW;
	header_size = header_words >> 2;

	if (check_mul_overflow((size_t)payload_stride, (size_t)aligned_h,
			       &payload_words) ||
	    check_mul_overflow(payload_words, (size_t)4, &payload_size))
		return -EOVERFLOW;

	if (check_add_overflow(header_size, payload_size,
			       &layout->total_size))
		return -EOVERFLOW;

	layout->yrgb_size = layout->total_size;

	return layout->total_size ? 0 : -EINVAL;
}

/*
 * Deprecated RGA2-Pro compressed-source helpers. They stay isolated so their
 * planned removal does not affect the normal RK3588 RGA2-Enhance/RGA3 paths.
 */

static u32 rk_rga_rkfbc_head_stride(const struct rga_img_info_t *img)
{
	return (ALIGN((u32)img->vir_w, 64) / 64) * 4;
}

static u32 rk_rga_afbc32x8_head_stride(const struct rga_img_info_t *img)
{
	return (ALIGN((u32)img->vir_w, 32) / 32) * 4;
}

static int rk_rga_rkfbc_layout(const struct rga_img_info_t *img,
			       struct rk_rga_img_layout *layout)
{
	size_t header_rows;
	size_t header_words;
	size_t header_size;
	u32 header_stride;

	header_stride = rk_rga_rkfbc_head_stride(img);
	header_rows = DIV_ROUND_UP((u32)img->vir_h, 4);
	if (check_mul_overflow((size_t)header_stride, header_rows,
			       &header_words) ||
	    check_mul_overflow(header_words, sizeof(u32), &header_size))
		return -EOVERFLOW;

	layout->yrgb_size = header_size;
	layout->total_size = header_size;

	return layout->total_size ? 0 : -EINVAL;
}

static int rk_rga_afbc32x8_layout(const struct rga_img_info_t *img,
				  struct rk_rga_img_layout *layout)
{
	size_t header_rows;
	size_t header_words;
	size_t header_size;
	u32 header_stride;

	header_stride = rk_rga_afbc32x8_head_stride(img);
	header_rows = DIV_ROUND_UP((u32)img->vir_h, 8);
	if (check_mul_overflow((size_t)header_stride, header_rows,
			       &header_words) ||
	    check_mul_overflow(header_words, sizeof(u32), &header_size))
		return -EOVERFLOW;

	layout->yrgb_size = header_size;
	layout->total_size = header_size;

	return layout->total_size ? 0 : -EINVAL;
}

static bool rk_rga_img_single_buffer_compressed(const struct rga_img_info_t *img)
{
	return img->rd_mode == RK_RGA_FBC_MODE ||
	       img->rd_mode == RK_RGA_RKFBC_MODE ||
	       img->rd_mode == RK_RGA_AFBC32X8_MODE;
}

static int rk_rga_img_layout(const struct rga_img_info_t *img,
			     struct rk_rga_img_layout *layout)
{
	size_t pixels;
	size_t total;
	int ret;

	if (!img->vir_w || !img->vir_h)
		return -EINVAL;
	if (check_mul_overflow((size_t)img->vir_w, (size_t)img->vir_h,
			       &pixels))
		return -EOVERFLOW;

	memset(layout, 0, sizeof(*layout));

	if (img->rd_mode == RK_RGA_FBC_MODE)
		return rk_rga_fbc_layout(img, layout);
	if (img->rd_mode == RK_RGA_RKFBC_MODE)
		return rk_rga_rkfbc_layout(img, layout);
	if (img->rd_mode == RK_RGA_AFBC32X8_MODE)
		return rk_rga_afbc32x8_layout(img, layout);

	switch (img->format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_BGRX_8888:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_XRGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XBGR_8888:
		ret = rk_rga_layout_size(pixels, 4, 1, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_RGB_888:
	case RK_RGA_FORMAT_BGR_888:
		ret = rk_rga_layout_size(pixels, 3, 1, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_RGB_565:
	case RK_RGA_FORMAT_RGBA_5551:
	case RK_RGA_FORMAT_RGBA_4444:
	case RK_RGA_FORMAT_BGR_565:
	case RK_RGA_FORMAT_BGRA_5551:
	case RK_RGA_FORMAT_BGRA_4444:
	case RK_RGA_FORMAT_ARGB_5551:
	case RK_RGA_FORMAT_ARGB_4444:
	case RK_RGA_FORMAT_ABGR_5551:
	case RK_RGA_FORMAT_ABGR_4444:
	case RK_RGA_FORMAT_YVYU_422:
	case RK_RGA_FORMAT_VYUY_422:
	case RK_RGA_FORMAT_YUYV_422:
	case RK_RGA_FORMAT_UYVY_422:
	case RK_RGA_FORMAT_YVYU_420:
	case RK_RGA_FORMAT_VYUY_420:
	case RK_RGA_FORMAT_YUYV_420:
	case RK_RGA_FORMAT_UYVY_420:
		ret = rk_rga_layout_size(pixels, 2, 1, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_YCBCR_444_SP:
	case RK_RGA_FORMAT_YCRCB_444_SP:
		layout->yrgb_size = pixels;
		ret = rk_rga_layout_size(pixels, 2, 1, &layout->uv_size);
		break;
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		layout->yrgb_size = pixels;
		layout->uv_size = pixels;
		ret = 0;
		break;
	case RK_RGA_FORMAT_YCBCR_422_P:
	case RK_RGA_FORMAT_YCRCB_422_P:
		layout->yrgb_size = pixels;
		layout->uv_size = pixels >> 1;
		layout->v_size = layout->uv_size;
		ret = 0;
		break;
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCRCB_420_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
		layout->yrgb_size = pixels;
		layout->uv_size = pixels >> 1;
		ret = 0;
		break;
	case RK_RGA_FORMAT_YCBCR_420_P:
	case RK_RGA_FORMAT_YCRCB_420_P:
		layout->yrgb_size = pixels;
		layout->uv_size = pixels >> 2;
		layout->v_size = layout->uv_size;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP8:
		ret = rk_rga_bpp_layout_size(img, 0, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_YCBCR_400:
	case RK_RGA_FORMAT_A8:
	case RK_RGA_FORMAT_Y8:
		layout->yrgb_size = pixels;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP4:
		ret = rk_rga_bpp_layout_size(img, 1, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_Y4:
		layout->yrgb_size = pixels >> 1;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP2:
		ret = rk_rga_bpp_layout_size(img, 2, &layout->yrgb_size);
		break;
	case RK_RGA_FORMAT_BPP1:
		ret = rk_rga_bpp_layout_size(img, 3, &layout->yrgb_size);
		break;
	default:
		return -EINVAL;
	}
	if (ret)
		return ret;

	if (check_add_overflow(layout->yrgb_size, layout->uv_size, &total))
		return -EOVERFLOW;
	if (check_add_overflow(total, layout->v_size, &layout->total_size))
		return -EOVERFLOW;
	if (!layout->total_size)
		return -EINVAL;

	return 0;
}

static int rk_rga_addr_add_size(__u64 base, size_t offset, __u64 *addr)
{
	if (check_add_overflow(base, (__u64)offset, addr))
		return -EOVERFLOW;

	return 0;
}

static int rk_rga_resolve_handle_locked(struct rk_rga_session *session,
					__u64 handle, size_t required_size,
					__u64 *addr,
					struct rk_rga_import **imports,
					u32 *import_count)
{
	struct rk_rga_import *import;

	if (!handle)
		return 0;

	if (handle > INT_MAX)
		return -EINVAL;

	import = idr_find(&session->imports, (int)handle);
	if (!import)
		return -EINVAL;
	if (required_size && import->size < required_size)
		return -EINVAL;

	refcount_inc(&import->refs);
	imports[*import_count] = import;
	(*import_count)++;
	*addr = import->iova;

	return 0;
}

static int rk_rga_materialize_img_import(struct rga_img_info_t *img,
					 struct rk_rga_import *import,
					 struct rk_rga_import **imports,
					 u32 *import_count)
{
	struct rk_rga_img_layout layout;
	int ret;

	ret = rk_rga_img_layout(img, &layout);
	if (ret)
		return ret;
	if (import->size < layout.total_size)
		return -EINVAL;

	imports[*import_count] = import;
	(*import_count)++;

	img->yrgb_addr = import->iova;
	if (rk_rga_img_single_buffer_compressed(img)) {
		img->uv_addr = img->yrgb_addr;
		img->v_addr = 0;
		return 0;
	}
	if (!layout.uv_size) {
		img->uv_addr = 0;
		img->v_addr = 0;
		return 0;
	}

	ret = rk_rga_addr_add_size(img->yrgb_addr, layout.yrgb_size,
				   &img->uv_addr);
	if (ret)
		return ret;
	if (layout.v_size) {
		ret = rk_rga_addr_add_size(img->uv_addr, layout.uv_size,
					   &img->v_addr);
		if (ret)
			return ret;
	} else {
		img->v_addr = 0;
	}

	return 0;
}

static bool rk_rga_direct_img_uses_mmu(const struct rga_req *task,
				       const struct rga_img_info_t *img)
{
	u32 flags = task->mmu_info.mmu_flag;

	if (img == &task->src)
		return flags & RK_RGA_MMU_SRC0;
	if (img == &task->dst)
		return flags & RK_RGA_MMU_DST;

	return flags & (RK_RGA_MMU_SRC1 | RK_RGA_MMU_ELSE);
}

enum rk_rga_direct_img_mem_type {
	RK_RGA_DIRECT_IMG_INVALID,
	RK_RGA_DIRECT_IMG_DMABUF,
	RK_RGA_DIRECT_IMG_USERPTR,
	RK_RGA_DIRECT_IMG_UNSUPPORTED_PHYS,
};

static enum rk_rga_direct_img_mem_type
rk_rga_classify_direct_img(const struct rga_req *task,
			   const struct rga_img_info_t *img)
{
	if (!rk_rga_direct_img_uses_mmu(task, img)) {
		if (!img->yrgb_addr && !img->uv_addr && !img->v_addr)
			return RK_RGA_DIRECT_IMG_INVALID;
		return RK_RGA_DIRECT_IMG_UNSUPPORTED_PHYS;
	}

	if (img->yrgb_addr && img->yrgb_addr <= INT_MAX)
		return RK_RGA_DIRECT_IMG_DMABUF;
	if (img->yrgb_addr || img->uv_addr)
		return RK_RGA_DIRECT_IMG_USERPTR;

	return RK_RGA_DIRECT_IMG_INVALID;
}

static int rk_rga_resolve_direct_img(struct rga_req *task,
				     struct rga_img_info_t *img,
				     struct rk_rga_import **imports,
				     u32 *import_count,
				     bool required)
{
	struct rga_external_buffer buffer = {};
	struct rk_rga_import *import;
	int ret;

	switch (rk_rga_classify_direct_img(task, img)) {
	case RK_RGA_DIRECT_IMG_INVALID:
		return required ? -EINVAL : 0;
	case RK_RGA_DIRECT_IMG_UNSUPPORTED_PHYS:
		return -EOPNOTSUPP;
	case RK_RGA_DIRECT_IMG_DMABUF:
		buffer.type = RGA_DMA_BUFFER;
		buffer.memory = img->yrgb_addr;
		break;
	case RK_RGA_DIRECT_IMG_USERPTR:
		buffer.type = RGA_VIRTUAL_ADDRESS;
		buffer.memory = img->yrgb_addr ? img->yrgb_addr : img->uv_addr;
		break;
	}

	buffer.memory_parm.width = img->vir_w;
	buffer.memory_parm.height = img->vir_h;
	buffer.memory_parm.format = img->format;

	if (buffer.type == RGA_DMA_BUFFER)
		ret = rk_rga_import_dmabuf(&buffer, &import);
	else
		ret = rk_rga_import_userptr(&buffer, &import);
	if (ret)
		return ret;

	ret = rk_rga_materialize_img_import(img, import, imports, import_count);
	if (ret) {
		rk_rga_import_put(import);
		return ret;
	}

	return 0;
}

static int rk_rga_resolve_img_handles_locked(struct rk_rga_session *session,
					     struct rga_img_info_t *img,
					     struct rk_rga_import **imports,
					     u32 *import_count,
					     bool required)
{
	struct rk_rga_img_layout layout;
	__u64 yrgb_handle = img->yrgb_addr;
	__u64 uv_handle = img->uv_addr;
	__u64 v_handle = img->v_addr;
	int ret;

	if (!yrgb_handle)
		return required ? -EINVAL : 0;

	ret = rk_rga_img_layout(img, &layout);
	if (ret)
		return ret;

	if (rk_rga_img_single_buffer_compressed(img) &&
	    (uv_handle || v_handle))
		return -EOPNOTSUPP;

	if (uv_handle || v_handle) {
		if (!uv_handle)
			return -EINVAL;
		if (uv_handle && !layout.uv_size)
			return -EINVAL;
		if (v_handle && !layout.v_size)
			return -EINVAL;
		if (layout.v_size && !v_handle)
			return -EINVAL;

		ret = rk_rga_resolve_handle_locked(session, yrgb_handle,
						   layout.yrgb_size,
						   &img->yrgb_addr, imports,
						   import_count);
		if (ret)
			return ret;
		ret = rk_rga_resolve_handle_locked(session, uv_handle,
						   layout.uv_size,
						   &img->uv_addr, imports,
						   import_count);
		if (ret)
			return ret;
		if (v_handle) {
			ret = rk_rga_resolve_handle_locked(session, v_handle,
							   layout.v_size,
							   &img->v_addr,
							   imports,
							   import_count);
			if (ret)
				return ret;
		}

		return 0;
	}

	ret = rk_rga_resolve_handle_locked(session, yrgb_handle,
					   layout.total_size, &img->yrgb_addr,
					   imports, import_count);
	if (ret)
		return ret;
	if (rk_rga_img_single_buffer_compressed(img)) {
		img->uv_addr = img->yrgb_addr;
		img->v_addr = 0;
		return 0;
	}
	if (!layout.uv_size) {
		img->uv_addr = 0;
		img->v_addr = 0;
		return 0;
	}

	ret = rk_rga_addr_add_size(img->yrgb_addr, layout.yrgb_size,
				   &img->uv_addr);
	if (ret)
		return ret;
	if (layout.v_size) {
		ret = rk_rga_addr_add_size(img->uv_addr, layout.uv_size,
					   &img->v_addr);
		if (ret)
			return ret;
	} else {
		img->v_addr = 0;
	}

	return 0;
}

static int rk_rga_resolve_task_handles_locked(struct rk_rga_session *session,
					      struct rga_req *task,
					      struct rk_rga_import **imports,
					      u32 *import_count)
{
	int ret;

	switch (task->render_mode) {
	case RK_RGA_RENDER_BITBLT:
	case RK_RGA_RENDER_COLOR_PALETTE:
		ret = rk_rga_resolve_img_handles_locked(session, &task->src,
							imports, import_count,
							true);
		if (ret)
			return ret;
		ret = rk_rga_resolve_img_handles_locked(session, &task->dst,
							imports, import_count,
							true);
		if (ret)
			return ret;
		if (task->pat.yrgb_addr || task->bsfilter_flag) {
			ret = rk_rga_resolve_img_handles_locked(session,
								&task->pat,
								imports,
								import_count,
								task->bsfilter_flag);
			if (ret)
				return ret;
		}
		break;
	case RK_RGA_RENDER_COLOR_FILL:
		ret = rk_rga_resolve_img_handles_locked(session, &task->dst,
							imports, import_count,
							true);
		if (ret)
			return ret;
		break;
	case RK_RGA_RENDER_UPDATE_PALETTE:
	case RK_RGA_RENDER_UPDATE_PATTERN:
		ret = rk_rga_resolve_img_handles_locked(session, &task->pat,
							imports, import_count,
							true);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	task->handle_flag &= ~1;

	return 0;
}

static int rk_rga_resolve_task_direct_buffers(struct rga_req *task,
					      struct rk_rga_import **imports,
					      u32 *import_count)
{
	int ret;

	switch (task->render_mode) {
	case RK_RGA_RENDER_BITBLT:
	case RK_RGA_RENDER_COLOR_PALETTE:
		ret = rk_rga_resolve_direct_img(task, &task->src, imports,
						import_count, true);
		if (ret)
			return ret;
		ret = rk_rga_resolve_direct_img(task, &task->dst, imports,
						import_count, true);
		if (ret)
			return ret;
		if (task->bsfilter_flag) {
			ret = rk_rga_resolve_direct_img(task, &task->pat,
							imports,
							import_count, true);
			if (ret)
				return ret;
		}
		break;
	case RK_RGA_RENDER_COLOR_FILL:
		ret = rk_rga_resolve_direct_img(task, &task->dst, imports,
						import_count, true);
		if (ret)
			return ret;
		break;
	case RK_RGA_RENDER_UPDATE_PALETTE:
	case RK_RGA_RENDER_UPDATE_PATTERN:
		ret = rk_rga_resolve_direct_img(task, &task->pat, imports,
						import_count, true);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rk_rga_record_acquire_fd(struct rk_rga_acquire_fd *acquire_fds,
				    u32 *acquire_fd_count, int fd,
				    bool kernel_close)
{
	u32 i;

	if (fd <= 0)
		return 0;

	for (i = 0; i < *acquire_fd_count; i++) {
		if (acquire_fds[i].fd == fd) {
			acquire_fds[i].kernel_close &= kernel_close;
			return 0;
		}
	}

	if (*acquire_fd_count >= RGA_TASK_NUM_MAX + 1)
		return -EOVERFLOW;

	acquire_fds[*acquire_fd_count].fd = fd;
	acquire_fds[*acquire_fd_count].kernel_close = kernel_close;
	(*acquire_fd_count)++;

	return 0;
}

static bool rk_rga_update_acquire_fd(struct rk_rga_acquire_fd *acquire_fds,
				     u32 acquire_fd_count, int fd,
				     bool kernel_close)
{
	u32 i;

	for (i = 0; i < acquire_fd_count; i++) {
		if (acquire_fds[i].fd == fd) {
			acquire_fds[i].kernel_close &= kernel_close;
			return true;
		}
	}

	return false;
}

static void rk_rga_close_kernel_acquire_fds(const struct rk_rga_acquire_fd *fds,
					    u32 fd_count)
{
	u32 i;

	for (i = 0; i < fd_count; i++) {
		if (fds[i].kernel_close)
			close_fd(fds[i].fd);
	}
}

static int rk_rga_get_fence_fd(int fd, bool kernel_close_fd,
			       struct dma_fence **fences, u32 *fence_count,
			       struct rk_rga_acquire_fd *acquire_fds,
			       u32 *acquire_fd_count)
{
	struct dma_fence *fence;
	int ret;

	if (fd <= 0)
		return 0;

	if (rk_rga_update_acquire_fd(acquire_fds, *acquire_fd_count, fd,
				     kernel_close_fd))
		return 0;

	fence = sync_file_get_fence(fd);
	if (!fence)
		return -EINVAL;

	ret = rk_rga_record_acquire_fd(acquire_fds, acquire_fd_count, fd,
				       kernel_close_fd);
	if (ret) {
		dma_fence_put(fence);
		return ret;
	}

	fences[*fence_count] = fence;
	(*fence_count)++;

	return 0;
}

static void rk_rga_put_import_array(struct rk_rga_import **imports,
				    u32 import_count)
{
	for (u32 i = 0; i < import_count; i++)
		rk_rga_import_put(imports[i]);
	kfree(imports);
}

static void rk_rga_sync_userptr_sgt(struct device *dev, struct sg_table *sgt,
				    bool for_device)
{
	if (for_device)
		dma_sync_sgtable_for_device(dev, sgt, DMA_BIDIRECTIONAL);
	else
		dma_sync_sgtable_for_cpu(dev, sgt, DMA_BIDIRECTIONAL);
}

static void rk_rga_job_sync_userptr_for_device(struct rk_rga_job *job,
					       struct device *dev)
{
	for (u32 i = 0; i < job->import_count; i++) {
		struct rk_rga_import *import = job->imports[i];

		if (import->type == RK_RGA_IMPORT_USERPTR &&
		    import->dev == dev && import->sgt)
			rk_rga_sync_userptr_sgt(dev, import->sgt, true);
	}

	for (u32 i = 0; i < job->mapping_count; i++) {
		struct rk_rga_job_mapping *mapping = &job->mappings[i];

		if (mapping->userptr && mapping->dev == dev && mapping->sgt)
			rk_rga_sync_userptr_sgt(dev, mapping->sgt, true);
	}
}

static void rk_rga_job_sync_userptr_for_cpu(struct rk_rga_job *job)
{
	for (u32 i = 0; i < job->import_count; i++) {
		struct rk_rga_import *import = job->imports[i];

		if (import->type == RK_RGA_IMPORT_USERPTR && import->sgt)
			rk_rga_sync_userptr_sgt(import->dev, import->sgt, false);
	}

	for (u32 i = 0; i < job->mapping_count; i++) {
		struct rk_rga_job_mapping *mapping = &job->mappings[i];

		if (mapping->userptr && mapping->sgt)
			rk_rga_sync_userptr_sgt(mapping->dev, mapping->sgt,
						false);
	}
}

static void rk_rga_job_clear_mappings(struct rk_rga_job *job)
{
	for (u32 i = 0; i < job->mapping_count; i++) {
		struct rk_rga_job_mapping *mapping = &job->mappings[i];

		if (mapping->sgt && mapping->userptr) {
			rk_rga_unmap_userptr_sgt(mapping->dev, mapping->sgt,
						 mapping->domain,
						 mapping->iova,
						 mapping->iova_size,
						 mapping->page_offset,
						 mapping->iommu_mapped);
		} else if (mapping->sgt) {
			dma_buf_unmap_attachment(mapping->attach, mapping->sgt,
						 DMA_BIDIRECTIONAL);
		}
		if (mapping->attach)
			dma_buf_detach(mapping->import->dmabuf, mapping->attach);
		if (mapping->dev)
			put_device(mapping->dev);
	}

	kfree(job->mappings);
	job->mappings = NULL;
	job->mapping_count = 0;
}

static void rk_rga_job_cancel_acquire_callbacks(struct rk_rga_job *job)
{
	if (!job->acquire_waiters)
		return;

	for (u32 i = 0; i < job->acquire_fence_count; i++) {
		struct rk_rga_fence_waiter *waiter = &job->acquire_waiters[i];
		struct rk_rga_job *owner;

		if (!READ_ONCE(waiter->job))
			continue;

		dma_fence_remove_callback(job->acquire_fences[i],
					  &waiter->cb);
		/* Removal and callback claim the waiter exactly once. */
		owner = xchg(&waiter->job, NULL);
		if (owner)
			atomic_dec(&owner->pending_acquire_count);
	}
}

static size_t rk_rga_cmd_size(struct rk_rga_hw *hw)
{
	if (hw->type == RK_RGA_HW_RGA3)
		return RK_RGA3_CMD_REG_COUNT * sizeof(u32);

	return RK_RGA2_CMD_REG_COUNT * sizeof(u32);
}

static void rk_rga_job_free_cmd(struct rk_rga_job *job)
{
	if (!job->cmd_vaddr)
		return;

	dma_free_coherent(job->cmd_dev, job->cmd_size, job->cmd_vaddr,
			  job->cmd_dma);
	put_device(job->cmd_dev);
	job->cmd_dev = NULL;
	job->cmd_vaddr = NULL;
	job->cmd_dma = 0;
	job->cmd_size = 0;
}

static int rk_rga_job_alloc_cmd(struct rk_rga_job *job, struct rk_rga_hw *hw)
{
	size_t size = rk_rga_cmd_size(hw);

	if (job->cmd_vaddr && job->cmd_dev == hw->dev &&
	    job->cmd_size >= size)
		return 0;

	rk_rga_job_free_cmd(job);

	job->cmd_size = size;
	job->cmd_dev = get_device(hw->dev);
	job->cmd_vaddr = dma_alloc_coherent(hw->dev, job->cmd_size,
					    &job->cmd_dma, GFP_KERNEL);
	if (!job->cmd_vaddr) {
		put_device(job->cmd_dev);
		job->cmd_dev = NULL;
		job->cmd_size = 0;
		return -ENOMEM;
	}
	if (rk_rga_check_iova_span(job->cmd_dma, job->cmd_size,
				   "command buffer", true)) {
		rk_rga_job_free_cmd(job);
		return -EOVERFLOW;
	}

	memset(job->cmd_vaddr, 0, job->cmd_size);
	atomic_inc(&rk_rga.cmd_alloc_count);

	return 0;
}

static int rk_rga_map_userptr_sgt(struct rk_rga_import *import,
				  struct device *dev,
				  struct sg_table **sgt_out,
				  dma_addr_t *iova_out,
				  struct iommu_domain **domain_out,
				  size_t *iova_size_out,
				  bool *iommu_mapped_out);
static int rk_rga_job_map_import(struct rk_rga_job *job,
				 struct rk_rga_import *import,
				 struct device *dev,
				 dma_addr_t *iova)
{
	struct dma_buf_attachment *attach;
	struct rk_rga_job_mapping *mappings;
	struct sg_table *sgt;
	size_t bytes;
	u32 count;
	int ret;

	if (import->dev == dev) {
		*iova = import->iova;
		return 0;
	}

	for (u32 i = 0; i < job->mapping_count; i++) {
		struct rk_rga_job_mapping *mapping = &job->mappings[i];

		if (mapping->import == import && mapping->dev == dev) {
			*iova = mapping->iova;
			return 0;
		}
	}

	if (check_add_overflow(job->mapping_count, 1U, &count))
		return -EOVERFLOW;
	bytes = array_size(count, sizeof(*mappings));
	if (bytes == SIZE_MAX)
		return -EOVERFLOW;

	mappings = krealloc(job->mappings, bytes, GFP_KERNEL);
	if (!mappings)
		return -ENOMEM;
	job->mappings = mappings;

	if (import->type == RK_RGA_IMPORT_USERPTR) {
		dma_addr_t mapped_iova;
		struct iommu_domain *domain;
		size_t iova_size;
		bool iommu_mapped;

		ret = rk_rga_map_userptr_sgt(import, dev, &sgt,
					     &mapped_iova, &domain,
					     &iova_size, &iommu_mapped);
		if (ret)
			return ret;

		job->mappings[job->mapping_count] = (struct rk_rga_job_mapping) {
			.import = import,
			.dev = get_device(dev),
			.sgt = sgt,
			.domain = domain,
			.iova = mapped_iova,
			.iova_size = iova_size,
			.page_offset = import->page_offset,
			.userptr = true,
			.iommu_mapped = iommu_mapped,
		};
		*iova = job->mappings[job->mapping_count].iova;
		job->mapping_count = count;

		return 0;
	}

	attach = dma_buf_attach(import->dmabuf, dev);
	if (IS_ERR(attach))
		return PTR_ERR(attach);

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		dma_buf_detach(import->dmabuf, attach);
		return PTR_ERR(sgt);
	}

	ret = rk_rga_check_dma_sgt(sgt, "dma-buf remap", import->size, iova,
				   true);
	if (ret) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(import->dmabuf, attach);
		return ret;
	}

	job->mappings[job->mapping_count] = (struct rk_rga_job_mapping) {
		.import = import,
		.dev = get_device(dev),
		.attach = attach,
		.sgt = sgt,
		.iova = *iova,
	};
	job->mapping_count = count;

	return 0;
}

static struct rk_rga_import *rk_rga_job_find_import_by_iova(
	struct rk_rga_job *job, dma_addr_t iova)
{
	for (u32 i = 0; i < job->import_count; i++) {
		if (job->imports[i]->iova == iova)
			return job->imports[i];
	}

	return NULL;
}

static int rk_rga_job_rebase_iova_to_hw(struct rk_rga_job *job,
					dma_addr_t old_iova,
					struct device *dev,
					dma_addr_t *new_iova)
{
	struct rk_rga_import *import;

	import = rk_rga_job_find_import_by_iova(job, old_iova);
	if (!import)
		return -EINVAL;

	return rk_rga_job_map_import(job, import, dev, new_iova);
}

static int rk_rga_job_rebase_img_to_hw(struct rk_rga_job *job,
				       struct rga_img_info_t *img,
				       struct device *dev)
{
	struct rk_rga_img_layout layout;
	struct rk_rga_import *import;
	__u64 old_y = img->yrgb_addr;
	__u64 old_uv = img->uv_addr;
	__u64 old_v = img->v_addr;
	dma_addr_t iova;
	int ret;

	for (u32 i = 0; i < job->mapping_count; i++) {
		struct rk_rga_job_mapping *mapping = &job->mappings[i];

		if (mapping->dev == dev && mapping->iova == img->yrgb_addr)
			return 0;
	}

	import = rk_rga_job_find_import_by_iova(job, img->yrgb_addr);
	if (!import) {
		if (job->import_count == 1)
			import = job->imports[0];
		else
			return -EINVAL;
	}

	ret = rk_rga_job_map_import(job, import, dev, &iova);
	if (ret)
		return ret;

	ret = rk_rga_img_layout(img, &layout);
	if (ret)
		return ret;

	img->yrgb_addr = iova;
	if (rk_rga_img_single_buffer_compressed(img)) {
		img->uv_addr = img->yrgb_addr;
		img->v_addr = 0;
		return 0;
	}
	if (!layout.uv_size) {
		img->uv_addr = 0;
		img->v_addr = 0;
		return 0;
	}

	if (!old_uv || old_uv == old_y + layout.yrgb_size) {
		ret = rk_rga_addr_add_size(img->yrgb_addr, layout.yrgb_size,
					   &img->uv_addr);
		if (ret)
			return ret;
	} else {
		ret = rk_rga_job_rebase_iova_to_hw(job, old_uv, dev, &iova);
		if (ret)
			return ret;
		img->uv_addr = iova;
	}

	if (layout.v_size) {
		if (!old_v || old_v == old_uv + layout.uv_size ||
		    old_v == old_y + layout.yrgb_size + layout.uv_size) {
			ret = rk_rga_addr_add_size(img->uv_addr,
						   layout.uv_size,
						   &img->v_addr);
			if (ret)
				return ret;
		} else {
			ret = rk_rga_job_rebase_iova_to_hw(job, old_v, dev,
							   &iova);
			if (ret)
				return ret;
			img->v_addr = iova;
		}
	} else {
		img->v_addr = 0;
	}

	return 0;
}

static void rk_rga_job_free(struct rk_rga_job *job)
{
	if (!job)
		return;

	if (job->release_fence) {
		rk_rga_fence_signal(job->release_fence,
				    RK_RGA_RELEASE_FENCE_ABORT_ERR);
		dma_fence_put(job->release_fence);
	}
	rk_rga_job_cancel_acquire_callbacks(job);
	rk_rga_job_free_cmd(job);
	rk_rga_job_clear_mappings(job);
	rk_rga_put_import_array(job->imports, job->import_count);
	rk_rga_put_fence_array(job->acquire_fences, job->acquire_fence_count);
	kfree(job->acquire_waiters);
	kfree(job->gauss_coeffs);
	kfree(job->tasks);
	kfree(job);
}

static void rk_rga_job_get(struct rk_rga_job *job)
{
	refcount_inc(&job->refs);
}

static void rk_rga_job_put(struct rk_rga_job *job)
{
	if (refcount_dec_and_test(&job->refs))
		rk_rga_job_free(job);
}

static int rk_rga_session_track_job(struct rk_rga_session *session,
				    struct rk_rga_job *job)
{
	unsigned long flags;
	int ret = 0;

	rk_rga_job_get(job);
	spin_lock_irqsave(&session->job_lock, flags);
	if (session->closing) {
		ret = -ESHUTDOWN;
	} else {
		job->session = session;
		job->session_linked = true;
		list_add_tail(&job->session_node, &session->jobs);
	}
	spin_unlock_irqrestore(&session->job_lock, flags);

	if (ret)
		rk_rga_job_put(job);

	return ret;
}

static bool rk_rga_session_begin_job_dispatch(struct rk_rga_session *session)
{
	unsigned long flags;
	bool dispatch = false;

	spin_lock_irqsave(&session->job_lock, flags);
	if (!session->closing) {
		session->dispatching_jobs++;
		dispatch = true;
	}
	spin_unlock_irqrestore(&session->job_lock, flags);

	return dispatch;
}

static void rk_rga_session_end_job_dispatch(struct rk_rga_session *session)
{
	unsigned long flags;

	spin_lock_irqsave(&session->job_lock, flags);
	WARN_ON_ONCE(!session->dispatching_jobs);
	if (session->dispatching_jobs)
		session->dispatching_jobs--;
	spin_unlock_irqrestore(&session->job_lock, flags);
	wake_up_all(&session->job_wait);
}

static void rk_rga_session_mark_closing(struct rk_rga_session *session)
{
	unsigned long flags;

	spin_lock_irqsave(&session->job_lock, flags);
	session->closing = true;
	spin_unlock_irqrestore(&session->job_lock, flags);
}

static bool rk_rga_session_dispatches_idle(struct rk_rga_session *session)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&session->job_lock, flags);
	idle = !session->dispatching_jobs;
	spin_unlock_irqrestore(&session->job_lock, flags);

	return idle;
}

static void rk_rga_job_unlink_session(struct rk_rga_job *job)
{
	struct rk_rga_session *session = job->session;
	unsigned long flags;
	bool linked = false;

	if (!session)
		return;

	spin_lock_irqsave(&session->job_lock, flags);
	if (job->session_linked) {
		list_del_init(&job->session_node);
		job->session_linked = false;
		job->session = NULL;
		linked = true;
	}
	spin_unlock_irqrestore(&session->job_lock, flags);

	if (linked) {
		wake_up_all(&session->job_wait);
		rk_rga_job_put(job);
	}
}

static void rk_rga_job_forget_release_fence_fd(struct rk_rga_job *job, int fd)
{
	if (!job || fd < 0)
		return;
	if (job->release_fence_fd == fd)
		job->release_fence_fd = -1;
}

static void rk_rga_job_install_fd(struct rk_rga_job *job,
				  struct sync_file *sync_file)
{
	int fd = job->release_fence_fd;

	rk_rga_fence_install_fd(fd, sync_file);
	rk_rga_job_forget_release_fence_fd(job, fd);
}

static void rk_rga_job_abort_fd(struct rk_rga_job *job,
				struct sync_file *sync_file)
{
	int fd = job->release_fence_fd;

	rk_rga_fence_abort_fd(fd, sync_file);
	rk_rga_job_forget_release_fence_fd(job, fd);
}

static void rk_rga_job_acquire_work(struct work_struct *work);

static void rk_rga_job_init(struct rk_rga_job *job)
{
	INIT_LIST_HEAD(&job->node);
	INIT_LIST_HEAD(&job->session_node);
	INIT_WORK(&job->acquire_work, rk_rga_job_acquire_work);
	refcount_set(&job->refs, 1);
	init_waitqueue_head(&job->wait);
	atomic_set(&job->pending_acquire_count, 0);
	atomic_set(&job->acquire_work_queued, 0);
	job->release_fence_fd = -1;
}

static u8 rk_rga_job_priority_from_tasks(const struct rga_req *tasks,
					 u32 task_count)
{
	u8 priority = 0;

	for (u32 i = 0; i < task_count; i++)
		priority = max_t(u8, priority,
				 min_t(u8, tasks[i].priority,
				       RK_RGA_SCHED_PRIORITY_MAX));

	return priority;
}

static int rk_rga_prepare_tasks_locked(struct rk_rga_session *session,
				       struct rga_req *tasks,
				       __u32 task_count,
				       __s32 acquire_fence_fd,
				       struct rk_rga_import ***imports_out,
				       u32 *import_count_out,
				       struct dma_fence ***fences_out,
				       u32 *fence_count_out,
				       struct rk_rga_acquire_fd *acquire_fds,
				       u32 *acquire_fd_count)
{
	struct dma_fence **fences;
	struct rk_rga_import **imports;
	u32 fence_count = 0;
	u32 import_count = 0;
	size_t fence_bytes;
	size_t import_bytes;
	int ret;

	import_bytes = array_size(task_count, 9 * sizeof(*imports));
	if (import_bytes == SIZE_MAX)
		return -EOVERFLOW;
	fence_bytes = array_size(task_count + 1, sizeof(*fences));
	if (fence_bytes == SIZE_MAX)
		return -EOVERFLOW;

	imports = kzalloc(import_bytes, GFP_KERNEL);
	if (!imports)
		return -ENOMEM;

	fences = kzalloc(fence_bytes, GFP_KERNEL);
	if (!fences) {
		kfree(imports);
		return -ENOMEM;
	}

	*acquire_fd_count = 0;

	ret = rk_rga_get_fence_fd(acquire_fence_fd,
				  !tasks[0].feature.user_close_fence,
				  fences, &fence_count,
				  acquire_fds, acquire_fd_count);
	if (ret)
		goto err_put_resources;

	for (u32 i = 0; i < task_count; i++) {
		struct rga_req *task = &tasks[i];

		ret = rk_rga_get_fence_fd(task->in_fence_fd,
					  !task->feature.user_close_fence,
					  fences, &fence_count,
					  acquire_fds, acquire_fd_count);
		if (ret)
			goto err_put_resources;

		if (task->handle_flag & 1)
			ret = rk_rga_resolve_task_handles_locked(session, task,
								 imports,
								 &import_count);
		else
			ret = rk_rga_resolve_task_direct_buffers(task, imports,
								&import_count);
		if (ret)
			goto err_put_resources;
	}

	*imports_out = imports;
	*import_count_out = import_count;
	*fences_out = fences;
	*fence_count_out = fence_count;

	return 0;

err_put_resources:
	rk_rga_put_import_array(imports, import_count);
	rk_rga_put_fence_array(fences, fence_count);
	rk_rga_close_kernel_acquire_fds(acquire_fds, *acquire_fd_count);

	return ret;
}

static int rk_rga_copy_gauss_coeffs(struct rga_req *tasks, u32 task_count,
				    u32 **coeffs_out)
{
	u32 *coeffs = NULL;
	int ret = 0;

	for (u32 i = 0; i < task_count; i++) {
		struct rga_req *task = &tasks[i];
		u32 user_coeffs[3];

		if (!task->gauss_config.size)
			continue;

		if (task->gauss_config.size != 3 || !task->gauss_config.coe_ptr) {
			ret = -EINVAL;
			goto err_free;
		}

		if (!coeffs) {
			coeffs = kcalloc(task_count, sizeof(*coeffs), GFP_KERNEL);
			if (!coeffs) {
				ret = -ENOMEM;
				goto err_free;
			}
		}

		if (copy_from_user(user_coeffs,
				   u64_to_user_ptr(task->gauss_config.coe_ptr),
				   sizeof(user_coeffs))) {
			ret = -EFAULT;
			goto err_free;
		}

		coeffs[i] = FIELD_PREP(RK_RGA2_GAUSS_COE0, user_coeffs[0]) |
			    FIELD_PREP(RK_RGA2_GAUSS_COE1, user_coeffs[1]) |
			    FIELD_PREP(RK_RGA2_GAUSS_COE2, user_coeffs[2]);
	}

	*coeffs_out = coeffs;

	return 0;

err_free:
	kfree(coeffs);

	return ret;
}

static int rk_rga_job_clone_request_locked(struct rk_rga_request *request,
					   struct rk_rga_job **job_out)
{
	struct dma_fence **fences = NULL;
	struct rk_rga_import **imports = NULL;
	struct rk_rga_job *job;
	size_t bytes;

	if (!request->configured)
		return -EINVAL;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;
	rk_rga_job_init(job);

	bytes = array_size(request->task_count, sizeof(*job->tasks));
	if (bytes == SIZE_MAX) {
		kfree(job);
		return -EOVERFLOW;
	}

	job->tasks = kmemdup(request->tasks, bytes, GFP_KERNEL);
	if (!job->tasks) {
		kfree(job);
		return -ENOMEM;
	}

	if (request->gauss_coeffs) {
		bytes = array_size(request->task_count,
				   sizeof(*job->gauss_coeffs));
		if (bytes == SIZE_MAX) {
			rk_rga_job_free(job);
			return -EOVERFLOW;
		}

		job->gauss_coeffs = kmemdup(request->gauss_coeffs, bytes,
					    GFP_KERNEL);
		if (!job->gauss_coeffs) {
			rk_rga_job_free(job);
			return -ENOMEM;
		}
	}

	if (request->import_count) {
		bytes = array_size(request->import_count, sizeof(*imports));
		if (bytes == SIZE_MAX) {
			rk_rga_job_free(job);
			return -EOVERFLOW;
		}

		imports = kmemdup(request->imports, bytes, GFP_KERNEL);
		if (!imports) {
			rk_rga_job_free(job);
			return -ENOMEM;
		}

		for (u32 i = 0; i < request->import_count; i++)
			refcount_inc(&imports[i]->refs);
	}

	if (request->acquire_fence_count) {
		bytes = array_size(request->acquire_fence_count,
				   sizeof(*fences));
		if (bytes == SIZE_MAX) {
			rk_rga_put_import_array(imports, request->import_count);
			rk_rga_job_free(job);
			return -EOVERFLOW;
		}

		fences = kmemdup(request->acquire_fences, bytes, GFP_KERNEL);
		if (!fences) {
			rk_rga_put_import_array(imports, request->import_count);
			rk_rga_job_free(job);
			return -ENOMEM;
		}

		for (u32 i = 0; i < request->acquire_fence_count; i++)
			dma_fence_get(fences[i]);
	}

	job->imports = imports;
	job->import_count = request->import_count;
	job->acquire_fences = fences;
	job->acquire_fence_count = request->acquire_fence_count;
	job->task_count = request->task_count;
	job->sync_mode = request->sync_mode;
	job->priority = rk_rga_job_priority_from_tasks(job->tasks, job->task_count);
	atomic_inc(&rk_rga.prepared_job_count);

	*job_out = job;

	return 0;
}

static int rk_rga_job_take_prepared(struct rga_req *tasks, u32 task_count,
				    __u32 sync_mode,
				    struct rk_rga_import **imports,
				    u32 import_count,
				    struct dma_fence **fences,
				    u32 fence_count,
				    u32 *gauss_coeffs,
				    struct rk_rga_job **job_out)
{
	struct rk_rga_job *job;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return -ENOMEM;
	rk_rga_job_init(job);

	job->tasks = tasks;
	job->task_count = task_count;
	job->sync_mode = sync_mode;
	job->imports = imports;
	job->import_count = import_count;
	job->acquire_fences = fences;
	job->acquire_fence_count = fence_count;
	job->gauss_coeffs = gauss_coeffs;
	job->priority = rk_rga_job_priority_from_tasks(job->tasks, job->task_count);
	atomic_inc(&rk_rga.prepared_job_count);

	*job_out = job;

	return 0;
}

static int rk_rga_job_acquire_status(struct rk_rga_job *job, bool *pending)
{
	for (u32 i = 0; i < job->acquire_fence_count; i++) {
		int status = dma_fence_get_status(job->acquire_fences[i]);

		if (status < 0)
			return status;
		if (!status)
			*pending = true;
	}

	return 0;
}

static int rk_rga_job_wait_acquire_fences(struct rk_rga_job *job)
{
	for (u32 i = 0; i < job->acquire_fence_count; i++) {
		struct dma_fence *fence = job->acquire_fences[i];
		int ret;

		ret = dma_fence_wait(fence, false);
		if (ret)
			return ret;

		ret = dma_fence_get_status(fence);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void rk_rga_job_set_acquire_result(struct rk_rga_job *job, int result)
{
	if (result < 0 && !READ_ONCE(job->result))
		WRITE_ONCE(job->result, result);
}

static void rk_rga_job_set_waiting_acquire(struct rk_rga_job *job,
					   bool waiting)
{
	struct rk_rga_session *session = READ_ONCE(job->session);
	unsigned long flags;

	if (!session) {
		WRITE_ONCE(job->waiting_acquire, waiting);
		return;
	}

	spin_lock_irqsave(&session->job_lock, flags);
	if (job->session == session && job->session_linked)
		job->waiting_acquire = waiting;
	spin_unlock_irqrestore(&session->job_lock, flags);
}

static void rk_rga_job_abort_acquire_state(struct rk_rga_job *job,
					   int result)
{
	struct rk_rga_session *session = READ_ONCE(job->session);
	unsigned long flags;

	if (!session) {
		if (READ_ONCE(job->waiting_acquire)) {
			rk_rga_job_set_acquire_result(job, result);
			WRITE_ONCE(job->waiting_acquire, false);
		}
		return;
	}

	spin_lock_irqsave(&session->job_lock, flags);
	if (job->session == session && job->session_linked &&
	    job->waiting_acquire) {
		rk_rga_job_set_acquire_result(job, result);
		job->waiting_acquire = false;
	}
	spin_unlock_irqrestore(&session->job_lock, flags);
}

static bool
rk_rga_session_begin_acquire_dispatch(struct rk_rga_session *session,
				      struct rk_rga_job *job)
{
	unsigned long flags;
	bool dispatch = false;

	spin_lock_irqsave(&session->job_lock, flags);
	if (job->session == session && job->session_linked &&
	    job->waiting_acquire) {
		job->waiting_acquire = false;
		if (!session->closing) {
			session->dispatching_jobs++;
			dispatch = true;
		}
	}
	spin_unlock_irqrestore(&session->job_lock, flags);

	return dispatch;
}

static void rk_rga_job_queue_acquire_work(struct rk_rga_job *job)
{
	if (atomic_cmpxchg(&job->acquire_work_queued, 0, 1) == 0)
		queue_work(system_highpri_wq, &job->acquire_work);
}

static bool rk_rga_job_is_pending_acquire(struct rk_rga_job *job)
{
	return job->waiting_acquire;
}

static void rk_rga_job_abort_pending_acquire(struct rk_rga_job *job,
					     int result)
{
	rk_rga_job_abort_acquire_state(job, result);
	rk_rga_job_cancel_acquire_callbacks(job);
	if (!atomic_read(&job->pending_acquire_count))
		rk_rga_job_queue_acquire_work(job);
}

static void rk_rga_session_abort_pending_acquire_jobs(
	struct rk_rga_session *session, int result)
{
	for (;;) {
		struct rk_rga_job *job = NULL;
		unsigned long flags;
		bool found = false;

		spin_lock_irqsave(&session->job_lock, flags);
		list_for_each_entry(job, &session->jobs, session_node) {
			if (rk_rga_job_is_pending_acquire(job)) {
				rk_rga_job_set_acquire_result(job, result);
				job->waiting_acquire = false;
				rk_rga_job_get(job);
				found = true;
				break;
			}
		}
		if (!found)
			job = NULL;
		spin_unlock_irqrestore(&session->job_lock, flags);

		if (!job)
			return;

		rk_rga_job_abort_pending_acquire(job, result);
		rk_rga_job_put(job);
	}
}

static void rk_rga_abort_all_pending_acquire_jobs(int result)
{
	struct rk_rga_session *session;

	mutex_lock(&rk_rga.session_lock);
	list_for_each_entry(session, &rk_rga.sessions, service_node)
		rk_rga_session_abort_pending_acquire_jobs(session, result);
	mutex_unlock(&rk_rga.session_lock);
}

static void rk_rga_abort_pending_if_no_hw(int result)
{
	if (!rk_rga_has_available_hw())
		rk_rga_abort_all_pending_acquire_jobs(result);
}

static void rk_rga_job_acquire_cb(struct dma_fence *fence,
				  struct dma_fence_cb *cb)
{
	struct rk_rga_fence_waiter *waiter =
		container_of(cb, struct rk_rga_fence_waiter, cb);
	struct rk_rga_job *job = xchg(&waiter->job, NULL);
	int status;

	if (!job)
		return;

	status = dma_fence_get_status_locked(fence);
	if (status < 0)
		rk_rga_job_set_acquire_result(job, status);

	if (atomic_dec_and_test(&job->pending_acquire_count))
		rk_rga_job_queue_acquire_work(job);
}

static int rk_rga_job_arm_acquire_callbacks(struct rk_rga_job *job)
{
	struct rk_rga_fence_waiter *waiters;
	int ret;

	waiters = kcalloc(job->acquire_fence_count, sizeof(*waiters),
			 GFP_KERNEL);
	if (!waiters)
		return -ENOMEM;

	job->acquire_waiters = waiters;
	atomic_set(&job->pending_acquire_count, 1);
	atomic_set(&job->acquire_work_queued, 0);
	WRITE_ONCE(job->result, 0);
	rk_rga_job_set_waiting_acquire(job, true);

	for (u32 i = 0; i < job->acquire_fence_count; i++) {
		struct dma_fence *fence = job->acquire_fences[i];
		int status;

		status = dma_fence_get_status(fence);
		if (status < 0) {
			rk_rga_job_set_acquire_result(job, status);
			continue;
		}
		if (status > 0)
			continue;

		atomic_inc(&job->pending_acquire_count);
		waiters[i].job = job;
		ret = dma_fence_add_callback(fence, &waiters[i].cb,
					     rk_rga_job_acquire_cb);
		if (ret == -ENOENT) {
			WRITE_ONCE(waiters[i].job, NULL);
			status = dma_fence_get_status(fence);
			if (status < 0)
				rk_rga_job_set_acquire_result(job, status);
			if (atomic_dec_and_test(&job->pending_acquire_count))
				rk_rga_job_queue_acquire_work(job);
		} else if (ret) {
			WRITE_ONCE(waiters[i].job, NULL);
			rk_rga_job_set_acquire_result(job, ret);
			if (atomic_dec_and_test(&job->pending_acquire_count))
				rk_rga_job_queue_acquire_work(job);
		}
	}

	if (atomic_dec_and_test(&job->pending_acquire_count))
		rk_rga_job_queue_acquire_work(job);

	return 0;
}

static int rk_rga_job_prepare_release_fence(struct rk_rga_job *job)
{
	struct dma_fence *fence;

	if (job->sync_mode != RGA_BLIT_ASYNC)
		return 0;

	fence = rk_rga_fence_alloc();
	if (IS_ERR(fence))
		return PTR_ERR(fence);

	job->release_fence = fence;

	return 0;
}

static void rk_rga_job_complete(struct rk_rga_job *job, int result)
{
	struct rk_rga_hw *hw = job->hw;

	rk_rga_job_note_hw_done(job);
	rk_rga_job_record_hw_stats(job);
	rk_rga_job_sync_userptr_for_cpu(job);
	WRITE_ONCE(job->result, result);
	/* Publish the result before waking synchronous waiters. */
	smp_store_release(&job->done, true);
	job->hw = NULL;
	rk_rga_fence_signal(job->release_fence, result);
	wake_up_all(&job->wait);
	atomic_inc(&rk_rga.completed_job_count);
	if (hw)
		rk_rga_hw_put(hw);
	rk_rga_job_unlink_session(job);
}

static void rk_rga_job_complete_queued(struct rk_rga_job *job, int result)
{
	rk_rga_job_complete(job, result);
	rk_rga_job_put(job);
}

static bool rk_rga_job_advance_task(struct rk_rga_job *job, int result)
{
	if (result)
		return false;
	if (job->current_task + 1 >= job->task_count)
		return false;

	job->current_task++;
	return true;
}

static void rk_rga_job_release_hw(struct rk_rga_job *job)
{
	struct rk_rga_hw *hw = job->hw;

	if (!hw)
		return;

	job->hw = NULL;
	rk_rga_hw_put(hw);
}

static void rk_rga_hw_dispatch(struct rk_rga_hw *hw);
static struct rk_rga_job *rk_rga_hw_take_active(struct rk_rga_hw *hw);
static void rk_rga_hw_timeout_work(struct work_struct *work);
static void rk_rga_hw_iommu_fault_work(struct work_struct *work);
static bool rk_rga_hw_iommu_fault_matches_locked(struct rk_rga_hw *hw);
static void rk_rga_hw_abort_jobs(struct rk_rga_hw *hw, int result);
static int rk_rga_job_queue_ref(struct rk_rga_job *job, bool take_ref);
static int rk_rga_job_queue_on_hw(struct rk_rga_job *job, struct rk_rga_hw *hw,
				  bool take_ref);

static inline u32 rk_rga_read(struct rk_rga_hw *hw, u32 offset)
{
	return readl(hw->regs + offset);
}

static inline void rk_rga_write(struct rk_rga_hw *hw, u32 value, u32 offset)
{
	writel(value, hw->regs + offset);
}

static u32 rk_rga2_csc_coeff(__s16 coeff)
{
	return (u32)(__u16)coeff;
}

static void rk_rga2_write_full_csc(struct rk_rga_hw *hw,
				   const struct rga_req *task)
{
	struct rga_csc_clip clip = {
		.y = { .max = 0xff, .min = 0 },
		.uv = { .max = 0xff, .min = 0 },
	};
	const struct rga_full_csc *csc = &task->full_csc;

	if (!(csc->flag & RK_RGA_FULL_CSC_ENABLE))
		return;

	if (task->feature.full_csc_clip_en)
		clip = task->full_csc_clip;

	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_y.r_v) |
		     ((u32)clip.y.max << 16) | ((u32)clip.y.min << 24),
		     RK_RGA2_DST_CSC_00_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_y.g_y) |
		     ((u32)clip.uv.max << 16) | ((u32)clip.uv.min << 24),
		     RK_RGA2_DST_CSC_01_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_y.b_u),
		     RK_RGA2_DST_CSC_02_OFFSET);
	rk_rga_write(hw, (__u32)csc->coe_y.off, RK_RGA2_DST_CSC_OFF0_OFFSET);

	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_u.r_v),
		     RK_RGA2_DST_CSC_10_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_u.g_y),
		     RK_RGA2_DST_CSC_11_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_u.b_u),
		     RK_RGA2_DST_CSC_12_OFFSET);
	rk_rga_write(hw, (__u32)csc->coe_u.off, RK_RGA2_DST_CSC_OFF1_OFFSET);

	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_v.r_v),
		     RK_RGA2_DST_CSC_20_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_v.g_y),
		     RK_RGA2_DST_CSC_21_OFFSET);
	rk_rga_write(hw, rk_rga2_csc_coeff(csc->coe_v.b_u),
		     RK_RGA2_DST_CSC_22_OFFSET);
	rk_rga_write(hw, (__u32)csc->coe_v.off, RK_RGA2_DST_CSC_OFF2_OFFSET);
}

static void rk_rga2_read_irq_status(struct rk_rga_hw *hw,
				    struct rk_rga_job *job)
{
	job->intr_status = rk_rga_read(hw, RK_RGA2_INT);
	job->hw_status = rk_rga_read(hw, RK_RGA2_STATUS2);
	job->cmd_status = rk_rga_read(hw, RK_RGA2_STATUS1);
	job->work_cycle = rk_rga_read(hw, RK_RGA2_WORK_CNT);
}

static void rk_rga3_read_irq_status(struct rk_rga_hw *hw,
				    struct rk_rga_job *job)
{
	job->intr_status = rk_rga_read(hw, RK_RGA3_INT_RAW);
	job->hw_status = rk_rga_read(hw, RK_RGA3_STATUS0);
	job->cmd_status = rk_rga_read(hw, RK_RGA3_CMD_STATE);
	job->work_cycle = 0;
}

static void rk_rga2_clear_irq(struct rk_rga_hw *hw)
{
	rk_rga_write(hw, rk_rga_read(hw, RK_RGA2_INT) | RK_RGA2_INT_CLEAR_MASK,
		     RK_RGA2_INT);
}

static void rk_rga3_clear_irq(struct rk_rga_hw *hw)
{
	rk_rga_write(hw, RK_RGA3_INT_DONE_MASK | RK_RGA3_INT_ERROR_MASK,
		     RK_RGA3_INT_CLR);
}

static int rk_rga2_soft_reset(struct rk_rga_hw *hw)
{
	u32 i;

	rk_rga_write(hw, RK_RGA2_SYS_CTRL_ACLK_SRESET |
		     RK_RGA2_SYS_CTRL_CCLK_SRESET |
		     RK_RGA2_SYS_CTRL_RST_PROTECT, RK_RGA2_SYS_CTRL);

	for (i = 0; i < RK_RGA_RESET_TIMEOUT_US; i++) {
		if (!(rk_rga_read(hw, RK_RGA2_SYS_CTRL) &
		      RK_RGA2_SYS_CTRL_CMD_OP_ST))
			return 0;

		udelay(1);
	}

	return -ETIMEDOUT;
}

static int rk_rga3_soft_reset(struct rk_rga_hw *hw)
{
	u32 i;

	rk_rga_write(hw, RK_RGA3_SYS_CTRL_CCLK_SRESET |
		     RK_RGA3_SYS_CTRL_ACLK_SRESET, RK_RGA3_SYS_CTRL);

	for (i = 0; i < RK_RGA_RESET_TIMEOUT_US; i++) {
		if (rk_rga_read(hw, RK_RGA3_RO_SRST) &
		    RK_RGA3_RO_SRST_RST_DONE)
			break;

		udelay(1);
	}

	rk_rga_write(hw, 0, RK_RGA3_SYS_CTRL);

	return i == RK_RGA_RESET_TIMEOUT_US ? -ETIMEDOUT : 0;
}

static void rk_rga_hw_refresh_iommu(struct rk_rga_hw *hw)
{
	if (!hw->iommu_domain)
		return;

	iommu_flush_iotlb_all(hw->iommu_domain);
	atomic_inc(&rk_rga.iommu_refresh_count);
}

static void rk_rga_hw_mark_recovery_failed(struct rk_rga_hw *hw, int error)
{
	unsigned long flags;
	bool newly_failed = false;

	spin_lock_irqsave(&hw->job_lock, flags);
	if (!hw->recovery_failed) {
		hw->recovery_failed = true;
		newly_failed = true;
	}
	spin_unlock_irqrestore(&hw->job_lock, flags);

	if (!newly_failed)
		return;

	atomic_inc(&rk_rga.recovery_failure_count);
	dev_err(hw->dev,
		"%s core %d quarantined after recovery reset failure: %d\n",
		hw->match->name, hw->index, error);
}

static int rk_rga_hw_reset_for_recovery(struct rk_rga_hw *hw)
{
	int ret;

	if (hw->type == RK_RGA_HW_RGA3)
		ret = rk_rga3_soft_reset(hw);
	else
		ret = rk_rga2_soft_reset(hw);

	if (!ret) {
		rk_rga_hw_refresh_iommu(hw);
		return 0;
	}

	dev_warn(hw->dev, "%s soft reset during recovery failed: %d\n",
		 hw->match->name, ret);

	if (!hw->resets)
		goto failed;

	ret = rk_rga_hw_reset_controls(hw);
	if (ret) {
		dev_warn(hw->dev,
			 "%s reset-control recovery fallback failed: %d\n",
			 hw->match->name, ret);
		goto failed;
	}

	rk_rga_hw_refresh_iommu(hw);

	return 0;

failed:
	rk_rga_hw_mark_recovery_failed(hw, ret);

	return ret;
}

static void rk_rga2_start_hw(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	const struct rga_pre_intr_info *intr =
		&job->tasks[job->current_task].pre_intr_info;
	u32 sys_ctrl = RK_RGA2_SYS_CTRL_AUTO_CKG |
		       RK_RGA2_SYS_CTRL_AUTO_RST |
		       RK_RGA2_SYS_CTRL_CMD_MODE;
	u32 int_enable = RK_RGA2_INT_ENABLE_MASK |
			 rk_rga2_pre_intr_int_enable(intr);

	rk_rga2_clear_irq(hw);
	rk_rga2_write_full_csc(hw, &job->tasks[job->current_task]);
	if (intr->enable) {
		u32 read_line = rk_rga2_pre_intr_read_line(intr);
		u32 write_line = rk_rga2_pre_intr_write_line(intr);

		if (intr->read_intr_en)
			rk_rga_write(hw, read_line, RK_RGA2_READ_LINE_CNT);
		if (intr->write_intr_en)
			rk_rga_write(hw, write_line, RK_RGA2_WRITE_LINE_CNT);
		sys_ctrl |= rk_rga2_pre_intr_sys_ctrl(intr);
	}
	rk_rga_write(hw, rk_rga_read(hw, RK_RGA2_INT) |
		     int_enable, RK_RGA2_INT);
	rk_rga_write(hw, lower_32_bits(job->cmd_dma), RK_RGA2_CMD_BASE);
	rk_rga_write(hw, sys_ctrl, RK_RGA2_SYS_CTRL);
	rk_rga_write(hw, rk_rga_read(hw, RK_RGA2_CMD_CTRL) |
		     RK_RGA2_CMD_CTRL_CMD_LINE_ST, RK_RGA2_CMD_CTRL);
}

static void rk_rga3_start_hw(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	rk_rga3_clear_irq(hw);
	rk_rga_write(hw, RK_RGA3_INT_DONE_MASK | RK_RGA3_INT_ERROR_MASK,
		     RK_RGA3_INT_EN);
	rk_rga_write(hw, lower_32_bits(job->cmd_dma), RK_RGA3_CMD_ADDR);
	rk_rga_write(hw, RK_RGA3_SYS_CTRL_CMD_MODE, RK_RGA3_SYS_CTRL);
	rk_rga_write(hw, RK_RGA3_CMD_CTRL_LINE_START, RK_RGA3_CMD_CTRL);
}

static struct rk_rga_job *rk_rga_hw_take_timeout_job(struct rk_rga_hw *hw)
{
	struct rk_rga_job *job;
	unsigned long flags;

	spin_lock_irqsave(&hw->job_lock, flags);
	job = hw->timeout_job;
	hw->timeout_job = NULL;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	return job;
}

static void rk_rga_hw_cancel_timeout(struct rk_rga_hw *hw)
{
	struct rk_rga_job *job;

	cancel_delayed_work(&hw->timeout_work);
	job = rk_rga_hw_take_timeout_job(hw);
	rk_rga_job_put(job);
}

static void rk_rga_hw_cancel_timeout_sync(struct rk_rga_hw *hw)
{
	struct rk_rga_job *job;

	cancel_delayed_work_sync(&hw->timeout_work);
	job = rk_rga_hw_take_timeout_job(hw);
	rk_rga_job_put(job);
}

static void rk_rga_hw_schedule_timeout(struct rk_rga_hw *hw,
				       struct rk_rga_job *job)
{
	struct rk_rga_job *old = NULL;
	unsigned long flags;

	spin_lock_irqsave(&hw->job_lock, flags);
	if (job != hw->timeout_job) {
		rk_rga_job_get(job);
		old = hw->timeout_job;
		hw->timeout_job = job;
	}
	spin_unlock_irqrestore(&hw->job_lock, flags);
	rk_rga_job_put(old);

	mod_delayed_work(system_wq, &hw->timeout_work,
			 msecs_to_jiffies(RK_RGA_JOB_TIMEOUT_MS));
}

static void rk_rga_hw_start(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	job->irq_result = 0;
	job->irq_seen = false;
	job->hw_start_ns = ktime_get_ns();

	if (hw->type == RK_RGA_HW_RGA3)
		rk_rga3_start_hw(hw, job);
	else
		rk_rga2_start_hw(hw, job);

	atomic_inc(&rk_rga.started_job_count);
	rk_rga_count_core(rk_rga.started_core_count, hw);
	rk_rga_hw_schedule_timeout(hw, job);
}

static int rk_rga2_irq_result(const struct rk_rga_job *job)
{
	if (job->intr_status & RK_RGA2_INT_ERROR_INT_FLAG)
		return -EFAULT;
	if (job->intr_status & RK_RGA2_INT_MMU_INT_FLAG)
		return -EACCES;
	if (job->intr_status & RK_RGA2_INT_SCL_ERROR_INTR)
		return -EACCES;
	if (job->intr_status & RK_RGA2_INT_FBCIN_DEC_ERROR)
		return -EACCES;
	if (job->hw_status & (RK_RGA2_STATUS2_RPP_ERROR |
			      RK_RGA2_STATUS2_BUS_ERROR))
		return -EFAULT;

	return -EFAULT;
}

static int rk_rga3_irq_result(const struct rk_rga_job *job)
{
	if (job->intr_status & RK_RGA3_INT_RGA_MMU_INTR)
		return -EACCES;
	if (job->intr_status & (RK_RGA3_INT_RGA_MI_RD_BUS_ERR |
				RK_RGA3_INT_RGA_MI_WR_BUS_ERR |
				RK_RGA3_INT_WIN0_FBCD_DEC_ERR |
				RK_RGA3_INT_WIN1_FBCD_DEC_ERR))
		return -EFAULT;

	return -EFAULT;
}

static int rk_rga_irq_completion_result(enum rk_rga_hw_type type,
					const struct rk_rga_job *job)
{
	if (type == RK_RGA_HW_RGA3)
		return job->intr_status & RK_RGA3_INT_ERROR_MASK ?
		       rk_rga3_irq_result(job) : 0;
	if (type == RK_RGA_HW_RGA2)
		return job->intr_status & RK_RGA2_INT_ERROR_MASK ?
		       rk_rga2_irq_result(job) : 0;

	return -EINVAL;
}

static irqreturn_t rk_rga_hw_irq_status(struct rk_rga_hw *hw,
					struct rk_rga_job *job)
{
	bool done;
	bool error;

	if (hw->type == RK_RGA_HW_RGA3) {
		rk_rga3_read_irq_status(hw, job);
		done = job->intr_status & RK_RGA3_INT_DONE_MASK;
		error = job->intr_status & RK_RGA3_INT_ERROR_MASK;
		if (!done && !error)
			return IRQ_NONE;
		job->irq_seen = true;
		rk_rga3_clear_irq(hw);
	} else {
		rk_rga2_read_irq_status(hw, job);
		done = job->intr_status & RK_RGA2_INT_DONE_MASK;
		error = job->intr_status & RK_RGA2_INT_ERROR_MASK;
		if (!done && !error) {
			if (job->intr_status & RK_RGA2_INT_LINE_MASK) {
				rk_rga2_clear_irq(hw);
				return IRQ_HANDLED;
			}
			return IRQ_NONE;
		}
		job->irq_seen = true;
		rk_rga2_clear_irq(hw);
	}
	job->irq_result = rk_rga_irq_completion_result(hw->type, job);

	if (job->irq_result)
		atomic_inc(&rk_rga.irq_error_count);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rk_rga_hw_clear_spurious_irq(struct rk_rga_hw *hw)
{
	u32 intr;

	if (hw->type == RK_RGA_HW_RGA3) {
		intr = rk_rga_read(hw, RK_RGA3_INT_RAW);
		if (!(intr & (RK_RGA3_INT_DONE_MASK | RK_RGA3_INT_ERROR_MASK)))
			return IRQ_NONE;
		rk_rga3_clear_irq(hw);
	} else {
		intr = rk_rga_read(hw, RK_RGA2_INT);
		if (!(intr & (RK_RGA2_INT_DONE_MASK | RK_RGA2_INT_ERROR_MASK |
			      RK_RGA2_INT_LINE_MASK)))
			return IRQ_NONE;
		rk_rga2_clear_irq(hw);
	}

	atomic_inc(&rk_rga.irq_spurious_count);

	return IRQ_HANDLED;
}

struct rk_rga3_format_info {
	u8 pic_format;
	u8 bus_format;
	u8 pixel_width;
	u8 pix_swap;
	u8 yc_swap;
	bool rgb;
	bool yuv;
	bool alpha;
	bool yuv_sp;
	bool yuv420_sp;
	bool yuv10;
};

enum rk_rga_alpha_blend_mode {
	RK_RGA_ALPHA_NONE = 0,
	RK_RGA_ALPHA_BLEND_SRC,
	RK_RGA_ALPHA_BLEND_DST,
	RK_RGA_ALPHA_BLEND_SRC_OVER,
	RK_RGA_ALPHA_BLEND_DST_OVER,
	RK_RGA_ALPHA_BLEND_SRC_IN,
	RK_RGA_ALPHA_BLEND_DST_IN,
	RK_RGA_ALPHA_BLEND_SRC_OUT,
	RK_RGA_ALPHA_BLEND_DST_OUT,
	RK_RGA_ALPHA_BLEND_SRC_ATOP,
	RK_RGA_ALPHA_BLEND_DST_ATOP,
	RK_RGA_ALPHA_BLEND_XOR,
	RK_RGA_ALPHA_BLEND_CLEAR,
};

struct rk_rga3_bitblt_profile {
	struct rk_rga3_format_info src_fmt;
	struct rk_rga3_format_info bg_fmt;
	struct rk_rga3_format_info dst_fmt;
	u32 src_mode;
	u32 bg_mode;
	u32 dst_mode;
	u32 rotate_flags;
	bool alpha_blend;
	bool pattern_blend;
	bool color_key;
	bool overlap_copy;
};

struct rk_rga2_format_info {
	u8 hw_format;
	u8 pixel_width;
	u8 plane_width;
	u8 x_div;
	u8 y_div;
	bool rb_swap;
	bool alpha_swap;
	bool uv_swap;
	bool yuv;
	bool yuv400;
	bool yuv10;
	bool alpha;
	bool y4;
	bool y4_lut;
	bool packed_yuv420;
	bool packed_yuv422;
	bool planar_420;
};

struct rk_rga2_transform {
	u8 src_rot_mode;
	u8 src_mir_mode;
	u16 dst_act_w;
	u16 dst_act_h;
	bool rot_90;
	bool x_mirror;
	bool y_mirror;
};

struct rk_rga2_bitblt_profile {
	struct rk_rga2_format_info src_fmt;
	struct rk_rga2_format_info pat_fmt;
	struct rk_rga2_format_info dst_fmt;
	struct rk_rga2_transform transform;
	bool alpha_bitmap;
	bool color_key;
	bool osd;
};

struct rk_rga2_fill_profile {
	struct rk_rga2_format_info dst_fmt;
};

struct rk_rga2_palette_profile {
	struct rk_rga2_format_info dst_fmt;
};

static bool rk_rga_format_is_yuv(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCBCR_422_P:
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCBCR_420_P:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_P:
	case RK_RGA_FORMAT_YCRCB_420_SP:
	case RK_RGA_FORMAT_YCRCB_420_P:
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
	case RK_RGA_FORMAT_YVYU_422:
	case RK_RGA_FORMAT_YVYU_420:
	case RK_RGA_FORMAT_VYUY_422:
	case RK_RGA_FORMAT_VYUY_420:
	case RK_RGA_FORMAT_YUYV_422:
	case RK_RGA_FORMAT_YUYV_420:
	case RK_RGA_FORMAT_UYVY_422:
	case RK_RGA_FORMAT_UYVY_420:
	case RK_RGA_FORMAT_YCBCR_444_SP:
	case RK_RGA_FORMAT_YCRCB_444_SP:
	case RK_RGA_FORMAT_YCBCR_400:
		return true;
	default:
		return false;
	}
}

static bool rk_rga_format_is_yuv10(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		return true;
	default:
		return false;
	}
}

static bool rk_rga_format_is_rga3_yuv422_rotate_blocked(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
	case RK_RGA_FORMAT_YVYU_422:
	case RK_RGA_FORMAT_VYUY_422:
	case RK_RGA_FORMAT_YUYV_422:
	case RK_RGA_FORMAT_UYVY_422:
		return true;
	default:
		return false;
	}
}

static bool rk_rga_format_is_alpha(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
		return true;
	default:
		return false;
	}
}

static bool rk_rga2_format_is_alpha(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_RGBA_5551:
	case RK_RGA_FORMAT_BGRA_5551:
	case RK_RGA_FORMAT_ARGB_5551:
	case RK_RGA_FORMAT_ABGR_5551:
	case RK_RGA_FORMAT_RGBA_4444:
	case RK_RGA_FORMAT_BGRA_4444:
	case RK_RGA_FORMAT_ARGB_4444:
	case RK_RGA_FORMAT_ABGR_4444:
		return true;
	default:
		return false;
	}
}

static int rk_rga3_format_info(u32 format, bool write,
			       struct rk_rga3_format_info *info)
{
	memset(info, 0, sizeof(*info));
	info->bus_format = 1;
	info->pixel_width = 1;
	info->yuv = rk_rga_format_is_yuv(format);
	info->rgb = !info->yuv;
	info->alpha = rk_rga_format_is_alpha(format);
	info->yuv10 = rk_rga_format_is_yuv10(format);

	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
		info->pic_format = write ? 0x6 : 0x8;
		info->pixel_width = 4;
		info->bus_format = 2;
		info->pix_swap = write ? 1 : 0;
		break;
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_BGRX_8888:
		info->pic_format = 0x6;
		info->pixel_width = 4;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_XRGB_8888:
		if (write)
			return -EOPNOTSUPP;
		info->pic_format = 0x9;
		info->pixel_width = 4;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XBGR_8888:
		if (write)
			return -EOPNOTSUPP;
		info->pic_format = 0x7;
		info->pixel_width = 4;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_RGB_888:
		info->pic_format = 0x5;
		info->pixel_width = 3;
		info->bus_format = 2;
		info->pix_swap = 1;
		break;
	case RK_RGA_FORMAT_BGR_888:
		info->pic_format = 0x5;
		info->pixel_width = 3;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_RGB_565:
		info->pic_format = 0x4;
		info->pixel_width = 2;
		info->bus_format = 2;
		info->pix_swap = 1;
		break;
	case RK_RGA_FORMAT_BGR_565:
		info->pic_format = 0x4;
		info->pixel_width = 2;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_YVYU_422:
		info->pic_format = 0x1;
		info->pixel_width = 2;
		info->bus_format = 2;
		info->pix_swap = 1;
		info->yc_swap = 1;
		break;
	case RK_RGA_FORMAT_VYUY_422:
		info->pic_format = 0x1;
		info->pixel_width = 2;
		info->bus_format = 2;
		info->pix_swap = 1;
		break;
	case RK_RGA_FORMAT_YUYV_422:
		info->pic_format = 0x1;
		info->pixel_width = 2;
		info->bus_format = 2;
		info->yc_swap = 1;
		break;
	case RK_RGA_FORMAT_UYVY_422:
		info->pic_format = 0x1;
		info->pixel_width = 2;
		info->bus_format = 2;
		break;
	case RK_RGA_FORMAT_YCBCR_422_SP:
		info->pic_format = 0x1;
		info->yuv_sp = true;
		break;
	case RK_RGA_FORMAT_YCBCR_420_SP:
		info->pic_format = 0x0;
		info->yuv_sp = true;
		info->yuv420_sp = true;
		break;
	case RK_RGA_FORMAT_YCRCB_422_SP:
		info->pic_format = 0x1;
		info->pix_swap = 1;
		info->yuv_sp = true;
		break;
	case RK_RGA_FORMAT_YCRCB_420_SP:
		info->pic_format = 0x0;
		info->pix_swap = 1;
		info->yuv_sp = true;
		info->yuv420_sp = true;
		break;
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
		info->pic_format = 0x2;
		info->yuv_sp = true;
		info->yuv420_sp = true;
		break;
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
		info->pic_format = 0x2;
		info->pix_swap = 1;
		info->yuv_sp = true;
		info->yuv420_sp = true;
		break;
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
		info->pic_format = 0x3;
		info->yuv_sp = true;
		break;
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		info->pic_format = 0x3;
		info->pix_swap = 1;
		info->yuv_sp = true;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static bool rk_rga_img_yuv10_compact(const struct rga_img_info_t *img)
{
	return img->compact_mode != RK_RGA_10BIT_INCOMPACT;
}

static bool rk_rga3_fbc_format_supported(u32 format, bool write)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_BGRX_8888:
	case RK_RGA_FORMAT_RGB_888:
	case RK_RGA_FORMAT_BGR_888:
	case RK_RGA_FORMAT_RGB_565:
	case RK_RGA_FORMAT_BGR_565:
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCRCB_420_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		return true;
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_XRGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XBGR_8888:
		return !write;
	default:
		return false;
	}
}

static bool rk_rga3_tile_format_supported(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCRCB_420_SP:
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		return true;
	default:
		return false;
	}
}

static int rk_rga2_format_info(u32 format, bool write,
			       struct rk_rga2_format_info *info)
{
	memset(info, 0, sizeof(*info));
	info->pixel_width = 1;
	info->x_div = 1;
	info->y_div = 1;
	info->yuv = rk_rga_format_is_yuv(format);
	info->alpha = rk_rga2_format_is_alpha(format);

	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
		info->hw_format = 0x0;
		info->pixel_width = 4;
		return 0;
	case RK_RGA_FORMAT_BGRA_8888:
		info->hw_format = 0x0;
		info->pixel_width = 4;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBX_8888:
		info->hw_format = 0x1;
		info->pixel_width = 4;
		return 0;
	case RK_RGA_FORMAT_BGRX_8888:
		info->hw_format = 0x1;
		info->pixel_width = 4;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGB_888:
		info->hw_format = 0x2;
		info->pixel_width = 3;
		return 0;
	case RK_RGA_FORMAT_BGR_888:
		info->hw_format = 0x2;
		info->pixel_width = 3;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGB_565:
		info->hw_format = 0x4;
		info->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGR_565:
		info->hw_format = 0x4;
		info->pixel_width = 2;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBA_5551:
		info->hw_format = 0x5;
		info->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGRA_5551:
		info->hw_format = 0x5;
		info->pixel_width = 2;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBA_4444:
		info->hw_format = 0x6;
		info->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGRA_4444:
		info->hw_format = 0x6;
		info->pixel_width = 2;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_8888:
		info->hw_format = 0x0;
		info->pixel_width = 4;
		info->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ABGR_8888:
		info->hw_format = 0x0;
		info->pixel_width = 4;
		info->alpha_swap = true;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_XRGB_8888:
		info->hw_format = 0x1;
		info->pixel_width = 4;
		info->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_XBGR_8888:
		info->hw_format = 0x1;
		info->pixel_width = 4;
		info->alpha_swap = true;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_5551:
		info->hw_format = 0x5;
		info->pixel_width = 2;
		info->alpha_swap = write;
		return 0;
	case RK_RGA_FORMAT_ABGR_5551:
		info->hw_format = 0x5;
		info->pixel_width = 2;
		info->alpha_swap = write;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_4444:
		info->hw_format = 0x6;
		info->pixel_width = 2;
		info->alpha_swap = write;
		return 0;
	case RK_RGA_FORMAT_ABGR_4444:
		info->hw_format = 0x6;
		info->pixel_width = 2;
		info->alpha_swap = write;
		info->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_YVYU_422:
		info->hw_format = write ? 0xe : 0x7;
		info->pixel_width = 2;
		info->uv_swap = !write;
		info->rb_swap = true;
		info->packed_yuv422 = true;
		return 0;
	case RK_RGA_FORMAT_VYUY_422:
		info->hw_format = write ? 0xc : 0x7;
		info->pixel_width = 2;
		info->uv_swap = !write;
		info->packed_yuv422 = true;
		return 0;
	case RK_RGA_FORMAT_YUYV_422:
		info->hw_format = write ? 0xe : 0x7;
		info->pixel_width = 2;
		info->uv_swap = write;
		info->rb_swap = !write;
		info->packed_yuv422 = true;
		return 0;
	case RK_RGA_FORMAT_UYVY_422:
		info->hw_format = write ? 0xc : 0x7;
		info->pixel_width = 2;
		info->uv_swap = write;
		info->packed_yuv422 = true;
		return 0;
	case RK_RGA_FORMAT_YVYU_420:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0xf;
		info->pixel_width = 2;
		info->packed_yuv420 = true;
		return 0;
	case RK_RGA_FORMAT_VYUY_420:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0xd;
		info->pixel_width = 2;
		info->packed_yuv420 = true;
		return 0;
	case RK_RGA_FORMAT_YUYV_420:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0xf;
		info->pixel_width = 2;
		info->uv_swap = true;
		info->packed_yuv420 = true;
		return 0;
	case RK_RGA_FORMAT_UYVY_420:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0xd;
		info->pixel_width = 2;
		info->uv_swap = true;
		info->packed_yuv420 = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_422_SP:
		info->hw_format = 0x8;
		info->plane_width = 2;
		info->x_div = 2;
		return 0;
	case RK_RGA_FORMAT_YCRCB_422_SP:
		info->hw_format = 0x8;
		info->plane_width = 2;
		info->x_div = 2;
		info->uv_swap = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_422_P:
		info->hw_format = 0x9;
		info->plane_width = 1;
		info->x_div = 2;
		return 0;
	case RK_RGA_FORMAT_YCRCB_422_P:
		info->hw_format = 0x9;
		info->plane_width = 1;
		info->x_div = 2;
		info->uv_swap = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_420_SP:
		info->hw_format = 0xa;
		info->plane_width = 2;
		info->x_div = 2;
		info->y_div = 2;
		return 0;
	case RK_RGA_FORMAT_YCRCB_420_SP:
		info->hw_format = 0xa;
		info->plane_width = 2;
		info->x_div = 2;
		info->y_div = 2;
		info->uv_swap = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
		if (write)
			return -EOPNOTSUPP;
		info->hw_format = 0xa;
		info->plane_width = 2;
		info->x_div = 2;
		info->y_div = 2;
		info->yuv10 = true;
		return 0;
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
		if (write)
			return -EOPNOTSUPP;
		info->hw_format = 0xa;
		info->plane_width = 2;
		info->x_div = 2;
		info->y_div = 2;
		info->uv_swap = true;
		info->yuv10 = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
		if (write)
			return -EOPNOTSUPP;
		info->hw_format = 0x8;
		info->plane_width = 2;
		info->x_div = 2;
		info->yuv10 = true;
		return 0;
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
		if (write)
			return -EOPNOTSUPP;
		info->hw_format = 0x8;
		info->plane_width = 2;
		info->x_div = 2;
		info->uv_swap = true;
		info->yuv10 = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_420_P:
		info->hw_format = 0xb;
		info->plane_width = 1;
		info->x_div = 2;
		info->y_div = 2;
		info->uv_swap = write;
		info->planar_420 = true;
		return 0;
	case RK_RGA_FORMAT_YCRCB_420_P:
		info->hw_format = 0xb;
		info->plane_width = 1;
		info->x_div = 2;
		info->y_div = 2;
		info->uv_swap = !write;
		info->planar_420 = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_444_SP:
		info->hw_format = 0x3;
		info->plane_width = 2;
		return 0;
	case RK_RGA_FORMAT_YCRCB_444_SP:
		info->hw_format = 0x3;
		info->plane_width = 2;
		info->uv_swap = true;
		return 0;
	case RK_RGA_FORMAT_Y4:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0x8;
		info->yuv400 = true;
		info->y4 = true;
		info->y4_lut = true;
		return 0;
	case RK_RGA_FORMAT_YCBCR_400:
		info->hw_format = 0x8;
		info->yuv400 = true;
		return 0;
	case RK_RGA_FORMAT_Y8:
		if (!write)
			return -EOPNOTSUPP;
		info->hw_format = 0x8;
		info->yuv400 = true;
		info->y4_lut = true;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rk_rga2_fill_format_info(u32 format,
				    struct rk_rga2_fill_profile *profile)
{
	memset(profile, 0, sizeof(*profile));

	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
	case RK_RGA_FORMAT_BGRX_8888:
	case RK_RGA_FORMAT_RGB_888:
	case RK_RGA_FORMAT_BGR_888:
	case RK_RGA_FORMAT_RGB_565:
	case RK_RGA_FORMAT_BGR_565:
	case RK_RGA_FORMAT_RGBA_5551:
	case RK_RGA_FORMAT_BGRA_5551:
	case RK_RGA_FORMAT_RGBA_4444:
	case RK_RGA_FORMAT_BGRA_4444:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XRGB_8888:
	case RK_RGA_FORMAT_XBGR_8888:
	case RK_RGA_FORMAT_ARGB_5551:
	case RK_RGA_FORMAT_ABGR_5551:
	case RK_RGA_FORMAT_ARGB_4444:
	case RK_RGA_FORMAT_ABGR_4444:
	case RK_RGA_FORMAT_YVYU_422:
	case RK_RGA_FORMAT_YVYU_420:
	case RK_RGA_FORMAT_VYUY_422:
	case RK_RGA_FORMAT_VYUY_420:
	case RK_RGA_FORMAT_YUYV_422:
	case RK_RGA_FORMAT_YUYV_420:
	case RK_RGA_FORMAT_UYVY_422:
	case RK_RGA_FORMAT_UYVY_420:
	case RK_RGA_FORMAT_YCBCR_422_SP:
	case RK_RGA_FORMAT_YCRCB_422_SP:
	case RK_RGA_FORMAT_YCBCR_422_P:
	case RK_RGA_FORMAT_YCRCB_422_P:
	case RK_RGA_FORMAT_YCBCR_420_SP:
	case RK_RGA_FORMAT_YCRCB_420_SP:
	case RK_RGA_FORMAT_YCBCR_420_P:
	case RK_RGA_FORMAT_YCRCB_420_P:
	case RK_RGA_FORMAT_YCBCR_444_SP:
	case RK_RGA_FORMAT_YCRCB_444_SP:
	case RK_RGA_FORMAT_YCBCR_400:
		return rk_rga2_format_info(format, true, &profile->dst_fmt);
	default:
		return -EOPNOTSUPP;
	}

	return rk_rga2_format_info(format, true, &profile->dst_fmt);
}

static int rk_rga3_hw_rd_mode(u32 user_mode, u32 *hw_mode)
{
	if (!user_mode || user_mode == RK_RGA_RASTER_MODE) {
		*hw_mode = 0;
		return 0;
	}
	if (user_mode == RK_RGA_FBC_MODE) {
		*hw_mode = 1;
		return 0;
	}
	if (user_mode == RK_RGA_TILE_MODE) {
		*hw_mode = 2;
		return 0;
	}

	return -EOPNOTSUPP;
}

static int rk_rga3_pack_pair(u32 low, u32 high, u32 *value)
{
	if (low > RK_RGA3_SIZE_MASK || high > RK_RGA3_SIZE_MASK)
		return -EINVAL;

	*value = low | (high << 16);

	return 0;
}

static int rk_rga3_stride(u32 width, u8 pixel_width, u32 *stride)
{
	u32 bytes;

	if (check_mul_overflow(width, (u32)pixel_width, &bytes))
		return -EOVERFLOW;

	*stride = ALIGN(bytes, 16) >> 2;

	return 0;
}

static int rk_rga3_tile_stride(u32 width, u8 pixel_width, u32 *stride)
{
	u32 bytes;

	if (check_mul_overflow(width, (u32)pixel_width * 8, &bytes))
		return -EOVERFLOW;

	*stride = ALIGN(bytes, 16) >> 2;

	return 0;
}

static int rk_rga3_read_strides(const struct rga_img_info_t *img,
				const struct rk_rga3_format_info *fmt,
				u32 rd_mode, u32 *stride, u32 *uv_stride)
{
	int ret;

	if (rd_mode == 1) {
		*stride = ALIGN((u32)img->vir_w, 16) >> 2;
		*uv_stride = *stride;
		return 0;
	}
	if (rd_mode == 2) {
		ret = rk_rga3_tile_stride(img->vir_w, fmt->pixel_width,
					  stride);
		if (ret)
			return ret;
		if (fmt->yuv420_sp)
			*uv_stride = ALIGN((u32)img->vir_w * 8, 16) >> 3;
		else
			*uv_stride = *stride;
		return 0;
	}

	if (rd_mode)
		return -EOPNOTSUPP;

	ret = rk_rga3_stride(img->vir_w, fmt->pixel_width, stride);
	if (ret)
		return ret;

	*uv_stride = fmt->yuv420_sp ? ALIGN((u32)img->vir_w, 16) >> 2 :
		     *stride;

	return 0;
}

static int rk_rga3_scale_axis(u32 src, u32 dst, u32 *factor, bool *up,
			      bool *bypass)
{
	u64 scaled;

	if (!src || !dst)
		return -EINVAL;

	if (src == dst) {
		*factor = 0;
		*up = false;
		*bypass = true;
		return 0;
	}

	if (src == 1 || dst == 1)
		return -EOPNOTSUPP;

	*bypass = false;
	if (src < dst) {
		u64 rem;

		scaled = (u64)RK_RGA3_FACTOR_MAX * (src - 1);
		*factor = div64_u64_rem(scaled, dst - 1, &rem);
		if (!rem)
			(*factor)--;
		*up = true;
	} else {
		scaled = (u64)RK_RGA3_FACTOR_MAX * (dst - 1);
		*factor = div_u64(scaled, src - 1) + 1;
		*up = false;
	}

	return 0;
}

static u32 rk_rga3_y2r_mode(u8 yuv2rgb_mode)
{
	switch (yuv2rgb_mode) {
	case 1:
		return 0;
	case 2:
		return 2;
	case 3:
		return 1;
	default:
		return 0;
	}
}

static u32 rk_rga3_r2y_mode(u8 yuv2rgb_mode)
{
	switch (yuv2rgb_mode >> 2) {
	case 2:
		return 0;
	case 1:
		return 2;
	case 3:
		return 1;
	default:
		return 0;
	}
}

static int rk_rga3_rotate_flags(const struct rga_req *task, u32 *rotate_flags)
{
	u32 flags = 0;

	if (task->src.rotate_mode || task->dst.rotate_mode)
		return -EOPNOTSUPP;

	switch (task->rotate_mode & 0x0f) {
	case 0:
		break;
	case 1:
		if (task->sina == 65536 && task->cosa == 0) {
			flags = RK_RGA3_ROT_BIT_ROT_90;
		} else if (task->sina == 0 && task->cosa == -65536) {
			flags = RK_RGA3_ROT_BIT_X_MIRROR |
				RK_RGA3_ROT_BIT_Y_MIRROR;
		} else if (task->sina == -65536 && task->cosa == 0) {
			flags = RK_RGA3_ROT_BIT_ROT_90 |
				RK_RGA3_ROT_BIT_X_MIRROR |
				RK_RGA3_ROT_BIT_Y_MIRROR;
		} else if (task->sina == 0 && task->cosa == 65536) {
			flags = 0;
		} else {
			flags = 0;
		}
		break;
	case 2:
		flags = RK_RGA3_ROT_BIT_X_MIRROR;
		break;
	case 3:
		flags = RK_RGA3_ROT_BIT_Y_MIRROR;
		break;
	case 4:
		flags = RK_RGA3_ROT_BIT_X_MIRROR |
			RK_RGA3_ROT_BIT_Y_MIRROR;
		break;
	default:
		flags = 0;
		break;
	}

	switch ((task->rotate_mode & 0xf0) >> 4) {
	case 0:
		break;
	case 2:
		flags ^= RK_RGA3_ROT_BIT_X_MIRROR;
		break;
	case 3:
		flags ^= RK_RGA3_ROT_BIT_Y_MIRROR;
		break;
	case 4:
		flags ^= RK_RGA3_ROT_BIT_X_MIRROR |
			 RK_RGA3_ROT_BIT_Y_MIRROR;
		break;
	default:
		break;
	}

	*rotate_flags = flags;
	return 0;
}

static void rk_rga_cmd_write(struct rk_rga_job *job, u32 offset, u32 value)
{
	u32 *cmd = job->cmd_vaddr;

	cmd[offset / sizeof(*cmd)] = value;
}

static int rk_rga3_validate_image(const struct rga_img_info_t *img)
{
	u32 width;
	u32 height;

	if (!img->act_w || !img->act_h || !img->vir_w || !img->vir_h)
		return -EINVAL;

	if (check_add_overflow((u32)img->x_offset, (u32)img->act_w, &width) ||
	    check_add_overflow((u32)img->y_offset, (u32)img->act_h, &height))
		return -EOVERFLOW;

	if (width > img->vir_w || height > img->vir_h)
		return -EINVAL;
	if (width > RK_RGA3_SIZE_MASK || height > RK_RGA3_SIZE_MASK)
		return -EINVAL;
	if (ALIGN(width, 16) > RK_RGA3_SIZE_MASK ||
	    ALIGN(height, 16) > RK_RGA3_SIZE_MASK)
		return -EINVAL;

	return 0;
}

static bool rk_rga_img_has_addr(const struct rga_img_info_t *img)
{
	return img->yrgb_addr || img->uv_addr || img->v_addr;
}

static bool rk_rga3_task_uses_alpha_blend(const struct rga_req *task)
{
	return task->alpha_rop_flag & BIT(0);
}

static bool rk_rga3_task_uses_color_key(const struct rga_req *task)
{
	return task->color_key_min || task->color_key_max;
}

static int rk_rga3_validate_alpha_blend(const struct rga_req *task)
{
	if (!rk_rga3_task_uses_alpha_blend(task))
		return 0;
	if (!(task->alpha_rop_flag & BIT(3)))
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag & ~RK_RGA3_ALPHA_SUPPORTED_FLAGS)
		return -EOPNOTSUPP;

	switch (task->PD_mode) {
	case RK_RGA_ALPHA_BLEND_SRC:
	case RK_RGA_ALPHA_BLEND_DST:
	case RK_RGA_ALPHA_BLEND_SRC_OVER:
	case RK_RGA_ALPHA_BLEND_DST_OVER:
	case RK_RGA_ALPHA_BLEND_SRC_IN:
	case RK_RGA_ALPHA_BLEND_DST_IN:
	case RK_RGA_ALPHA_BLEND_SRC_OUT:
	case RK_RGA_ALPHA_BLEND_DST_OUT:
	case RK_RGA_ALPHA_BLEND_SRC_ATOP:
	case RK_RGA_ALPHA_BLEND_DST_ATOP:
	case RK_RGA_ALPHA_BLEND_XOR:
	case RK_RGA_ALPHA_BLEND_CLEAR:
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rk_rga3_validate_color_key(const struct rga_req *task, bool has_pat)
{
	if (!rk_rga3_task_uses_color_key(task))
		return 0;
	if (has_pat)
		return -EOPNOTSUPP;
	if (!rk_rga3_task_uses_alpha_blend(task))
		return -EOPNOTSUPP;
	if (task->src_trans_mode != 0x1e && task->src_trans_mode != 0x1f)
		return -EOPNOTSUPP;
	if (task->alpha_rop_mode != 0x11)
		return -EOPNOTSUPP;
	if (task->PD_mode != RK_RGA_ALPHA_BLEND_SRC)
		return -EOPNOTSUPP;

	return 0;
}

static int rk_rga2_validate_full_csc(const struct rga_req *task)
{
	if (!task->full_csc.flag)
		return 0;
	if (task->full_csc.flag & ~RK_RGA_FULL_CSC_ENABLE)
		return -EOPNOTSUPP;
	if (!task->feature.full_csc_clip_en)
		return 0;
	if (task->full_csc_clip.y.max > 0xff ||
	    task->full_csc_clip.y.min > 0xff ||
	    task->full_csc_clip.uv.max > 0xff ||
	    task->full_csc_clip.uv.min > 0xff)
		return -EINVAL;

	return 0;
}

static int rk_rga2_decode_transform(const struct rga_req *task,
				    struct rk_rga2_transform *transform)
{
	u8 rot = 0;
	u8 mir = 0;
	bool base_mirror;

	memset(transform, 0, sizeof(*transform));

	if (task->src.rotate_mode || task->dst.rotate_mode)
		return -EOPNOTSUPP;

	switch (task->rotate_mode & 0x0f) {
	case 0:
		break;
	case 1:
		if (task->sina == 0 && task->cosa == 65536) {
			rot = 0;
		} else if (task->sina == 65536 && task->cosa == 0) {
			rot = 1;
		} else if (task->sina == 0 && task->cosa == -65536) {
			rot = 2;
		} else if (task->sina == -65536 && task->cosa == 0) {
			rot = 3;
		} else {
			rot = 0;
		}
		break;
	case 2:
		mir |= 1;
		break;
	case 3:
		mir |= 2;
		break;
	case 4:
		mir |= 3;
		break;
	default:
		break;
	}

	switch ((task->rotate_mode & 0xf0) >> 4) {
	case 0:
		break;
	case 2:
		mir |= 1;
		break;
	case 3:
		mir |= 2;
		break;
	case 4:
		mir |= 3;
		break;
	default:
		break;
	}

	transform->src_rot_mode = rot;
	transform->src_mir_mode = mir;
	transform->rot_90 = rot & 1;
	transform->dst_act_w = transform->rot_90 ? task->dst.act_h :
			       task->dst.act_w;
	transform->dst_act_h = transform->rot_90 ? task->dst.act_w :
			       task->dst.act_h;
	if (!transform->dst_act_w || !transform->dst_act_h)
		return -EINVAL;

	base_mirror = rot > 1;
	transform->x_mirror = (base_mirror + (mir & 1)) & 1;
	transform->y_mirror = (base_mirror + ((mir >> 1) & 1)) & 1;

	return 0;
}

static void rk_rga2_normalized_dst(const struct rga_req *task,
				   const struct rk_rga2_transform *transform,
				   struct rga_img_info_t *dst)
{
	*dst = task->dst;
	dst->act_w = transform->dst_act_w;
	dst->act_h = transform->dst_act_h;
}

static bool rk_rga_img_rects_disjoint(const struct rga_img_info_t *a,
				      const struct rga_img_info_t *b)
{
	u32 a_right;
	u32 a_bottom;
	u32 b_right;
	u32 b_bottom;

	if (check_add_overflow((u32)a->x_offset, (u32)a->act_w, &a_right) ||
	    check_add_overflow((u32)a->y_offset, (u32)a->act_h, &a_bottom) ||
	    check_add_overflow((u32)b->x_offset, (u32)b->act_w, &b_right) ||
	    check_add_overflow((u32)b->y_offset, (u32)b->act_h, &b_bottom))
		return false;

	return a_right <= b->x_offset || b_right <= a->x_offset ||
	       a_bottom <= b->y_offset || b_bottom <= a->y_offset;
}

static bool rk_rga_mirror_only_rotate_mode(u16 rotate_mode)
{
	u16 low = rotate_mode & 0x0f;
	u16 high = (rotate_mode >> 4) & 0x0f;

	if (rotate_mode & ~0xff)
		return false;

	switch (low) {
	case 0:
	case 2:
	case 3:
	case 4:
		break;
	default:
		return false;
	}

	switch (high) {
	case 0:
	case 2:
	case 3:
	case 4:
		return true;
	default:
		return false;
	}
}

static bool rk_rga_in_place_bitblt_allowed(const struct rga_req *task)
{
	if (task->src.yrgb_addr != task->dst.yrgb_addr)
		return true;
	if (!task->src.yrgb_addr)
		return false;
	if (task->src.uv_addr != task->dst.uv_addr ||
	    task->src.v_addr != task->dst.v_addr)
		return false;
	if (task->src.format != task->dst.format)
		return false;
	if (task->src.rd_mode != task->dst.rd_mode)
		return false;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return false;
	if (task->src.vir_w != task->dst.vir_w ||
	    task->src.vir_h != task->dst.vir_h)
		return false;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return false;
	if (task->src.rotate_mode || task->dst.rotate_mode)
		return false;
	if (!rk_rga_mirror_only_rotate_mode(task->rotate_mode))
		return false;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->feature.global_alpha_en)
		return false;
	if (task->bsfilter_flag || rk_rga_img_has_addr(&task->pat))
		return false;
	if (task->full_csc.flag || task->yuv2rgb_mode)
		return false;

	return rk_rga_img_rects_disjoint(&task->src, &task->dst);
}

static bool rk_rga2_in_place_mosaic_allowed(const struct rga_req *task)
{
	if (!task->mosaic_info.enable)
		return false;
	if (task->mosaic_info.mode > 4)
		return false;
	if (!task->src.yrgb_addr || task->src.yrgb_addr != task->dst.yrgb_addr)
		return false;
	if (task->src.uv_addr != task->dst.uv_addr ||
	    task->src.v_addr != task->dst.v_addr)
		return false;
	if (task->src.format != task->dst.format)
		return false;
	if (task->src.rd_mode != task->dst.rd_mode)
		return false;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return false;
	if (task->src.vir_w != task->dst.vir_w ||
	    task->src.vir_h != task->dst.vir_h)
		return false;
	if (task->src.x_offset != task->dst.x_offset ||
	    task->src.y_offset != task->dst.y_offset)
		return false;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return false;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return false;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return false;

	return true;
}

static bool rk_rga2_task_uses_y4_lut_dst(const struct rga_req *task)
{
	return task->dst.format == RK_RGA_FORMAT_Y4 ||
	       task->dst.format == RK_RGA_FORMAT_Y8;
}

static bool rk_rga2_dither_flags_allowed(const struct rga_req *task)
{
	u16 flags = task->alpha_rop_flag;

	if (!rk_rga2_task_uses_y4_lut_dst(task))
		return !flags;
	if (flags & ~(RK_RGA2_ALPHA_FLAG_ENABLE |
		      RK_RGA2_ALPHA_FLAG_DST_DITHER_DOWN))
		return false;

	return !(flags & RK_RGA2_ALPHA_FLAG_DST_DITHER_DOWN) ||
	       (flags & RK_RGA2_ALPHA_FLAG_ENABLE);
}

static bool rk_rga2_task_uses_rop(const struct rga_req *task)
{
	return task->rop_code ||
	       (!rk_rga2_dither_flags_allowed(task) &&
		(task->alpha_rop_flag & ~BIT(8))) ||
	       task->alpha_rop_mode;
}

static bool rk_rga2_task_uses_color_key(const struct rga_req *task)
{
	return task->color_key_min || task->color_key_max;
}

static bool rk_rga2_task_uses_quantize(const struct rga_req *task)
{
	return task->alpha_rop_flag & BIT(8);
}

static bool rk_rga2_task_uses_alpha_bitmap(const struct rga_req *task)
{
	return task->rgba5551_alpha.flags & BIT(0);
}

static bool rk_rga2_alpha_bitmap_format(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_5551:
	case RK_RGA_FORMAT_BGRA_5551:
	case RK_RGA_FORMAT_ARGB_5551:
	case RK_RGA_FORMAT_ABGR_5551:
		return true;
	default:
		return false;
	}
}

static bool rk_rga2_rop_bitblt_allowed(const struct rga_req *task)
{
	if (task->src.yrgb_addr != task->dst.yrgb_addr)
		return true;
	if (!task->src.yrgb_addr)
		return false;
	if (task->src.uv_addr != task->dst.uv_addr ||
	    task->src.v_addr != task->dst.v_addr)
		return false;
	if (task->src.format != task->dst.format)
		return false;
	if (task->src.rd_mode != task->dst.rd_mode)
		return false;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return false;
	if (task->src.vir_w != task->dst.vir_w ||
	    task->src.vir_h != task->dst.vir_h)
		return false;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return false;

	return rk_rga_img_rects_disjoint(&task->src, &task->dst);
}

static int rk_rga2_rop_ctrl(u16 rop_code, u32 *rop_ctrl)
{
	switch (rop_code) {
	case RK_RGA_ROP_AND:
		*rop_ctrl = 0x00000070;
		return 0;
	case RK_RGA_ROP_OR:
		*rop_ctrl = 0x00000050;
		return 0;
	case RK_RGA_ROP_NOT_DST:
		*rop_ctrl = 0x00800004;
		return 0;
	case RK_RGA_ROP_NOT_SRC:
		*rop_ctrl = 0x00800006;
		return 0;
	case RK_RGA_ROP_XOR:
		*rop_ctrl = 0x008004b0;
		return 0;
	case RK_RGA_ROP_NOT_XOR:
		*rop_ctrl = 0x00004830;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int rk_rga2_validate_rop(const struct rga_req *task)
{
	u32 rop_ctrl;

	if (!rk_rga2_task_uses_rop(task))
		return 0;
	if (task->alpha_rop_flag != (BIT(0) | BIT(1)))
		return -EOPNOTSUPP;
	if (task->alpha_rop_mode != 0x1)
		return -EOPNOTSUPP;
	if (task->PD_mode || task->feature.global_alpha_en)
		return -EOPNOTSUPP;
	if (task->src.format != task->dst.format)
		return -EOPNOTSUPP;
	if ((task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->osd_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;

	return rk_rga2_rop_ctrl(task->rop_code, &rop_ctrl);
}

static int rk_rga2_validate_color_key(const struct rga_req *task)
{
	u16 supported_flags = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_PD_ENABLE |
			      RK_RGA2_ALPHA_FLAG_CAL_MODE |
			      RK_RGA2_ALPHA_FLAG_REAL_COLOR;

	if (!rk_rga2_task_uses_color_key(task))
		return 0;
	if (task->alpha_rop_flag != supported_flags)
		return -EOPNOTSUPP;
	if (task->alpha_rop_mode != 0x11)
		return -EOPNOTSUPP;
	if (task->PD_mode != RK_RGA_ALPHA_BLEND_SRC)
		return -EOPNOTSUPP;
	if (task->src_trans_mode != 0x1e && task->src_trans_mode != 0x1f)
		return -EOPNOTSUPP;
	if (task->feature.global_alpha_en &&
	    (task->fg_global_alpha != 0xff || task->bg_global_alpha != 0xff))
		return -EOPNOTSUPP;
	if (task->rop_code || task->rgba5551_alpha.flags ||
	    task->mosaic_info.enable || task->osd_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if ((task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;

	return 0;
}

static bool rk_rga2_task_uses_gauss(const struct rga_req *task)
{
	return task->gauss_config.size;
}

static int rk_rga2_validate_quantize_value(__s16 value, bool scale)
{
	if (scale && value < 0)
		return -EINVAL;
	if (value < -255 || value > 0x3ff)
		return -EINVAL;

	return 0;
}

static int rk_rga2_validate_quantize(const struct rga_req *task)
{
	if (!rk_rga2_task_uses_quantize(task))
		return 0;
	if (task->alpha_rop_flag != BIT(8))
		return -EOPNOTSUPP;
	if (task->PD_mode || task->feature.global_alpha_en ||
	    task->rop_code || task->alpha_rop_mode)
		return -EOPNOTSUPP;
	if (task->src.format != task->dst.format)
		return -EOPNOTSUPP;
	if ((task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->interp.horiz || task->interp.verti)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->osd_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if (rk_rga2_validate_quantize_value(task->gr_color.gr_x_r, true) ||
	    rk_rga2_validate_quantize_value(task->gr_color.gr_x_g, true) ||
	    rk_rga2_validate_quantize_value(task->gr_color.gr_x_b, true) ||
	    rk_rga2_validate_quantize_value(task->gr_color.gr_y_r, false) ||
	    rk_rga2_validate_quantize_value(task->gr_color.gr_y_g, false) ||
	    rk_rga2_validate_quantize_value(task->gr_color.gr_y_b, false))
		return -EINVAL;

	return 0;
}

static int rk_rga2_validate_alpha_bitmap(const struct rga_req *task)
{
	u16 supported_flags = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_PD_ENABLE |
			      RK_RGA2_ALPHA_FLAG_CAL_MODE |
			      RK_RGA2_ALPHA_FLAG_REAL_COLOR;

	if (!rk_rga2_task_uses_alpha_bitmap(task))
		return 0;
	if (task->rgba5551_alpha.flags != 1)
		return -EOPNOTSUPP;
	if (!rk_rga_img_has_addr(&task->pat))
		return -EINVAL;
	if (task->bsfilter_flag != 1)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag != supported_flags)
		return -EOPNOTSUPP;
	if (task->alpha_rop_mode != 0x1)
		return -EOPNOTSUPP;
	if (task->PD_mode != RK_RGA_ALPHA_BLEND_DST_OVER)
		return -EOPNOTSUPP;
	if (task->feature.global_alpha_en &&
	    (task->fg_global_alpha != 0xff || task->bg_global_alpha != 0xff))
		return -EOPNOTSUPP;
	if (task->rop_code || task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->src.yrgb_addr == task->dst.yrgb_addr)
		return -EOPNOTSUPP;
	if (task->src.format != task->dst.format)
		return -EOPNOTSUPP;
	if ((task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->pat.rd_mode && task->pat.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h ||
	    task->pat.act_w != task->dst.act_w ||
	    task->pat.act_h != task->dst.act_h)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->pat.rotate_mode || task->rotate_mode ||
	    task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->interp.horiz || task->interp.verti)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->osd_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if (!rk_rga2_alpha_bitmap_format(task->pat.format))
		return -EOPNOTSUPP;

	return 0;
}

static bool rk_rga2_task_uses_osd(const struct rga_req *task)
{
	return task->osd_info.enable;
}

static bool rk_rga2_osd_format(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
		return true;
	default:
		return false;
	}
}

static bool rk_rga2_osd_in_place_allowed(const struct rga_req *task)
{
	if (!task->src.yrgb_addr || task->src.yrgb_addr != task->dst.yrgb_addr)
		return false;
	if (task->src.uv_addr != task->dst.uv_addr ||
	    task->src.v_addr != task->dst.v_addr)
		return false;
	if (task->src.format != task->dst.format)
		return false;
	if (task->src.rd_mode != task->dst.rd_mode)
		return false;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return false;
	if (task->src.vir_w != task->dst.vir_w ||
	    task->src.vir_h != task->dst.vir_h)
		return false;
	if (task->src.x_offset != task->dst.x_offset ||
	    task->src.y_offset != task->dst.y_offset)
		return false;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return false;

	return true;
}

static int rk_rga2_validate_osd_info(const struct rga_osd_info *osd)
{
	const struct rga_osd_mode_ctrl *mode = &osd->mode_ctrl;
	u16 fix_width;

	if (!osd->enable)
		return -EOPNOTSUPP;
	if (mode->mode > 3 || mode->direction_mode > 1 ||
	    mode->width_mode != 0 || mode->color_mode > 1 ||
	    mode->invert_flags_mode > 1 || mode->default_color_sel > 1 ||
	    mode->invert_enable > 7 || mode->invert_mode > 1)
		return -EOPNOTSUPP;
	if (!mode->block_num || mode->block_num > 32)
		return -EINVAL;
	if (mode->block_fix_width < 2 || mode->block_fix_width % 2)
		return -EINVAL;

	fix_width = mode->block_fix_width / 2 - 1;
	if (fix_width > 0x3ff || mode->flags_index > 0x3ff ||
	    mode->unfix_index > 0xf)
		return -EINVAL;

	return 0;
}

static int rk_rga2_validate_osd(const struct rga_req *task)
{
	if (!rk_rga2_task_uses_osd(task))
		return 0;
	if (!rk_rga_img_has_addr(&task->pat))
		return -EINVAL;
	if (task->bsfilter_flag != 1)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag != (RK_RGA2_ALPHA_FLAG_ENABLE |
				     RK_RGA2_ALPHA_FLAG_PD_ENABLE |
				     RK_RGA2_ALPHA_FLAG_CAL_MODE))
		return -EOPNOTSUPP;
	if (task->alpha_rop_mode != 0x1)
		return -EOPNOTSUPP;
	if (task->PD_mode != RK_RGA_ALPHA_BLEND_DST_OVER)
		return -EOPNOTSUPP;
	if (!task->feature.global_alpha_en ||
	    task->fg_global_alpha != 0xff || task->bg_global_alpha != 0xff)
		return -EOPNOTSUPP;
	if (task->rop_code || task->color_key_min || task->color_key_max ||
	    task->rgba5551_alpha.flags)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->pat.rotate_mode || task->rotate_mode ||
	    task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->interp.horiz || task->interp.verti)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->gauss_config.size)
		return -EOPNOTSUPP;
	if (!rk_rga2_osd_in_place_allowed(task))
		return -EOPNOTSUPP;
	if ((task->pat.rd_mode && task->pat.rd_mode != RK_RGA_RASTER_MODE) ||
	    task->pat.x_offset || task->pat.y_offset)
		return -EOPNOTSUPP;
	if (task->pat.act_w != task->src.act_w ||
	    task->pat.act_h != task->src.act_h)
		return -EOPNOTSUPP;
	if (!rk_rga2_osd_format(task->src.format) ||
	    !rk_rga2_osd_format(task->pat.format))
		return -EOPNOTSUPP;

	return rk_rga2_validate_osd_info(&task->osd_info);
}

static int rk_rga2_validate_gauss(const struct rga_req *task)
{
	if (!rk_rga2_task_uses_gauss(task))
		return 0;
	if (task->gauss_config.size != 3 || !task->gauss_config.coe_ptr)
		return -EINVAL;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->rop_code || task->alpha_rop_mode)
		return -EOPNOTSUPP;
	if (task->src.format != task->dst.format)
		return -EOPNOTSUPP;
	if ((task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE) ||
	    (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->interp.horiz || task->interp.verti)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->osd_info.enable)
		return -EOPNOTSUPP;

	return 0;
}

#if IS_ENABLED(CONFIG_ROCKCHIP_RGA_REWRITE_KUNIT_TEST)
static int rk_rga2_select_dst_addresses(const struct rga_img_info_t *dst,
					const struct rk_rga2_format_info *fmt,
					const struct rk_rga2_transform *transform,
					u32 stride, u32 uv_stride,
					__u64 y_lt, __u64 u_lt, __u64 v_lt,
					__u64 *y_addr, __u64 *u_addr,
					__u64 *v_addr);
static int __maybe_unused rk_rga_job_hw_type(struct rk_rga_job *job,
					     enum rk_rga_hw_type *type);
static int rk_rga_job_hw_type_mask(struct rk_rga_job *job, u32 *type_mask);
static int rk_rga_job_emit_cmd(struct rk_rga_hw *hw, struct rk_rga_job *job);
static int rk_rga2_emit_simple_bitblt(struct rk_rga_job *job);
static int rk_rga2_emit_color_fill(struct rk_rga_job *job);
static int rk_rga2_emit_color_palette(struct rk_rga_job *job);
static int rk_rga2_emit_update_palette(struct rk_rga_job *job);
static u32 rk_rga2_alpha_bitmap_ctrl1(void);
static u32 rk_rga2_osd_alpha_ctrl1(void);
static u32 rk_rga2_pack_nn_quantize(__s16 r, __s16 g, __s16 b);
static int rk_rga3_emit_simple_bitblt(struct rk_rga_job *job);
static int rk_rga3_alpha_factors(u8 pd_mode, u32 *top_factor,
				 u32 *bottom_factor);
static int rk_rga3_validate_bitblt(const struct rga_req *task,
				   struct rk_rga3_bitblt_profile *profile);
static int rk_rga_request_check(const struct rga_user_request *user);
static int rk_rga_request_ioctl_ret(int ret);
static int rk_rga_request_config(struct rk_rga_session *session,
				 const struct rga_user_request *user,
				 struct rk_rga_job **job_out);
static int rk_rga_import_buffer_size(const struct rga_external_buffer *buffer,
				     size_t *size);
static int rk_rga_import_one(struct rk_rga_session *session,
			     struct rga_external_buffer *buffer);
static struct rk_rga_hw *
rk_rga_find_best_hw_for_job(struct list_head *hw_list, struct rk_rga_job *job,
			    u32 type_mask, u32 rr_start);
static u32 rk_rga_find_free_core_mask(struct list_head *hw_list,
				      enum rk_rga_hw_type type);
static void rk_rga_hw_enqueue_job_locked(struct rk_rga_hw *hw,
					 struct rk_rga_job *job);
static bool rk_rga_hw_abort_session_jobs(struct rk_rga_hw *hw,
					 struct rk_rga_session *session,
					 int result);
static struct rk_rga_hw *
rk_rga_iommu_find_fault_hw(struct list_head *fault_hws,
			   struct iommu_domain *domain,
			   struct device *iommu_dev);

static void rk_rga2_transform_expect(struct kunit *test, u8 rotate_mode,
				     s32 sina, s32 cosa, u8 rot, u8 mir,
				     bool x_mirror, bool y_mirror,
				     u16 dst_w, u16 dst_h)
{
	struct rga_req task = {
		.rotate_mode = rotate_mode,
		.sina = sina,
		.cosa = cosa,
		.dst = {
			.act_w = 640,
			.act_h = 480,
		},
	};
	struct rk_rga2_transform transform;

	KUNIT_EXPECT_EQ(test, rk_rga2_decode_transform(&task, &transform), 0);
	KUNIT_EXPECT_EQ(test, transform.src_rot_mode, rot);
	KUNIT_EXPECT_EQ(test, transform.src_mir_mode, mir);
	KUNIT_EXPECT_EQ(test, transform.rot_90, !!(rot & 1));
	KUNIT_EXPECT_EQ(test, transform.x_mirror, x_mirror);
	KUNIT_EXPECT_EQ(test, transform.y_mirror, y_mirror);
	KUNIT_EXPECT_EQ(test, transform.dst_act_w, dst_w);
	KUNIT_EXPECT_EQ(test, transform.dst_act_h, dst_h);
}

static void rk_rga2_decode_transform_kunit(struct kunit *test)
{
	struct rk_rga2_transform transform;
	struct rga_req bad = {
		.rotate_mode = 1,
		.sina = 1,
		.cosa = 1,
		.dst = {
			.act_w = 640,
			.act_h = 480,
		},
	};

	rk_rga2_transform_expect(test, 0, 0, 65536, 0, 0, false, false,
				 640, 480);
	rk_rga2_transform_expect(test, 1, 0, 65536, 0, 0, false, false,
				 640, 480);
	rk_rga2_transform_expect(test, 1, 65536, 0, 1, 0, false, false,
				 480, 640);
	rk_rga2_transform_expect(test, 1, 0, -65536, 2, 0, true, true,
				 640, 480);
	rk_rga2_transform_expect(test, 1, -65536, 0, 3, 0, true, true,
				 480, 640);
	rk_rga2_transform_expect(test, 2, 0, 0, 0, 1, true, false,
				 640, 480);
	rk_rga2_transform_expect(test, 3, 0, 0, 0, 2, false, true,
				 640, 480);
	rk_rga2_transform_expect(test, 4, 0, 0, 0, 3, true, true,
				 640, 480);
	rk_rga2_transform_expect(test, 1 | (2 << 4), -65536, 0, 3, 1,
				 false, true, 480, 640);

	KUNIT_EXPECT_EQ(test, rk_rga2_decode_transform(&bad, &transform), 0);
	KUNIT_EXPECT_EQ(test, transform.src_rot_mode, 0);
	KUNIT_EXPECT_EQ(test, transform.src_mir_mode, 0);
	KUNIT_EXPECT_FALSE(test, transform.rot_90);
	KUNIT_EXPECT_EQ(test, transform.dst_act_w, 640);
	KUNIT_EXPECT_EQ(test, transform.dst_act_h, 480);
	bad.sina = 0;
	bad.cosa = 65536;
	bad.src.rotate_mode = 1;
	KUNIT_EXPECT_EQ(test, rk_rga2_decode_transform(&bad, &transform),
			-EOPNOTSUPP);
}

static void rk_rga3_rotate_flags_kunit(struct kunit *test)
{
	struct rga_req task = {};
	u32 flags = U32_MAX;

	task.rotate_mode = 1;
	task.sina = 65536;
	task.cosa = 0;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags, RK_RGA3_ROT_BIT_ROT_90);

	task.rotate_mode = 1;
	task.sina = 0;
	task.cosa = -65536;
	flags = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags,
			RK_RGA3_ROT_BIT_X_MIRROR |
			RK_RGA3_ROT_BIT_Y_MIRROR);

	task.rotate_mode = 1;
	task.sina = -65536;
	task.cosa = 0;
	flags = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags,
			RK_RGA3_ROT_BIT_ROT_90 |
			RK_RGA3_ROT_BIT_X_MIRROR |
			RK_RGA3_ROT_BIT_Y_MIRROR);

	task.rotate_mode = 1 | (2 << 4);
	task.sina = -65536;
	task.cosa = 0;
	flags = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags,
			RK_RGA3_ROT_BIT_ROT_90 | RK_RGA3_ROT_BIT_Y_MIRROR);

	task.rotate_mode = 0x5;
	task.sina = 0;
	task.cosa = 0;
	flags = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags, 0);

	task.rotate_mode = 0xf0;
	flags = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags), 0);
	KUNIT_EXPECT_EQ(test, flags, 0);

	task.src.rotate_mode = 1;
	KUNIT_EXPECT_EQ(test, rk_rga3_rotate_flags(&task, &flags),
			-EOPNOTSUPP);
}

static void rk_rga2_dst_corner_expect(struct kunit *test,
				      const struct rk_rga2_transform *transform,
				      u64 expected)
{
	struct rga_img_info_t dst = {
		.yrgb_addr = 0x1000,
		.act_w = 4,
		.act_h = 3,
	};
	struct rk_rga2_format_info fmt = {
		.pixel_width = 4,
		.x_div = 1,
		.y_div = 1,
	};
	u64 y_addr;
	u64 u_addr;
	u64 v_addr;

	KUNIT_EXPECT_EQ(test,
			rk_rga2_select_dst_addresses(&dst, &fmt, transform,
						     32, 0, dst.yrgb_addr,
						     0, 0, &y_addr,
						     &u_addr, &v_addr),
			0);
	KUNIT_EXPECT_EQ(test, y_addr, expected);
	KUNIT_EXPECT_EQ(test, u_addr, 0);
	KUNIT_EXPECT_EQ(test, v_addr, 0);
}

static void rk_rga2_dst_corner_kunit(struct kunit *test)
{
	struct rk_rga2_transform identity = { };
	struct rk_rga2_transform x_mirror = {
		.src_mir_mode = 1,
		.x_mirror = true,
	};
	struct rk_rga2_transform y_mirror = {
		.src_mir_mode = 2,
		.y_mirror = true,
	};
	struct rk_rga2_transform rot_180 = {
		.src_rot_mode = 2,
		.x_mirror = true,
		.y_mirror = true,
	};
	struct rk_rga2_transform rot_90 = {
		.src_rot_mode = 1,
		.rot_90 = true,
	};
	struct rk_rga2_transform rot_270 = {
		.src_rot_mode = 3,
		.rot_90 = true,
		.x_mirror = true,
		.y_mirror = true,
	};

	rk_rga2_dst_corner_expect(test, &identity, 0x1000);
	rk_rga2_dst_corner_expect(test, &x_mirror, 0x100c);
	rk_rga2_dst_corner_expect(test, &y_mirror, 0x1040);
	rk_rga2_dst_corner_expect(test, &rot_180, 0x104c);
	rk_rga2_dst_corner_expect(test, &rot_90, 0x100c);
	rk_rga2_dst_corner_expect(test, &rot_270, 0x1040);
}

static struct rga_req rk_rga_fill_task(u32 core)
{
	return (struct rga_req) {
		.render_mode = RK_RGA_RENDER_COLOR_FILL,
		.core = core,
		.dst = {
			.act_w = 64,
			.act_h = 32,
			.vir_w = 64,
			.vir_h = 32,
			.format = RK_RGA_FORMAT_RGBA_8888,
		},
	};
}

static DEFINE_SPINLOCK(rk_rga_kunit_fence_lock);

static long rk_rga_ioctl_get_version(unsigned long arg);
static long rk_rga_ioctl_get_rga2_version(unsigned long arg);
static long rk_rga_ioctl_get_hw_versions(unsigned long arg);
static long rk_rga_ioctl_get_driver_version(unsigned long arg);
static long rk_rga_ioctl_blit(unsigned long arg, struct rk_rga_session *session,
			      __u32 sync_mode);
static long rk_rga_ioctl_request_create(unsigned long arg,
					struct rk_rga_session *session);
static long rk_rga_ioctl_request_cancel(unsigned long arg,
					struct rk_rga_session *session);
static long rk_rga_ioctl_request_submit(unsigned long arg,
					struct rk_rga_session *session,
					bool run);
static long rk_rga_ioctl_import_buffer(unsigned long arg,
				       struct rk_rga_session *session);
static long rk_rga_ioctl_release_buffer(unsigned long arg,
					struct rk_rga_session *session);
static long rk_rga_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

struct rk_rga_kunit_sync_ioctl {
	struct work_struct work;
	struct rk_rga_session *session;
	void __user *task_user;
	long ret;
	bool done;
};

static void rk_rga_kunit_sync_ioctl_work(struct work_struct *work)
{
	struct rk_rga_kunit_sync_ioctl *ioctl =
		container_of(work, struct rk_rga_kunit_sync_ioctl, work);

	ioctl->ret = rk_rga_ioctl_blit((unsigned long)ioctl->task_user,
				       ioctl->session, RGA_BLIT_SYNC);
	WRITE_ONCE(ioctl->done, true);
}

static u32 rk_rga_kunit_hw_queued_jobs(struct rk_rga_hw *hw)
{
	unsigned long flags;
	u32 queued;

	spin_lock_irqsave(&hw->job_lock, flags);
	queued = hw->queued_jobs;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	return queued;
}
static struct rk_rga_import *rk_rga_kunit_import(struct kunit *test);

static void __user *rk_rga_kunit_user_buffer(struct kunit *test, size_t size)
{
	unsigned long useraddr;

	useraddr = kunit_vm_mmap(test, NULL, 0, PAGE_ALIGN(size ? size : 1),
				 PROT_READ | PROT_WRITE,
				 MAP_ANONYMOUS | MAP_PRIVATE, 0);
	if (!useraddr || useraddr >= TASK_SIZE) {
		KUNIT_FAIL(test, "failed to allocate userspace buffer");
		return NULL;
	}

	return (void __user *)useraddr;
}

static struct dma_fence *rk_rga_kunit_alloc_fence(void)
{
	struct dma_fence *fence;

	fence = kzalloc_obj(*fence, GFP_KERNEL);
	if (!fence)
		return NULL;

	dma_fence_init(fence, &rk_rga_fence_ops, &rk_rga_kunit_fence_lock,
		       dma_fence_context_alloc(1), 1);

	return fence;
}

static int rk_rga_kunit_install_fence_fd(struct dma_fence *fence)
{
	struct sync_file *sync_file;
	int fd;

	fd = rk_rga_fence_create_fd(fence, &sync_file);
	if (fd < 0)
		return fd;

	rk_rga_fence_install_fd(fd, sync_file);

	return fd;
}

static struct rga_img_info_t rk_rga_kunit_img(u64 addr, u32 format,
					      u16 width, u16 height)
{
	return (struct rga_img_info_t) {
		.yrgb_addr = addr,
		.uv_addr = rk_rga_format_is_yuv(format) ? addr + 0x100000 : 0,
		.format = format,
		.act_w = width,
		.act_h = height,
		.vir_w = width,
		.vir_h = height,
	};
}

static struct rga_req rk_rga_ffmpeg_bitblt_task(u32 src_format,
						u32 dst_format)
{
	return (struct rga_req) {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000, src_format, 1920, 1080),
		.dst = rk_rga_kunit_img(0x20000000, dst_format, 1280, 720),
		.yuv2rgb_mode = 1,
	};
}

static struct rga_req rk_rga_librga_splice_task(u64 src_addr, u32 dst_x)
{
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(src_addr, RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000,
					RK_RGA_FORMAT_RGBA_8888, 2560, 720),
	};

	task.dst.x_offset = dst_x;
	task.dst.act_w = 1280;

	return task;
}

static struct rga_req rk_rga_librga_side_border_task(u16 src_x, u16 dst_x,
						    u16 width, bool reflect,
						    u32 core)
{
	struct rga_img_info_t img =
		rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				  width, 48);
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.core = core,
		.rotate_mode = reflect ? 2 : 0,
	};

	img.vir_w = 64;
	task.src = img;
	task.dst = img;
	task.src.x_offset = src_x;
	task.dst.x_offset = dst_x;

	return task;
}

static struct rga_req rk_rga_librga_padding_task(u16 src_y, u16 dst_y,
						 u16 height, bool reflect)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    2048, 912);
	task.src.y_offset = src_y;
	task.src.act_h = height;
	task.dst.x_offset = 256;
	task.dst.y_offset = dst_y;
	task.dst.act_w = 1280;
	task.dst.act_h = height;
	task.rotate_mode = reflect ? 3 : 0;
	task.yuv2rgb_mode = 0;

	return task;
}

static void rk_rga_fill_hw_type_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	struct rga_req task = rk_rga_fill_task(0);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	task = rk_rga_fill_task(BIT(2));
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	task = rk_rga_fill_task(BIT(0));
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_fill_task(BIT(7));
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), -EINVAL);
}

static void rk_rga2_fill_dst_offset_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req task = rk_rga_fill_task(BIT(2));
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;
	u32 expected_base;

	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    300, 200);
	task.dst.vir_w = 640;
	task.dst.vir_h = 480;
	task.dst.x_offset = 100;
	task.dst.y_offset = 50;
	task.fg_color = 0xff00ff00;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	stride_bytes = ALIGN((u32)task.dst.vir_w * 4, 4);
	expected_base = lower_32_bits(task.dst.yrgb_addr +
				      (u64)task.dst.y_offset * stride_bytes +
				      (u64)task.dst.x_offset * 4);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			stride_bytes >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			((u32)task.dst.act_w - 1) |
			(((u32)task.dst.act_h - 1) << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_FG_COLOR_OFFSET / 4],
			task.fg_color);
}

static void rk_rga2_pre_intr_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req task = rk_rga_fill_task(BIT(2));
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	struct rga_pre_intr_info intr = {
		.enable = 1,
		.read_intr_en = 1,
		.write_intr_en = 1,
		.read_hold_en = 1,
		.read_threshold = 0x0345,
		.write_start = 0x1234,
		.write_step = 0x1456,
	};

	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_read_line(&intr),
			FIELD_PREP(RK_RGA2_LINE_RD_THRESHOLD,
				   intr.read_threshold));
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_write_line(&intr),
			FIELD_PREP(RK_RGA2_LINE_WR_START,
				   intr.write_start) |
			FIELD_PREP(RK_RGA2_LINE_WR_STEP, intr.write_step));
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_int_enable(&intr),
			RK_RGA2_INT_LINE_RD_EN | RK_RGA2_INT_LINE_WR_EN);
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_sys_ctrl(&intr),
			RK_RGA2_SYS_CTRL_HOLD_MODE_EN);

	intr.enable = 0;
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_read_line(&intr), 0U);
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_write_line(&intr), 0U);
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_int_enable(&intr), 0U);
	KUNIT_EXPECT_EQ(test, rk_rga2_pre_intr_sys_ctrl(&intr), 0U);

	intr.enable = 1;
	task.pre_intr_info = intr;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	memset(cmd, 0, sizeof(cmd));
	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_RGBA_8888);
	task.core = BIT(2);
	task.pre_intr_info = intr;
	job.tasks = &task;
	job.import_count = 2;
	job.cmd_ready = false;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
}

static void rk_rga2_fill_yuv_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req task = rk_rga_fill_task(BIT(2));
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 y_stride_bytes;
	u32 uv_stride_bytes;
	u32 expected_base;
	u32 expected_dst_info;

	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    128, 64);
	task.dst.vir_w = 256;
	task.dst.vir_h = 128;
	task.dst.x_offset = 16;
	task.dst.y_offset = 8;
	task.yuv2rgb_mode = 2 << 2;
	task.fg_color = 0xff00ff00;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	y_stride_bytes = ALIGN((u32)task.dst.vir_w, 4);
	uv_stride_bytes = ALIGN((u32)task.dst.vir_w, 4);
	expected_base = lower_32_bits(task.dst.yrgb_addr +
				      (u64)task.dst.y_offset *
				      y_stride_bytes +
				      task.dst.x_offset);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);
	expected_base = lower_32_bits(task.dst.uv_addr +
				      (u64)(task.dst.y_offset / 2) *
				      uv_stride_bytes +
				      task.dst.x_offset);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE1_OFFSET / 4],
			expected_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			y_stride_bytes >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			((u32)task.dst.act_w - 1) |
			(((u32)task.dst.act_h - 1) << 16));
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xa) |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_FG_COLOR_OFFSET / 4],
			task.fg_color);

	memset(cmd, 0, sizeof(cmd));
	task.dst.x_offset = 1;
	job.cmd_ready = false;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), -EINVAL);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.dst.x_offset = 16;
	task.yuv2rgb_mode = 0;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), -EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);
}

static void rk_rga2_fill_packed_yuv_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req task = rk_rga_fill_task(BIT(2));
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;
	u32 expected_base;
	u32 expected_dst_info;

	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YUYV_422,
				    128, 64);
	task.dst.vir_w = 256;
	task.dst.vir_h = 128;
	task.dst.x_offset = 16;
	task.dst.y_offset = 8;
	task.yuv2rgb_mode = 2 << 2;
	task.fg_color = 0xff00ff00;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	stride_bytes = ALIGN((u32)task.dst.vir_w * 2, 4);
	expected_base = lower_32_bits(task.dst.yrgb_addr +
				      (u64)task.dst.y_offset *
				      stride_bytes +
				      (u64)task.dst.x_offset * 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE1_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE2_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			stride_bytes >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			((u32)task.dst.act_w - 1) |
			(((u32)task.dst.act_h - 1) << 16));
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xe) |
			    RK_RGA2_DST_UV_SWAP |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_FG_COLOR_OFFSET / 4],
			task.fg_color);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_UYVY_422;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xc) |
			    RK_RGA2_DST_UV_SWAP |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_YUYV_420;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xf) |
			    RK_RGA2_DST_UV_SWAP |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_YVYU_420;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xf) |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_VYUY_420;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xd) |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_UYVY_420;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0xd) |
			    RK_RGA2_DST_UV_SWAP |
			    FIELD_PREP(RK_RGA2_DST_CSC_MODE, 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_base);
}

static void rk_rga2_fill_multitask_hw_type_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	struct rga_req *tasks;
	struct rk_rga_job job = {
		.task_count = 4,
		.import_count = 1,
	};

	tasks = kunit_kcalloc(test, job.task_count, sizeof(*tasks), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	job.tasks = tasks;

	for (u32 i = 0; i < job.task_count; i++) {
		tasks[i] = rk_rga_fill_task(BIT(2));
		tasks[i].dst.vir_w = 640;
		tasks[i].dst.vir_h = 480;
	}

	tasks[0].dst.act_w = 300;
	tasks[0].dst.act_h = 4;
	tasks[1].dst.x_offset = 100;
	tasks[1].dst.y_offset = 100;
	tasks[1].dst.act_w = 300;
	tasks[1].dst.act_h = 4;
	tasks[2].dst.x_offset = 100;
	tasks[2].dst.y_offset = 4;
	tasks[2].dst.act_w = 4;
	tasks[2].dst.act_h = 96;
	tasks[3].dst.x_offset = 396;
	tasks[3].dst.y_offset = 4;
	tasks[3].dst.act_w = 4;
	tasks[3].dst.act_h = 96;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
}

static void rk_rga2_rectangle_task_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = {};
	static const struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} rects[] = {
		{ 100, 200, 300, 4 },
		{ 100, 292, 300, 4 },
		{ 100, 204, 4, 88 },
		{ 396, 204, 4, 88 },
	};
	struct rga_req *tasks;
	struct rk_rga_job job = {
		.task_count = ARRAY_SIZE(rects),
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;

	tasks = kunit_kcalloc(test, job.task_count, sizeof(*tasks), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	job.tasks = tasks;

	for (u32 i = 0; i < job.task_count; i++) {
		tasks[i] = rk_rga_fill_task(BIT(2));
		tasks[i].dst = rk_rga_kunit_img(0x20000000,
						 RK_RGA_FORMAT_RGBA_8888,
						 rects[i].w, rects[i].h);
		tasks[i].dst.vir_w = 640;
		tasks[i].dst.vir_h = 480;
		tasks[i].dst.x_offset = rects[i].x;
		tasks[i].dst.y_offset = rects[i].y;
		tasks[i].fg_color = 0xff00ff00;
	}

	stride_bytes = ALIGN((u32)tasks[0].dst.vir_w * 4, 4);
	for (u32 i = 0; i < job.task_count; i++) {
		enum rk_rga_hw_type type = 0;
		u32 expected_base;

		memset(cmd, 0, sizeof(cmd));
		job.cmd_ready = false;

		KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
		KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
		KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_fill(&job), 0);
		KUNIT_EXPECT_TRUE(test, job.cmd_ready);

		expected_base = lower_32_bits(tasks[i].dst.yrgb_addr +
					      (u64)rects[i].y *
					      stride_bytes +
					      (u64)rects[i].x * 4);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
				expected_base);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
				((u32)rects[i].w - 1) |
				(((u32)rects[i].h - 1) << 16));
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_FG_COLOR_OFFSET / 4],
				tasks[i].fg_color);

		if (i + 1 < job.task_count)
			KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job,
									0));
		else
			KUNIT_EXPECT_FALSE(test, rk_rga_job_advance_task(&job,
									 0));
	}
}

static void rk_rga2_mosaic_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_img_info_t img =
		rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				 1280, 720);
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	img.act_w = 300;
	img.act_h = 200;
	task.src = img;
	task.dst = img;
	task.yuv2rgb_mode = 0;
	task.mosaic_info.enable = 1;
	task.mosaic_info.mode = 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
			  RK_RGA2_MODE_MOSAIC_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MOSAIC_MODE_OFFSET / 4], 2U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			lower_32_bits(img.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			lower_32_bits(img.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			299U | (199U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			299U | (199U << 16));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.mosaic_info.mode = 5;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.mosaic_info.mode = 2;
	task.dst.x_offset = 4;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);
}

static void rk_rga2_mosaic_task_array_emit_kunit(struct kunit *test)
{
	static const struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
		u8 mode;
	} rects[] = {
		{ 32, 16, 96, 48, 1 },
		{ 320, 180, 160, 90, 3 },
	};
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req *tasks;
	struct rk_rga_job job = {
		.task_count = ARRAY_SIZE(rects),
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;

	tasks = kunit_kcalloc(test, job.task_count, sizeof(*tasks), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	job.tasks = tasks;

	for (u32 i = 0; i < job.task_count; i++) {
		struct rga_img_info_t img =
			rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
					 rects[i].w, rects[i].h);

		tasks[i] = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
						     RK_RGA_FORMAT_RGBA_8888);
		img.vir_w = 640;
		img.vir_h = 480;
		img.x_offset = rects[i].x;
		img.y_offset = rects[i].y;
		tasks[i].src = img;
		tasks[i].dst = img;
		tasks[i].core = RK_RGA_CORE_RGA2_MASK;
		tasks[i].yuv2rgb_mode = 0;
		tasks[i].mosaic_info.enable = 1;
		tasks[i].mosaic_info.mode = rects[i].mode;
	}

	stride_bytes = ALIGN((u32)tasks[0].src.vir_w * 4, 4);
	for (u32 i = 0; i < job.task_count; i++) {
		enum rk_rga_hw_type type = 0;
		u32 expected_base;

		memset(cmd, 0, sizeof(cmd));
		job.cmd_ready = false;

		KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
		KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
		KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
		KUNIT_EXPECT_TRUE(test, job.cmd_ready);

		expected_base = lower_32_bits(tasks[i].src.yrgb_addr +
					      (u64)rects[i].y *
					      stride_bytes +
					      (u64)rects[i].x * 4);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
				expected_base);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
				expected_base);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
				((u32)rects[i].w - 1) |
				(((u32)rects[i].h - 1) << 16));
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
				((u32)rects[i].w - 1) |
				(((u32)rects[i].h - 1) << 16));
		KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
				  RK_RGA2_MODE_MOSAIC_EN);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MOSAIC_MODE_OFFSET / 4],
				(u32)rects[i].mode);

		if (i + 1 < job.task_count)
			KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job,
									0));
		else
			KUNIT_EXPECT_FALSE(test, rk_rga_job_advance_task(&job,
									 0));
	}
}

static void rk_rga2_rop_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.yuv2rgb_mode = 0;
	task.alpha_rop_flag = BIT(0) | BIT(1);
	task.alpha_rop_mode = 0x1;
	task.rop_code = RK_RGA_ROP_AND;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4],
			RK_RGA2_ALPHA_ROP_0 | RK_RGA2_ALPHA_ROP_SEL |
			FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL1_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ROP_CTRL0_OFFSET / 4],
			0x00000070U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ROP_CTRL1_OFFSET / 4], 0U);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.rop_code = 0x12;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.rop_code = RK_RGA_ROP_AND;
	task.dst.act_w = 640;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);
}

static void rk_rga2_colorkey_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 src_info;
	u32 expected_trans;
	u32 expected_ctrl1;

	task.core = BIT(2);
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    320, 240);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRA_8888,
				    320, 240);
	task.yuv2rgb_mode = 0;
	task.alpha_rop_flag = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_PD_ENABLE |
			      RK_RGA2_ALPHA_FLAG_CAL_MODE |
			      RK_RGA2_ALPHA_FLAG_REAL_COLOR;
	task.alpha_rop_mode = 0x11;
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.src_trans_mode = 0x1e;
	task.color_key_min = 0x00112233;
	task.color_key_max = 0x00445566;
	expected_ctrl1 =
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_COLOR_M0,
			   RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_COLOR_M0,
			   RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M0,
			   RK_RGA2_ALPHA_ONE) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_BLEND_M0,
			   RK_RGA2_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M0,
			   RK_RGA2_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M1,
			   RK_RGA2_ALPHA_ONE) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_BLEND_M1,
			   RK_RGA2_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M1,
			   RK_RGA2_ALPHA_PER_PIXEL);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	expected_trans =
		FIELD_PREP(RK_RGA2_SRC_TRANS_MODE, task.src_trans_mode) |
		FIELD_PREP(RK_RGA2_SRC_TRANS_ENABLE,
			   task.src_trans_mode >> 1);
	KUNIT_EXPECT_EQ(test, src_info & (RK_RGA2_SRC_TRANS_MODE |
					 RK_RGA2_SRC_TRANS_ENABLE),
			expected_trans);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_TR_COLOR0_OFFSET / 4],
			task.color_key_min);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_TR_COLOR1_OFFSET / 4],
			task.color_key_max);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4],
			RK_RGA2_ALPHA_ROP_0 |
			FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL1_OFFSET / 4],
			expected_ctrl1);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.src_trans_mode = 0x1f;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	expected_trans =
		FIELD_PREP(RK_RGA2_SRC_TRANS_MODE, task.src_trans_mode) |
		FIELD_PREP(RK_RGA2_SRC_TRANS_ENABLE,
			   task.src_trans_mode >> 1);
	KUNIT_EXPECT_EQ(test, src_info & (RK_RGA2_SRC_TRANS_MODE |
					 RK_RGA2_SRC_TRANS_ENABLE),
			expected_trans);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.src_trans_mode = 0x1d;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.src_trans_mode = 0x1e;
	task.src.format = RK_RGA_FORMAT_RGB_888;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga2_gauss_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	u32 gauss_coeffs[] = {
		FIELD_PREP(RK_RGA2_GAUSS_COE0, 1) |
		FIELD_PREP(RK_RGA2_GAUSS_COE1, 2) |
		FIELD_PREP(RK_RGA2_GAUSS_COE2, 3),
	};
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.gauss_coeffs = gauss_coeffs,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.yuv2rgb_mode = 0;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xfe;
	task.gauss_config.size = 3;
	task.gauss_config.coe_ptr = 0x1000;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
			  RK_RGA2_MODE_SRC_GAUSS_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_GAUSS_COE_OFFSET / 4],
			gauss_coeffs[0]);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4],
			FIELD_PREP(RK_RGA2_ALPHA_SRC_GLOBAL, 0xfe));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.act_w = 640;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.dst.act_w = task.src.act_w;
	task.gauss_config.size = 5;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), -EINVAL);
}

static void rk_rga2_quantize_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_scale;
	u32 expected_offset;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.yuv2rgb_mode = 0;
	task.alpha_rop_flag = BIT(8);
	task.gr_color.gr_x_r = 0x100;
	task.gr_color.gr_x_g = 0x080;
	task.gr_color.gr_x_b = 0x3ff;
	task.gr_color.gr_y_r = -1;
	task.gr_color.gr_y_g = 0x020;
	task.gr_color.gr_y_b = 0x100;
	expected_scale = rk_rga2_pack_nn_quantize(0x100, 0x080, 0x3ff);
	expected_offset = rk_rga2_pack_nn_quantize(-1, 0x020, 0x100);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4] &
			  RK_RGA2_DST_NN_QUANTIZE_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_QUANTIZE_SCALE_OFFSET / 4],
			expected_scale);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_QUANTIZE_OFFSET_OFFSET / 4],
			expected_offset);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL1_OFFSET / 4], 0U);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.gr_color.gr_x_g = 0x400;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), -EINVAL);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.gr_color.gr_x_g = 0x080;
	task.alpha_rop_flag = BIT(8) | BIT(0);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job),
			-EOPNOTSUPP);
}

static void rk_rga2_alpha_bitmap_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_img_info_t src =
		rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				 1920, 1080);
	struct rga_img_info_t dst =
		rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				 1920, 1080);
	struct rga_img_info_t pat =
		rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_ARGB_5551,
				 1280, 720);
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_ctrl0;
	u32 expected_dst_info;
	u32 expected_dst_vir_info;

	src.x_offset = 100;
	src.y_offset = 200;
	src.act_w = 1280;
	src.act_h = 720;
	dst.x_offset = 100;
	dst.y_offset = 200;
	dst.act_w = 1280;
	dst.act_h = 720;
	task.src = src;
	task.dst = dst;
	task.pat = pat;
	task.yuv2rgb_mode = 0;
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_PD_ENABLE |
			      RK_RGA2_ALPHA_FLAG_CAL_MODE |
			      RK_RGA2_ALPHA_FLAG_REAL_COLOR;
	task.alpha_rop_mode = 0x1;
	task.PD_mode = RK_RGA_ALPHA_BLEND_DST_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.rgba5551_alpha.flags = 1;
	task.rgba5551_alpha.alpha0 = 0x20;
	task.rgba5551_alpha.alpha1 = 0xe0;
	expected_ctrl0 = FIELD_PREP(RK_RGA2_ALPHA_ROP_0,
				    task.alpha_rop_flag & 0x1) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_SEL,
				    (task.alpha_rop_flag >> 1) & 0x1) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE,
				    task.alpha_rop_mode);
	expected_dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, 0x0) |
			    FIELD_PREP(RK_RGA2_DST_SRC1_FORMAT, 0x5) |
			    RK_RGA2_DST_SRC1_A1555_ALPHA_EN;
	expected_dst_vir_info = (1920U * 4 / 4) |
				((1280U * 2 / 4) << 16);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
			  RK_RGA2_MODE_BITBLT_MODE);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4],
			expected_dst_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			expected_dst_vir_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE3_OFFSET / 4],
			lower_32_bits(pat.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BG_COLOR_OFFSET / 4],
			0x20000000U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_FG_COLOR_OFFSET / 4],
			0xe0000000U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4],
			expected_ctrl0);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL1_OFFSET / 4],
			rk_rga2_alpha_bitmap_ctrl1());

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.pat.format = RK_RGA_FORMAT_RGB_565;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.pat.format = RK_RGA_FORMAT_ARGB_5551;
	task.alpha_rop_flag &= ~RK_RGA2_ALPHA_FLAG_REAL_COLOR;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job),
			-EOPNOTSUPP);
}

static void rk_rga2_osd_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_img_info_t bg =
		rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				 1280, 720);
	struct rga_img_info_t osd =
		rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				 64, 576);
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_ctrl0 = FIELD_PREP(RK_RGA2_OSD_CTRL0_MODE, 3) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL0_DIRECTION, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL0_BLOCK_COUNT, 5) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL0_FLAGS_INDEX, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL0_FIX_WIDTH, 47);
	u32 expected_ctrl1 = FIELD_PREP(RK_RGA2_OSD_CTRL1_FLAGS_MODE, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_MODE, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL1_DEFAULT_COLOR, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL1_THRESH, 40) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_Y, 1) |
			     FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_C, 1);
	u32 expected_dst_vir_info = (1280U * 4 / 4) |
				    ((64U * 4 / 4) << 16);
	u32 expected_cal0 = (211U << 24) | (17U << 16) |
			    (201U << 8) | 9U;
	u32 expected_cal1 = (231U << 8) | 23U;

	bg.x_offset = 100;
	bg.y_offset = 100;
	bg.act_w = 64;
	bg.act_h = 576;
	task.src = bg;
	task.dst = bg;
	task.pat = osd;
	task.yuv2rgb_mode = 0;
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_PD_ENABLE |
			      RK_RGA2_ALPHA_FLAG_CAL_MODE;
	task.alpha_rop_mode = 0x1;
	task.PD_mode = RK_RGA_ALPHA_BLEND_DST_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.osd_info.enable = 1;
	task.osd_info.mode_ctrl.mode = 3;
	task.osd_info.mode_ctrl.direction_mode = 1;
	task.osd_info.mode_ctrl.block_fix_width = 96;
	task.osd_info.mode_ctrl.block_num = 6;
	task.osd_info.mode_ctrl.invert_flags_mode = 1;
	task.osd_info.mode_ctrl.flags_index = 1;
	task.osd_info.mode_ctrl.default_color_sel = 1;
	task.osd_info.mode_ctrl.invert_enable = 0x6;
	task.osd_info.mode_ctrl.invert_mode = 1;
	task.osd_info.mode_ctrl.invert_thresh = 40;
	task.osd_info.cal_factor.yg_min = 9;
	task.osd_info.cal_factor.yg_max = 201;
	task.osd_info.cal_factor.crb_min = 17;
	task.osd_info.cal_factor.crb_max = 211;
	task.osd_info.cal_factor.alpha_min = 23;
	task.osd_info.cal_factor.alpha_max = 231;
	task.osd_info.last_flags0 = 0x2a;
	task.osd_info.last_flags1 = 0x01020304;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
			  RK_RGA2_MODE_BITBLT_MODE);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4] &
			  RK_RGA2_MODE_OSD_EN);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4] &
			   RK_RGA2_DST_SRC1_A1555_ALPHA_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			expected_dst_vir_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE3_OFFSET / 4],
			lower_32_bits(osd.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL0_OFFSET / 4],
			RK_RGA2_ALPHA_ROP_0 |
			FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_ALPHA_CTRL1_OFFSET / 4],
			rk_rga2_osd_alpha_ctrl1());
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_CTRL0_OFFSET / 4],
			expected_ctrl0);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_CTRL1_OFFSET / 4],
			expected_ctrl1);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_INVERSION_CAL0_OFFSET / 4],
			expected_cal0);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_INVERSION_CAL1_OFFSET / 4],
			expected_cal1);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_LAST_FLAGS0_OFFSET / 4],
			0x2aU);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_LAST_FLAGS1_OFFSET / 4],
			0x01020304U);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.osd_info.mode_ctrl.color_mode = 1;
	task.osd_info.bpp2_info.color0.value = 0xff336699;
	task.osd_info.bpp2_info.color1.value = 0x80123456;
	expected_ctrl1 |= FIELD_PREP(RK_RGA2_OSD_CTRL1_COLOR_MODE, 1);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_CTRL1_OFFSET / 4],
			expected_ctrl1);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_COLOR0_OFFSET / 4],
			0x336699U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_OSD_COLOR1_OFFSET / 4],
			0x123456U);

	task.osd_info.mode_ctrl.color_mode = 0;
	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.pat.format = RK_RGA_FORMAT_RGB_565;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.pat.format = RK_RGA_FORMAT_RGBA_8888;
	task.osd_info.mode_ctrl.block_fix_width = 95;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), -EINVAL);
}

static void rk_rga2_palette_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_img_info_t lut =
		rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				 16, 16);
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_COLOR_PALETTE,
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_400,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.pat = lut,
		.palette_mode = 3,
		.endian_mode = 1,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride = ALIGN((u32)task.src.vir_w, 4);
	u32 expected_src_info = FIELD_PREP(RK_RGA2_SRC_FORMAT, 0xf) |
				RK_RGA2_SRC_CP_ENDIAN;

	task.src.x_offset = 3;
	task.src.y_offset = 5;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_palette(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				   RK_RGA_RENDER_COLOR_PALETTE) |
			RK_RGA2_MODE_INTR_CF_E);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr +
				      (u64)task.src.y_offset * stride +
				      task.src.x_offset));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_INFO_OFFSET / 4],
			expected_src_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_VIR_INFO_OFFSET / 4],
			stride >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			((u32)task.src.act_w - 1) |
			(((u32)task.src.act_h - 1) << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_INFO_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.src.format = RK_RGA_FORMAT_BPP4;
	task.palette_mode = 2;
	task.endian_mode = 0;
	task.src.x_offset = 6;
	task.src.y_offset = 7;
	stride = ALIGN((u32)task.src.vir_w >> 1, 4);
	expected_src_info = FIELD_PREP(RK_RGA2_SRC_FORMAT, 0xe);

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_palette(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr +
				      (u64)task.src.y_offset * stride +
				      (task.src.x_offset >> 1)));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_INFO_OFFSET / 4],
			expected_src_info);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_VIR_INFO_OFFSET / 4],
			stride >> 2);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.palette_mode = 1;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_color_palette(&job), -EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);
}

static void rk_rga2_update_palette_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_UPDATE_PALETTE,
		.pat = rk_rga_kunit_img(0x30000000,
					RK_RGA_FORMAT_RGBA_8888, 16, 16),
		.palette_mode = 3,
		.fading = {
			.g = 0xff,
		},
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 1,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_update_palette(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MODE_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				   RK_RGA2_HW_RENDER_UPDATE_PALETTE) |
			RK_RGA2_MODE_INTR_CF_E);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_FADING_CTRL_OFFSET / 4],
			0xff00U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MASK_BASE_OFFSET / 4],
			lower_32_bits(task.pat.yrgb_addr));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.palette_mode = 0;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_update_palette(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_MASK_BASE_OFFSET / 4],
			lower_32_bits(task.pat.yrgb_addr));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.palette_mode = 4;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_update_palette(&job),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.palette_mode = 3;
	task.pat.act_w = 8;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_update_palette(&job),
			-EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);
}

static void rk_rga_request_check_kunit(struct kunit *test)
{
	struct rga_user_request user = {
		.task_ptr = 0x1000,
		.task_num = 1,
		.id = 1,
	};

	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), 0);

	user.id = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), -EINVAL);

	user.id = 1;
	user.task_ptr = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), -EINVAL);

	user.task_ptr = 0x1000;
	user.task_num = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), -EINVAL);

	user.task_num = RGA_TASK_NUM_MAX + 1;
	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), -EFBIG);

	user.task_num = RGA_TASK_NUM_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_request_check(&user), 0);
}

static void rk_rga_request_ioctl_ret_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_rga_request_ioctl_ret(0), 0);
	KUNIT_EXPECT_EQ(test, rk_rga_request_ioctl_ret(-EINVAL), -EFAULT);
	KUNIT_EXPECT_EQ(test, rk_rga_request_ioctl_ret(-ENOMEM), -EFAULT);
}

static void rk_rga_request_create_cancel_ioctl_kunit(struct kunit *test)
{
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	void __user *user;
	unsigned long uncopied;
	__u32 value = 0x55;
	__u32 id = 0;

	user = rk_rga_kunit_user_buffer(test, sizeof(value));
	KUNIT_ASSERT_NOT_NULL(test, user);
	uncopied = copy_to_user(user, &value, sizeof(value));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_request_create((unsigned long)user,
						    &session),
			0L);
	uncopied = copy_from_user(&id, user, sizeof(id));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_NE(test, id, 0U);
	request = idr_find(&session.requests, id);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_EXPECT_EQ(test, request->flags, 0x55U);

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_request_cancel((unsigned long)user,
						    &session),
			0L);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, id), NULL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_request_cancel((unsigned long)user,
						    &session),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_request_create(TASK_SIZE, &session),
			-EFAULT);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_request_cancel(TASK_SIZE, &session),
			-EFAULT);

	idr_destroy(&session.requests);
}

static void rk_rga_request_config_handles_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 7,
		.sync_mode = RGA_BLIT_ASYNC,
		.mpi_config_flags = 0x5a,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct rk_rga_job *job = NULL;
	void __user *task_user;
	unsigned long uncopied;
	int ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	task.handle_flag = 1;
	task.src.yrgb_addr = 11;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 12;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	user.task_ptr = (uintptr_t)task_user;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, request);
	src_import = kzalloc_obj(*src_import, GFP_KERNEL);
	dst_import = kzalloc_obj(*dst_import, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);

	src_import->type = RK_RGA_IMPORT_USERPTR;
	refcount_set(&src_import->refs, 1);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->type = RK_RGA_IMPORT_USERPTR;
	refcount_set(&dst_import->refs, 1);
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 7, 8,
				  GFP_KERNEL),
			7);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 12, 13,
				  GFP_KERNEL),
			12);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 7), request);
	KUNIT_EXPECT_TRUE(test, request->configured);
	KUNIT_EXPECT_EQ(test, request->task_count, 1U);
	KUNIT_EXPECT_EQ(test, request->sync_mode, (u32)RGA_BLIT_ASYNC);
	KUNIT_EXPECT_EQ(test, request->mpi_config_flags, 0x5aU);
	KUNIT_EXPECT_EQ(test, request->release_fence_fd, -1);
	KUNIT_EXPECT_EQ(test, request->import_count, 2U);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);
	KUNIT_EXPECT_EQ(test, request->tasks[0].handle_flag & 1, 0U);
	KUNIT_EXPECT_EQ(test, request->tasks[0].src.yrgb_addr,
			src_import->iova);
	KUNIT_EXPECT_EQ(test, request->tasks[0].dst.yrgb_addr,
			dst_import->iova);

	ret = rk_rga_request_config(&session, &user, &job);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_NOT_NULL(test, job);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 7), request);
	KUNIT_EXPECT_EQ(test, job->task_count, 1U);
	KUNIT_EXPECT_EQ(test, job->sync_mode, (u32)RGA_BLIT_ASYNC);
	KUNIT_EXPECT_EQ(test, job->release_fence_fd, -1);
	KUNIT_EXPECT_EQ(test, job->import_count, 2U);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 3);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 3);
	KUNIT_EXPECT_EQ(test, job->tasks[0].src.yrgb_addr,
			src_import->iova);
	KUNIT_EXPECT_EQ(test, job->tasks[0].dst.yrgb_addr,
			dst_import->iova);

	rk_rga_job_put(job);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 7));
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 11),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 12),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_direct_img_mem_type_kunit(struct kunit *test)
{
	struct rga_req task = {};

	task.src.yrgb_addr = 5;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.src),
			RK_RGA_DIRECT_IMG_UNSUPPORTED_PHYS);

	task.mmu_info.mmu_flag = RK_RGA_MMU_SRC0;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.src),
			RK_RGA_DIRECT_IMG_DMABUF);

	task.src.yrgb_addr = 0;
	task.src.uv_addr = 0x1000;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.src),
			RK_RGA_DIRECT_IMG_USERPTR);

	task.src.yrgb_addr = (u64)INT_MAX + 1;
	task.src.uv_addr = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.src),
			RK_RGA_DIRECT_IMG_USERPTR);

	task.src.yrgb_addr = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.src),
			RK_RGA_DIRECT_IMG_INVALID);

	task.mmu_info.mmu_flag = RK_RGA_MMU_DST;
	task.dst.yrgb_addr = 7;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.dst),
			RK_RGA_DIRECT_IMG_DMABUF);

	task.mmu_info.mmu_flag = RK_RGA_MMU_SRC1;
	task.pat.yrgb_addr = 9;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.pat),
			RK_RGA_DIRECT_IMG_DMABUF);

	task.mmu_info.mmu_flag = RK_RGA_MMU_ELSE;
	task.pat.yrgb_addr = 0;
	task.pat.uv_addr = 0x2000;
	KUNIT_EXPECT_EQ(test, rk_rga_classify_direct_img(&task, &task.pat),
			RK_RGA_DIRECT_IMG_USERPTR);
}

static void rk_rga_request_config_direct_phys_reject_kunit(struct kunit *test)
{
	struct rga_req tasks[2] = {
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888),
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888),
	};
	struct rga_user_request user = {
		.task_num = ARRAY_SIZE(tasks),
		.id = 8,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	void __user *task_user;
	unsigned long uncopied;
	int ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(tasks));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	tasks[0].handle_flag = 1;
	tasks[0].src.yrgb_addr = 81;
	tasks[0].src.uv_addr = 0;
	tasks[0].src.v_addr = 0;
	tasks[0].dst.yrgb_addr = 82;
	tasks[0].dst.uv_addr = 0;
	tasks[0].dst.v_addr = 0;
	tasks[1].handle_flag = 0;
	tasks[1].mmu_info.mmu_flag = 0;
	tasks[1].src.yrgb_addr = 0x10000000;
	tasks[1].src.uv_addr = 0;
	tasks[1].src.v_addr = 0;
	tasks[1].dst.yrgb_addr = 0x20000000;
	tasks[1].dst.uv_addr = 0;
	tasks[1].dst.v_addr = 0;
	user.task_ptr = (uintptr_t)task_user;

	uncopied = copy_to_user(task_user, tasks, sizeof(tasks));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 8, 9,
				  GFP_KERNEL),
			8);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 81, 82,
				  GFP_KERNEL),
			81);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 82, 83,
				  GFP_KERNEL),
			82);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 8), request);
	KUNIT_EXPECT_FALSE(test, request->configured);
	KUNIT_EXPECT_PTR_EQ(test, request->tasks, NULL);
	KUNIT_EXPECT_PTR_EQ(test, request->imports, NULL);
	KUNIT_EXPECT_EQ(test, request->import_count, 0U);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 8));
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 81),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 82),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_request_reconfig_resources_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 9,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *old_src;
	struct rk_rga_import *old_dst;
	struct rk_rga_import *new_src;
	struct rk_rga_import *new_dst;
	void __user *task_user;
	unsigned long uncopied;
	int ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	user.task_ptr = (uintptr_t)task_user;

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	old_src = rk_rga_kunit_import(test);
	old_dst = rk_rga_kunit_import(test);
	new_src = rk_rga_kunit_import(test);
	new_dst = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, old_src);
	KUNIT_ASSERT_NOT_NULL(test, old_dst);
	KUNIT_ASSERT_NOT_NULL(test, new_src);
	KUNIT_ASSERT_NOT_NULL(test, new_dst);

	old_src->iova = 0x10000000;
	old_src->size = (size_t)1920 * 1080 * 4;
	old_dst->iova = 0x20000000;
	old_dst->size = (size_t)1280 * 720 * 4;
	new_src->iova = 0x30000000;
	new_src->size = (size_t)1920 * 1080 * 4;
	new_dst->iova = 0x40000000;
	new_dst->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 9, 10,
				  GFP_KERNEL),
			9);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, old_src, 21, 22,
				  GFP_KERNEL),
			21);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, old_dst, 22, 23,
				  GFP_KERNEL),
			22);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, new_src, 23, 24,
				  GFP_KERNEL),
			23);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, new_dst, 24, 25,
				  GFP_KERNEL),
			24);

	task.handle_flag = 1;
	task.src.yrgb_addr = 21;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 22;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, request->import_count, 2U);
	KUNIT_EXPECT_EQ(test, refcount_read(&old_src->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&old_dst->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&new_src->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&new_dst->refs), 1);
	KUNIT_EXPECT_EQ(test, request->tasks[0].src.yrgb_addr,
			old_src->iova);
	KUNIT_EXPECT_EQ(test, request->tasks[0].dst.yrgb_addr,
			old_dst->iova);

	task.src.yrgb_addr = 23;
	task.dst.yrgb_addr = 24;
	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, request->import_count, 2U);
	KUNIT_EXPECT_EQ(test, refcount_read(&old_src->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&old_dst->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&new_src->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&new_dst->refs), 2);
	KUNIT_EXPECT_EQ(test, request->tasks[0].src.yrgb_addr,
			new_src->iova);
	KUNIT_EXPECT_EQ(test, request->tasks[0].dst.yrgb_addr,
			new_dst->iova);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 9));
	KUNIT_EXPECT_EQ(test, refcount_read(&new_src->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&new_dst->refs), 1);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 21), old_src);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 22), old_dst);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 23), new_src);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 24), new_dst);
	rk_rga_import_put(old_src);
	rk_rga_import_put(old_dst);
	rk_rga_import_put(new_src);
	rk_rga_import_put(new_dst);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_request_reconfig_fences_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 10,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *old_fence;
	struct dma_fence *new_fence;
	void __user *task_user;
	unsigned int old_base_refs;
	unsigned int new_base_refs;
	unsigned long uncopied;
	int old_fd;
	int new_fd;
	int ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	user.task_ptr = (uintptr_t)task_user;

	old_fence = rk_rga_kunit_alloc_fence();
	new_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, old_fence);
	KUNIT_ASSERT_NOT_NULL(test, new_fence);
	old_fd = rk_rga_kunit_install_fence_fd(old_fence);
	new_fd = rk_rga_kunit_install_fence_fd(new_fence);
	KUNIT_ASSERT_GE(test, old_fd, 0);
	KUNIT_ASSERT_GE(test, new_fd, 0);
	old_base_refs = kref_read(&old_fence->refcount);
	new_base_refs = kref_read(&new_fence->refcount);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.in_fence_fd = -1;
	task.src.yrgb_addr = 31;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 32;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 10, 11,
				  GFP_KERNEL),
			10);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 31, 32,
				  GFP_KERNEL),
			31);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 32, 33,
				  GFP_KERNEL),
			32);

	user.acquire_fence_fd = old_fd;
	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, request->acquire_fence_count, 1U);
	KUNIT_EXPECT_PTR_EQ(test, request->acquire_fences[0], old_fence);
	KUNIT_EXPECT_EQ(test, kref_read(&old_fence->refcount),
			old_base_refs + 1);
	KUNIT_EXPECT_EQ(test, kref_read(&new_fence->refcount),
			new_base_refs);

	user.acquire_fence_fd = new_fd;
	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, request->acquire_fence_count, 1U);
	KUNIT_EXPECT_PTR_EQ(test, request->acquire_fences[0], new_fence);
	KUNIT_EXPECT_EQ(test, kref_read(&old_fence->refcount),
			old_base_refs);
	KUNIT_EXPECT_EQ(test, kref_read(&new_fence->refcount),
			new_base_refs + 1);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 10));
	KUNIT_EXPECT_EQ(test, kref_read(&new_fence->refcount), new_base_refs);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 31),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 32),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
	close_fd(old_fd);
	close_fd(new_fd);
	dma_fence_put(old_fence);
	dma_fence_put(new_fence);
}

static void rk_rga_request_config_ioctl_acquire_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 12,
		.sync_mode = RGA_BLIT_ASYNC,
		.release_fence_fd = 0x12345678,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct dma_fence *fd_fence;
	void __user *task_user;
	void __user *request_user;
	unsigned long uncopied;
	int acquire_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	request_user = rk_rga_kunit_user_buffer(test, sizeof(user));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	KUNIT_ASSERT_NOT_NULL(test, request_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);

	task.handle_flag = 1;
	task.src.yrgb_addr = 51;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 52;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	user.task_ptr = (uintptr_t)task_user;
	user.acquire_fence_fd = acquire_fd;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(request_user, &user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 12, 13,
				  GFP_KERNEL),
			12);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 51, 52,
				  GFP_KERNEL),
			51);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 52, 53,
				  GFP_KERNEL),
			52);

	ret = rk_rga_ioctl_request_submit((unsigned long)request_user,
					  &session, false);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 12), request);
	KUNIT_EXPECT_TRUE(test, request->configured);
	KUNIT_EXPECT_EQ(test, request->release_fence_fd, -1);
	KUNIT_EXPECT_EQ(test, request->acquire_fence_count, 1U);
	KUNIT_EXPECT_PTR_EQ(test, request->acquire_fences[0], acquire_fence);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);

	uncopied = copy_from_user(&user, request_user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, user.release_fence_fd, 0x12345678U);

	fd_fence = sync_file_get_fence(acquire_fd);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, NULL);
	if (fd_fence)
		dma_fence_put(fd_fence);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 12));
	KUNIT_EXPECT_EQ(test, kref_read(&acquire_fence->refcount), 1U);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 51),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 52),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	dma_fence_put(acquire_fence);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_request_cancel_configured_ioctl_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	u32 coeffs[3] = { 7, 8, 9 };
	struct rga_user_request user = {
		.task_num = 1,
		.id = 13,
		.sync_mode = RGA_BLIT_ASYNC,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	void __user *task_user;
	void __user *request_user;
	void __user *cancel_user;
	void __user *coeff_user;
	unsigned int acquire_base_refs;
	unsigned long uncopied;
	__u32 cancel_id = 13;
	int acquire_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	request_user = rk_rga_kunit_user_buffer(test, sizeof(user));
	cancel_user = rk_rga_kunit_user_buffer(test, sizeof(cancel_id));
	coeff_user = rk_rga_kunit_user_buffer(test, sizeof(coeffs));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	KUNIT_ASSERT_NOT_NULL(test, request_user);
	KUNIT_ASSERT_NOT_NULL(test, cancel_user);
	KUNIT_ASSERT_NOT_NULL(test, coeff_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);
	acquire_base_refs = kref_read(&acquire_fence->refcount);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.src.yrgb_addr = 61;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 62;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xfe;
	task.gauss_config.size = 3;
	task.gauss_config.coe_ptr = (uintptr_t)coeff_user;
	user.task_ptr = (uintptr_t)task_user;
	user.acquire_fence_fd = acquire_fd;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(request_user, &user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(cancel_user, &cancel_id, sizeof(cancel_id));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(coeff_user, coeffs, sizeof(coeffs));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 13, 14,
				  GFP_KERNEL),
			13);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 61, 62,
				  GFP_KERNEL),
			61);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 62, 63,
				  GFP_KERNEL),
			62);

	ret = rk_rga_ioctl_request_submit((unsigned long)request_user,
					  &session, false);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_TRUE(test, request->configured);
	KUNIT_EXPECT_NOT_NULL(test, request->gauss_coeffs);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);
	KUNIT_EXPECT_EQ(test, kref_read(&acquire_fence->refcount),
			acquire_base_refs + 1);

	ret = rk_rga_ioctl_request_cancel((unsigned long)cancel_user,
					  &session);
	KUNIT_EXPECT_EQ(test, ret, 0L);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 13), NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_EQ(test, kref_read(&acquire_fence->refcount),
			acquire_base_refs);

	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 61),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 62),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_release_configured_request_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 14,
	};
	struct rk_rga_session *session;
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct file file = {};
	void __user *task_user;
	void __user *request_user;
	unsigned int acquire_base_refs;
	unsigned long uncopied;
	int acquire_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	request_user = rk_rga_kunit_user_buffer(test, sizeof(user));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	KUNIT_ASSERT_NOT_NULL(test, request_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);
	acquire_base_refs = kref_read(&acquire_fence->refcount);

	session = kzalloc_obj(*session, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	rk_rga_session_init(session);
	file.private_data = session;

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	refcount_inc(&src_import->refs);
	refcount_inc(&dst_import->refs);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->requests, request, 14, 15,
				  GFP_KERNEL),
			14);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, src_import, 71, 72,
				  GFP_KERNEL),
			71);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, dst_import, 72, 73,
				  GFP_KERNEL),
			72);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.src.yrgb_addr = 71;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 72;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	user.task_ptr = (uintptr_t)task_user;
	user.acquire_fence_fd = acquire_fd;
	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(request_user, &user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_ioctl_request_submit((unsigned long)request_user,
					  session, false);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 3);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 3);
	KUNIT_EXPECT_EQ(test, kref_read(&acquire_fence->refcount),
			acquire_base_refs + 1);

	mutex_init(&rk_rga.session_lock);
	INIT_LIST_HEAD(&rk_rga.sessions);
	KUNIT_EXPECT_EQ(test, rk_rga_release(NULL, &file), 0);
	KUNIT_EXPECT_PTR_EQ(test, file.private_data, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_EQ(test, kref_read(&acquire_fence->refcount),
			acquire_base_refs);

	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
}

static void rk_rga_release_pending_acquire_job_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_session *session;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct dma_fence *release_fence;
	struct file file = {};
	struct rk_rga_hw hw = { };
	void __user *task_user;
	unsigned long uncopied;
	int acquire_fd;
	int release_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);

	session = kzalloc_obj(*session, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	rk_rga_session_init(session);
	file.private_data = session;

	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	refcount_inc(&src_import->refs);
	refcount_inc(&dst_import->refs);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, src_import, 81, 82,
				  GFP_KERNEL),
			81);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, dst_import, 82, 83,
				  GFP_KERNEL),
			82);

	mutex_init(&rk_rga.hw_lock);
	mutex_init(&rk_rga.session_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	INIT_LIST_HEAD(&rk_rga.sessions);
	INIT_LIST_HEAD(&hw.node);
	INIT_LIST_HEAD(&hw.job_queue);
	spin_lock_init(&hw.job_lock);
	mutex_init(&hw.run_lock);
	init_waitqueue_head(&hw.idle);
	refcount_set(&hw.refs, 1);
	list_add_tail(&hw.node, &rk_rga.hw_list);
	spin_lock_init(&rk_rga.fence_lock);
	rk_rga.fence_context = dma_fence_context_alloc(1);
	rk_rga.fence_seqno = 0;

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.in_fence_fd = acquire_fd;
	task.src.yrgb_addr = 81;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 82;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.out_fence_fd = -1;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_ioctl_blit((unsigned long)task_user, session,
				RGA_BLIT_ASYNC);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 3);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 3);

	uncopied = copy_from_user(&task, task_user, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	release_fd = task.out_fence_fd;
	KUNIT_ASSERT_GE(test, release_fd, 0);

	release_fence = sync_file_get_fence(release_fd);
	KUNIT_ASSERT_NOT_NULL(test, release_fence);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), 0);

	KUNIT_EXPECT_EQ(test, rk_rga_release(NULL, &file), 0);
	KUNIT_EXPECT_PTR_EQ(test, file.private_data, NULL);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -EFAULT);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	list_del_init(&hw.node);

	rk_rga_fence_signal(acquire_fence, 0);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -EFAULT);

	close_fd(release_fd);
	dma_fence_put(release_fence);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
}

static void rk_rga_last_hw_remove_pending_acquire_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_session *session;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct dma_fence *release_fence;
	struct file file = {};
	struct rk_rga_hw hw = { };
	void __user *task_user;
	unsigned long uncopied;
	int acquire_fd;
	int release_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);

	session = kzalloc_obj(*session, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	rk_rga_session_init(session);
	file.private_data = session;

	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	refcount_inc(&src_import->refs);
	refcount_inc(&dst_import->refs);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, src_import, 81, 82,
				  GFP_KERNEL),
			81);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session->imports, dst_import, 82, 83,
				  GFP_KERNEL),
			82);

	mutex_init(&rk_rga.hw_lock);
	mutex_init(&rk_rga.session_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	INIT_LIST_HEAD(&rk_rga.sessions);
	INIT_LIST_HEAD(&hw.node);
	list_add_tail(&hw.node, &rk_rga.hw_list);
	spin_lock_init(&rk_rga.fence_lock);
	rk_rga.fence_context = dma_fence_context_alloc(1);
	rk_rga.fence_seqno = 0;
	rk_rga_session_link(session);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.in_fence_fd = acquire_fd;
	task.src.yrgb_addr = 81;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 82;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.out_fence_fd = -1;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_ioctl_blit((unsigned long)task_user, session,
				RGA_BLIT_ASYNC);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 3);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 3);

	uncopied = copy_from_user(&task, task_user, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	release_fd = task.out_fence_fd;
	KUNIT_ASSERT_GE(test, release_fd, 0);

	release_fence = sync_file_get_fence(release_fd);
	KUNIT_ASSERT_NOT_NULL(test, release_fence);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), 0);

	list_del_init(&hw.node);
	rk_rga_abort_all_pending_acquire_jobs(-ENODEV);
	KUNIT_EXPECT_GT(test,
			dma_fence_wait_timeout(release_fence, false,
					       msecs_to_jiffies(1000)),
			0L);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);

	KUNIT_EXPECT_EQ(test, rk_rga_release(NULL, &file), 0);
	KUNIT_EXPECT_PTR_EQ(test, file.private_data, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);

	rk_rga_fence_signal(acquire_fence, 0);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);

	close_fd(release_fd);
	dma_fence_put(release_fence);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
}

static void rk_rga_release_queued_job_kunit(struct kunit *test)
{
	struct rk_rga_session *session;
	struct rk_rga_job *job;
	struct dma_fence *release_fence;
	struct file file = {};
	struct rk_rga_hw hw = { };

	session = kzalloc_obj(*session, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	rk_rga_session_init(session);
	file.private_data = session;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);
	KUNIT_ASSERT_EQ(test, rk_rga_session_track_job(session, job), 0);

	release_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, release_fence);
	dma_fence_get(release_fence);
	job->release_fence = release_fence;

	mutex_init(&hw.run_lock);
	spin_lock_init(&hw.job_lock);
	init_waitqueue_head(&hw.idle);
	INIT_LIST_HEAD(&hw.node);
	INIT_LIST_HEAD(&hw.job_queue);
	refcount_set(&hw.refs, 2);

	job->hw = &hw;
	rk_rga_job_get(job);
	job->queued = true;
	list_add_tail(&job->node, &hw.job_queue);
	hw.queued_jobs = 1;

	mutex_init(&rk_rga.hw_lock);
	mutex_init(&rk_rga.session_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	INIT_LIST_HEAD(&rk_rga.sessions);
	list_add_tail(&hw.node, &rk_rga.hw_list);

	KUNIT_EXPECT_EQ(test, rk_rga_release(NULL, &file), 0);
	KUNIT_EXPECT_PTR_EQ(test, file.private_data, NULL);
	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_FALSE(test, job->queued);
	KUNIT_EXPECT_TRUE(test, job->done);
	KUNIT_EXPECT_EQ(test, job->result, -EFAULT);
	KUNIT_EXPECT_PTR_EQ(test, job->session, NULL);
	KUNIT_EXPECT_FALSE(test, job->session_linked);
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -EFAULT);

	list_del_init(&hw.node);
	rk_rga_job_put(job);
	dma_fence_put(release_fence);
}

static void rk_rga_request_reconfig_gauss_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	u32 coeffs[3] = { 1, 2, 3 };
	u32 expected_old =
		FIELD_PREP(RK_RGA2_GAUSS_COE0, 1) |
		FIELD_PREP(RK_RGA2_GAUSS_COE1, 2) |
		FIELD_PREP(RK_RGA2_GAUSS_COE2, 3);
	u32 expected_new =
		FIELD_PREP(RK_RGA2_GAUSS_COE0, 4) |
		FIELD_PREP(RK_RGA2_GAUSS_COE1, 5) |
		FIELD_PREP(RK_RGA2_GAUSS_COE2, 6);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 11,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct rk_rga_job *job = NULL;
	void __user *task_user;
	void __user *coeff_user;
	unsigned long uncopied;
	int ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	coeff_user = rk_rga_kunit_user_buffer(test, sizeof(coeffs));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	KUNIT_ASSERT_NOT_NULL(test, coeff_user);

	task.handle_flag = 1;
	task.src.yrgb_addr = 41;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 42;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xfe;
	task.gauss_config.size = 3;
	task.gauss_config.coe_ptr = (uintptr_t)coeff_user;
	user.task_ptr = (uintptr_t)task_user;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(coeff_user, coeffs, sizeof(coeffs));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.requests);
	idr_init(&session.imports);

	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 41, 42,
				  GFP_KERNEL),
			41);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 42, 43,
				  GFP_KERNEL),
			42);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_NOT_NULL(test, request->gauss_coeffs);
	KUNIT_EXPECT_EQ(test, request->gauss_coeffs[0], expected_old);

	coeffs[0] = 4;
	coeffs[1] = 5;
	coeffs[2] = 6;
	uncopied = copy_to_user(coeff_user, coeffs, sizeof(coeffs));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	ret = rk_rga_request_config(&session, &user, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_NOT_NULL(test, request->gauss_coeffs);
	KUNIT_EXPECT_EQ(test, request->gauss_coeffs[0], expected_new);

	ret = rk_rga_job_clone_request_locked(request, &job);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_NOT_NULL(test, job);
	KUNIT_ASSERT_NOT_NULL(test, job->gauss_coeffs);
	KUNIT_EXPECT_PTR_NE(test, job->gauss_coeffs, request->gauss_coeffs);
	KUNIT_EXPECT_EQ(test, job->gauss_coeffs[0], expected_new);
	rk_rga_job_put(job);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 11));
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 41),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 42),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_legacy_blit_sync_wait_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_req user_task;
	struct rk_rga_session session = {};
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct rk_rga_hw hw = { };
	struct rk_rga_job active = { };
	struct rk_rga_kunit_sync_ioctl ioctl = {
		.session = &session,
		.ret = -EINVAL,
	};
	void __user *task_user;
	unsigned long uncopied;
	bool queued = false;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	task.handle_flag = 1;
	task.src.yrgb_addr = 11;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 12;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.out_fence_fd = -1;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	rk_rga_session_init(&session);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 12, 13,
				  GFP_KERNEL),
			12);

	mutex_init(&hw.run_lock);
	spin_lock_init(&hw.job_lock);
	init_waitqueue_head(&hw.idle);
	INIT_LIST_HEAD(&hw.node);
	INIT_LIST_HEAD(&hw.job_queue);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_rga_hw_timeout_work);
	refcount_set(&hw.refs, 1);
	hw.type = RK_RGA_HW_RGA3;
	hw.core_mask = BIT(0);
	hw.active_job = &active;

	mutex_init(&rk_rga.hw_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	rk_rga.core_select_seq = 0;
	list_add_tail(&hw.node, &rk_rga.hw_list);

	ioctl.task_user = task_user;
	INIT_WORK(&ioctl.work, rk_rga_kunit_sync_ioctl_work);
	schedule_work(&ioctl.work);

	for (u32 i = 0; i < 100; i++) {
		if (rk_rga_kunit_hw_queued_jobs(&hw) == 1) {
			queued = true;
			break;
		}
		if (READ_ONCE(ioctl.done))
			break;
		usleep_range(1000, 2000);
	}

	if (queued) {
		KUNIT_EXPECT_FALSE(test, READ_ONCE(ioctl.done));
		KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 2);
		KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 2);
		KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 2);
		KUNIT_EXPECT_TRUE(test,
				  rk_rga_hw_abort_session_jobs(&hw, &session,
							       0));
	} else {
		unsigned long flags;

		KUNIT_FAIL(test, "sync legacy blit did not queue before returning");
		spin_lock_irqsave(&hw.job_lock, flags);
		hw.removing = true;
		spin_unlock_irqrestore(&hw.job_lock, flags);
		rk_rga_hw_abort_session_jobs(&hw, &session, -ETIMEDOUT);
	}

	flush_work(&ioctl.work);
	KUNIT_EXPECT_TRUE(test, READ_ONCE(ioctl.done));
	KUNIT_EXPECT_EQ(test, ioctl.ret, 0L);
	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);

	uncopied = copy_from_user(&user_task, task_user, sizeof(user_task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, user_task.handle_flag & 1, 1U);
	KUNIT_EXPECT_EQ(test, user_task.src.yrgb_addr, 11ULL);
	KUNIT_EXPECT_EQ(test, user_task.dst.yrgb_addr, 12ULL);
	KUNIT_EXPECT_EQ(test, user_task.out_fence_fd, -1);

	hw.active_job = NULL;
	list_del_init(&hw.node);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 11),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 12),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_legacy_blit_async_acquire_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_session session = {};
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct dma_fence *release_fence;
	struct dma_fence *fd_fence;
	void __user *task_user;
	unsigned long uncopied;
	int acquire_fd;
	int release_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	KUNIT_ASSERT_NOT_NULL(test, task_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.in_fence_fd = acquire_fd;
	task.src.yrgb_addr = 11;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 12;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	task.out_fence_fd = -1;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	rk_rga_session_init(&session);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 12, 13,
				  GFP_KERNEL),
			12);

	mutex_init(&rk_rga.hw_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	spin_lock_init(&rk_rga.fence_lock);
	rk_rga.fence_context = dma_fence_context_alloc(1);
	rk_rga.fence_seqno = 0;

	ret = rk_rga_ioctl_blit((unsigned long)task_user, &session,
				RGA_BLIT_ASYNC);
	KUNIT_ASSERT_EQ(test, ret, 0L);

	uncopied = copy_from_user(&task, task_user, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, task.handle_flag & 1, 0U);
	KUNIT_EXPECT_EQ(test, task.src.yrgb_addr, src_import->iova);
	KUNIT_EXPECT_EQ(test, task.dst.yrgb_addr, dst_import->iova);
	release_fd = task.out_fence_fd;
	KUNIT_ASSERT_GE(test, release_fd, 0);

	fd_fence = sync_file_get_fence(acquire_fd);
	KUNIT_ASSERT_NOT_NULL(test, fd_fence);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, acquire_fence);
	dma_fence_put(fd_fence);

	release_fence = sync_file_get_fence(release_fd);
	KUNIT_ASSERT_NOT_NULL(test, release_fence);
	KUNIT_EXPECT_GT(test,
			dma_fence_wait_timeout(release_fence, false,
					       msecs_to_jiffies(1000)),
			0L);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);

	rk_rga_fence_signal(acquire_fence, 0);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);

	close_fd(release_fd);
	dma_fence_put(release_fence);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 11),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 12),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_request_submit_async_acquire_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rga_user_request user = {
		.task_num = 1,
		.id = 7,
		.sync_mode = RGA_BLIT_ASYNC,
	};
	struct rk_rga_session session = {};
	struct rk_rga_request *request;
	struct rk_rga_import *src_import;
	struct rk_rga_import *dst_import;
	struct dma_fence *acquire_fence;
	struct dma_fence *release_fence;
	struct dma_fence *fd_fence;
	void __user *task_user;
	void __user *request_user;
	unsigned long uncopied;
	int acquire_fd = -1;
	int release_fd;
	long ret;

	task_user = rk_rga_kunit_user_buffer(test, sizeof(task));
	request_user = rk_rga_kunit_user_buffer(test, sizeof(user));
	KUNIT_ASSERT_NOT_NULL(test, task_user);
	KUNIT_ASSERT_NOT_NULL(test, request_user);

	acquire_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, acquire_fence);
	acquire_fd = rk_rga_kunit_install_fence_fd(acquire_fence);
	KUNIT_ASSERT_GE(test, acquire_fd, 0);

	task.handle_flag = 1;
	task.feature.user_close_fence = 1;
	task.src.yrgb_addr = 11;
	task.src.uv_addr = 0;
	task.src.v_addr = 0;
	task.dst.yrgb_addr = 12;
	task.dst.uv_addr = 0;
	task.dst.v_addr = 0;
	user.task_ptr = (uintptr_t)task_user;
	user.acquire_fence_fd = acquire_fd;

	uncopied = copy_to_user(task_user, &task, sizeof(task));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(request_user, &user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	rk_rga_session_init(&session);
	request = kzalloc_obj(*request, GFP_KERNEL);
	src_import = rk_rga_kunit_import(test);
	dst_import = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_NOT_NULL(test, src_import);
	KUNIT_ASSERT_NOT_NULL(test, dst_import);
	src_import->iova = 0x10000000;
	src_import->size = (size_t)1920 * 1080 * 4;
	dst_import->iova = 0x20000000;
	dst_import->size = (size_t)1280 * 720 * 4;

	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.requests, request, 7, 8,
				  GFP_KERNEL),
			7);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, src_import, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, dst_import, 12, 13,
				  GFP_KERNEL),
			12);

	mutex_init(&rk_rga.hw_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	spin_lock_init(&rk_rga.fence_lock);
	rk_rga.fence_context = dma_fence_context_alloc(1);
	rk_rga.fence_seqno = 0;

	ret = rk_rga_ioctl_request_submit((unsigned long)request_user,
					  &session, true);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 7), NULL);

	uncopied = copy_from_user(&user, request_user, sizeof(user));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	release_fd = (int)user.release_fence_fd;
	KUNIT_ASSERT_GE(test, release_fd, 0);

	fd_fence = sync_file_get_fence(acquire_fd);
	KUNIT_ASSERT_NOT_NULL(test, fd_fence);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, acquire_fence);
	dma_fence_put(fd_fence);

	release_fence = sync_file_get_fence(release_fd);
	KUNIT_ASSERT_NOT_NULL(test, release_fence);
	KUNIT_EXPECT_GT(test,
			dma_fence_wait_timeout(release_fence, false,
					       msecs_to_jiffies(1000)),
			0L);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);
	KUNIT_EXPECT_EQ(test, refcount_read(&src_import->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&dst_import->refs), 1);

	rk_rga_fence_signal(acquire_fence, 0);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(release_fence), -ENODEV);

	close_fd(release_fd);
	dma_fence_put(release_fence);
	close_fd(acquire_fd);
	dma_fence_put(acquire_fence);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 11),
			    src_import);
	KUNIT_EXPECT_PTR_EQ(test, idr_remove(&session.imports, 12),
			    dst_import);
	rk_rga_import_put(src_import);
	rk_rga_import_put(dst_import);
	idr_destroy(&session.requests);
	idr_destroy(&session.imports);
}

static void rk_rga_version_queries_kunit(struct kunit *test)
{
	const struct rk_rga_hw_match rga3_match = {
		.type = RK_RGA_HW_RGA3,
		.name = "rga3",
		.version_major = 3,
		.version_minor = 0,
		.version_revision = 0x76831,
	};
	const struct rk_rga_hw_match rga2_match = {
		.type = RK_RGA_HW_RGA2,
		.name = "rga2",
		.version_major = 3,
		.version_minor = 2,
		.version_revision = 0x63318,
	};
	struct rk_rga_hw rga3 = {
		.type = RK_RGA_HW_RGA3,
		.match = &rga3_match,
		.version = {
			.major = 3,
			.minor = 0,
			.revision = 0x76831,
		},
	};
	struct rk_rga_hw rga2 = {
		.type = RK_RGA_HW_RGA2,
		.match = &rga2_match,
		.version = {
			.major = 3,
			.minor = 2,
			.revision = 0x63318,
		},
	};
	struct rga_hw_versions_t hw_versions = {};
	struct rga_version_t driver_version = {};
	char legacy_version[RGA_VERSION_SIZE] = {};
	char rga2_version[RGA_VERSION_SIZE] = {};
	void __user *legacy_user;
	void __user *rga2_user;
	void __user *hw_user;
	void __user *driver_user;
	unsigned long uncopied;

	legacy_user = rk_rga_kunit_user_buffer(test, sizeof(legacy_version));
	rga2_user = rk_rga_kunit_user_buffer(test, sizeof(rga2_version));
	hw_user = rk_rga_kunit_user_buffer(test, sizeof(hw_versions));
	driver_user = rk_rga_kunit_user_buffer(test, sizeof(driver_version));
	KUNIT_ASSERT_NOT_NULL(test, legacy_user);
	KUNIT_ASSERT_NOT_NULL(test, rga2_user);
	KUNIT_ASSERT_NOT_NULL(test, hw_user);
	KUNIT_ASSERT_NOT_NULL(test, driver_user);

	mutex_init(&rk_rga.hw_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	INIT_LIST_HEAD(&rga3.node);
	INIT_LIST_HEAD(&rga2.node);
	list_add_tail(&rga3.node, &rk_rga.hw_list);
	list_add_tail(&rga2.node, &rk_rga.hw_list);
	rk_rga_refresh_hw_versions();

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_get_version((unsigned long)legacy_user),
			0L);
	uncopied = copy_from_user(legacy_version, legacy_user,
				  sizeof(legacy_version));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_STREQ(test, legacy_version, "3.00");

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_get_rga2_version((unsigned long)rga2_user),
			1L);
	uncopied = copy_from_user(rga2_version, rga2_user,
				  sizeof(rga2_version));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_STREQ(test, rga2_version, "3.2.63318");

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_get_hw_versions((unsigned long)hw_user),
			1L);
	uncopied = copy_from_user(&hw_versions, hw_user, sizeof(hw_versions));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, hw_versions.size, 2U);
	KUNIT_EXPECT_STREQ(test, hw_versions.version[0].str, "3.0.76831");
	KUNIT_EXPECT_STREQ(test, hw_versions.version[1].str, "3.2.63318");

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_get_driver_version((unsigned long)driver_user),
			1L);
	uncopied = copy_from_user(&driver_version, driver_user,
				  sizeof(driver_version));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, driver_version.major, 1U);
	KUNIT_EXPECT_EQ(test, driver_version.minor, 3U);
	KUNIT_EXPECT_EQ(test, driver_version.revision, 11U);
	KUNIT_EXPECT_STREQ(test, driver_version.str, DRIVER_VERSION);

	strscpy(rga2_version, "unchanged", sizeof(rga2_version));
	uncopied = copy_to_user(rga2_user, rga2_version,
				sizeof(rga2_version));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	list_del_init(&rga2.node);
	rk_rga_refresh_hw_versions();
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_get_rga2_version((unsigned long)rga2_user),
			-EFAULT);
	memset(rga2_version, 0, sizeof(rga2_version));
	uncopied = copy_from_user(rga2_version, rga2_user,
				  sizeof(rga2_version));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_STREQ(test, rga2_version, "unchanged");

	list_del_init(&rga3.node);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	rk_rga_refresh_hw_versions();
}

static void rk_rga_legacy_noop_ioctls_kunit(struct kunit *test)
{
	static const unsigned int cmds[] = {
		RGA_CACHE_FLUSH,
		RGA_FLUSH,
		RGA2_FLUSH,
		RGA_GET_RESULT,
		RGA2_GET_RESULT,
	};
	struct rk_rga_session session = {};
	struct file file = {
		.private_data = &session,
	};
	int base_count;
	size_t i;

	base_count = atomic_read(&rk_rga.ioctl_count);

	for (i = 0; i < ARRAY_SIZE(cmds); i++) {
		KUNIT_EXPECT_EQ(test, rk_rga_ioctl(&file, cmds[i], 0), 0L);
		KUNIT_EXPECT_EQ(test, atomic_read(&rk_rga.ioctl_count),
				base_count + (int)i + 1);
	}

	file.private_data = NULL;
	KUNIT_EXPECT_EQ(test, rk_rga_ioctl(&file, RGA_FLUSH, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, atomic_read(&rk_rga.ioctl_count),
			base_count + (int)ARRAY_SIZE(cmds));
}

static void rk_rga_request_remove_free_kunit(struct kunit *test)
{
	struct rk_rga_session session;
	struct rk_rga_request *request;

	memset(&session, 0, sizeof(session));
	mutex_init(&session.lock);
	idr_init(&session.requests);

	request = kzalloc_obj(*request, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, request);
	KUNIT_ASSERT_EQ(test, idr_alloc(&session.requests, request, 7, 8,
				       GFP_KERNEL), 7);

	KUNIT_EXPECT_TRUE(test, rk_rga_request_remove_free(&session, 7));
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.requests, 7), NULL);
	KUNIT_EXPECT_FALSE(test, rk_rga_request_remove_free(&session, 7));

	idr_destroy(&session.requests);
}

static void rk_rga_import_buffer_size_kunit(struct kunit *test)
{
	struct rga_external_buffer buffer = {
		.type = RGA_VIRTUAL_ADDRESS,
		.memory = 0x100000,
		.memory_parm = {
			.size = 4096,
		},
	};
	size_t size = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_import_buffer_size(&buffer, &size), 0);
	KUNIT_EXPECT_EQ(test, size, (size_t)4096);

	buffer.memory_parm = (struct rga_memory_parm) {
		.width = 64,
		.height = 32,
		.format = RK_RGA_FORMAT_RGBA_8888,
	};
	size = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_import_buffer_size(&buffer, &size), 0);
	KUNIT_EXPECT_EQ(test, size, (size_t)(64 * 32 * 4));

	buffer.memory_parm.format = RK_RGA_FORMAT_YCBCR_420_SP;
	size = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_import_buffer_size(&buffer, &size), 0);
	KUNIT_EXPECT_EQ(test, size, (size_t)(64 * 32 * 3 / 2));

	buffer.memory_parm.width = 6;
	buffer.memory_parm.height = 5;
	buffer.memory_parm.format = RK_RGA_FORMAT_BPP4;
	size = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_import_buffer_size(&buffer, &size), 0);
	KUNIT_EXPECT_EQ(test, size, (size_t)(ALIGN(6U >> 1, 4) * 5));

	buffer.type = RGA_PHYSICAL_ADDRESS;
	buffer.memory_parm.size = 4096;
	KUNIT_EXPECT_EQ(test, rk_rga_import_one(NULL, &buffer), -EOPNOTSUPP);
}

static void rk_rga_iova_span_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_rga_check_iova_span(0, 1, "test", false), 0);
	KUNIT_EXPECT_EQ(test,
			rk_rga_check_iova_span(U32_MAX, 1, "test", false), 0);
	KUNIT_EXPECT_EQ(test,
			rk_rga_check_iova_span(0, 0, "test", false), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_check_iova_span(U32_MAX, 2, "test", false),
			-EOVERFLOW);
	KUNIT_EXPECT_EQ(test,
			rk_rga_check_iova_span((dma_addr_t)U32_MAX + 1, 1,
					       "test", false),
			-EOVERFLOW);
}

static void rk_rga_clock_count_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_rga_validate_clock_count(-EPROBE_DEFER),
			-EPROBE_DEFER);
	KUNIT_EXPECT_EQ(test, rk_rga_validate_clock_count(0), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_rga_validate_clock_count(3), 3);
}

static void rk_rga_mmio_size_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_mmio_size(RK_RGA2_MIN_REG_SIZE,
						  RK_RGA2_MIN_REG_SIZE - 1),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_mmio_size(RK_RGA2_MIN_REG_SIZE,
						  RK_RGA2_MIN_REG_SIZE),
			0);
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_mmio_size(RK_RGA3_MIN_REG_SIZE,
						  RK_RGA3_MIN_REG_SIZE - 1),
			-EINVAL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_mmio_size(RK_RGA3_MIN_REG_SIZE,
						  RK_RGA3_MIN_REG_SIZE),
			0);
}

static void rk_rga_hw_version_kunit(struct kunit *test)
{
	const struct rk_rga_hw_match rga2_match = {
		.type = RK_RGA_HW_RGA2,
		.version_major = 3,
		.version_minor = 2,
		.version_revision = 0x63318,
	};
	const struct rk_rga_hw_match rga3_match = {
		.type = RK_RGA_HW_RGA3,
		.version_major = 3,
		.version_minor = 0,
		.version_revision = 0x76831,
	};
	struct rga_version_t version = {};

	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_hw_version(&rga2_match, 0x03263318,
						   &version),
			0);
	KUNIT_EXPECT_STREQ(test, version.str, "3.2.63318");
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_hw_version(&rga3_match, 0x30076831,
						   &version),
			0);
	KUNIT_EXPECT_STREQ(test, version.str, "3.0.76831");

	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_hw_version(&rga2_match, 0x03363318,
						   &version),
			-ENODEV);
	KUNIT_EXPECT_STREQ(test, version.str, "3.3.63318");
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_hw_version(&rga3_match, 0x30176831,
						   &version),
			-ENODEV);
	KUNIT_EXPECT_STREQ(test, version.str, "3.1.76831");
	KUNIT_EXPECT_EQ(test,
			rk_rga_validate_hw_version(&rga2_match, 0, &version),
			-ENODEV);
}

static void rk_rga_import_dmabuf_fd_kunit(struct kunit *test)
{
	struct rga_external_buffer buffer = {};
	int fd = -1;

	KUNIT_EXPECT_EQ(test, rk_rga_import_dmabuf_fd(&buffer, &fd), 0);
	KUNIT_EXPECT_EQ(test, fd, 0);

	buffer.memory = INT_MAX;
	fd = -1;
	KUNIT_EXPECT_EQ(test, rk_rga_import_dmabuf_fd(&buffer, &fd), 0);
	KUNIT_EXPECT_EQ(test, fd, INT_MAX);

	buffer.memory = (u64)INT_MAX + 1;
	fd = -1;
	KUNIT_EXPECT_EQ(test, rk_rga_import_dmabuf_fd(&buffer, &fd), -EINVAL);
	KUNIT_EXPECT_EQ(test, fd, -1);

	buffer.memory = U64_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_import_dmabuf_fd(&buffer, &fd), -EINVAL);
	KUNIT_EXPECT_EQ(test, fd, -1);
}

static void rk_rga_import_buffer_ioctl_errors_kunit(struct kunit *test)
{
	struct rga_external_buffer buffer = {
		.type = RGA_PHYSICAL_ADDRESS,
		.memory = 0x100000,
		.memory_parm = {
			.size = 4096,
		},
	};
	struct rga_buffer_pool pool = {
		.size = 1,
	};
	struct rk_rga_session session = {};
	void __user *pool_user;
	void __user *buffer_user;
	unsigned long uncopied;

	pool_user = rk_rga_kunit_user_buffer(test, sizeof(pool));
	buffer_user = rk_rga_kunit_user_buffer(test, sizeof(buffer));
	KUNIT_ASSERT_NOT_NULL(test, pool_user);
	KUNIT_ASSERT_NOT_NULL(test, buffer_user);

	pool.buffers_ptr = (uintptr_t)buffer_user;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(buffer_user, &buffer, sizeof(buffer));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.imports);

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EOPNOTSUPP);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.imports, 1), NULL);

	buffer.type = RGA_DMA_BUFFER;
	buffer.memory = (u64)INT_MAX + 1;
	uncopied = copy_to_user(buffer_user, &buffer, sizeof(buffer));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EINVAL);
	KUNIT_EXPECT_TRUE(test, idr_is_empty(&session.imports));

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer(TASK_SIZE, &session),
			-EFAULT);

	pool.size = RGA_BUFFER_POOL_SIZE_MAX + 1;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EFBIG);

	pool.size = 1;
	pool.buffers_ptr = 0;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EFAULT);

	pool.buffers_ptr = TASK_SIZE;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EFAULT);

	idr_destroy(&session.imports);
}

static struct rk_rga_import *rk_rga_kunit_import(struct kunit *test)
{
	struct rk_rga_import *import;

	import = kzalloc_obj(*import, GFP_KERNEL);
	if (!import) {
		KUNIT_FAIL(test, "failed to allocate fake import");
		return NULL;
	}
	import->type = RK_RGA_IMPORT_USERPTR;
	refcount_set(&import->refs, 1);

	return import;
}

static void rk_rga_release_buffer_ioctl_kunit(struct kunit *test)
{
	struct rga_external_buffer buffers[2] = {
		{ .handle = 11 },
		{ .handle = 12 },
	};
	struct rga_buffer_pool pool = {
		.size = ARRAY_SIZE(buffers),
	};
	struct rk_rga_session session = {};
	struct rk_rga_import *import_a;
	struct rk_rga_import *import_b;
	void __user *pool_user;
	void __user *buffers_user;
	unsigned long uncopied;

	pool_user = rk_rga_kunit_user_buffer(test, sizeof(pool));
	buffers_user = rk_rga_kunit_user_buffer(test, sizeof(buffers));
	KUNIT_ASSERT_NOT_NULL(test, pool_user);
	KUNIT_ASSERT_NOT_NULL(test, buffers_user);

	pool.buffers_ptr = (uintptr_t)buffers_user;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	uncopied = copy_to_user(buffers_user, buffers, sizeof(buffers));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);

	mutex_init(&session.lock);
	idr_init(&session.imports);
	import_a = rk_rga_kunit_import(test);
	import_b = rk_rga_kunit_import(test);
	KUNIT_ASSERT_NOT_NULL(test, import_a);
	KUNIT_ASSERT_NOT_NULL(test, import_b);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, import_a, 11, 12,
				  GFP_KERNEL),
			11);
	KUNIT_ASSERT_EQ(test,
			idr_alloc(&session.imports, import_b, 12, 13,
				  GFP_KERNEL),
			12);
	import_a->counted = true;
	import_b->counted = true;
	atomic_set(&rk_rga.import_count, 2);

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			0L);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.imports, 11), NULL);
	KUNIT_EXPECT_PTR_EQ(test, idr_find(&session.imports, 12), NULL);
	KUNIT_EXPECT_EQ(test, atomic_read(&rk_rga.import_count), 0);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			-EINVAL);

	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer(TASK_SIZE, &session),
			-EFAULT);

	pool.size = RGA_BUFFER_POOL_SIZE_MAX + 1;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			-EFBIG);

	pool.size = 1;
	pool.buffers_ptr = 0;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			-EFAULT);

	idr_destroy(&session.imports);
}

static void rk_rga_buffer_pool_zero_count_kunit(struct kunit *test)
{
	struct rga_external_buffer buffer = {};
	struct rga_buffer_pool pool = {};
	struct rk_rga_session session = {};
	void __user *pool_user;
	void __user *buffer_user;
	unsigned long uncopied;

	pool_user = rk_rga_kunit_user_buffer(test, sizeof(pool));
	buffer_user = rk_rga_kunit_user_buffer(test, sizeof(buffer));
	KUNIT_ASSERT_NOT_NULL(test, pool_user);
	KUNIT_ASSERT_NOT_NULL(test, buffer_user);

	mutex_init(&session.lock);
	idr_init(&session.imports);

	pool.size = 0;
	pool.buffers_ptr = (uintptr_t)buffer_user;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			0L);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			0L);
	KUNIT_EXPECT_TRUE(test, idr_is_empty(&session.imports));

	pool.buffers_ptr = 0;
	uncopied = copy_to_user(pool_user, &pool, sizeof(pool));
	KUNIT_ASSERT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_import_buffer((unsigned long)pool_user,
						   &session),
			-EFAULT);
	KUNIT_EXPECT_EQ(test,
			rk_rga_ioctl_release_buffer((unsigned long)pool_user,
						    &session),
			-EFAULT);
	KUNIT_EXPECT_TRUE(test, idr_is_empty(&session.imports));

	idr_destroy(&session.imports);
}

static void rk_rga_acquire_fd_ownership_kunit(struct kunit *test)
{
	struct rk_rga_acquire_fd fds[2] = {};
	u32 count = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_record_acquire_fd(fds, &count, 5, true),
			0);
	KUNIT_EXPECT_EQ(test, count, 1U);
	KUNIT_EXPECT_EQ(test, fds[0].fd, 5);
	KUNIT_EXPECT_TRUE(test, fds[0].kernel_close);

	KUNIT_EXPECT_TRUE(test, rk_rga_update_acquire_fd(fds, count, 5,
							 false));
	KUNIT_EXPECT_FALSE(test, fds[0].kernel_close);

	KUNIT_EXPECT_TRUE(test, rk_rga_update_acquire_fd(fds, count, 5,
							 true));
	KUNIT_EXPECT_FALSE(test, fds[0].kernel_close);
	KUNIT_EXPECT_FALSE(test, rk_rga_update_acquire_fd(fds, count, 7,
							  true));

	KUNIT_EXPECT_EQ(test, rk_rga_record_acquire_fd(fds, &count, 6, true),
			0);
	KUNIT_EXPECT_EQ(test, count, 2U);
	KUNIT_EXPECT_EQ(test, rk_rga_record_acquire_fd(fds, &count, 7, true),
			-EOVERFLOW);
	KUNIT_EXPECT_EQ(test, count, 2U);
}

static void rk_rga_acquire_fence_status_kunit(struct kunit *test)
{
	struct dma_fence *fences[2];
	struct dma_fence *err_fence;
	struct rk_rga_job job = {
		.acquire_fences = fences,
		.acquire_fence_count = ARRAY_SIZE(fences),
	};
	bool pending = false;

	fences[0] = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fences[0]);
	fences[1] = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fences[1]);

	KUNIT_EXPECT_EQ(test, rk_rga_job_acquire_status(&job, &pending), 0);
	KUNIT_EXPECT_TRUE(test, pending);

	rk_rga_fence_signal(fences[0], 0);
	pending = false;
	KUNIT_EXPECT_EQ(test, rk_rga_job_acquire_status(&job, &pending), 0);
	KUNIT_EXPECT_TRUE(test, pending);

	rk_rga_fence_signal(fences[1], 0);
	pending = false;
	KUNIT_EXPECT_EQ(test, rk_rga_job_acquire_status(&job, &pending), 0);
	KUNIT_EXPECT_FALSE(test, pending);
	KUNIT_EXPECT_EQ(test, rk_rga_job_wait_acquire_fences(&job), 0);

	dma_fence_put(fences[0]);
	dma_fence_put(fences[1]);

	err_fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, err_fence);
	rk_rga_fence_signal(err_fence, -EIO);
	job.acquire_fences = &err_fence;
	job.acquire_fence_count = 1;
	pending = false;

	KUNIT_EXPECT_EQ(test, rk_rga_job_acquire_status(&job, &pending),
			-EIO);
	KUNIT_EXPECT_FALSE(test, pending);
	KUNIT_EXPECT_EQ(test, rk_rga_job_wait_acquire_fences(&job), -EIO);

	dma_fence_put(err_fence);
}

static void rk_rga_acquire_callbacks_result_kunit(struct kunit *test)
{
	struct rk_rga_job *job;
	int ret;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);

	job->acquire_fences = kcalloc(2, sizeof(*job->acquire_fences),
				      GFP_KERNEL);
	if (!job->acquire_fences) {
		kfree(job);
		KUNIT_FAIL(test, "failed to allocate acquire fence array");
		return;
	}
	job->acquire_fence_count = 2;

	job->acquire_fences[0] = rk_rga_kunit_alloc_fence();
	if (!job->acquire_fences[0]) {
		kfree(job->acquire_fences);
		kfree(job);
		KUNIT_FAIL(test, "failed to allocate ready fence");
		return;
	}
	job->acquire_fences[1] = rk_rga_kunit_alloc_fence();
	if (!job->acquire_fences[1]) {
		dma_fence_put(job->acquire_fences[0]);
		kfree(job->acquire_fences);
		kfree(job);
		KUNIT_FAIL(test, "failed to allocate pending fence");
		return;
	}

	rk_rga_fence_signal(job->acquire_fences[0], 0);
	rk_rga_job_get(job);
	ret = rk_rga_job_arm_acquire_callbacks(job);
	KUNIT_EXPECT_EQ(test, ret, 0);
	if (ret) {
		rk_rga_job_put(job);
		rk_rga_job_put(job);
		return;
	}

	KUNIT_EXPECT_EQ(test, atomic_read(&job->pending_acquire_count), 1);
	KUNIT_EXPECT_TRUE(test, job->waiting_acquire);
	KUNIT_EXPECT_FALSE(test, job->done);

	rk_rga_fence_signal(job->acquire_fences[1], -EIO);
	flush_work(&job->acquire_work);

	KUNIT_EXPECT_EQ(test, job->result, -EIO);
	KUNIT_EXPECT_FALSE(test, job->waiting_acquire);
	KUNIT_EXPECT_TRUE(test, job->done);

	rk_rga_job_put(job);
}

static void rk_rga_acquire_abort_during_arming_kunit(struct kunit *test)
{
	struct rk_rga_job *job;
	struct dma_fence *fences[2];
	bool last;
	int ret;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);

	job->acquire_fences = kcalloc(2, sizeof(*job->acquire_fences),
				      GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job->acquire_fences);
	job->acquire_waiters = kcalloc(2, sizeof(*job->acquire_waiters),
				       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job->acquire_waiters);
	job->acquire_fence_count = 2;
	for (u32 i = 0; i < ARRAY_SIZE(fences); i++) {
		fences[i] = rk_rga_kunit_alloc_fence();
		KUNIT_ASSERT_NOT_NULL(test, fences[i]);
		job->acquire_fences[i] = fences[i];
	}

	atomic_set(&job->pending_acquire_count, 1);
	atomic_set(&job->acquire_work_queued, 0);
	WRITE_ONCE(job->result, 0);
	WRITE_ONCE(job->waiting_acquire, true);
	rk_rga_job_get(job);
	atomic_inc(&job->pending_acquire_count);
	job->acquire_waiters[0].job = job;
	ret = dma_fence_add_callback(fences[0],
				     &job->acquire_waiters[0].cb,
				     rk_rga_job_acquire_cb);
	KUNIT_ASSERT_EQ(test, ret, 0);

	rk_rga_job_abort_pending_acquire(job, -ECANCELED);
	KUNIT_EXPECT_EQ(test, atomic_read(&job->pending_acquire_count), 1);
	KUNIT_EXPECT_PTR_EQ(test, job->acquire_waiters[0].job, NULL);
	KUNIT_EXPECT_EQ(test, atomic_read(&job->acquire_work_queued), 0);
	KUNIT_EXPECT_FALSE(test, job->done);

	atomic_inc(&job->pending_acquire_count);
	job->acquire_waiters[1].job = job;
	ret = dma_fence_add_callback(fences[1],
				     &job->acquire_waiters[1].cb,
				     rk_rga_job_acquire_cb);
	KUNIT_ASSERT_EQ(test, ret, 0);
	last = atomic_dec_and_test(&job->pending_acquire_count);
	KUNIT_EXPECT_FALSE(test, last);

	rk_rga_fence_signal(fences[1], 0);
	flush_work(&job->acquire_work);
	KUNIT_EXPECT_TRUE(test, job->done);
	KUNIT_EXPECT_EQ(test, job->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);

	rk_rga_fence_signal(fences[0], 0);
	KUNIT_EXPECT_EQ(test, job->result, -ECANCELED);
	rk_rga_job_put(job);
}

static void rk_rga_session_dispatch_close_handoff_kunit(struct kunit *test)
{
	struct rk_rga_session dispatch_session = { };
	struct rk_rga_session worker_session = { };
	struct rk_rga_job *job;

	rk_rga_session_init(&dispatch_session);
	/* Acquire workers and multi-task IRQ handoffs use this same claim. */
	KUNIT_EXPECT_TRUE(test,
			  rk_rga_session_begin_job_dispatch(&dispatch_session));
	KUNIT_EXPECT_EQ(test, dispatch_session.dispatching_jobs, 1U);

	rk_rga_session_mark_closing(&dispatch_session);
	KUNIT_EXPECT_FALSE(test,
			   rk_rga_session_dispatches_idle(&dispatch_session));
	KUNIT_EXPECT_FALSE(test,
			   rk_rga_session_begin_job_dispatch(&dispatch_session));
	rk_rga_session_end_job_dispatch(&dispatch_session);
	KUNIT_EXPECT_TRUE(test,
			  rk_rga_session_dispatches_idle(&dispatch_session));
	idr_destroy(&dispatch_session.imports);
	idr_destroy(&dispatch_session.requests);

	rk_rga_session_init(&worker_session);
	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);
	KUNIT_ASSERT_EQ(test,
			rk_rga_session_track_job(&worker_session, job), 0);

	/* Model the shared reference owned by queued acquire work. */
	rk_rga_job_get(job);
	rk_rga_session_mark_closing(&worker_session);
	rk_rga_job_acquire_work(&job->acquire_work);

	KUNIT_EXPECT_TRUE(test, job->done);
	KUNIT_EXPECT_EQ(test, job->result, -EFAULT);
	KUNIT_EXPECT_PTR_EQ(test, job->session, NULL);
	KUNIT_EXPECT_TRUE(test, list_empty(&worker_session.jobs));
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);

	rk_rga_job_put(job);
	idr_destroy(&worker_session.imports);
	idr_destroy(&worker_session.requests);
}

static void rk_rga_job_free_release_fence_kunit(struct kunit *test)
{
	struct rk_rga_job *job;
	struct dma_fence *fence;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);

	fence = rk_rga_kunit_alloc_fence();
	if (!fence) {
		kfree(job);
		KUNIT_FAIL(test, "failed to allocate dma fence");
		return;
	}

	dma_fence_get(fence);
	job->release_fence = fence;

	rk_rga_job_free(job);

	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence),
			RK_RGA_RELEASE_FENCE_ABORT_ERR);
	dma_fence_put(fence);

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);

	fence = rk_rga_kunit_alloc_fence();
	if (!fence) {
		kfree(job);
		KUNIT_FAIL(test, "failed to allocate dma fence");
		return;
	}

	rk_rga_fence_signal(fence, 0);
	dma_fence_get(fence);
	job->release_fence = fence;

	rk_rga_job_free(job);

	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence), 1);
	dma_fence_put(fence);
}

static void rk_rga_release_fence_fd_state_kunit(struct kunit *test)
{
	struct sync_file *sync_file;
	struct dma_fence *fd_fence;
	struct dma_fence *fence;
	struct rk_rga_job job = { };
	int fd;

	rk_rga_job_init(&job);
	job.release_fence_fd = 9;

	rk_rga_job_forget_release_fence_fd(&job, -1);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, 9);

	rk_rga_job_forget_release_fence_fd(&job, 8);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, 9);

	rk_rga_job_forget_release_fence_fd(NULL, 9);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, 9);

	rk_rga_job_forget_release_fence_fd(&job, 9);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, -1);

	fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence);
	fd = rk_rga_fence_create_fd(fence, &sync_file);
	KUNIT_ASSERT_GE(test, fd, 0);
	job.release_fence_fd = fd;

	/* Reserved descriptors must stay invisible until user copy succeeds. */
	fd_fence = sync_file_get_fence(fd);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, NULL);
	if (fd_fence)
		dma_fence_put(fd_fence);

	rk_rga_job_install_fd(&job, sync_file);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, -1);
	fd_fence = sync_file_get_fence(fd);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, fence);
	if (fd_fence)
		dma_fence_put(fd_fence);
	close_fd(fd);
	dma_fence_put(fence);

	fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence);
	fd = rk_rga_fence_create_fd(fence, &sync_file);
	KUNIT_ASSERT_GE(test, fd, 0);
	job.release_fence_fd = fd;
	rk_rga_job_abort_fd(&job, sync_file);
	KUNIT_EXPECT_EQ(test, job.release_fence_fd, -1);
	fd_fence = sync_file_get_fence(fd);
	KUNIT_EXPECT_PTR_EQ(test, fd_fence, NULL);
	if (fd_fence)
		dma_fence_put(fd_fence);
	dma_fence_put(fence);
}

static void rk_rga_hw_abort_queued_jobs_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = { };
	struct rk_rga_job *job0;
	struct rk_rga_job *job1;
	struct dma_fence *fence0;
	struct dma_fence *fence1;

	mutex_init(&hw.run_lock);
	spin_lock_init(&hw.job_lock);
	init_waitqueue_head(&hw.idle);
	INIT_LIST_HEAD(&hw.job_queue);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_rga_hw_timeout_work);
	refcount_set(&hw.refs, 3);

	job0 = kzalloc_obj(*job0, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kzalloc_obj(*job1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	rk_rga_job_init(job0);
	rk_rga_job_init(job1);
	rk_rga_job_get(job0);
	rk_rga_job_get(job1);
	job0->hw = &hw;
	job1->hw = &hw;
	job0->queued = true;
	job1->queued = true;

	fence0 = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence0);
	fence1 = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence1);
	dma_fence_get(fence0);
	dma_fence_get(fence1);
	job0->release_fence = fence0;
	job1->release_fence = fence1;

	list_add_tail(&job0->node, &hw.job_queue);
	list_add_tail(&job1->node, &hw.job_queue);
	hw.queued_jobs = 2;

	rk_rga_hw_abort_jobs(&hw, -ENODEV);

	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_FALSE(test, job0->queued);
	KUNIT_EXPECT_FALSE(test, job1->queued);
	KUNIT_EXPECT_TRUE(test, job0->done);
	KUNIT_EXPECT_TRUE(test, job1->done);
	KUNIT_EXPECT_EQ(test, job0->result, -ENODEV);
	KUNIT_EXPECT_EQ(test, job1->result, -ENODEV);
	KUNIT_EXPECT_PTR_EQ(test, job0->hw, NULL);
	KUNIT_EXPECT_PTR_EQ(test, job1->hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&job0->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence0), -ENODEV);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence1), -ENODEV);

	rk_rga_job_put(job0);
	rk_rga_job_put(job1);
	dma_fence_put(fence0);
	dma_fence_put(fence1);
}

static void rk_rga_queue_on_removing_hw_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = { };
	struct rk_rga_job *job;
	struct dma_fence *fence;

	spin_lock_init(&hw.job_lock);
	INIT_LIST_HEAD(&hw.job_queue);
	refcount_set(&hw.refs, 2);
	hw.removing = true;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);
	job->hw = NULL;

	fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence);
	dma_fence_get(fence);
	job->release_fence = fence;

	KUNIT_EXPECT_EQ(test, rk_rga_job_queue_on_hw(job, &hw, true),
			-ENODEV);
	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_FALSE(test, job->queued);
	KUNIT_EXPECT_TRUE(test, job->done);
	KUNIT_EXPECT_EQ(test, job->result, -ENODEV);
	KUNIT_EXPECT_PTR_EQ(test, job->hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence), -ENODEV);

	rk_rga_job_put(job);
	dma_fence_put(fence);
}

static void rk_rga_queue_on_recovery_failed_hw_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = { };
	struct rk_rga_job *job;
	struct dma_fence *fence;

	spin_lock_init(&hw.job_lock);
	INIT_LIST_HEAD(&hw.job_queue);
	refcount_set(&hw.refs, 2);
	hw.recovery_failed = true;

	job = kzalloc_obj(*job, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	rk_rga_job_init(job);

	fence = rk_rga_kunit_alloc_fence();
	KUNIT_ASSERT_NOT_NULL(test, fence);
	dma_fence_get(fence);
	job->release_fence = fence;

	KUNIT_EXPECT_EQ(test, rk_rga_job_queue_on_hw(job, &hw, true), -EIO);
	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_FALSE(test, job->queued);
	KUNIT_EXPECT_TRUE(test, job->done);
	KUNIT_EXPECT_EQ(test, job->result, -EIO);
	KUNIT_EXPECT_PTR_EQ(test, job->hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);
	KUNIT_EXPECT_EQ(test, dma_fence_get_status(fence), -EIO);

	rk_rga_job_put(job);
	dma_fence_put(fence);
}

static void rk_rga_recovery_failed_dispatch_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = { .recovery_failed = true };
	struct rk_rga_job *job0;
	struct rk_rga_job *job1;

	mutex_init(&hw.run_lock);
	spin_lock_init(&hw.job_lock);
	init_waitqueue_head(&hw.idle);
	INIT_LIST_HEAD(&hw.job_queue);
	refcount_set(&hw.refs, 3);

	job0 = kzalloc_obj(*job0, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kzalloc_obj(*job1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	rk_rga_job_init(job0);
	rk_rga_job_init(job1);
	rk_rga_job_get(job0);
	rk_rga_job_get(job1);
	job0->hw = &hw;
	job1->hw = &hw;
	job0->queued = true;
	job1->queued = true;
	list_add_tail(&job0->node, &hw.job_queue);
	list_add_tail(&job1->node, &hw.job_queue);
	hw.queued_jobs = 2;

	rk_rga_hw_dispatch(&hw);

	KUNIT_EXPECT_TRUE(test, list_empty(&hw.job_queue));
	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 0U);
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, NULL);
	KUNIT_EXPECT_TRUE(test, job0->done);
	KUNIT_EXPECT_TRUE(test, job1->done);
	KUNIT_EXPECT_EQ(test, job0->result, -EIO);
	KUNIT_EXPECT_EQ(test, job1->result, -EIO);
	KUNIT_EXPECT_EQ(test, refcount_read(&job0->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&hw.refs), 1);

	rk_rga_job_put(job0);
	rk_rga_job_put(job1);
}

static void rk_rga_irq_completion_result_kunit(struct kunit *test)
{
	struct rk_rga_job job = { };

	job.intr_status = RK_RGA3_INT_FRM_DONE;
	KUNIT_EXPECT_EQ(test,
			rk_rga_irq_completion_result(RK_RGA_HW_RGA3, &job), 0);
	job.intr_status = RK_RGA3_INT_FRM_DONE | RK_RGA3_INT_RGA_MMU_INTR;
	KUNIT_EXPECT_EQ(test,
			rk_rga_irq_completion_result(RK_RGA_HW_RGA3, &job),
			-EACCES);
	job.intr_status = RK_RGA3_INT_FRM_DONE |
			  RK_RGA3_INT_RGA_MI_RD_BUS_ERR;
	KUNIT_EXPECT_EQ(test,
			rk_rga_irq_completion_result(RK_RGA_HW_RGA3, &job),
			-EFAULT);

	job.intr_status = RK_RGA2_INT_ALL_CMD_DONE_INT_FLAG |
			  RK_RGA2_INT_MMU_INT_FLAG;
	KUNIT_EXPECT_EQ(test,
			rk_rga_irq_completion_result(RK_RGA_HW_RGA2, &job),
			-EACCES);
	KUNIT_EXPECT_EQ(test,
			rk_rga_irq_completion_result((enum rk_rga_hw_type)-1,
						     &job),
			-EINVAL);
}

static void rk_rga_mixed_task_hw_type_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	u32 type_mask = 0;
	struct rga_req tasks[2] = {
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_RGB_888),
		rk_rga_fill_task(0),
	};
	struct rk_rga_job job = {
		.tasks = tasks,
		.task_count = ARRAY_SIZE(tasks),
		.import_count = 2,
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_ALL);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	job.current_task = 1;
	type_mask = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_RGA2);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	tasks[0].core = BIT(1);
	tasks[1].core = BIT(3);
	job.current_task = 0;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	job.current_task = 1;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	tasks[1].core = BIT(0);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga_mixed_task_core_handoff_kunit(struct kunit *test)
{
	u32 type_mask = 0;
	struct rga_req tasks[2] = {
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888),
		rk_rga_fill_task(0),
	};
	struct rk_rga_job job = {
		.tasks = tasks,
		.task_count = ARRAY_SIZE(tasks),
		.import_count = 2,
	};
	struct rk_rga_hw *rga3;
	struct rk_rga_hw *rga2;
	struct rk_rga_hw *selected;
	LIST_HEAD(hw_list);

	rga3 = kunit_kzalloc(test, sizeof(*rga3), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, rga3);
	rga2 = kunit_kzalloc(test, sizeof(*rga2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, rga2);
	rga3->type = RK_RGA_HW_RGA3;
	rga3->core_mask = BIT(0);
	rga2->type = RK_RGA_HW_RGA2;
	rga2->core_mask = BIT(2);
	spin_lock_init(&rga3->job_lock);
	spin_lock_init(&rga2->job_lock);
	refcount_set(&rga3->refs, 2);
	refcount_set(&rga2->refs, 1);
	list_add_tail(&rga3->node, &hw_list);
	list_add_tail(&rga2->node, &hw_list);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_ALL);
	selected = rk_rga_find_best_hw_for_job(&hw_list, &job, type_mask, 0);
	KUNIT_ASSERT_PTR_EQ(test, selected, rga3);

	job.hw = selected;
	KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job, 0));
	rk_rga_job_release_hw(&job);
	KUNIT_EXPECT_PTR_EQ(test, job.hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&rga3->refs), 1);
	KUNIT_EXPECT_EQ(test, job.current_task, 1U);

	tasks[1].core = BIT(2);
	type_mask = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_RGA2);
	selected = rk_rga_find_best_hw_for_job(&hw_list, &job, type_mask, 0);
	KUNIT_EXPECT_PTR_EQ(test, selected, rga2);
	KUNIT_EXPECT_EQ(test, refcount_read(&rga2->refs), 1);
}

static void rk_rga_bitblt_hw_type_mask_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	u32 type_mask = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_ALL);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.core = BIT(2);
	type_mask = 0;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	task.core = BIT(1);
	type_mask = 0;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.core = 0;
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	type_mask = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask), 0);
	KUNIT_EXPECT_EQ(test, type_mask, RK_RGA_HW_TYPE_MASK_RGA3);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					 RK_RGA_FORMAT_RGBA_8888);
	task.src.rd_mode = RK_RGA_RKFBC_MODE;
	type_mask = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask),
			-EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, type_mask, 0U);
}

static void rk_rga_task_core_invalid_mask_kunit(struct kunit *test)
{
	const u32 invalid_core = RK_RGA_CORE_MASK | BIT(4);
	u32 type_mask;
	struct rga_req task;
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
	};

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_BGRA_8888);
	task.core = invalid_core;
	job.import_count = 2;
	type_mask = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, type_mask, 0U);

	task = rk_rga_fill_task(invalid_core);
	job.import_count = 1;
	type_mask = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, type_mask, 0U);

	task = (struct rga_req) {
		.render_mode = RK_RGA_RENDER_COLOR_PALETTE,
		.core = invalid_core,
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_400,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
					16, 16),
		.palette_mode = 3,
	};
	job.import_count = 3;
	type_mask = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, type_mask, 0U);

	task = (struct rga_req) {
		.render_mode = RK_RGA_RENDER_UPDATE_PALETTE,
		.core = invalid_core,
		.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
					16, 16),
		.palette_mode = 3,
	};
	job.import_count = 1;
	type_mask = U32_MAX;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type_mask(&job, &type_mask),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, type_mask, 0U);
}

static void rk_rga_find_best_hw_for_job_kunit(struct kunit *test)
{
	struct rga_req *task;
	struct rga_req *tasks;
	struct rk_rga_job *active;
	struct rk_rga_job *job;
	struct rk_rga_hw *busy;
	struct rk_rga_hw *idle;
	struct rk_rga_hw *rga2_idle;
	struct rk_rga_hw *rga2_other;
	struct rk_rga_hw *selected;
	u32 rr_start;
	LIST_HEAD(hw_list);

	task = kunit_kzalloc(test, sizeof(*task), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, task);
	tasks = kunit_kcalloc(test, 2, sizeof(*tasks), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	active = kunit_kzalloc(test, sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);
	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	busy = kunit_kzalloc(test, sizeof(*busy), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, busy);
	idle = kunit_kzalloc(test, sizeof(*idle), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, idle);
	rga2_idle = kunit_kzalloc(test, sizeof(*rga2_idle), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, rga2_idle);
	rga2_other = kunit_kzalloc(test, sizeof(*rga2_other), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, rga2_other);

	*task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	job->tasks = task;
	job->task_count = 1;
	busy->type = RK_RGA_HW_RGA3;
	busy->core_mask = BIT(0);
	busy->queued_jobs = 2;
	idle->type = RK_RGA_HW_RGA3;
	idle->core_mask = BIT(1);
	rga2_idle->type = RK_RGA_HW_RGA2;
	rga2_idle->core_mask = BIT(2);
	rga2_other->type = RK_RGA_HW_RGA2;
	rga2_other->core_mask = BIT(3);

	spin_lock_init(&busy->job_lock);
	spin_lock_init(&idle->job_lock);
	spin_lock_init(&rga2_idle->job_lock);
	spin_lock_init(&rga2_other->job_lock);
	list_add_tail(&busy->node, &hw_list);
	list_add_tail(&idle->node, &hw_list);
	list_add_tail(&rga2_idle->node, &hw_list);

	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA3,
							0),
			    idle);

	task->core = BIT(0);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    busy);

	task->core = BIT(1);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    idle);

	task->core = 0;
	idle->active_job = active;
	idle->queued_jobs = 3;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA3,
							0),
			    busy);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    rga2_idle);

	task->core = BIT(3);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    NULL);

	task->core = BIT(2) | BIT(3);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    rga2_idle);

	task->core = BIT(0);
	busy->removing = true;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    NULL);
	busy->removing = false;
	busy->recovery_failed = true;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    NULL);

	tasks[0] = *task;
	tasks[0].core = BIT(0);
	tasks[1] = *task;
	tasks[1].core = BIT(1);
	busy->recovery_failed = false;
	job->tasks = tasks;
	job->task_count = 2;
	job->current_task = 1;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							0),
			    idle);

	job->tasks = task;
	job->task_count = 1;
	job->current_task = 0;
	task->core = 0;
	idle->active_job = NULL;
	idle->queued_jobs = 0;
	busy->active_job = NULL;
	busy->queued_jobs = 0;
	busy->removing = false;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA3,
							0),
			    busy);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA3,
							1),
			    idle);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_ALL,
							2),
			    busy);

	task->core = BIT(2) | BIT(3);
	list_add_tail(&rga2_other->node, &hw_list);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA2,
							2),
			    rga2_idle);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, job,
							RK_RGA_HW_TYPE_MASK_RGA2,
							3),
			    rga2_other);

	task->core = 0;
	rr_start = 0;
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA3,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, busy);
	rr_start = rk_rga_core_select_next(selected, rr_start);
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA3,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, idle);
	rr_start = rk_rga_core_select_next(selected, rr_start);
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA3,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, busy);
	rr_start = rk_rga_core_select_next(selected, rr_start);
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA3,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, idle);

	task->core = BIT(2) | BIT(3);
	rr_start = 2;
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA2,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, rga2_idle);
	rr_start = rk_rga_core_select_next(selected, rr_start);
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA2,
					       rr_start);
	KUNIT_ASSERT_PTR_EQ(test, selected, rga2_other);
	rr_start = rk_rga_core_select_next(selected, rr_start);
	selected = rk_rga_find_best_hw_for_job(&hw_list, job,
					       RK_RGA_HW_TYPE_MASK_RGA2,
					       rr_start);
	KUNIT_EXPECT_PTR_EQ(test, selected, rga2_idle);
}

static void rk_rga_core_counter_kunit(struct kunit *test)
{
	atomic_t counters[RK_RGA_CORE_COUNTER_COUNT];
	atomic64_t total_ns[RK_RGA_CORE_COUNTER_COUNT];
	atomic64_t max_ns[RK_RGA_CORE_COUNTER_COUNT];
	struct rk_rga_hw hw = { .core_mask = BIT(1) };

	for (u32 i = 0; i < ARRAY_SIZE(counters); i++) {
		atomic_set(&counters[i], 0);
		atomic64_set(&total_ns[i], 0);
		atomic64_set(&max_ns[i], 0);
	}

	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(0)), 0);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(1)), 1);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(2)), 2);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(3)), 3);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(0), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(0) | BIT(1)),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, rk_rga_core_counter_index(BIT(4)), -EINVAL);

	rk_rga_count_core(counters, &hw);
	rk_rga_count_core_ns(total_ns, &hw, 100, false);
	rk_rga_count_core_ns(max_ns, &hw, 7, true);
	rk_rga_count_core_ns(max_ns, &hw, 3, true);
	KUNIT_EXPECT_EQ(test, atomic_read(&counters[0]), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&counters[1]), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&counters[2]), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&counters[3]), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&total_ns[1]), 100LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&max_ns[1]), 7LL);

	hw.core_mask = BIT(4);
	rk_rga_count_core(counters, &hw);
	rk_rga_count_core_ns(total_ns, &hw, 1000, false);
	rk_rga_count_core_ns(max_ns, &hw, 1000, true);
	KUNIT_EXPECT_EQ(test, atomic_read(&counters[1]), 1);
	KUNIT_EXPECT_EQ(test, atomic64_read(&total_ns[1]), 100LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&max_ns[1]), 7LL);
}

static void rk_rga_core_slot_reprobe_kunit(struct kunit *test)
{
	struct rk_rga_hw rga3_0 = {
		.type = RK_RGA_HW_RGA3,
		.core_mask = BIT(0),
	};
	struct rk_rga_hw rga3_1 = {
		.type = RK_RGA_HW_RGA3,
		.core_mask = BIT(1),
	};
	struct rk_rga_hw rga2_0 = {
		.type = RK_RGA_HW_RGA2,
		.core_mask = BIT(2),
	};
	struct rk_rga_hw rga2_1 = {
		.type = RK_RGA_HW_RGA2,
		.core_mask = BIT(3),
	};
	LIST_HEAD(hw_list);

	list_add_tail(&rga3_0.node, &hw_list);
	list_add_tail(&rga3_1.node, &hw_list);
	list_add_tail(&rga2_0.node, &hw_list);
	list_add_tail(&rga2_1.node, &hw_list);

	KUNIT_EXPECT_EQ(test,
			rk_rga_find_free_core_mask(&hw_list,
						   RK_RGA_HW_RGA3), 0U);
	KUNIT_EXPECT_EQ(test,
			rk_rga_find_free_core_mask(&hw_list,
						   RK_RGA_HW_RGA2), 0U);

	list_del_init(&rga3_0.node);
	KUNIT_EXPECT_EQ(test,
			rk_rga_find_free_core_mask(&hw_list,
						   RK_RGA_HW_RGA3),
			BIT(0));
	list_del_init(&rga2_0.node);
	KUNIT_EXPECT_EQ(test,
			rk_rga_find_free_core_mask(&hw_list,
						   RK_RGA_HW_RGA2),
			BIT(2));
}

static void rk_rga_priority_enqueue_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = { };
	struct rk_rga_job low = { .priority = 1 };
	struct rk_rga_job default_prio = { };
	struct rk_rga_job high = { .priority = 3 };
	struct rk_rga_job equal = { .priority = 2 };
	struct rk_rga_job *pos;

	INIT_LIST_HEAD(&hw.job_queue);
	INIT_LIST_HEAD(&low.node);
	INIT_LIST_HEAD(&default_prio.node);
	INIT_LIST_HEAD(&high.node);
	INIT_LIST_HEAD(&equal.node);

	rk_rga_hw_enqueue_job_locked(&hw, &low);
	rk_rga_hw_enqueue_job_locked(&hw, &default_prio);
	rk_rga_hw_enqueue_job_locked(&hw, &high);
	rk_rga_hw_enqueue_job_locked(&hw, &equal);

	KUNIT_EXPECT_EQ(test, hw.queued_jobs, 4U);
	pos = list_first_entry(&hw.job_queue, struct rk_rga_job, node);
	KUNIT_EXPECT_PTR_EQ(test, pos, &high);
	pos = list_next_entry(pos, node);
	KUNIT_EXPECT_PTR_EQ(test, pos, &low);
	pos = list_next_entry(pos, node);
	KUNIT_EXPECT_PTR_EQ(test, pos, &equal);
	pos = list_next_entry(pos, node);
	KUNIT_EXPECT_PTR_EQ(test, pos, &default_prio);

	KUNIT_EXPECT_EQ(test, low.priority, 2);
	KUNIT_EXPECT_EQ(test, default_prio.priority, 2);
	KUNIT_EXPECT_EQ(test, equal.priority, 2);
}

static void rk_rga_iommu_fault_generation_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = {};
	struct rk_rga_job target = {};
	struct rk_rga_job replacement = {};
	unsigned long flags;
	bool matches;
	bool queued;

	spin_lock_init(&hw.job_lock);
	mutex_init(&hw.run_lock);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_rga_hw_timeout_work);
	INIT_WORK(&hw.iommu_fault_work, rk_rga_hw_iommu_fault_work);

	hw.active_job = &target;
	hw.active_generation = 1;
	hw.iommu_fault_generation = 1;
	spin_lock_irqsave(&hw.job_lock, flags);
	matches = rk_rga_hw_iommu_fault_matches_locked(&hw);
	spin_unlock_irqrestore(&hw.job_lock, flags);
	KUNIT_EXPECT_TRUE(test, matches);
	KUNIT_EXPECT_EQ(test, hw.iommu_fault_generation, 0ULL);

	hw.active_job = &replacement;
	hw.active_generation = 2;
	hw.iommu_fault_generation = 1;
	queued = schedule_delayed_work(&hw.timeout_work,
				       msecs_to_jiffies(60000));
	KUNIT_ASSERT_TRUE(test, queued);

	rk_rga_hw_iommu_fault_work(&hw.iommu_fault_work);

	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, &replacement);
	KUNIT_EXPECT_EQ(test, hw.iommu_fault_generation, 0ULL);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&hw.timeout_work));
	cancel_delayed_work_sync(&hw.timeout_work);
}

static void rk_rga_timeout_target_replacement_kunit(struct kunit *test)
{
	struct rk_rga_hw hw = {};
	struct rk_rga_job target = {};
	struct rk_rga_job replacement = {};

	spin_lock_init(&hw.job_lock);
	mutex_init(&hw.run_lock);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_rga_hw_timeout_work);
	refcount_set(&target.refs, 1);
	refcount_set(&replacement.refs, 1);
	hw.active_job = &replacement;
	hw.timeout_job = &target;
	rk_rga_job_get(&target);

	rk_rga_hw_timeout_work(&hw.timeout_work.work);

	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, &replacement);
	KUNIT_EXPECT_PTR_EQ(test, hw.timeout_job, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&target.refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&replacement.refs), 1);

	rk_rga_hw_schedule_timeout(&hw, &replacement);
	KUNIT_EXPECT_PTR_EQ(test, hw.timeout_job, &replacement);
	KUNIT_EXPECT_EQ(test, refcount_read(&replacement.refs), 2);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&hw.timeout_work));
	rk_rga_hw_cancel_timeout_sync(&hw);
	KUNIT_EXPECT_PTR_EQ(test, hw.timeout_job, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&replacement.refs), 1);
}

static void rk_rga_iommu_fault_match_kunit(struct kunit *test)
{
	struct rk_rga_iommu_fault_match_fixture {
		struct iommu_domain domain0;
		struct iommu_domain domain1;
		struct iommu_domain domain2;
		struct device_node node0;
		struct device_node node1;
		struct device_node node2;
		struct device iommu_dev;
		struct device unknown_dev;
		struct rk_rga_hw hw0;
		struct rk_rga_hw hw1;
		struct rk_rga_hw hw2;
	} *fixture;
	struct device *iommu_dev;
	struct rk_rga_hw *hw0;
	struct rk_rga_hw *hw1;
	struct rk_rga_hw *hw2;
	LIST_HEAD(fault_hws);

	fixture = kunit_kzalloc(test, sizeof(*fixture), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fixture);

	iommu_dev = &fixture->iommu_dev;
	iommu_dev->of_node = &fixture->node1;

	hw0 = &fixture->hw0;
	hw0->iommu_domain = &fixture->domain0;
	hw0->iommu_node = &fixture->node0;
	hw1 = &fixture->hw1;
	hw1->iommu_domain = &fixture->domain0;
	hw1->iommu_node = &fixture->node1;
	hw2 = &fixture->hw2;
	hw2->iommu_domain = &fixture->domain1;
	hw2->iommu_node = &fixture->node1;

	INIT_LIST_HEAD(&hw0->fault_node);
	INIT_LIST_HEAD(&hw1->fault_node);
	INIT_LIST_HEAD(&hw2->fault_node);
	list_add_tail(&hw0->fault_node, &fault_hws);
	list_add_tail(&hw1->fault_node, &fault_hws);
	list_add_tail(&hw2->fault_node, &fault_hws);

	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       iommu_dev),
			    hw1);

	iommu_dev->of_node = &fixture->node2;
	hw0->dev = iommu_dev;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       iommu_dev),
			    hw0);

	fixture->unknown_dev.of_node = &fixture->node2;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       &fixture->unknown_dev),
			    NULL);

	iommu_dev->of_node = &fixture->node1;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain1,
						       iommu_dev),
			    hw2);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       NULL),
			    hw0);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain2,
						       iommu_dev),
			    NULL);
}

static void rk_rga_iommu_refresh_kunit(struct kunit *test)
{
	static const struct iommu_domain_ops iommu_ops;
	struct iommu_domain domain = {
		.ops = &iommu_ops,
	};
	struct rk_rga_hw hw = {
		.iommu_domain = &domain,
	};

	atomic_set(&rk_rga.iommu_refresh_count, 0);
	rk_rga_hw_refresh_iommu(&hw);
	KUNIT_EXPECT_EQ(test, atomic_read(&rk_rga.iommu_refresh_count), 1);

	hw.iommu_domain = NULL;
	rk_rga_hw_refresh_iommu(&hw);
	KUNIT_EXPECT_EQ(test, atomic_read(&rk_rga.iommu_refresh_count), 1);
}

static void rk_rga_ffmpeg_rga3_profiles_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_RGB_888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YUYV_422,
					 RK_RGA_FORMAT_BGRA_8888);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP_10B,
					 RK_RGA_FORMAT_YCBCR_420_SP);
	task.src.compact_mode = RK_RGA_10BIT_INCOMPACT;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_BGRA_8888);
	task.rotate_mode = 1;
	task.sina = 65536;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
}

static void rk_rga_gstreamer_legacy_convert_profiles_kunit(struct kunit *test)
{
	u32 rga3_cmd[RK_RGA3_CMD_REG_COUNT] = { };
	u32 rga2_cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task;
	struct rk_rga_job job;
	u32 ctrl;

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_BGRX_8888,
					 RK_RGA_FORMAT_YCBCR_420_SP);
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_BGRX_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    640, 360);
	job = (struct rk_rga_job) {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = rga3_cmd,
		.cmd_size = sizeof(rga3_cmd),
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	ctrl = rga3_cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_Y2R_EN);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_ROT);

	memset(rga3_cmd, 0, sizeof(rga3_cmd));
	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					 RK_RGA_FORMAT_BGRX_8888);
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_SP, 640, 360);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRX_8888,
				    360, 640);
	task.rotate_mode = 1;
	task.sina = 65536;
	task.cosa = 0;
	type = 0;
	job = (struct rk_rga_job) {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = rga3_cmd,
		.cmd_size = sizeof(rga3_cmd),
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	ctrl = rga3_cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_Y2R_EN);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ROT);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_P,
					 RK_RGA_FORMAT_BGRA_8888);
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_P, 640, 360);
	task.src.v_addr = 0x10180000;
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRA_8888,
				    640, 360);
	type = 0;
	job = (struct rk_rga_job) {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = rga2_cmd,
		.cmd_size = sizeof(rga2_cmd),
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
}

struct rk_rga_gstreamer_format_case {
	u32 src_format;
	u32 dst_format;
	enum rk_rga_hw_type expected_type;
	bool rotate_90;
	bool expect_y2r;
	bool expect_r2y;
	bool expect_src_yuv10_compact;
};

static bool rk_rga_kunit_format_is_planar_yuv(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_422_P:
	case RK_RGA_FORMAT_YCRCB_422_P:
	case RK_RGA_FORMAT_YCBCR_420_P:
	case RK_RGA_FORMAT_YCRCB_420_P:
		return true;
	default:
		return false;
	}
}

static void
rk_rga_gstreamer_legacy_convert_matrix_case(struct kunit *test,
			const struct rk_rga_gstreamer_format_case *profile)
{
	u32 rga3_cmd[RK_RGA3_CMD_REG_COUNT] = { };
	u32 rga2_cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task;
	struct rk_rga_job job;
	u32 ctrl;
	int ret;

	task = rk_rga_ffmpeg_bitblt_task(profile->src_format,
					 profile->dst_format);
	task.src = rk_rga_kunit_img(0x10000000, profile->src_format,
				    640, 360);
	task.dst = rk_rga_kunit_img(0x20000000, profile->dst_format,
				    profile->rotate_90 ? 360 : 640,
				    profile->rotate_90 ? 640 : 360);
	if (rk_rga_kunit_format_is_planar_yuv(profile->src_format))
		task.src.v_addr = 0x10180000;
	if (rk_rga_kunit_format_is_planar_yuv(profile->dst_format))
		task.dst.v_addr = 0x20180000;
	if (profile->rotate_90) {
		task.rotate_mode = 1;
		task.sina = 65536;
		task.cosa = 0;
	}

	job = (struct rk_rga_job) {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = profile->expected_type == RK_RGA_HW_RGA3 ?
			     rga3_cmd : rga2_cmd,
		.cmd_size = profile->expected_type == RK_RGA_HW_RGA3 ?
			    sizeof(rga3_cmd) : sizeof(rga2_cmd),
	};

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, profile->expected_type);

	if (profile->expected_type == RK_RGA_HW_RGA3)
		ret = rk_rga3_emit_simple_bitblt(&job);
	else
		ret = rk_rga2_emit_simple_bitblt(&job);

	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	if (profile->expected_type != RK_RGA_HW_RGA3)
		return;

	ctrl = rga3_cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_Y2R_EN),
			!!profile->expect_y2r);
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_R2Y_EN),
			!!profile->expect_r2y);
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_ROT),
			!!profile->rotate_90);
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_YUV10_COMPACT),
			!!profile->expect_src_yuv10_compact);
}

static void rk_rga_gstreamer_legacy_format_matrix_kunit(struct kunit *test)
{
	static const struct rk_rga_gstreamer_format_case profiles[] = {
		{
			.src_format = RK_RGA_FORMAT_BGR_565,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_RGB_888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_BGR_888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_RGBA_8888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_BGRA_8888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_RGBX_8888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_BGRX_8888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_r2y = true,
		}, {
			.src_format = RK_RGA_FORMAT_BGRX_8888,
			.dst_format = RK_RGA_FORMAT_BGRX_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.rotate_90 = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_422_SP,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_422_SP,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP_10B,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_src_yuv10_compact = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_422_SP_10B,
			.dst_format = RK_RGA_FORMAT_YCBCR_422_SP,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_src_yuv10_compact = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_BGR_565,
			.expected_type = RK_RGA_HW_RGA3,
			.rotate_90 = true,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_420_SP,
			.dst_format = RK_RGA_FORMAT_RGB_888,
			.expected_type = RK_RGA_HW_RGA3,
			.rotate_90 = true,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_422_SP,
			.dst_format = RK_RGA_FORMAT_BGR_888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_422_SP,
			.dst_format = RK_RGA_FORMAT_RGBA_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_BGRA_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_RGBX_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_BGRX_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP_10B,
			.dst_format = RK_RGA_FORMAT_BGRX_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
			.expect_src_yuv10_compact = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_422_SP_10B,
			.dst_format = RK_RGA_FORMAT_BGRA_8888,
			.expected_type = RK_RGA_HW_RGA3,
			.expect_y2r = true,
			.expect_src_yuv10_compact = true,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_P,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA2,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_420_P,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expected_type = RK_RGA_HW_RGA2,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_P,
			.expected_type = RK_RGA_HW_RGA2,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_420_SP,
			.dst_format = RK_RGA_FORMAT_YCRCB_420_P,
			.expected_type = RK_RGA_HW_RGA2,
		},
	};
	size_t i;

	for (i = 0; i < ARRAY_SIZE(profiles); i++)
		rk_rga_gstreamer_legacy_convert_matrix_case(test,
							    &profiles[i]);
}

struct rk_rga_rknn_case {
	u32 src_format;
	u32 dst_format;
	u16 src_w;
	u16 src_h;
	u16 src_x;
	u16 src_y;
	u16 dst_w;
	u16 dst_h;
	u16 dst_x;
	u16 dst_y;
	u16 dst_vir_w;
	u16 dst_vir_h;
	bool expect_y2r;
	bool expect_r2y;
	u32 expected_wr_stride_bytes;
	u32 expected_wr_uv_stride_bytes;
};

static void
rk_rga_rknn_preprocess_profile_case(struct kunit *test,
				    const struct rk_rga_rknn_case *profile)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(profile->src_format,
					  profile->dst_format);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u16 src_w = profile->src_w ?: 64;
	u16 src_h = profile->src_h ?: 64;
	u16 dst_w = profile->dst_w ?: 32;
	u16 dst_h = profile->dst_h ?: 32;
	u16 dst_vir_w = profile->dst_vir_w ?: dst_w;
	u16 dst_vir_h = profile->dst_vir_h ?: dst_h;
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, profile->src_format,
				    src_w, src_h);
	task.dst = rk_rga_kunit_img(0x20000000, profile->dst_format,
				    dst_w, dst_h);
	task.src.x_offset = profile->src_x;
	task.src.y_offset = profile->src_y;
	task.dst.x_offset = profile->dst_x;
	task.dst.y_offset = profile->dst_y;
	task.dst.vir_w = dst_vir_w;
	task.dst.vir_h = dst_vir_h;
	task.yuv2rgb_mode = profile->expect_y2r || profile->expect_r2y;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_Y2R_EN),
			!!profile->expect_y2r);
	KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_R2Y_EN),
			!!profile->expect_r2y);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			(u32)profile->src_x | ((u32)profile->src_y << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			(u32)dst_w | ((u32)dst_h << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      (u64)profile->dst_y *
				      profile->expected_wr_stride_bytes +
				      profile->dst_x *
				      (profile->expected_wr_stride_bytes /
				       dst_vir_w)));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2,
			profile->expected_wr_stride_bytes);
	if (profile->expected_wr_uv_stride_bytes) {
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
				lower_32_bits(task.dst.uv_addr +
					      (u64)(profile->dst_y / 2) *
					      profile->expected_wr_uv_stride_bytes +
					      profile->dst_x));
		KUNIT_EXPECT_EQ(test,
				cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4] << 2,
				profile->expected_wr_uv_stride_bytes);
	}
}

static void rk_rga_rknn_preprocess_profiles_kunit(struct kunit *test)
{
	static const struct rk_rga_rknn_case profiles[] = {
		{
			.src_format = RK_RGA_FORMAT_RGB_888,
			.dst_format = RK_RGA_FORMAT_RGB_888,
			.expected_wr_stride_bytes = 32U * 3U,
		}, {
			.src_format = RK_RGA_FORMAT_RGB_888,
			.dst_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.expect_r2y = true,
			.expected_wr_stride_bytes = 32U,
			.expected_wr_uv_stride_bytes = 32U,
		}, {
			.src_format = RK_RGA_FORMAT_YCBCR_420_SP,
			.dst_format = RK_RGA_FORMAT_RGB_888,
			.expect_y2r = true,
			.expected_wr_stride_bytes = 32U * 3U,
		}, {
			.src_format = RK_RGA_FORMAT_YCRCB_420_SP,
			.dst_format = RK_RGA_FORMAT_RGB_888,
			.expect_y2r = true,
			.expected_wr_stride_bytes = 32U * 3U,
		}, {
			.src_format = RK_RGA_FORMAT_RGBA_8888,
			.dst_format = RK_RGA_FORMAT_RGB_888,
			.src_w = 64,
			.src_h = 64,
			.src_x = 8,
			.src_y = 6,
			.dst_w = 40,
			.dst_h = 32,
			.dst_x = 12,
			.dst_y = 8,
			.dst_vir_w = 64,
			.dst_vir_h = 48,
			.expected_wr_stride_bytes = 64U * 3U,
		},
	};
	size_t i;

	for (i = 0; i < ARRAY_SIZE(profiles); i++)
		rk_rga_rknn_preprocess_profile_case(test, &profiles[i]);
}

static void rk_rga_gstreamer_legacy_rotation_extrema_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_BGRX_8888,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_BGRX_8888,
				    640, 360);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    320, 180);
	task.rotate_mode = 1;
	task.sina = 0;
	task.cosa = -65536;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_YMIRROR);

	memset(cmd, 0, sizeof(cmd));
	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					 RK_RGA_FORMAT_BGRX_8888);
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_SP, 640, 360);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRX_8888,
				    360, 640);
	task.rotate_mode = 1;
	task.sina = -65536;
	task.cosa = 0;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_Y2R_EN);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_YMIRROR);
}

static void rk_rga3_librga_resize_interp_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.yuv2rgb_mode = 0;
	task.interp.horiz = RK_RGA2_INTERP_LINEAR;
	task.interp.verti = RK_RGA2_INTERP_LINEAR;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ENABLE);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_HOR_BY);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_HOR_UP);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_VER_BY);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_VER_UP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SCL_FAC_OFFSET / 4],
			0xaa97aaa0U);
}

static void rk_rga3_librga_drm_abgr_copy_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_PIC_FORMAT, 0x8));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_RD_FORMAT, 2));
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_PIX_SWAP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			1280U);

	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WR_PIC_FORMAT, 0x6));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_FORMAT,
			FIELD_PREP(RK_RGA3_WR_FORMAT, 2));
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_PIX_SWAP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			1280U);
}

static void rk_rga3_librga_copy_splice_task_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req tasks[] = {
		rk_rga_librga_splice_task(0x10000000, 0),
		rk_rga_librga_splice_task(0x30000000, 1280),
	};
	struct rk_rga_job job = {
		.tasks = tasks,
		.task_count = ARRAY_SIZE(tasks),
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	enum rk_rga_hw_type type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(0x10000000));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(0x20000000));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			2560U);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job, 0));
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(0x30000000));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(0x20000000 + 1280 * 4));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			2560U);
	KUNIT_EXPECT_FALSE(test, rk_rga_job_advance_task(&job, 0));
}

static void rk_rga3_multitask_emit_clears_stale_regs_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rk_rga_hw hw = {
		.type = RK_RGA_HW_RGA3,
	};
	struct rga_req tasks[] = {
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888),
		rk_rga_librga_splice_task(0x40000000, 0),
	};
	struct rk_rga_job job = {
		.tasks = tasks,
		.task_count = ARRAY_SIZE(tasks),
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	tasks[0].src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720);
	tasks[0].dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720);
	tasks[0].pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720);
	tasks[0].bsfilter_flag = 1;
	tasks[0].alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	tasks[0].PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER;
	tasks[0].feature.global_alpha_en = true;
	tasks[0].fg_global_alpha = 0xff;
	tasks[0].bg_global_alpha = 0xff;
	tasks[0].yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_emit_cmd(&hw, &job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_NE(test, cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4], 0U);
	KUNIT_EXPECT_NE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4], 0U);

	KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job, 0));
	job.cmd_ready = true;
	KUNIT_EXPECT_EQ(test, rk_rga_job_emit_cmd(&hw, &job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(0x40000000));
}

static void rk_rga3_librga_translate_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;

	task.src.act_w = 1620;
	task.src.act_h = 780;
	task.src.vir_w = 1920;
	task.src.vir_h = 1080;
	task.dst.act_w = 1620;
	task.dst.act_h = 780;
	task.dst.vir_w = 1920;
	task.dst.vir_h = 1080;
	task.dst.x_offset = 300;
	task.dst.y_offset = 300;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
	KUNIT_EXPECT_EQ(test, stride_bytes, 1920U * 4U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			1920U | (1080U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			1620U | (780U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1620U | (780U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      300ULL * stride_bytes + 300ULL * 4ULL));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    640, 360);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1920, 1080);
	task.dst.act_w = 640;
	task.dst.act_h = 360;
	task.dst.x_offset = 320;
	task.dst.y_offset = 180;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4] << 2,
			640U * 4U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			640U | (360U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			640U | (360U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			640U | (360U << 16));
	stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
	KUNIT_EXPECT_EQ(test, stride_bytes, 1920U * 4U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      180ULL * stride_bytes + 320ULL * 4ULL));
}

static void rk_rga3_librga_rotate_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    720, 1280);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.rotate_mode = 1;
	task.sina = 65536;
	task.cosa = 0;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ENABLE);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_YMIRROR);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			720U | (1280U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			720U | (1280U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
}

static void rk_rga3_librga_rotate_flip_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    720, 1280);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.rotate_mode = 1 | (4 << 4);
	task.sina = 65536;
	task.cosa = 0;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ENABLE);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_YMIRROR);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			720U | (1280U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			720U | (1280U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
}

static void rk_rga3_librga_center_rotate_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst.x_offset = 437;
	task.dst.y_offset = 0;
	task.dst.act_w = 720;
	task.dst.act_h = 406;
	task.rotate_mode = 1;
	task.sina = 65536;
	task.cosa = 0;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4], 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			406U | (720U << 16));
	KUNIT_EXPECT_EQ(test, stride_bytes, 1280U * 4U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr + 437ULL * 4ULL));
}

static void rk_rga3_librga_flip_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.rotate_mode = 2;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ENABLE);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_YMIRROR);
	KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_HOR_BY);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_VER_BY);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
}

static void rk_rga3_yuv422_rotate_policy_kunit(struct kunit *test)
{
	struct rk_rga3_bitblt_profile profile;
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YUYV_422,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
	};

	task.rotate_mode = 1;
	task.sina = 65536;
	task.cosa = 0;
	task.dst.act_w = 1080;
	task.dst.act_h = 1920;
	task.dst.vir_w = 1920;
	task.dst.vir_h = 1920;

	KUNIT_EXPECT_EQ(test, rk_rga3_validate_bitblt(&task, &profile),
			-EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);

	task.core = RK_RGA_CORE_RGA3_MASK;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga3_librga_side_border_kunit(struct kunit *test)
{
	u32 rga3_cmd[RK_RGA3_CMD_REG_COUNT] = { };
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req *tasks;
	struct rk_rga_job job = {
		.task_count = 4,
		.import_count = 2,
		.cmd_vaddr = rga3_cmd,
		.cmd_size = sizeof(rga3_cmd),
	};
	struct rga_req task;
	struct rk_rga_job single_job;
	enum rk_rga_hw_type type;

	tasks = kunit_kcalloc(test, job.task_count, sizeof(*tasks),
			      GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	tasks[0] = rk_rga_librga_side_border_task(16, 0, 16, true, 0);
	tasks[1] = rk_rga_librga_side_border_task(32, 48, 16, true, 0);
	tasks[2] = rk_rga_librga_side_border_task(32, 0, 16, false, 0);
	tasks[3] = rk_rga_librga_side_border_task(16, 48, 16, false, 0);
	job.tasks = tasks;

	for (u32 i = 0; i < job.task_count; i++) {
		u32 win1_ctrl;
		bool reflect = tasks[i].rotate_mode == 2;

		memset(rga3_cmd, 0, sizeof(rga3_cmd));
		job.cmd_ready = false;
		type = 0;

		KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
		KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
		KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
		KUNIT_EXPECT_TRUE(test, job.cmd_ready);
		KUNIT_EXPECT_TRUE(test,
				  rga3_cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
				  RK_RGA3_WIN0_ENABLE);

		win1_ctrl = rga3_cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4];
		KUNIT_EXPECT_TRUE(test, win1_ctrl & RK_RGA3_WIN0_ENABLE);
		KUNIT_EXPECT_EQ(test, !!(win1_ctrl & RK_RGA3_WIN0_XMIRROR),
				reflect);
		KUNIT_EXPECT_FALSE(test, win1_ctrl & RK_RGA3_WIN0_YMIRROR);
		KUNIT_EXPECT_FALSE(test, win1_ctrl & RK_RGA3_WIN0_ROT);
		KUNIT_EXPECT_EQ(test,
				rga3_cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4] &
				RK_RGA3_OVLP_MODE,
				FIELD_PREP(RK_RGA3_OVLP_MODE, 1));
		KUNIT_EXPECT_EQ(test, rga3_cmd[RK_RGA3_OVLP_OFF_OFFSET / 4],
				(u32)tasks[i].dst.x_offset |
				((u32)tasks[i].dst.y_offset << 16));
		KUNIT_EXPECT_EQ(test,
				rga3_cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
				(u32)tasks[i].dst.x_offset |
				((u32)tasks[i].dst.y_offset << 16));
		KUNIT_EXPECT_EQ(test,
				rga3_cmd[RK_RGA3_WIN1_ACT_OFF_OFFSET / 4],
				(u32)tasks[i].src.x_offset |
				((u32)tasks[i].src.y_offset << 16));
		KUNIT_EXPECT_EQ(test, rga3_cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
				lower_32_bits(tasks[i].dst.yrgb_addr));

		if (i + 1 < job.task_count)
			KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job,
									0));
		else
			KUNIT_EXPECT_FALSE(test, rk_rga_job_advance_task(&job,
									 0));
	}

	task = rk_rga_librga_side_border_task(16, 48, 16, true, BIT(2));
	single_job = (struct rk_rga_job) {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&single_job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&single_job), 0);
	KUNIT_EXPECT_TRUE(test, single_job.cmd_ready);

	task.core = BIT(0);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&single_job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task = rk_rga_librga_side_border_task(16, 20, 16, true, 0);
	single_job.cmd_ready = false;
	task.dst.x_offset = 20;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&single_job, &type),
			-EOPNOTSUPP);

	task = rk_rga_librga_side_border_task(16, 48, 16, true, 0);
	task.dst.act_w = 8;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&single_job, &type),
			-EOPNOTSUPP);
}

static void rk_rga3_librga_padding_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req *tasks;
	struct rk_rga_job job = {
		.task_count = 4,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 stride_bytes;

	tasks = kunit_kcalloc(test, job.task_count, sizeof(*tasks),
			      GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tasks);
	tasks[0] = rk_rga_librga_padding_task(0, 0, 64, true);
	tasks[1] = rk_rga_librga_padding_task(592, 784, 128, true);
	tasks[2] = rk_rga_librga_padding_task(656, 0, 64, false);
	tasks[3] = rk_rga_librga_padding_task(0, 784, 128, false);
	job.tasks = tasks;

	for (u32 i = 0; i < job.task_count; i++) {
		enum rk_rga_hw_type type = 0;
		u32 expected_wr_base;
		u32 ctrl;
		bool reflect = tasks[i].rotate_mode == 3;

		memset(cmd, 0, sizeof(cmd));
		job.cmd_ready = false;

		KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
		KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
		KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
		KUNIT_EXPECT_TRUE(test, job.cmd_ready);

		ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
		KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_ENABLE);
		KUNIT_EXPECT_EQ(test, !!(ctrl & RK_RGA3_WIN0_YMIRROR),
				reflect);
		KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_XMIRROR);
		KUNIT_EXPECT_FALSE(test, ctrl & RK_RGA3_WIN0_ROT);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
				(u32)tasks[i].src.x_offset |
				((u32)tasks[i].src.y_offset << 16));
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
				1280U | ((u32)tasks[i].src.act_h << 16));
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
				1280U | ((u32)tasks[i].dst.act_h << 16));

		stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
		KUNIT_EXPECT_EQ(test, stride_bytes, 2048U * 4U);
		expected_wr_base = lower_32_bits(tasks[i].dst.yrgb_addr +
					(u64)tasks[i].dst.y_offset *
					stride_bytes +
					(u64)tasks[i].dst.x_offset * 4);
		KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
				expected_wr_base);

		if (i + 1 < job.task_count)
			KUNIT_EXPECT_TRUE(test, rk_rga_job_advance_task(&job,
									0));
		else
			KUNIT_EXPECT_FALSE(test, rk_rga_job_advance_task(&job,
									 0));
	}
}

static void rk_rga2_compact_10bit_profile_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP_10B,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 src_info;

	task.core = BIT(2);
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_FORMAT, 0xa);
	KUNIT_EXPECT_TRUE(test, src_info & RK_RGA2_SRC_YUV10_EN);
	KUNIT_EXPECT_TRUE(test, src_info & RK_RGA2_SRC_YUV10_ROUND_EN);

	memset(cmd, 0, sizeof(cmd));
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1920, 1080);
	job.cmd_ready = false;
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_HSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_HSCL_MODE,
				   RK_RGA2_SCALE_FORCE_TILE));
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_VSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_VSCL_MODE,
				   RK_RGA2_SCALE_FORCE_TILE));

	task.src.compact_mode = RK_RGA_10BIT_INCOMPACT;
	job.cmd_ready = false;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					 RK_RGA_FORMAT_YCBCR_420_SP_10B);
	task.core = BIT(2);
	job.tasks = &task;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga2_librga_interp_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 src_info;

	task.core = BIT(2);
	task.yuv2rgb_mode = 0;
	task.interp.horiz = RK_RGA2_INTERP_LINEAR;
	task.interp.verti = RK_RGA2_INTERP_LINEAR;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_HSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_HSCL_MODE,
				   RK_RGA2_SCALE_DOWN));
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_VSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_VSCL_MODE,
				   RK_RGA2_SCALE_DOWN));
	KUNIT_EXPECT_TRUE(test, src_info & RK_RGA2_SRC_HSD_MODE_SEL);
	KUNIT_EXPECT_TRUE(test, src_info & RK_RGA2_SRC_VSD_MODE_SEL);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_X_FACTOR_OFFSET / 4],
			0x080017ffU);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_Y_FACTOR_OFFSET / 4],
			0x080017ffU);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			1919U | (1079U << 16));

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.interp.horiz = RK_RGA2_INTERP_BICUBIC;
	task.interp.verti = RK_RGA2_INTERP_BICUBIC;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	KUNIT_EXPECT_FALSE(test, src_info & RK_RGA2_SRC_HSD_MODE_SEL);
	KUNIT_EXPECT_FALSE(test, src_info & RK_RGA2_SRC_VSD_MODE_SEL);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_X_FACTOR_OFFSET / 4],
			0xaaabU);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_Y_FACTOR_OFFSET / 4],
			0xaaabU);
}

static void rk_rga2_librga_y400_uv_downsample_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.core = BIT(2),
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_400,
					1920, 2160),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_400,
					1280, 1080),
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_src_base;
	u32 expected_dst_base;
	u32 src_info;
	u32 dst_info;

	task.src.act_h = 1080;
	task.src.y_offset = 1080;
	task.dst.act_h = 360;
	task.dst.y_offset = 720;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	expected_src_base = lower_32_bits(task.src.yrgb_addr +
					  (u64)task.src.y_offset *
					  task.src.vir_w);
	expected_dst_base = lower_32_bits(task.dst.yrgb_addr +
					  (u64)task.dst.y_offset *
					  task.dst.vir_w);
	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];

	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_FORMAT,
			FIELD_PREP(RK_RGA2_SRC_FORMAT, 0x8));
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_FORMAT,
			FIELD_PREP(RK_RGA2_DST_FORMAT, 0x8));
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_YUV400_EN);
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_HSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_HSCL_MODE,
				   RK_RGA2_SCALE_DOWN));
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_VSCL_MODE,
			FIELD_PREP(RK_RGA2_SRC_VSCL_MODE,
				   RK_RGA2_SCALE_DOWN));
	KUNIT_EXPECT_FALSE(test, src_info & RK_RGA2_SRC_HSD_MODE_SEL);
	KUNIT_EXPECT_FALSE(test, src_info & RK_RGA2_SRC_VSD_MODE_SEL);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			expected_src_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE1_OFFSET / 4],
			expected_src_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE2_OFFSET / 4],
			expected_src_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			expected_dst_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_VIR_INFO_OFFSET / 4],
			task.src.vir_w >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			task.dst.vir_w >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_X_FACTOR_OFFSET / 4],
			0xaaabU);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_Y_FACTOR_OFFSET / 4],
			0x5556U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			1919U | (1079U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			1279U | (359U << 16));
}

static void rk_rga2_librga_gray256_cvtcolor_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.core = BIT(2),
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_400,
					1280, 720),
		.yuv2rgb_mode = 1 << 2,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 src_info;
	u32 dst_info;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];

	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_FORMAT,
			FIELD_PREP(RK_RGA2_SRC_FORMAT, 0x0));
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_CSC_MODE, 0U);
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_FORMAT,
			FIELD_PREP(RK_RGA2_DST_FORMAT, 0x8));
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_CSC_MODE,
			FIELD_PREP(RK_RGA2_DST_CSC_MODE, 1));
	KUNIT_EXPECT_FALSE(test, dst_info & RK_RGA2_DST_CSC_CLIP);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_YUV400_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_VIR_INFO_OFFSET / 4],
			(u32)task.src.vir_w);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			(u32)task.dst.vir_w >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			1279U | (719U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			1279U | (719U << 16));
}

static void rk_rga2_librga_y4_dither_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.core = BIT(2),
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_Y4,
					1280, 720),
		.alpha_rop_flag = RK_RGA2_ALPHA_FLAG_ENABLE |
				  RK_RGA2_ALPHA_FLAG_DST_DITHER_DOWN,
		.dither_mode = 1,
		.yuv2rgb_mode = 1 << 2,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 dst_info;

	task.gr_color.gr_x_r = 0x3210;
	task.gr_color.gr_x_g = 0x7654;
	task.gr_color.gr_y_r = (__s16)0xba98;
	task.gr_color.gr_y_g = (__s16)0xfedc;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_FORMAT,
			FIELD_PREP(RK_RGA2_DST_FORMAT, 0x8));
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_YUV400_EN);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_Y4_EN);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_DITHER_DOWN_EN);
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_DITHER_MODE,
			FIELD_PREP(RK_RGA2_DST_DITHER_MODE, 1));
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_CSC_MODE,
			FIELD_PREP(RK_RGA2_DST_CSC_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			((u32)task.dst.vir_w / 2) >> 2);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_Y4MAP_LUT0_OFFSET / 4],
			0x76543210U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_Y4MAP_LUT1_OFFSET / 4],
			0xfedcba98U);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.dst.format = RK_RGA_FORMAT_Y8;
	task.alpha_rop_flag = 0;
	task.dither_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_YUV400_EN);
	KUNIT_EXPECT_FALSE(test, dst_info & RK_RGA2_DST_Y4_EN);
	KUNIT_EXPECT_FALSE(test, dst_info & RK_RGA2_DST_DITHER_DOWN_EN);

	job.cmd_ready = false;
	memset(cmd, 0, sizeof(cmd));
	task.full_csc.flag = RK_RGA_FULL_CSC_ENABLE;
	task.yuv2rgb_mode = 3 << 2;
	task.alpha_rop_flag = RK_RGA2_ALPHA_FLAG_ENABLE |
			      RK_RGA2_ALPHA_FLAG_DST_DITHER_DOWN;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_FULL_CSC_EN);
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_CSC_MODE,
			FIELD_PREP(RK_RGA2_DST_CSC_MODE, 3));
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_DITHER_DOWN_EN);
	KUNIT_EXPECT_FALSE(test, dst_info & RK_RGA2_DST_Y4_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_Y4MAP_LUT0_OFFSET / 4],
			0x76543210U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_Y4MAP_LUT1_OFFSET / 4],
			0xfedcba98U);

	job.cmd_ready = false;
	memset(cmd, 0, sizeof(cmd));
	task.dst.format = RK_RGA_FORMAT_Y4;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_FULL_CSC_EN);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_Y4_EN);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_DITHER_DOWN_EN);
}

static void rk_rga2_librga_full_csc_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 dst_info;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.yuv2rgb_mode = 3 << 2;
	task.full_csc.flag = RK_RGA_FULL_CSC_ENABLE;
	task.full_csc.coe_y.r_v = 187;
	task.full_csc.coe_y.g_y = 628;
	task.full_csc.coe_y.b_u = 63;
	task.full_csc.coe_y.off = 16368;
	task.full_csc.coe_u.r_v = -102;
	task.full_csc.coe_u.g_y = -346;
	task.full_csc.coe_u.b_u = 449;
	task.full_csc.coe_u.off = 130944;
	task.full_csc.coe_v.r_v = 449;
	task.full_csc.coe_v.g_y = -407;
	task.full_csc.coe_v.b_u = -40;
	task.full_csc.coe_v.off = 130944;
	task.feature.full_csc_clip_en = true;
	task.full_csc_clip.y.max = 0xeb;
	task.full_csc_clip.y.min = 0x10;
	task.full_csc_clip.uv.max = 0xf0;
	task.full_csc_clip.uv.min = 0x10;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_FULL_CSC_EN);
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_CSC_MODE,
			FIELD_PREP(RK_RGA2_DST_CSC_MODE, 3));
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_FORMAT,
			FIELD_PREP(RK_RGA2_DST_FORMAT, 0xa));

	task.core = RK_RGA_CORE_RGA3_MASK;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), -EOPNOTSUPP);
}

static void rk_rga2_src_crop_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_RGB_888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_uv_base;
	u32 expected_y_base;
	u32 stride_bytes;
	u32 uv_stride_bytes;

	task.core = BIT(2);
	task.src.act_w = 640;
	task.src.act_h = 360;
	task.src.x_offset = 16;
	task.src.y_offset = 8;
	task.dst.act_w = 320;
	task.dst.act_h = 180;

	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	stride_bytes = ALIGN((u32)task.src.vir_w, 4);
	uv_stride_bytes = ALIGN((u32)task.src.vir_w, 4);
	expected_y_base = lower_32_bits(task.src.yrgb_addr +
				       (u64)task.src.y_offset * stride_bytes +
				       task.src.x_offset);
	expected_uv_base = lower_32_bits(task.src.uv_addr +
					(u64)(task.src.y_offset / 2) *
					uv_stride_bytes + task.src.x_offset);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			expected_y_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE1_OFFSET / 4],
			expected_uv_base);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			((u32)task.src.act_w - 1) |
			(((u32)task.src.act_h - 1) << 16));
}

static void rk_rga_ffmpeg_fbc_profiles_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
	};

	task.src.rd_mode = RK_RGA_FBC_MODE;
	task.src.vir_h = 1088;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.src.rd_mode = RK_RGA_RASTER_MODE;
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	task.dst.vir_h = 720;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	/* RGA2-Pro compressed source modes are recognized but unsupported. */
	task.src.rd_mode = RK_RGA_RKFBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	task.src.x_offset = 64;
	task.src.y_offset = 4;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP_10B,
					 RK_RGA_FORMAT_RGBA_8888);
	task.src.rd_mode = RK_RGA_RKFBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	task.src.x_offset = 96;
	task.src.y_offset = 12;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_422_SP_10B,
					 RK_RGA_FORMAT_RGBA_8888);
	task.src.rd_mode = RK_RGA_RKFBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	task.src.x_offset = 128;
	task.src.y_offset = 8;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_BGRA_8888);
	task.src.rd_mode = RK_RGA_AFBC32X8_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	task.src.x_offset = 32;
	task.src.y_offset = 8;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGB_888,
					 RK_RGA_FORMAT_BGRA_8888);
	task.src.rd_mode = RK_RGA_AFBC32X8_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					 RK_RGA_FORMAT_RGBA_8888);
	task.src.rd_mode = RK_RGA_AFBC32X8_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);

	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_422_SP_10B,
					 RK_RGA_FORMAT_RGBA_8888);
	task.src.rd_mode = RK_RGA_RKFBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	task.core = BIT(0);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga3_librga_afbc_copy_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 aligned_w;
	u32 aligned_h;
	u32 fbc_header_stride;
	u32 fbc_payload_stride;
	u32 fbc_header_size;
	u32 raster_stride;
	u32 ctrl;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.yuv2rgb_mode = 0;
	task.dst.rd_mode = RK_RGA_FBC_MODE;

	aligned_w = ALIGN((u32)task.dst.vir_w, 16);
	aligned_h = ALIGN((u32)task.dst.vir_h, 16);
	fbc_header_stride = aligned_w >> 2;
	fbc_payload_stride = (aligned_w >> 3) * 3;
	fbc_header_size = (fbc_header_stride * aligned_h) >> 2;
	raster_stride = aligned_w >> 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE, 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_FBCE_SPARSE_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			fbc_payload_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr + fbc_header_size));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000,
				    RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	job.cmd_ready = false;
	type = 0;

	fbc_payload_stride = aligned_w >> 1;
	fbc_header_size = (fbc_header_stride * aligned_h) >> 2;
	raster_stride = aligned_w >> 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WR_PIC_FORMAT, 0x2));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_FORMAT,
			FIELD_PREP(RK_RGA3_WR_FORMAT, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			fbc_payload_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr + fbc_header_size));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	job.cmd_ready = false;
	type = 0;

	fbc_payload_stride = aligned_w;
	fbc_header_size = (fbc_header_stride * aligned_h) >> 2;
	raster_stride = aligned_w;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WR_PIC_FORMAT, 0x6));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_FORMAT,
			FIELD_PREP(RK_RGA3_WR_FORMAT, 2));
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_PIX_SWAP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			fbc_payload_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr + fbc_header_size));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.src.rd_mode = RK_RGA_FBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	job.cmd_ready = false;
	type = 0;

	fbc_payload_stride = (aligned_w >> 3) * 3;
	raster_stride = aligned_w >> 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_U_BASE_OFFSET / 4],
			lower_32_bits(task.src.uv_addr));
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE, 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr));

	memset(cmd, 0, sizeof(cmd));
	task.src.x_offset = 0;
	task.src.y_offset = 32;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			1280U | (752U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			32U << 16);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));

	memset(cmd, 0, sizeof(cmd));
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_U_BASE_OFFSET / 4],
			lower_32_bits(task.src.uv_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			1280U | (752U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			32U << 16);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280U | (720U << 16));
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_FBCE_SPARSE_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			fbc_payload_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr + fbc_header_size));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000,
				    RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.src.rd_mode = RK_RGA_FBC_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	job.cmd_ready = false;
	type = 0;

	raster_stride = aligned_w >> 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 1));
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_PIC_FORMAT, 0x2));
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_RD_FORMAT, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			fbc_header_stride);
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE, 0U);
	KUNIT_EXPECT_TRUE(test, ctrl & RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			raster_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			raster_stride);
}

static void rk_rga3_tile8x8_profile_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 dst_tile_stride = ALIGN((u32)task.dst.vir_w * 8, 16) >> 2;
	u32 dst_tile_uv_stride = ALIGN((u32)task.dst.vir_w * 8, 16) >> 3;
	u32 src_tile_stride = ALIGN((u32)task.src.vir_w * 8, 16) >> 2;
	u32 src_tile_uv_stride = ALIGN((u32)task.src.vir_w * 8, 16) >> 3;
	u32 ctrl;

	task.dst.rd_mode = RK_RGA_TILE_MODE;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			dst_tile_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			dst_tile_uv_stride);

	memset(cmd, 0, sizeof(cmd));
	task.src.rd_mode = RK_RGA_TILE_MODE;
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	job.cmd_ready = false;
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			src_tile_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			src_tile_uv_stride);

	memset(cmd, 0, sizeof(cmd));
	task.dst.rd_mode = RK_RGA_TILE_MODE;
	job.cmd_ready = false;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			src_tile_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET / 4],
			src_tile_uv_stride);
	ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, ctrl & RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			dst_tile_stride);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			dst_tile_uv_stride);

	task.dst.format = RK_RGA_FORMAT_RGBA_8888;
	task.dst.rd_mode = RK_RGA_TILE_MODE;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga_ffmpeg_alpha_overlay_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
	};

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_BGRA_8888,
				    1280, 720);
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_DST_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRA_8888,
				    1280, 720);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.yuv2rgb_mode = 1;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.yuv2rgb_mode = 1 | (2 << 2);
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

	task.rotate_mode = 1;
	task.sina = 65536;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
}

static void rk_rga3_librga_alpha_yuv_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000,
					RK_RGA_FORMAT_YCBCR_420_SP,
					1920, 1080),
		.dst = rk_rga_kunit_img(0x20000000,
					RK_RGA_FORMAT_YCBCR_420_SP,
					1920, 1080),
		.pat = rk_rga_kunit_img(0x30000000,
					RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.bsfilter_flag = 1,
		.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9),
		.PD_mode = RK_RGA_ALPHA_BLEND_DST_OVER,
		.feature.global_alpha_en = true,
		.fg_global_alpha = 0xff,
		.bg_global_alpha = 0xff,
		.yuv2rgb_mode = 1 | (2 << 2),
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.src.x_offset = 100;
	task.src.y_offset = 200;
	task.src.act_w = 1280;
	task.src.act_h = 720;
	task.dst.x_offset = 100;
	task.dst.y_offset = 200;
	task.dst.act_w = 1280;
	task.dst.act_h = 720;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			   (RK_RGA3_WIN0_R2Y_EN | RK_RGA3_WIN0_Y2R_EN));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_ACT_OFF_OFFSET / 4],
			100U | (200U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_OFF_OFFSET / 4],
			100U | (200U << 16));

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000,
				    RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			   (RK_RGA3_WIN0_R2Y_EN | RK_RGA3_WIN0_Y2R_EN));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			RK_RGA3_WR_MODE, FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_FBCE_SPARSE_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      ((1280U >> 2) * 720U) / 4));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000,
				    RK_RGA_FORMAT_YCRCB_420_SP,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000,
				    RK_RGA_FORMAT_YCBCR_422_SP,
				    1280, 720);
	memset(&task.pat, 0, sizeof(task.pat));
	task.bsfilter_flag = 0;
	task.core = BIT(0);
	job.import_count = 2;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			   (RK_RGA3_WIN0_R2Y_EN | RK_RGA3_WIN0_Y2R_EN));
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_PIX_SWAP);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_PIX_SWAP);

	task.src.format = RK_RGA_FORMAT_YCRCB_420_SP_10B;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga3_librga_slt_alpha_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000,
					RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.dst = rk_rga_kunit_img(0x20000000,
					RK_RGA_FORMAT_YCBCR_420_SP,
					1280, 720),
		.pat = rk_rga_kunit_img(0x30000000,
					RK_RGA_FORMAT_RGBA_8888,
					1280, 720),
		.bsfilter_flag = 1,
		.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9),
		.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER,
		.feature.global_alpha_en = true,
		.fg_global_alpha = 0xff,
		.bg_global_alpha = 0xff,
		.rotate_mode = 1 | (4 << 4),
		.cosa = -65536,
		.yuv2rgb_mode = 2 << 2,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 win0_ctrl;
	u32 win1_ctrl;

	task.src.x_offset = 100;
	task.src.y_offset = 100;
	task.src.act_w = 480;
	task.src.act_h = 320;
	task.dst.x_offset = 100;
	task.dst.y_offset = 100;
	task.dst.act_w = 720;
	task.dst.act_h = 540;
	task.pat.x_offset = 100;
	task.pat.y_offset = 100;
	task.pat.act_w = 720;
	task.pat.act_h = 540;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	win0_ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	win1_ctrl = cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, win0_ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_TRUE(test, win1_ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, win1_ctrl & (RK_RGA3_WIN0_ROT |
					     RK_RGA3_WIN0_XMIRROR |
					     RK_RGA3_WIN0_YMIRROR));
	KUNIT_EXPECT_TRUE(test, win1_ctrl & RK_RGA3_WIN0_HOR_UP);
	KUNIT_EXPECT_TRUE(test, win1_ctrl & RK_RGA3_WIN0_VER_UP);
	KUNIT_EXPECT_FALSE(test, win1_ctrl & RK_RGA3_WIN0_HOR_BY);
	KUNIT_EXPECT_FALSE(test, win1_ctrl & RK_RGA3_WIN0_VER_BY);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.pat.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			100U | (100U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			720U | (540U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			720U | (540U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_ACT_OFF_OFFSET / 4],
			100U | (100U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_ACT_SIZE_OFFSET / 4],
			480U | (320U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_DST_SIZE_OFFSET / 4],
			720U | (540U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_OFF_OFFSET / 4],
			100U | (100U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr));
}

static void rk_rga3_librga_global_alpha_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_top_ctrl;
	u32 expected_bottom_ctrl;
	u32 expected_top_alpha;
	u32 expected_bottom_alpha;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0x80;
	task.bg_global_alpha = 0xe0;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
			RK_RGA3_OVLP_TOP_ALPHA_EN);

	expected_top_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL_GLOBAL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0x80);
	expected_bottom_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL_GLOBAL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xe0);
	expected_top_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL_GLOBAL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE);
	expected_bottom_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL_GLOBAL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4],
			expected_top_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_CTRL_OFFSET / 4],
			expected_bottom_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_ALPHA_OFFSET / 4],
			expected_top_alpha);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_ALPHA_OFFSET / 4],
			expected_bottom_alpha);
}

static void rk_rga3_librga_rgb_composite_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_top_ctrl;
	u32 expected_bottom_ctrl;
	u32 expected_top_alpha;
	u32 expected_bottom_alpha;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.pat.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			RK_RGA3_OVLP_TOP_ALPHA_EN);

	expected_top_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xff);
	expected_bottom_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xff);
	expected_top_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE);
	expected_bottom_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4],
			expected_top_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_CTRL_OFFSET / 4],
			expected_bottom_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_ALPHA_OFFSET / 4],
			expected_top_alpha);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_ALPHA_OFFSET / 4],
			expected_bottom_alpha);
}

static void rk_rga3_display_partial_alpha_blend_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000,
					RK_RGA_FORMAT_BGRA_8888,
					64, 48),
		.dst = rk_rga_kunit_img(0x20000000,
					RK_RGA_FORMAT_BGRA_8888,
					80, 64),
		.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9),
		.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER,
		.feature.global_alpha_en = true,
		.fg_global_alpha = 0xff,
		.bg_global_alpha = 0xff,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_top_ctrl;
	u32 expected_bottom_ctrl;
	u32 expected_top_alpha;
	u32 expected_bottom_alpha;

	task.src.x_offset = 8;
	task.src.y_offset = 6;
	task.src.act_w = 28;
	task.src.act_h = 20;
	task.dst.x_offset = 24;
	task.dst.y_offset = 18;
	task.dst.act_w = 28;
	task.dst.act_h = 20;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			64U | (48U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			24U | (18U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			28U | (20U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			28U | (20U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_SRC_SIZE_OFFSET / 4],
			36U | (26U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_ACT_OFF_OFFSET / 4],
			8U | (6U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_ACT_SIZE_OFFSET / 4],
			28U | (20U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_DST_SIZE_OFFSET / 4],
			28U | (20U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
			RK_RGA3_OVLP_TOP_ALPHA_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_OFF_OFFSET / 4],
			24U | (18U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			80U);

	expected_top_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xff);
	expected_bottom_ctrl =
		FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE) |
		FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xff);
	expected_top_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR, RK_RGA3_ALPHA_ONE);
	expected_bottom_alpha =
		FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			   RK_RGA3_ALPHA_PER_PIXEL) |
		FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
			   RK_RGA3_ALPHA_OPPOSITE_INVERSE);

	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4],
			expected_top_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_CTRL_OFFSET / 4],
			expected_bottom_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_ALPHA_OFFSET / 4],
			expected_top_alpha);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_BOT_ALPHA_OFFSET / 4],
			expected_bottom_alpha);
}

static void rk_rga3_display_rgb565_rotate_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGB_565,
					64, 32),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGB_565,
					64, 32),
		.rotate_mode = 1,
		.cosa = -65536,
		.core = BIT(0),
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 rd_ctrl;
	u32 wr_ctrl;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	rd_ctrl = cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4];
	KUNIT_EXPECT_TRUE(test, rd_ctrl & RK_RGA3_WIN0_ENABLE);
	KUNIT_EXPECT_EQ(test, rd_ctrl & RK_RGA3_WIN0_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_PIC_FORMAT, 0x4));
	KUNIT_EXPECT_EQ(test, rd_ctrl & RK_RGA3_WIN0_RD_FORMAT,
			FIELD_PREP(RK_RGA3_WIN0_RD_FORMAT, 2));
	KUNIT_EXPECT_TRUE(test, rd_ctrl & RK_RGA3_WIN0_PIX_SWAP);
	KUNIT_EXPECT_FALSE(test, rd_ctrl & RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, rd_ctrl & RK_RGA3_WIN0_XMIRROR);
	KUNIT_EXPECT_TRUE(test, rd_ctrl & RK_RGA3_WIN0_YMIRROR);
	KUNIT_EXPECT_FALSE(test, rd_ctrl & RK_RGA3_WIN0_Y2R_EN);
	KUNIT_EXPECT_FALSE(test, rd_ctrl & RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_VIR_STRIDE_OFFSET / 4],
			32U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			64U | (32U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			64U | (32U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			64U | (32U << 16));

	wr_ctrl = cmd[RK_RGA3_WR_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, wr_ctrl & RK_RGA3_WR_PIC_FORMAT,
			FIELD_PREP(RK_RGA3_WR_PIC_FORMAT, 0x4));
	KUNIT_EXPECT_EQ(test, wr_ctrl & RK_RGA3_WR_FORMAT,
			FIELD_PREP(RK_RGA3_WR_FORMAT, 2));
	KUNIT_EXPECT_TRUE(test, wr_ctrl & RK_RGA3_WR_PIX_SWAP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4],
			32U);
}

static void rk_rga2_display_xrgb_rotate_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA2_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task = {
		.render_mode = RK_RGA_RENDER_BITBLT,
		.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_XRGB_8888,
					64, 32),
		.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_XRGB_8888,
					32, 64),
		.rotate_mode = 1,
		.sina = -65536,
	};
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 mode_ctrl;
	u32 src_info;
	u32 dst_info;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA2);
	KUNIT_EXPECT_EQ(test, rk_rga2_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	mode_ctrl = cmd[RK_RGA2_MODE_CTRL_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, mode_ctrl & RK_RGA2_MODE_RENDER_MODE,
			FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				   RK_RGA_RENDER_BITBLT));
	KUNIT_EXPECT_TRUE(test, mode_ctrl & RK_RGA2_MODE_INTR_CF_E);

	src_info = cmd[RK_RGA2_SRC_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_FORMAT,
			FIELD_PREP(RK_RGA2_SRC_FORMAT, 0x1));
	KUNIT_EXPECT_FALSE(test, src_info & RK_RGA2_SRC_RB_SWAP);
	KUNIT_EXPECT_TRUE(test, src_info & RK_RGA2_SRC_ALPHA_SWAP);
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_ROT_MODE,
			FIELD_PREP(RK_RGA2_SRC_ROT_MODE, 3));
	KUNIT_EXPECT_EQ(test, src_info & RK_RGA2_SRC_MIR_MODE, 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_BASE0_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_VIR_INFO_OFFSET / 4],
			64U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_SRC_ACT_INFO_OFFSET / 4],
			63U | (31U << 16));

	dst_info = cmd[RK_RGA2_DST_INFO_OFFSET / 4];
	KUNIT_EXPECT_EQ(test, dst_info & RK_RGA2_DST_FORMAT,
			FIELD_PREP(RK_RGA2_DST_FORMAT, 0x1));
	KUNIT_EXPECT_FALSE(test, dst_info & RK_RGA2_DST_RB_SWAP);
	KUNIT_EXPECT_TRUE(test, dst_info & RK_RGA2_DST_ALPHA_SWAP);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_BASE0_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr + 31U * 128U));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_VIR_INFO_OFFSET / 4],
			32U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA2_DST_ACT_INFO_OFFSET / 4],
			63U | (31U << 16));
}

static void rk_rga3_pattern_rotate_reject_kunit(struct kunit *test)
{
	struct rk_rga3_bitblt_profile profile;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.yuv2rgb_mode = 0;

	KUNIT_EXPECT_EQ(test, rk_rga3_validate_bitblt(&task, &profile), 0);

	task.pat.rotate_mode = 1;
	KUNIT_EXPECT_EQ(test, rk_rga3_validate_bitblt(&task, &profile),
			-EOPNOTSUPP);
}

static void rk_rga3_librga_blend_modes_kunit(struct kunit *test)
{
	static const struct {
		u8 mode;
		u32 top_factor;
		u32 bottom_factor;
	} modes[] = {
		{
			RK_RGA_ALPHA_BLEND_SRC,
			RK_RGA3_ALPHA_ONE,
			RK_RGA3_ALPHA_ZERO,
		}, {
			RK_RGA_ALPHA_BLEND_DST,
			RK_RGA3_ALPHA_ZERO,
			RK_RGA3_ALPHA_ONE,
		}, {
			RK_RGA_ALPHA_BLEND_SRC_OVER,
			RK_RGA3_ALPHA_ONE,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
		}, {
			RK_RGA_ALPHA_BLEND_DST_OVER,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
			RK_RGA3_ALPHA_ONE,
		}, {
			RK_RGA_ALPHA_BLEND_SRC_IN,
			RK_RGA3_ALPHA_OPPOSITE,
			RK_RGA3_ALPHA_ZERO,
		}, {
			RK_RGA_ALPHA_BLEND_DST_IN,
			RK_RGA3_ALPHA_ZERO,
			RK_RGA3_ALPHA_OPPOSITE,
		}, {
			RK_RGA_ALPHA_BLEND_SRC_OUT,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
			RK_RGA3_ALPHA_ZERO,
		}, {
			RK_RGA_ALPHA_BLEND_DST_OUT,
			RK_RGA3_ALPHA_ZERO,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
		}, {
			RK_RGA_ALPHA_BLEND_SRC_ATOP,
			RK_RGA3_ALPHA_OPPOSITE,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
		}, {
			RK_RGA_ALPHA_BLEND_DST_ATOP,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
			RK_RGA3_ALPHA_OPPOSITE,
		}, {
			RK_RGA_ALPHA_BLEND_XOR,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
			RK_RGA3_ALPHA_OPPOSITE_INVERSE,
		}, {
			RK_RGA_ALPHA_BLEND_CLEAR,
			RK_RGA3_ALPHA_ZERO,
			RK_RGA3_ALPHA_ZERO,
		},
	};
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_RGBA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	enum rk_rga_hw_type type;
	u32 top_factor;
	u32 bottom_factor;
	unsigned int i;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_RGBA_8888,
				    1280, 720);
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.yuv2rgb_mode = 0;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		type = 0;
		top_factor = U32_MAX;
		bottom_factor = U32_MAX;
		task.PD_mode = modes[i].mode;

		KUNIT_EXPECT_EQ(test,
				rk_rga3_alpha_factors(task.PD_mode,
						      &top_factor,
						      &bottom_factor),
				0);
		KUNIT_EXPECT_EQ(test, top_factor, modes[i].top_factor);
		KUNIT_EXPECT_EQ(test, bottom_factor,
				modes[i].bottom_factor);
		KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
		KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);

		memset(cmd, 0, sizeof(cmd));
		job.cmd_ready = false;
		KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
		KUNIT_EXPECT_TRUE(test, job.cmd_ready);
		KUNIT_EXPECT_EQ(test,
				cmd[RK_RGA3_OVLP_TOP_CTRL_OFFSET / 4] &
				RK_RGA3_ALPHA_FACTOR,
				FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
					   modes[i].top_factor));
		KUNIT_EXPECT_EQ(test,
				cmd[RK_RGA3_OVLP_BOT_CTRL_OFFSET / 4] &
				RK_RGA3_ALPHA_FACTOR,
				FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
					   modes[i].bottom_factor));
		KUNIT_EXPECT_EQ(test,
				cmd[RK_RGA3_OVLP_TOP_ALPHA_OFFSET / 4] &
				RK_RGA3_ALPHA_FACTOR,
				FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
					   modes[i].top_factor));
		KUNIT_EXPECT_EQ(test,
				cmd[RK_RGA3_OVLP_BOT_ALPHA_OFFSET / 4] &
				RK_RGA3_ALPHA_FACTOR,
				FIELD_PREP(RK_RGA3_ALPHA_FACTOR,
					   modes[i].bottom_factor));
	}

	type = 0;
	top_factor = U32_MAX;
	bottom_factor = U32_MAX;
	task.PD_mode = RK_RGA_ALPHA_BLEND_CLEAR + 1;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test,
			rk_rga3_alpha_factors(task.PD_mode, &top_factor,
					      &bottom_factor),
			-EOPNOTSUPP);
}

static void rk_rga3_alpha_yuv10_overlay_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP_10B,
					  RK_RGA_FORMAT_YCBCR_420_SP_10B);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_RGBA_8888,
				    task.dst.act_w, task.dst.act_h);
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_DST_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.yuv2rgb_mode = 1 | (2 << 2);

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			   RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_R2Y_EN);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			RK_RGA3_WR_MODE, FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4],
			1280U >> 1);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      ((1280U >> 2) * 720U) / 4));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 0) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
	memset(&task.pat, 0, sizeof(task.pat));
	task.bsfilter_flag = 0;
	job.import_count = 2;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
			RK_RGA3_OVLP_FIELD | RK_RGA3_OVLP_TOP_ALPHA_EN);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			   RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);

	memset(cmd, 0, sizeof(cmd));
	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_YCBCR_420_SP_10B,
				    1280, 720);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_YCBCR_420_SP,
				    1280, 720);
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			   RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_YUV10_COMPACT);
}

static void rk_rga3_colorkey_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 expected_ctrl;
	u32 expected_min;
	u32 expected_max;

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    320, 240);
	task.dst = rk_rga_kunit_img(0x20000000, RK_RGA_FORMAT_BGRA_8888,
				    320, 240);
	task.yuv2rgb_mode = 0;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.alpha_rop_mode = 0x11;
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC;
	task.src_trans_mode = 0x1e;
	task.color_key_min = 0x00112233;
	task.color_key_max = 0x00445566;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	expected_ctrl = FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
			RK_RGA3_OVLP_TOP_ALPHA_EN |
			FIELD_PREP(RK_RGA3_OVLP_TOP_KEY_EN, 1);
	expected_min = (0x33 << 22) | (0x22 << 2) | (0x11 << 12);
	expected_max = (0x66 << 22) | (0x55 << 2) | (0x44 << 12);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			expected_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_KEY_MIN_OFFSET / 4],
			expected_min);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_KEY_MAX_OFFSET / 4],
			expected_max);

	memset(cmd, 0, sizeof(cmd));
	job.cmd_ready = false;
	task.src_trans_mode = 0x1f;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_CTRL_OFFSET / 4],
			expected_ctrl);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_KEY_MIN_OFFSET / 4],
			expected_min);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_TOP_KEY_MAX_OFFSET / 4],
			expected_max);

	task.src_trans_mode = 0x1d;
	type = 0;
	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), -EOPNOTSUPP);
}

static void rk_rga3_alpha_rotate_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 3,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.src = rk_rga_kunit_img(0x10000000, RK_RGA_FORMAT_RGBA_8888,
				    720, 1280);
	task.pat = rk_rga_kunit_img(0x30000000, RK_RGA_FORMAT_BGRA_8888,
				    1280, 720);
	task.bsfilter_flag = 1;
	task.alpha_rop_flag = BIT(0) | BIT(3) | BIT(4) | BIT(9);
	task.PD_mode = RK_RGA_ALPHA_BLEND_SRC_OVER;
	task.feature.global_alpha_en = true;
	task.fg_global_alpha = 0xff;
	task.bg_global_alpha = 0xff;
	task.rotate_mode = 1;
	task.sina = 65536;

	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			   RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_ROT);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			1280 | (720 << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_DST_SIZE_OFFSET / 4],
			1280 | (720 << 16));

	memset(cmd, 0, sizeof(cmd));
	memset(&task.pat, 0, sizeof(task.pat));
	task.bsfilter_flag = 0;
	job.import_count = 2;
	job.cmd_ready = false;

	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			720 | (1280 << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			720 | (1280 << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_DST_SIZE_OFFSET / 4],
			720 | (1280 << 16));
}

static void rk_rga3_dst_offset_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_YCBCR_420_SP);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 uv_stride_bytes;
	u32 y_stride_bytes;
	u32 afbc_header_size;

	task.dst.act_w = 640;
	task.dst.act_h = 360;
	task.dst.x_offset = 2;
	task.dst.y_offset = 4;

	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);

	y_stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
	uv_stride_bytes = cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4] << 2;
	KUNIT_EXPECT_EQ(test, y_stride_bytes, 1280);
	KUNIT_EXPECT_EQ(test, uv_stride_bytes, 1280);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      4 * y_stride_bytes + 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      2 * uv_stride_bytes + 2));

	memset(cmd, 0, sizeof(cmd));
	task.dst.x_offset = 1;
	task.dst.y_offset = 4;
	job.cmd_ready = false;
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), -EINVAL);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	task.dst.x_offset = 2;
	task.dst.y_offset = 3;
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), -EINVAL);
	KUNIT_EXPECT_FALSE(test, job.cmd_ready);

	memset(cmd, 0, sizeof(cmd));
	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_YCBCR_420_SP_10B);
	task.dst.act_w = 640;
	task.dst.act_h = 360;
	task.dst.x_offset = 64;
	task.dst.y_offset = 8;
	task.dst.compact_mode = RK_RGA_10BIT_INCOMPACT;
	task.dst.is_10b_endian = 1;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_ENDIAN_MODE);

	y_stride_bytes = (cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2) * 2;
	uv_stride_bytes =
		(cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4] << 2) * 2;
	KUNIT_EXPECT_EQ(test, y_stride_bytes, 2560);
	KUNIT_EXPECT_EQ(test, uv_stride_bytes, 2560);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      8 * y_stride_bytes + 64 * 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      4 * uv_stride_bytes + 64 * 2));

	memset(cmd, 0, sizeof(cmd));
	task.dst.compact_mode = 0;
	task.dst.is_10b_endian = 0;
	job.cmd_ready = false;
	type = 0;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_ENDIAN_MODE);

	y_stride_bytes = cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2;
	uv_stride_bytes = cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4] << 2;
	KUNIT_EXPECT_EQ(test, y_stride_bytes, 1280U);
	KUNIT_EXPECT_EQ(test, uv_stride_bytes, 1280U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      8 * y_stride_bytes + 64));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      4 * uv_stride_bytes + 64));

	memset(cmd, 0, sizeof(cmd));
	task = rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					 RK_RGA_FORMAT_RGBA_8888);
	task.dst.rd_mode = RK_RGA_FBC_MODE;
	task.dst.x_offset = 32;
	task.dst.y_offset = 16;
	job.cmd_ready = false;
	type = 0;

	afbc_header_size = ((ALIGN((u32)task.dst.vir_w, 16) >> 2) *
			    ALIGN((u32)task.dst.vir_h, 16)) >> 2;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			RK_RGA3_WIN0_RD_MODE,
			FIELD_PREP(RK_RGA3_WIN0_RD_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN1_RD_CTRL_OFFSET / 4] &
			RK_RGA3_WIN0_RD_MODE, 0U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			RK_RGA3_WR_MODE,
			FIELD_PREP(RK_RGA3_WR_MODE, 1));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_OVLP_OFF_OFFSET / 4],
			32U | (16U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr + afbc_header_size));
}

static void rk_rga3_ffmpeg_p210_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	enum rk_rga_hw_type type = 0;
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_422_SP_10B,
					  RK_RGA_FORMAT_YCBCR_422_SP_10B);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};
	u32 uv_stride_bytes;
	u32 y_stride_bytes;

	task.src.compact_mode = RK_RGA_10BIT_INCOMPACT;
	task.src.is_10b_endian = 1;
	task.dst.act_w = 640;
	task.dst.act_h = 360;
	task.dst.x_offset = 32;
	task.dst.y_offset = 7;
	task.dst.compact_mode = RK_RGA_10BIT_INCOMPACT;
	task.dst.is_10b_endian = 1;

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type), 0);
	KUNIT_EXPECT_EQ(test, type, RK_RGA_HW_RGA3);
	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			   RK_RGA3_WIN0_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WIN0_RD_CTRL_OFFSET / 4] &
			  RK_RGA3_WIN0_ENDIAN_MODE);
	KUNIT_EXPECT_FALSE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			   RK_RGA3_WR_YUV10_COMPACT);
	KUNIT_EXPECT_TRUE(test, cmd[RK_RGA3_WR_CTRL_OFFSET / 4] &
			  RK_RGA3_WR_ENDIAN_MODE);

	y_stride_bytes = (cmd[RK_RGA3_WR_VIR_STRIDE_OFFSET / 4] << 2) * 2;
	uv_stride_bytes =
		(cmd[RK_RGA3_WR_PL_VIR_STRIDE_OFFSET / 4] << 2) * 2;
	KUNIT_EXPECT_EQ(test, y_stride_bytes, 2560U);
	KUNIT_EXPECT_EQ(test, uv_stride_bytes, 2560U);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_Y_BASE_OFFSET / 4],
			lower_32_bits(task.dst.yrgb_addr +
				      7 * y_stride_bytes + 32 * 2));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WR_U_BASE_OFFSET / 4],
			lower_32_bits(task.dst.uv_addr +
				      7 * uv_stride_bytes + 32 * 2));
}

static void rk_rga3_src_crop_emit_kunit(struct kunit *test)
{
	u32 cmd[RK_RGA3_CMD_REG_COUNT] = { };
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_YCBCR_420_SP,
					  RK_RGA_FORMAT_RGB_888);
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
		.import_count = 2,
		.cmd_vaddr = cmd,
		.cmd_size = sizeof(cmd),
	};

	task.src.act_w = 640;
	task.src.act_h = 360;
	task.src.x_offset = 128;
	task.src.y_offset = 64;
	task.dst.act_w = 320;
	task.dst.act_h = 180;

	KUNIT_EXPECT_EQ(test, rk_rga3_emit_simple_bitblt(&job), 0);
	KUNIT_EXPECT_TRUE(test, job.cmd_ready);
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_Y_BASE_OFFSET / 4],
			lower_32_bits(task.src.yrgb_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_U_BASE_OFFSET / 4],
			lower_32_bits(task.src.uv_addr));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_SRC_SIZE_OFFSET / 4],
			768U | (432U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_OFF_OFFSET / 4],
			128U | (64U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_ACT_SIZE_OFFSET / 4],
			640U | (360U << 16));
	KUNIT_EXPECT_EQ(test, cmd[RK_RGA3_WIN0_DST_SIZE_OFFSET / 4],
			320U | (180U << 16));
}

static struct kunit_case rk_rga_rewrite_test_cases[] = {
	KUNIT_CASE(rk_rga2_decode_transform_kunit),
	KUNIT_CASE(rk_rga3_rotate_flags_kunit),
	KUNIT_CASE(rk_rga2_dst_corner_kunit),
	KUNIT_CASE(rk_rga_fill_hw_type_kunit),
	KUNIT_CASE(rk_rga2_fill_dst_offset_emit_kunit),
	KUNIT_CASE(rk_rga2_pre_intr_kunit),
	KUNIT_CASE(rk_rga2_fill_yuv_emit_kunit),
	KUNIT_CASE(rk_rga2_fill_packed_yuv_emit_kunit),
	KUNIT_CASE(rk_rga2_fill_multitask_hw_type_kunit),
	KUNIT_CASE(rk_rga2_rectangle_task_emit_kunit),
	KUNIT_CASE(rk_rga2_mosaic_emit_kunit),
	KUNIT_CASE(rk_rga2_mosaic_task_array_emit_kunit),
	KUNIT_CASE(rk_rga2_rop_emit_kunit),
	KUNIT_CASE(rk_rga2_colorkey_emit_kunit),
	KUNIT_CASE(rk_rga2_gauss_emit_kunit),
	KUNIT_CASE(rk_rga2_quantize_emit_kunit),
	KUNIT_CASE(rk_rga2_alpha_bitmap_emit_kunit),
	KUNIT_CASE(rk_rga2_osd_emit_kunit),
	KUNIT_CASE(rk_rga2_palette_emit_kunit),
	KUNIT_CASE(rk_rga2_update_palette_emit_kunit),
	KUNIT_CASE(rk_rga_request_check_kunit),
	KUNIT_CASE(rk_rga_request_ioctl_ret_kunit),
	KUNIT_CASE(rk_rga_request_create_cancel_ioctl_kunit),
	KUNIT_CASE(rk_rga_request_config_handles_kunit),
	KUNIT_CASE(rk_rga_direct_img_mem_type_kunit),
	KUNIT_CASE(rk_rga_request_config_direct_phys_reject_kunit),
	KUNIT_CASE(rk_rga_request_reconfig_resources_kunit),
	KUNIT_CASE(rk_rga_request_reconfig_fences_kunit),
	KUNIT_CASE(rk_rga_request_config_ioctl_acquire_kunit),
	KUNIT_CASE(rk_rga_request_cancel_configured_ioctl_kunit),
	KUNIT_CASE(rk_rga_release_configured_request_kunit),
	KUNIT_CASE(rk_rga_release_pending_acquire_job_kunit),
	KUNIT_CASE(rk_rga_last_hw_remove_pending_acquire_kunit),
	KUNIT_CASE(rk_rga_release_queued_job_kunit),
	KUNIT_CASE(rk_rga_request_reconfig_gauss_kunit),
	KUNIT_CASE(rk_rga_legacy_blit_sync_wait_kunit),
	KUNIT_CASE(rk_rga_legacy_blit_async_acquire_kunit),
	KUNIT_CASE(rk_rga_request_submit_async_acquire_kunit),
	KUNIT_CASE(rk_rga_version_queries_kunit),
	KUNIT_CASE(rk_rga_legacy_noop_ioctls_kunit),
	KUNIT_CASE(rk_rga_request_remove_free_kunit),
	KUNIT_CASE(rk_rga_import_buffer_size_kunit),
	KUNIT_CASE(rk_rga_iova_span_kunit),
	KUNIT_CASE(rk_rga_clock_count_kunit),
	KUNIT_CASE(rk_rga_mmio_size_kunit),
	KUNIT_CASE(rk_rga_hw_version_kunit),
	KUNIT_CASE(rk_rga_import_dmabuf_fd_kunit),
	KUNIT_CASE(rk_rga_import_buffer_ioctl_errors_kunit),
	KUNIT_CASE(rk_rga_release_buffer_ioctl_kunit),
	KUNIT_CASE(rk_rga_buffer_pool_zero_count_kunit),
	KUNIT_CASE(rk_rga_acquire_fd_ownership_kunit),
	KUNIT_CASE(rk_rga_acquire_fence_status_kunit),
	KUNIT_CASE(rk_rga_acquire_callbacks_result_kunit),
	KUNIT_CASE(rk_rga_acquire_abort_during_arming_kunit),
	KUNIT_CASE(rk_rga_session_dispatch_close_handoff_kunit),
	KUNIT_CASE(rk_rga_job_free_release_fence_kunit),
	KUNIT_CASE(rk_rga_release_fence_fd_state_kunit),
	KUNIT_CASE(rk_rga_hw_abort_queued_jobs_kunit),
	KUNIT_CASE(rk_rga_queue_on_removing_hw_kunit),
	KUNIT_CASE(rk_rga_queue_on_recovery_failed_hw_kunit),
	KUNIT_CASE(rk_rga_recovery_failed_dispatch_kunit),
	KUNIT_CASE(rk_rga_irq_completion_result_kunit),
	KUNIT_CASE(rk_rga_mixed_task_hw_type_kunit),
	KUNIT_CASE(rk_rga_mixed_task_core_handoff_kunit),
	KUNIT_CASE(rk_rga_bitblt_hw_type_mask_kunit),
	KUNIT_CASE(rk_rga_task_core_invalid_mask_kunit),
	KUNIT_CASE(rk_rga_find_best_hw_for_job_kunit),
	KUNIT_CASE(rk_rga_core_counter_kunit),
	KUNIT_CASE(rk_rga_core_slot_reprobe_kunit),
	KUNIT_CASE(rk_rga_priority_enqueue_kunit),
	KUNIT_CASE(rk_rga_iommu_fault_generation_kunit),
	KUNIT_CASE(rk_rga_timeout_target_replacement_kunit),
	KUNIT_CASE(rk_rga_iommu_fault_match_kunit),
	KUNIT_CASE(rk_rga_iommu_refresh_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_rga3_profiles_kunit),
	KUNIT_CASE(rk_rga_gstreamer_legacy_convert_profiles_kunit),
	KUNIT_CASE(rk_rga_gstreamer_legacy_format_matrix_kunit),
	KUNIT_CASE(rk_rga_rknn_preprocess_profiles_kunit),
	KUNIT_CASE(rk_rga_gstreamer_legacy_rotation_extrema_kunit),
	KUNIT_CASE(rk_rga3_librga_resize_interp_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_drm_abgr_copy_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_copy_splice_task_kunit),
	KUNIT_CASE(rk_rga3_multitask_emit_clears_stale_regs_kunit),
	KUNIT_CASE(rk_rga3_librga_translate_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_rotate_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_rotate_flip_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_center_rotate_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_flip_emit_kunit),
	KUNIT_CASE(rk_rga3_yuv422_rotate_policy_kunit),
	KUNIT_CASE(rk_rga3_librga_side_border_kunit),
	KUNIT_CASE(rk_rga3_librga_padding_kunit),
	KUNIT_CASE(rk_rga2_compact_10bit_profile_kunit),
	KUNIT_CASE(rk_rga2_librga_interp_emit_kunit),
	KUNIT_CASE(rk_rga2_librga_y400_uv_downsample_kunit),
	KUNIT_CASE(rk_rga2_librga_gray256_cvtcolor_kunit),
	KUNIT_CASE(rk_rga2_librga_y4_dither_emit_kunit),
	KUNIT_CASE(rk_rga2_librga_full_csc_emit_kunit),
	KUNIT_CASE(rk_rga2_src_crop_emit_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_fbc_profiles_kunit),
	KUNIT_CASE(rk_rga3_librga_afbc_copy_emit_kunit),
	KUNIT_CASE(rk_rga3_tile8x8_profile_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_alpha_overlay_kunit),
	KUNIT_CASE(rk_rga3_librga_alpha_yuv_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_slt_alpha_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_global_alpha_emit_kunit),
	KUNIT_CASE(rk_rga3_librga_rgb_composite_emit_kunit),
	KUNIT_CASE(rk_rga3_display_partial_alpha_blend_kunit),
	KUNIT_CASE(rk_rga3_display_rgb565_rotate_kunit),
	KUNIT_CASE(rk_rga2_display_xrgb_rotate_kunit),
	KUNIT_CASE(rk_rga3_pattern_rotate_reject_kunit),
	KUNIT_CASE(rk_rga3_librga_blend_modes_kunit),
	KUNIT_CASE(rk_rga3_alpha_yuv10_overlay_emit_kunit),
	KUNIT_CASE(rk_rga3_colorkey_emit_kunit),
	KUNIT_CASE(rk_rga3_alpha_rotate_emit_kunit),
	KUNIT_CASE(rk_rga3_dst_offset_emit_kunit),
	KUNIT_CASE(rk_rga3_ffmpeg_p210_emit_kunit),
	KUNIT_CASE(rk_rga3_src_crop_emit_kunit),
	{ }
};

static struct kunit_suite rk_rga_rewrite_test_suite = {
	.name = "rockchip-rga-rewrite",
	.test_cases = rk_rga_rewrite_test_cases,
};

kunit_test_suite(rk_rga_rewrite_test_suite);
#endif

static int rk_rga2_validate_image(const struct rga_img_info_t *img,
				  const struct rk_rga2_format_info *fmt);

static int
rk_rga2_validate_fill_csc(const struct rga_req *task,
			  const struct rk_rga2_format_info *dst_fmt)
{
	u8 mode = task->yuv2rgb_mode;

	if (!dst_fmt->yuv)
		return mode ? -EOPNOTSUPP : 0;
	if (!mode)
		return -EOPNOTSUPP;
	if (mode & ~GENMASK(4, 2))
		return -EOPNOTSUPP;
	if (!((mode >> 2) & 0x3))
		return -EOPNOTSUPP;

	return 0;
}

static int rk_rga2_validate_color_fill(const struct rga_req *task,
				       struct rk_rga2_fill_profile *profile)
{
	int ret;

	if (task->render_mode != RK_RGA_RENDER_COLOR_FILL)
		return -EOPNOTSUPP;
	if (task->color_fill_mode)
		return -EOPNOTSUPP;
	if (task->src.yrgb_addr || task->src.uv_addr || task->src.v_addr)
		return -EOPNOTSUPP;
	if (task->pat.yrgb_addr || task->pat.uv_addr || task->pat.v_addr)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr || task->bsfilter_flag ||
	    task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->feature.global_alpha_en)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->full_csc.flag || task->mosaic_info.enable ||
	    task->osd_info.enable || task->gauss_config.size)
		return -EOPNOTSUPP;
	if (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;

	ret = rk_rga2_fill_format_info(task->dst.format, profile);
	if (ret)
		return ret;
	ret = rk_rga2_validate_fill_csc(task, &profile->dst_fmt);
	if (ret)
		return ret;

	return rk_rga2_validate_image(&task->dst, &profile->dst_fmt);
}

static int rk_rga2_palette_source_mode(const struct rga_req *task)
{
	u8 expected;

	switch (task->src.format) {
	case RK_RGA_FORMAT_BPP1:
		expected = 0;
		break;
	case RK_RGA_FORMAT_BPP2:
		expected = 1;
		break;
	case RK_RGA_FORMAT_BPP4:
		expected = 2;
		break;
	case RK_RGA_FORMAT_BPP8:
	case RK_RGA_FORMAT_YCBCR_400:
		expected = 3;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (task->palette_mode != expected)
		return -EOPNOTSUPP;

	return 0;
}

static int rk_rga2_palette_shift(const struct rga_req *task, u8 *shift)
{
	int ret;

	ret = rk_rga2_palette_source_mode(task);
	if (ret)
		return ret;

	*shift = 3 - task->palette_mode;
	return 0;
}

static bool rk_rga2_palette_dst_format(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_RGBA_8888:
	case RK_RGA_FORMAT_BGRA_8888:
	case RK_RGA_FORMAT_RGBX_8888:
	case RK_RGA_FORMAT_BGRX_8888:
	case RK_RGA_FORMAT_RGB_888:
	case RK_RGA_FORMAT_BGR_888:
	case RK_RGA_FORMAT_RGB_565:
	case RK_RGA_FORMAT_BGR_565:
	case RK_RGA_FORMAT_RGBA_5551:
	case RK_RGA_FORMAT_BGRA_5551:
	case RK_RGA_FORMAT_RGBA_4444:
	case RK_RGA_FORMAT_BGRA_4444:
	case RK_RGA_FORMAT_ARGB_8888:
	case RK_RGA_FORMAT_ABGR_8888:
	case RK_RGA_FORMAT_XRGB_8888:
	case RK_RGA_FORMAT_XBGR_8888:
	case RK_RGA_FORMAT_ARGB_5551:
	case RK_RGA_FORMAT_ABGR_5551:
	case RK_RGA_FORMAT_ARGB_4444:
	case RK_RGA_FORMAT_ABGR_4444:
		return true;
	default:
		return false;
	}
}

static int rk_rga2_validate_color_palette(const struct rga_req *task,
					  struct rk_rga2_palette_profile *profile)
{
	int ret;

	if (task->render_mode != RK_RGA_RENDER_COLOR_PALETTE)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr || task->bsfilter_flag ||
	    task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->feature.global_alpha_en || task->rop_code ||
	    task->alpha_rop_mode)
		return -EOPNOTSUPP;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;
	if (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;
	if (task->src.rotate_mode || task->dst.rotate_mode ||
	    task->rotate_mode || task->sina || task->cosa)
		return -EOPNOTSUPP;
	if (task->interp.horiz || task->interp.verti)
		return -EOPNOTSUPP;
	if (task->yuv2rgb_mode || task->full_csc.flag)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable || task->osd_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if (task->src.act_w != task->dst.act_w ||
	    task->src.act_h != task->dst.act_h)
		return -EOPNOTSUPP;
	if (rk_rga2_palette_source_mode(task))
		return -EOPNOTSUPP;
	if (!rk_rga2_palette_dst_format(task->dst.format))
		return -EOPNOTSUPP;
	if (rk_rga_img_has_addr(&task->pat) &&
	    (task->pat.format != RK_RGA_FORMAT_RGBA_8888 ||
	     (task->pat.rd_mode && task->pat.rd_mode != RK_RGA_RASTER_MODE) ||
	     task->pat.x_offset || task->pat.y_offset ||
	     task->pat.act_w != 16 || task->pat.act_h != 16 ||
	     task->pat.vir_w < 16 || task->pat.vir_h < 16))
		return -EOPNOTSUPP;

	ret = rk_rga2_format_info(task->dst.format, true, &profile->dst_fmt);
	if (ret)
		return ret;
	ret = rk_rga2_validate_image(&task->src, &(struct rk_rga2_format_info) {
		.hw_format = 0xf,
		.pixel_width = 1,
		.x_div = 1,
		.y_div = 1,
	});
	if (ret)
		return ret;

	return rk_rga2_validate_image(&task->dst, &profile->dst_fmt);
}

static int rk_rga2_validate_update_palette(const struct rga_req *task)
{
	if (task->render_mode != RK_RGA_RENDER_UPDATE_PALETTE)
		return -EOPNOTSUPP;
	if (task->palette_mode > 3)
		return -EOPNOTSUPP;
	if (task->fading.g != 0xff)
		return -EOPNOTSUPP;
	if (!rk_rga_img_has_addr(&task->pat))
		return -EINVAL;
	if (task->src.yrgb_addr || task->src.uv_addr || task->src.v_addr ||
	    task->dst.yrgb_addr || task->dst.uv_addr || task->dst.v_addr)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr || task->bsfilter_flag ||
	    task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->feature.global_alpha_en || task->rop_code ||
	    task->alpha_rop_mode)
		return -EOPNOTSUPP;
	if (task->pat.format != RK_RGA_FORMAT_RGBA_8888 ||
	    (task->pat.rd_mode && task->pat.rd_mode != RK_RGA_RASTER_MODE))
		return -EOPNOTSUPP;
	if (task->pat.x_offset || task->pat.y_offset ||
	    task->pat.act_w != 16 || task->pat.act_h != 16 ||
	    task->pat.vir_w < 16 || task->pat.vir_h < 16)
		return -EOPNOTSUPP;

	return 0;
}

static int rk_rga2_validate_image(const struct rga_img_info_t *img,
				  const struct rk_rga2_format_info *fmt)
{
	u32 width;
	u32 height;

	if (!img->act_w || !img->act_h || !img->vir_w || !img->vir_h)
		return -EINVAL;

	if (check_add_overflow((u32)img->x_offset, (u32)img->act_w, &width) ||
	    check_add_overflow((u32)img->y_offset, (u32)img->act_h, &height))
		return -EOVERFLOW;

	if (width > img->vir_w || height > img->vir_h)
		return -EINVAL;
	if (width > 8192 || height > 8192)
		return -EINVAL;

	if (fmt->y4_lut &&
	    ((img->x_offset | img->y_offset | img->act_w | img->act_h |
	      img->vir_w | img->vir_h) & 0x1))
		return -EINVAL;
	if (fmt->y4 && (img->vir_w & 0x7))
		return -EINVAL;

	if (fmt->plane_width) {
		if ((img->x_offset % fmt->x_div) || (img->act_w % fmt->x_div) ||
		    (img->vir_w % fmt->x_div))
			return -EINVAL;
		if ((img->y_offset % fmt->y_div) || (img->act_h % fmt->y_div) ||
		    (img->vir_h % fmt->y_div))
			return -EINVAL;
	}

	return 0;
}

static int rk_rga2_validate_bitblt(const struct rga_req *task,
				   struct rk_rga2_bitblt_profile *profile)
{
	struct rga_img_info_t dst;
	bool uses_alpha_bitmap = rk_rga2_task_uses_alpha_bitmap(task);
	bool uses_color_key = rk_rga2_task_uses_color_key(task);
	bool uses_rop = !uses_color_key && rk_rga2_task_uses_rop(task);
	bool uses_quantize = rk_rga2_task_uses_quantize(task);
	bool uses_gauss = rk_rga2_task_uses_gauss(task);
	bool uses_osd = rk_rga2_task_uses_osd(task);
	int ret;

	profile->alpha_bitmap = false;
	profile->color_key = false;
	profile->osd = false;

	if (task->render_mode != RK_RGA_RENDER_BITBLT)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr)
		return -EOPNOTSUPP;
	if (task->bsfilter_flag && !uses_alpha_bitmap && !uses_osd)
		return -EOPNOTSUPP;
	if (rk_rga_img_has_addr(&task->pat) && !uses_alpha_bitmap && !uses_osd)
		return -EOPNOTSUPP;
	if (task->mosaic_info.enable) {
		if (!rk_rga2_in_place_mosaic_allowed(task))
			return -EOPNOTSUPP;
	} else if (uses_alpha_bitmap) {
		if (rk_rga_img_has_addr(&task->pat) &&
		    task->src.yrgb_addr == task->dst.yrgb_addr)
			return -EOPNOTSUPP;
	} else if (uses_osd) {
		if (!rk_rga2_osd_in_place_allowed(task))
			return -EOPNOTSUPP;
	} else if (uses_color_key) {
		if (!rk_rga_in_place_bitblt_allowed(task))
			return -EOPNOTSUPP;
	} else if (uses_rop) {
		if (!rk_rga2_rop_bitblt_allowed(task))
			return -EOPNOTSUPP;
	} else if (!rk_rga_in_place_bitblt_allowed(task)) {
		return -EOPNOTSUPP;
	}
	if (uses_alpha_bitmap) {
		ret = rk_rga2_validate_alpha_bitmap(task);
		if (ret)
			return ret;
	} else if (uses_quantize) {
		ret = rk_rga2_validate_quantize(task);
		if (ret)
			return ret;
	} else if (uses_color_key) {
		ret = rk_rga2_validate_color_key(task);
		if (ret)
			return ret;
	} else if (uses_rop) {
		ret = rk_rga2_validate_rop(task);
		if (ret)
			return ret;
	} else if (uses_gauss) {
		ret = rk_rga2_validate_gauss(task);
		if (ret)
			return ret;
	} else if (uses_osd) {
		ret = rk_rga2_validate_osd(task);
		if (ret)
			return ret;
	} else {
		if (task->alpha_rop_flag || task->PD_mode ||
		    task->feature.global_alpha_en)
			return -EOPNOTSUPP;
	}
	ret = rk_rga2_decode_transform(task, &profile->transform);
	if (ret)
		return ret;
	ret = rk_rga2_validate_full_csc(task);
	if (ret)
		return ret;
	if (task->src.rd_mode && task->src.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;
	if (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;
	if (task->interp.horiz > RK_RGA2_INTERP_AVERAGE ||
	    task->interp.verti > RK_RGA2_INTERP_AVERAGE)
		return -EOPNOTSUPP;

	ret = rk_rga2_format_info(task->src.format, false,
				  &profile->src_fmt);
	if (ret)
		return ret;
	ret = rk_rga2_format_info(task->dst.format, true,
				  &profile->dst_fmt);
	if (ret)
		return ret;
	if (uses_alpha_bitmap || uses_osd) {
		ret = rk_rga2_format_info(task->pat.format, false,
					  &profile->pat_fmt);
		if (ret)
			return ret;
		profile->alpha_bitmap = uses_alpha_bitmap;
		profile->osd = uses_osd;
	}
	profile->color_key = uses_color_key;
	if (profile->src_fmt.yuv10 &&
	    !rk_rga_img_yuv10_compact(&task->src))
		return -EOPNOTSUPP;
	ret = rk_rga2_validate_image(&task->src, &profile->src_fmt);
	if (ret)
		return ret;
	rk_rga2_normalized_dst(task, &profile->transform, &dst);
	ret = rk_rga2_validate_image(&dst, &profile->dst_fmt);
	if (ret)
		return ret;
	if (uses_alpha_bitmap) {
		ret = rk_rga2_validate_image(&task->pat, &profile->pat_fmt);
		if (ret)
			return ret;
	}
	if (task->mosaic_info.enable &&
	    (profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10))
		return -EOPNOTSUPP;
	if (uses_rop &&
	    (profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10))
		return -EOPNOTSUPP;
	if (uses_gauss &&
	    (profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10))
		return -EOPNOTSUPP;
	if (uses_quantize &&
	    (profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10))
		return -EOPNOTSUPP;
	if (uses_color_key &&
	    (!profile->src_fmt.alpha || !profile->dst_fmt.alpha ||
	     profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10))
		return -EOPNOTSUPP;
	if (profile->dst_fmt.y4_lut) {
		if (!rk_rga2_dither_flags_allowed(task))
			return -EOPNOTSUPP;
		if (task->PD_mode || task->feature.global_alpha_en ||
		    task->rop_code || task->alpha_rop_mode)
			return -EOPNOTSUPP;
		if (task->full_csc.flag & ~RK_RGA_FULL_CSC_ENABLE)
			return -EOPNOTSUPP;
		if (task->dither_mode > 1)
			return -EOPNOTSUPP;
		if (profile->transform.src_rot_mode ||
		    profile->transform.src_mir_mode ||
		    task->src.act_w != dst.act_w ||
		    task->src.act_h != dst.act_h)
			return -EOPNOTSUPP;
	}
	if ((uses_alpha_bitmap || uses_osd) &&
	    (profile->src_fmt.yuv || profile->src_fmt.yuv400 ||
	     profile->src_fmt.yuv10 || profile->dst_fmt.yuv ||
	     profile->dst_fmt.yuv400 || profile->dst_fmt.yuv10 ||
	     profile->pat_fmt.yuv || profile->pat_fmt.yuv400 ||
	     profile->pat_fmt.yuv10))
		return -EOPNOTSUPP;

	return 0;
}

static int rk_rga3_validate_bitblt(const struct rga_req *task,
				   struct rk_rga3_bitblt_profile *profile)
{
	bool has_pat = rk_rga_img_has_addr(&task->pat);
	int ret;

	if (task->render_mode != RK_RGA_RENDER_BITBLT)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr)
		return -EOPNOTSUPP;
	if (task->bsfilter_flag != has_pat)
		return -EOPNOTSUPP;
	if (!rk_rga_in_place_bitblt_allowed(task))
		return -EOPNOTSUPP;
	if (task->full_csc.flag || task->mosaic_info.enable ||
	    task->osd_info.enable || task->pre_intr_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag & BIT(8))
		return -EOPNOTSUPP;
	if (task->rgba5551_alpha.flags)
		return -EOPNOTSUPP;

	ret = rk_rga3_validate_alpha_blend(task);
	if (ret)
		return ret;
	ret = rk_rga3_validate_color_key(task, has_pat);
	if (ret)
		return ret;

	profile->alpha_blend = rk_rga3_task_uses_alpha_blend(task);
	profile->pattern_blend = has_pat;
	profile->color_key = rk_rga3_task_uses_color_key(task);
	profile->overlap_copy = false;
	if (profile->pattern_blend && !profile->alpha_blend)
		return -EOPNOTSUPP;
	if (profile->pattern_blend && task->pat.rotate_mode)
		return -EOPNOTSUPP;

	ret = rk_rga3_rotate_flags(task, &profile->rotate_flags);
	if (ret)
		return ret;
	if ((profile->rotate_flags & RK_RGA3_ROT_BIT_ROT_90) &&
	    rk_rga_format_is_rga3_yuv422_rotate_blocked(task->src.format))
		return -EOPNOTSUPP;

	ret = rk_rga3_hw_rd_mode(task->src.rd_mode, &profile->src_mode);
	if (ret)
		return ret;
	ret = rk_rga3_hw_rd_mode(task->dst.rd_mode, &profile->dst_mode);
	if (ret)
		return ret;
	if (profile->pattern_blend) {
		ret = rk_rga3_hw_rd_mode(task->pat.rd_mode, &profile->bg_mode);
		if (ret)
			return ret;
	} else {
		profile->bg_mode = profile->dst_mode;
	}
	if (profile->src_mode == 2 || profile->dst_mode == 2 ||
	    profile->bg_mode == 2) {
		if (profile->alpha_blend || profile->pattern_blend ||
		    profile->color_key)
			return -EOPNOTSUPP;
		if (task->src.yrgb_addr == task->dst.yrgb_addr)
			return -EOPNOTSUPP;
	}
	if (profile->dst_mode == 2 &&
	    (task->dst.x_offset || task->dst.y_offset))
		return -EOPNOTSUPP;
	if (profile->alpha_blend && !profile->pattern_blend &&
	    profile->bg_mode == 1 && profile->dst_mode == 1)
		return -EOPNOTSUPP;

	ret = rk_rga3_validate_image(&task->src);
	if (ret)
		return ret;

	ret = rk_rga3_validate_image(&task->dst);
	if (ret)
		return ret;
	if (profile->pattern_blend) {
		ret = rk_rga3_validate_image(&task->pat);
		if (ret)
			return ret;
		if (task->pat.act_w != task->dst.act_w ||
		    task->pat.act_h != task->dst.act_h)
			return -EOPNOTSUPP;
	}

	ret = rk_rga3_format_info(task->src.format, false,
				  &profile->src_fmt);
	if (ret)
		return ret;
	if (profile->src_mode == 1 &&
	    !rk_rga3_fbc_format_supported(task->src.format, false))
		return -EOPNOTSUPP;
	if (profile->src_mode == 2 &&
	    !rk_rga3_tile_format_supported(task->src.format))
		return -EOPNOTSUPP;
	ret = rk_rga3_format_info(task->dst.format, true,
				  &profile->dst_fmt);
	if (ret)
		return ret;
	if (profile->dst_mode == 1 &&
	    !rk_rga3_fbc_format_supported(task->dst.format, true))
		return -EOPNOTSUPP;
	if (profile->dst_mode == 2 &&
	    !rk_rga3_tile_format_supported(task->dst.format))
		return -EOPNOTSUPP;
	if (profile->color_key &&
	    (profile->src_mode || profile->dst_mode ||
	     !profile->src_fmt.rgb || !profile->dst_fmt.rgb))
		return -EOPNOTSUPP;
	profile->overlap_copy = !profile->alpha_blend &&
				(task->src.yrgb_addr == task->dst.yrgb_addr ||
				 (profile->dst_mode == 1 &&
				  (task->dst.x_offset || task->dst.y_offset)));
	if (!profile->alpha_blend)
		return 0;

	ret = rk_rga3_format_info(profile->pattern_blend ? task->pat.format :
				  task->dst.format, false,
				  &profile->bg_fmt);
	if (ret)
		return ret;
	if (profile->bg_mode == 1 &&
	    !rk_rga3_fbc_format_supported(profile->pattern_blend ?
					  task->pat.format : task->dst.format,
					  false))
		return -EOPNOTSUPP;
	/*
	 * Keep the alpha subset to the formats emitted by librga/ffmpeg:
	 * RGB/RGBA pattern over an RGB destination, the default RKMPP overlay
	 * path where RGB/RGBA pattern data converts into a YUV write domain,
	 * and librga no-pattern A+B->B updates where foreground/background/
	 * writeback are semiplanar YUV formats.
	 */
	if (profile->pattern_blend) {
		if (!profile->bg_fmt.rgb)
			return -EOPNOTSUPP;
		if (!profile->dst_fmt.rgb &&
		    !(profile->dst_fmt.yuv &&
		      (profile->src_fmt.yuv || profile->src_fmt.rgb)))
			return -EOPNOTSUPP;
	} else if (profile->dst_fmt.yuv) {
		if (!profile->src_fmt.yuv || !profile->src_fmt.yuv_sp ||
		    !profile->dst_fmt.yuv_sp)
			return -EOPNOTSUPP;
	} else if (!profile->bg_fmt.rgb) {
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rk_rga3_emit_read_window(struct rk_rga_job *job,
				    const struct rga_img_info_t *img,
				    const struct rk_rga3_format_info *rd_fmt,
				    const struct rk_rga3_format_info *dst_fmt,
				    u32 rd_mode, u32 rotate_flags,
				    u32 dst_w, u32 dst_h, u32 base,
				    bool align_src_size, u8 yuv2rgb_mode,
				    bool rotate_dst_size)
{
	u32 hw_src_w = img->act_w;
	u32 hw_src_h = img->act_h;
	u32 hw_dst_w = dst_w;
	u32 hw_dst_h = dst_h;
	u32 x_factor;
	u32 y_factor;
	u32 stride;
	u32 uv_stride;
	u32 src_size;
	u32 act_off;
	u32 act_size;
	u32 dst_size;
	u32 width;
	u32 height;
	u32 reg = 0;
	bool x_up;
	bool y_up;
	bool x_bypass;
	bool y_bypass;
	int ret;

	if (rotate_flags & RK_RGA3_ROT_BIT_ROT_90) {
		hw_src_w = img->act_h;
		hw_src_h = img->act_w;
		if (rotate_dst_size) {
			hw_dst_w = dst_h;
			hw_dst_h = dst_w;
		}
	}

	ret = rk_rga3_scale_axis(hw_src_w, hw_dst_w, &x_factor,
				 &x_up, &x_bypass);
	if (ret)
		return ret;
	ret = rk_rga3_scale_axis(hw_src_h, hw_dst_h, &y_factor,
				 &y_up, &y_bypass);
	if (ret)
		return ret;
	ret = rk_rga3_read_strides(img, rd_fmt, rd_mode, &stride,
				   &uv_stride);
	if (ret)
		return ret;

	if (check_add_overflow((u32)img->act_w, (u32)img->x_offset,
			       &width) ||
	    check_add_overflow((u32)img->act_h, (u32)img->y_offset,
			       &height))
		return -EOVERFLOW;
	if (align_src_size || rd_mode == 1) {
		width = ALIGN(width, 16);
		height = ALIGN(height, 16);
	}

	ret = rk_rga3_pack_pair(width, height, &src_size);
	if (ret)
		return ret;
	ret = rk_rga3_pack_pair(img->x_offset, img->y_offset, &act_off);
	if (ret)
		return ret;
	ret = rk_rga3_pack_pair(img->act_w, img->act_h, &act_size);
	if (ret)
		return ret;
	ret = rk_rga3_pack_pair(hw_dst_w, hw_dst_h, &dst_size);
	if (ret)
		return ret;

	reg = RK_RGA3_WIN0_ENABLE |
	      FIELD_PREP(RK_RGA3_WIN0_RD_MODE, rd_mode) |
	      FIELD_PREP(RK_RGA3_WIN0_PIC_FORMAT, rd_fmt->pic_format) |
	      FIELD_PREP(RK_RGA3_WIN0_RD_FORMAT, rd_fmt->bus_format) |
	      FIELD_PREP(RK_RGA3_WIN0_PIX_SWAP, rd_fmt->pix_swap) |
	      FIELD_PREP(RK_RGA3_WIN0_YC_SWAP, rd_fmt->yc_swap) |
	      FIELD_PREP(RK_RGA3_WIN0_ROT,
			 !!(rotate_flags & RK_RGA3_ROT_BIT_ROT_90)) |
	      FIELD_PREP(RK_RGA3_WIN0_XMIRROR,
			 !!(rotate_flags & RK_RGA3_ROT_BIT_X_MIRROR)) |
	      FIELD_PREP(RK_RGA3_WIN0_YMIRROR,
			 !!(rotate_flags & RK_RGA3_ROT_BIT_Y_MIRROR)) |
	      FIELD_PREP(RK_RGA3_WIN0_HOR_BY, x_bypass) |
	      FIELD_PREP(RK_RGA3_WIN0_HOR_UP, x_up) |
	      FIELD_PREP(RK_RGA3_WIN0_VER_BY, y_bypass) |
	      FIELD_PREP(RK_RGA3_WIN0_VER_UP, y_up);

	if (rd_fmt->yuv10) {
		if (rd_mode == 1 || rk_rga_img_yuv10_compact(img))
			reg |= RK_RGA3_WIN0_YUV10_COMPACT;
		if (rd_mode == 0 && img->is_10b_endian)
			reg |= RK_RGA3_WIN0_ENDIAN_MODE;
	}

	if (rd_fmt->rgb && dst_fmt->yuv)
		reg |= RK_RGA3_WIN0_R2Y_EN |
		       FIELD_PREP(RK_RGA3_WIN0_CSC_MODE,
				  rk_rga3_r2y_mode(yuv2rgb_mode));
	else if (rd_fmt->yuv && dst_fmt->rgb)
		reg |= RK_RGA3_WIN0_Y2R_EN |
		       FIELD_PREP(RK_RGA3_WIN0_CSC_MODE,
				  rk_rga3_y2r_mode(yuv2rgb_mode));

	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_RD_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_Y_BASE_OFFSET,
			 lower_32_bits(img->yrgb_addr));
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_U_BASE_OFFSET,
			 lower_32_bits(img->uv_addr));
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_V_BASE_OFFSET,
			 lower_32_bits(img->v_addr));
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_VIR_STRIDE_OFFSET,
			 stride);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_SRC_SIZE_OFFSET,
			 src_size);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_ACT_OFF_OFFSET,
			 act_off);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_ACT_SIZE_OFFSET,
			 act_size);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_DST_SIZE_OFFSET,
			 dst_size);
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_SCL_FAC_OFFSET,
			 x_factor | (y_factor << 16));
	rk_rga_cmd_write(job, base + RK_RGA3_WIN0_UV_VIR_STRIDE_OFFSET,
			 uv_stride);

	return 0;
}

static int rk_rga3_emit_win0(struct rk_rga_job *job,
			     const struct rga_req *task,
			     const struct rk_rga3_format_info *src_fmt,
			     const struct rk_rga3_format_info *dst_fmt,
			     u32 rd_mode, u32 rotate_flags)
{
	return rk_rga3_emit_read_window(job, &task->src, src_fmt, dst_fmt,
					rd_mode, rotate_flags,
					task->dst.act_w, task->dst.act_h,
					RK_RGA3_WIN0_RD_CTRL_OFFSET, true,
					task->yuv2rgb_mode, true);
}

static int rk_rga3_emit_wr(struct rk_rga_job *job,
			   const struct rga_req *task,
			   const struct rk_rga3_format_info *dst_fmt,
			   u32 wr_mode, bool apply_dst_offset)
{
	__u64 y_addr = task->dst.yrgb_addr;
	__u64 u_addr = task->dst.uv_addr;
	__u64 v_addr = task->dst.v_addr;
	u32 y_stride_bytes;
	u32 uv_stride_bytes;
	u32 stride;
	u32 uv_stride;
	u32 y_offset;
	u32 reg;
	u8 pix_swap = dst_fmt->pix_swap;
	int ret;

	if (wr_mode == 1) {
		size_t header_words;
		size_t header_size;
		u32 aligned_h = ALIGN((u32)task->dst.vir_h, 16);

		ret = rk_rga_fbc_strides(&task->dst, &stride, &uv_stride);
		if (ret)
			return ret;
		if (check_mul_overflow((size_t)stride, (size_t)aligned_h,
				       &header_words))
			return -EOVERFLOW;
		header_size = header_words >> 2;
		if (check_add_overflow(u_addr, (__u64)header_size, &u_addr))
			return -EOVERFLOW;

		switch (task->dst.format) {
		case RK_RGA_FORMAT_RGBA_8888:
		case RK_RGA_FORMAT_RGBX_8888:
			pix_swap = 0;
			break;
		case RK_RGA_FORMAT_BGRA_8888:
		case RK_RGA_FORMAT_BGRX_8888:
			pix_swap = 1;
			break;
		default:
			break;
		}
	} else {
		if (wr_mode == 2) {
			ret = rk_rga3_tile_stride(task->dst.vir_w,
						  dst_fmt->pixel_width,
						  &stride);
			if (ret)
				return ret;
			uv_stride = dst_fmt->yuv420_sp ?
				    ALIGN((u32)task->dst.vir_w * 8, 16) >> 3 :
				    stride;
		} else {
			ret = rk_rga3_stride(task->dst.vir_w,
					     dst_fmt->pixel_width, &stride);
			if (ret)
				return ret;
			uv_stride = dst_fmt->yuv_sp ?
				    ALIGN((u32)task->dst.vir_w, 16) >> 2 :
				    stride;
		}
		y_stride_bytes = stride << 2;
		uv_stride_bytes = uv_stride << 2;

		if (apply_dst_offset &&
		    (task->dst.x_offset || task->dst.y_offset)) {
			u32 x_offset = task->dst.x_offset;
			u32 x_offset_bytes;
			u32 y_plane_offset;

			if (dst_fmt->yuv_sp && (x_offset & 1))
				return -EINVAL;
			if (dst_fmt->yuv420_sp && (task->dst.y_offset & 1))
				return -EINVAL;

			if (dst_fmt->yuv_sp) {
				if (dst_fmt->yuv10 &&
				    !rk_rga_img_yuv10_compact(&task->dst)) {
					if (check_mul_overflow(x_offset, 2U,
							       &x_offset_bytes))
						return -EOVERFLOW;
					if (check_mul_overflow(y_stride_bytes, 2U,
							       &y_stride_bytes))
						return -EOVERFLOW;
					if (check_mul_overflow(uv_stride_bytes, 2U,
							       &uv_stride_bytes))
						return -EOVERFLOW;
				} else {
					x_offset_bytes = x_offset;
				}
			} else {
				if (check_mul_overflow(x_offset,
						       (u32)dst_fmt->pixel_width,
						       &x_offset_bytes))
					return -EOVERFLOW;
			}

			if (check_mul_overflow((u32)task->dst.y_offset,
					       y_stride_bytes, &y_offset))
				return -EOVERFLOW;
			if (check_add_overflow(y_offset, x_offset_bytes,
					       &y_offset))
				return -EOVERFLOW;

			if (check_add_overflow(y_addr, (__u64)y_offset,
					       &y_addr))
				return -EOVERFLOW;

			if (dst_fmt->yuv_sp) {
				u32 uv_y_offset = dst_fmt->yuv420_sp ?
						  task->dst.y_offset / 2 :
						  task->dst.y_offset;

				if (check_mul_overflow(uv_y_offset,
						       uv_stride_bytes,
						       &y_plane_offset))
					return -EOVERFLOW;
				if (check_add_overflow(y_plane_offset,
						       x_offset_bytes,
						       &y_plane_offset))
					return -EOVERFLOW;
				if (check_add_overflow(u_addr,
						       (__u64)y_plane_offset,
						       &u_addr))
					return -EOVERFLOW;
			}
		}
	}

	reg = FIELD_PREP(RK_RGA3_WR_MODE, wr_mode) |
	      RK_RGA3_WR_FBCE_SPARSE_EN |
	      FIELD_PREP(RK_RGA3_WR_PIC_FORMAT, dst_fmt->pic_format) |
	      FIELD_PREP(RK_RGA3_WR_FORMAT, dst_fmt->bus_format) |
	      FIELD_PREP(RK_RGA3_WR_PIX_SWAP, pix_swap) |
	      FIELD_PREP(RK_RGA3_WR_OUTSTANDING_MAX, 0xf) |
	      FIELD_PREP(RK_RGA3_WR_YC_SWAP, dst_fmt->yc_swap);

	if (dst_fmt->yuv10) {
		if (wr_mode == 1 || rk_rga_img_yuv10_compact(&task->dst))
			reg |= RK_RGA3_WR_YUV10_COMPACT;
		if (wr_mode == 0 && task->dst.is_10b_endian)
			reg |= RK_RGA3_WR_ENDIAN_MODE;
	}

	rk_rga_cmd_write(job, RK_RGA3_WR_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, RK_RGA3_WR_FBCE_CTRL_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA3_WR_VIR_STRIDE_OFFSET, stride);
	rk_rga_cmd_write(job, RK_RGA3_WR_PL_VIR_STRIDE_OFFSET, uv_stride);
	rk_rga_cmd_write(job, RK_RGA3_WR_Y_BASE_OFFSET,
			 lower_32_bits(y_addr));
	rk_rga_cmd_write(job, RK_RGA3_WR_U_BASE_OFFSET,
			 lower_32_bits(u_addr));
	rk_rga_cmd_write(job, RK_RGA3_WR_V_BASE_OFFSET,
			 lower_32_bits(v_addr));

	return 0;
}

static int rk_rga2_stride(const struct rga_img_info_t *img,
			  const struct rk_rga2_format_info *fmt,
			  u32 *stride, u32 *uv_stride)
{
	u32 bytes;

	if (fmt->y4) {
		*stride = (u32)img->vir_w / 2;
		*uv_stride = 0;
		return 0;
	}
	if (check_mul_overflow((u32)img->vir_w, (u32)fmt->pixel_width,
			       &bytes))
		return -EOVERFLOW;

	*stride = ALIGN(bytes, 4);
	*uv_stride = 0;

	if (fmt->plane_width) {
		u32 uv_width;

		if (check_mul_overflow((u32)(img->vir_w / fmt->x_div),
				       (u32)fmt->plane_width, &uv_width))
			return -EOVERFLOW;
		*uv_stride = ALIGN(uv_width, 4);
	}

	return 0;
}

static int rk_rga2_image_offsets(const struct rga_img_info_t *img,
				 const struct rk_rga2_format_info *fmt,
				 u32 *y_offset, u32 *uv_offset)
{
	u32 stride;
	u32 uv_stride;
	u32 y;
	u32 x;
	int ret;

	ret = rk_rga2_stride(img, fmt, &stride, &uv_stride);
	if (ret)
		return ret;

	if (check_mul_overflow((u32)img->y_offset, stride, &y))
		return -EOVERFLOW;
	if (check_mul_overflow((u32)img->x_offset, (u32)fmt->pixel_width,
			       &x))
		return -EOVERFLOW;
	if (fmt->y4)
		x /= 2;
	if (check_add_overflow(y, x, y_offset))
		return -EOVERFLOW;

	*uv_offset = 0;
	if (fmt->plane_width) {
		u32 uv_y;
		u32 uv_x;

		if (check_mul_overflow((u32)(img->y_offset / fmt->y_div),
				       uv_stride, &uv_y))
			return -EOVERFLOW;
		if (check_mul_overflow((u32)(img->x_offset / fmt->x_div),
				       (u32)fmt->plane_width, &uv_x))
			return -EOVERFLOW;
		if (check_add_overflow(uv_y, uv_x, uv_offset))
			return -EOVERFLOW;
	}

	return 0;
}

static int rk_rga2_addr_add_u64(__u64 base, u64 offset, __u64 *addr)
{
	if (check_add_overflow(base, offset, addr))
		return -EOVERFLOW;

	return 0;
}

static int rk_rga2_addr_add_mul(__u64 base, u32 value, u32 scale,
				__u64 *addr)
{
	u64 offset;

	if (check_mul_overflow((u64)value, (u64)scale, &offset))
		return -EOVERFLOW;

	return rk_rga2_addr_add_u64(base, offset, addr);
}

static int rk_rga2_select_dst_addresses(const struct rga_img_info_t *dst,
					const struct rk_rga2_format_info *fmt,
					const struct rk_rga2_transform *transform,
					u32 stride, u32 uv_stride,
					__u64 y_lt, __u64 u_lt, __u64 v_lt,
					__u64 *y_addr, __u64 *u_addr,
					__u64 *v_addr)
{
	__u64 y_ld;
	__u64 y_rt;
	__u64 y_rd;
	__u64 u_ld = 0;
	__u64 u_rt = 0;
	__u64 u_rd = 0;
	__u64 v_ld = 0;
	__u64 v_rt = 0;
	__u64 v_rd = 0;
	u64 offset;
	u32 right;
	u32 row;
	int ret;

	if (fmt->y4) {
		ret = rk_rga2_addr_add_mul(y_lt, dst->act_h - 1, stride,
					   &y_ld);
		if (ret)
			return ret;
		right = dst->act_w / 2 - 1;
		ret = rk_rga2_addr_add_u64(y_lt, right, &y_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(y_ld, right, &y_rd);
		if (ret)
			return ret;
	} else if (fmt->packed_yuv422) {
		ret = rk_rga2_addr_add_mul(y_lt, dst->act_h - 1, stride,
					   &y_ld);
		if (ret)
			return ret;
		if (check_mul_overflow((u32)dst->act_w, 2u, &right))
			return -EOVERFLOW;
		right--;
		ret = rk_rga2_addr_add_u64(y_lt, right, &y_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(y_ld, right, &y_rd);
		if (ret)
			return ret;
	} else if (fmt->packed_yuv420) {
		if (check_add_overflow((u32)dst->y_offset,
				       (u32)dst->act_h - 1, &row))
			return -EOVERFLOW;
		if (check_mul_overflow((u64)row, (u64)stride, &offset))
			return -EOVERFLOW;
		if (check_add_overflow(offset, (u64)dst->x_offset, &offset))
			return -EOVERFLOW;
		ret = rk_rga2_addr_add_u64(dst->yrgb_addr, offset, &y_ld);
		if (ret)
			return ret;
		if (check_mul_overflow((u32)dst->act_w, 2u, &right))
			return -EOVERFLOW;
		right--;
		ret = rk_rga2_addr_add_u64(y_lt, right, &y_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(y_ld, (u32)dst->act_w - 1, &y_rd);
		if (ret)
			return ret;
	} else {
		ret = rk_rga2_addr_add_mul(y_lt, dst->act_h - 1, stride,
					   &y_ld);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_mul(y_lt, dst->act_w - 1,
					   fmt->pixel_width, &y_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_mul(y_ld, dst->act_w - 1,
					   fmt->pixel_width, &y_rd);
		if (ret)
			return ret;
	}

	if (fmt->plane_width) {
		u32 uv_h = dst->act_h / fmt->y_div;
		u32 uv_w = dst->act_w / fmt->x_div;

		ret = rk_rga2_addr_add_mul(u_lt, uv_h - 1, uv_stride,
					   &u_ld);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_mul(v_lt, uv_h - 1, uv_stride,
					   &v_ld);
		if (ret)
			return ret;
		if (check_mul_overflow(uv_w, (u32)fmt->plane_width, &right))
			return -EOVERFLOW;
		right--;
		ret = rk_rga2_addr_add_u64(u_lt, right, &u_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(v_lt, right, &v_rt);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(u_ld, right, &u_rd);
		if (ret)
			return ret;
		ret = rk_rga2_addr_add_u64(v_ld, right, &v_rd);
		if (ret)
			return ret;
	}

	if (!transform->rot_90) {
		if (transform->y_mirror) {
			if (transform->x_mirror) {
				*y_addr = y_rd;
				*u_addr = u_rd;
				*v_addr = v_rd;
			} else {
				*y_addr = y_ld;
				*u_addr = u_ld;
				*v_addr = v_ld;
			}
		} else if (transform->x_mirror) {
			*y_addr = y_rt;
			*u_addr = u_rt;
			*v_addr = v_rt;
		} else {
			*y_addr = y_lt;
			*u_addr = u_lt;
			*v_addr = v_lt;
		}
	} else {
		if (transform->y_mirror) {
			if (transform->x_mirror) {
				*y_addr = y_ld;
				*u_addr = u_ld;
				*v_addr = v_ld;
			} else {
				*y_addr = y_rd;
				*u_addr = u_rd;
				*v_addr = v_rd;
			}
		} else if (transform->x_mirror) {
			*y_addr = y_lt;
			*u_addr = u_lt;
			*v_addr = v_lt;
		} else {
			*y_addr = y_rt;
			*u_addr = u_rt;
			*v_addr = v_rt;
		}
	}

	return 0;
}

static void rk_rga2_scale_down_bilinear_protect(u32 src, u32 dst,
						u32 *factor,
						u32 *active_src)
{
	u32 offset = (1 << RK_RGA2_BILINEAR_PREC) >> 1;
	u32 param = ((u64)src << RK_RGA2_BILINEAR_PREC) / dst;
	u64 final_coor;
	u64 final_limit = (u64)(src - 1) << RK_RGA2_BILINEAR_PREC;
	u32 final_steps;

	for (;;) {
		final_coor = offset + (u64)param * (dst - 1);
		if (final_coor < final_limit)
			break;
		param--;
	}

	final_steps = DIV_ROUND_UP_ULL(final_coor,
				       1 << RK_RGA2_BILINEAR_PREC);
	*factor = param | (offset << 16);
	*active_src = final_steps + 1;
}

static u32 rk_rga2_scale_down_average_factor(u32 src, u32 dst)
{
	u32 param = div_u64((u64)dst << 16, src) + 1;

	while (param && (u64)param * (src - 1) > (u64)dst << 16)
		param--;

	return param;
}

static int rk_rga2_scale_factor(u32 src, u32 dst, u8 interp, u32 *mode,
				u32 *factor, bool *filter, u32 *active_src)
{
	u32 param;

	*filter = false;
	*active_src = src;
	if (src == dst) {
		*mode = RK_RGA2_SCALE_BYPASS;
		*factor = 0;
		return 0;
	}

	if (src > dst) {
		*mode = RK_RGA2_SCALE_DOWN;
		if (interp == RK_RGA2_INTERP_LINEAR) {
			param = ((u64)src << RK_RGA2_BILINEAR_PREC) / dst;
			if (param > 0xffff)
				return -EOPNOTSUPP;
			rk_rga2_scale_down_bilinear_protect(src, dst, factor,
							    active_src);
			*filter = true;
			return 0;
		}

		*factor = rk_rga2_scale_down_average_factor(src, dst);
		return 0;
	}

	*mode = RK_RGA2_SCALE_UP;
	if (dst <= 1)
		return -EINVAL;

	param = ((src - 1) << 16) / (dst - 1);
	*factor = param << 16;
	*filter = interp == RK_RGA2_INTERP_LINEAR;

	return 0;
}

static bool rk_rga2_format_needs_force_tile(u32 format)
{
	switch (format) {
	case RK_RGA_FORMAT_YCBCR_420_SP_10B:
	case RK_RGA_FORMAT_YCRCB_420_SP_10B:
	case RK_RGA_FORMAT_YCBCR_422_SP_10B:
	case RK_RGA_FORMAT_YCRCB_422_SP_10B:
	case RK_RGA_FORMAT_YCBCR_444_SP:
	case RK_RGA_FORMAT_YCRCB_444_SP:
		return true;
	default:
		return false;
	}
}

static bool rk_rga2_needs_force_tile(const struct rga_req *task,
				     const struct rk_rga2_transform *transform,
				     u32 dst_w, u32 dst_h)
{
	if (task->src.act_w != dst_w || task->src.act_h != dst_h)
		return false;
	if (transform->src_rot_mode || transform->src_mir_mode)
		return false;

	return rk_rga2_format_needs_force_tile(task->src.format) ||
	       rk_rga2_format_needs_force_tile(task->dst.format);
}

static int rk_rga2_emit_src(struct rk_rga_job *job,
			    const struct rga_req *task,
			    const struct rk_rga2_format_info *src_fmt,
			    const struct rk_rga2_transform *transform)
{
	u32 stride;
	u32 uv_stride;
	u32 y_offset;
	u32 uv_offset;
	u32 dst_w = transform->rot_90 ? transform->dst_act_h :
		    transform->dst_act_w;
	u32 dst_h = transform->rot_90 ? transform->dst_act_w :
		    transform->dst_act_h;
	u32 h_mode;
	u32 v_mode;
	u32 x_factor;
	u32 y_factor;
	u32 src_info;
	__u64 y_addr;
	u32 u_addr = 0;
	u32 v_addr = 0;
	bool h_filter;
	bool v_filter;
	u32 active_w;
	u32 active_h;
	int ret;

	ret = rk_rga2_stride(&task->src, src_fmt, &stride, &uv_stride);
	if (ret)
		return ret;
	ret = rk_rga2_image_offsets(&task->src, src_fmt, &y_offset,
				    &uv_offset);
	if (ret)
		return ret;
	ret = rk_rga2_scale_factor(task->src.act_w, dst_w,
				   task->interp.horiz, &h_mode, &x_factor,
				   &h_filter, &active_w);
	if (ret)
		return ret;
	ret = rk_rga2_scale_factor(task->src.act_h, dst_h,
				   task->interp.verti, &v_mode, &y_factor,
				   &v_filter, &active_h);
	if (ret)
		return ret;
	if (rk_rga2_needs_force_tile(task, transform, dst_w, dst_h)) {
		h_mode = RK_RGA2_SCALE_FORCE_TILE;
		v_mode = RK_RGA2_SCALE_FORCE_TILE;
		x_factor = 0;
		y_factor = 0;
		h_filter = false;
		v_filter = false;
		active_w = task->src.act_w;
		active_h = task->src.act_h;
	}

	src_info = FIELD_PREP(RK_RGA2_SRC_RB_SWAP, src_fmt->rb_swap) |
		   FIELD_PREP(RK_RGA2_SRC_ALPHA_SWAP, src_fmt->alpha_swap) |
		   FIELD_PREP(RK_RGA2_SRC_UV_SWAP, src_fmt->uv_swap) |
		   FIELD_PREP(RK_RGA2_SRC_CSC_MODE, task->yuv2rgb_mode) |
		   FIELD_PREP(RK_RGA2_SRC_ROT_MODE,
			      transform->src_rot_mode) |
		   FIELD_PREP(RK_RGA2_SRC_MIR_MODE,
			      transform->src_mir_mode) |
		   FIELD_PREP(RK_RGA2_SRC_HSCL_MODE, h_mode) |
		   FIELD_PREP(RK_RGA2_SRC_VSCL_MODE, v_mode);
	if (h_filter) {
		if (h_mode == RK_RGA2_SCALE_UP)
			src_info |= RK_RGA2_SRC_HSP_MODE_SEL;
		else if (h_mode == RK_RGA2_SCALE_DOWN)
			src_info |= RK_RGA2_SRC_HSD_MODE_SEL;
	}
	if (v_filter) {
		if (v_mode == RK_RGA2_SCALE_UP)
			src_info |= RK_RGA2_SRC_VSP_MODE_SEL;
		else if (v_mode == RK_RGA2_SCALE_DOWN)
			src_info |= RK_RGA2_SRC_VSD_MODE_SEL;
	}
	if (src_fmt->yuv10)
		src_info |= RK_RGA2_SRC_YUV10_EN |
			    RK_RGA2_SRC_YUV10_ROUND_EN;
	if (rk_rga2_task_uses_color_key(task))
		src_info |= FIELD_PREP(RK_RGA2_SRC_TRANS_MODE,
				       task->src_trans_mode) |
			    FIELD_PREP(RK_RGA2_SRC_TRANS_ENABLE,
				       task->src_trans_mode >> 1);

	src_info |= FIELD_PREP(RK_RGA2_SRC_FORMAT, src_fmt->hw_format);
	if (check_add_overflow(task->src.yrgb_addr, (__u64)y_offset,
			       &y_addr))
		return -EOVERFLOW;
	if (src_fmt->plane_width) {
		__u64 u64_addr;

		if (check_add_overflow(task->src.uv_addr,
				       (__u64)uv_offset, &u64_addr))
			return -EOVERFLOW;
		u_addr = lower_32_bits(u64_addr);
		if (check_add_overflow(task->src.v_addr,
				       (__u64)uv_offset, &u64_addr))
			return -EOVERFLOW;
		v_addr = lower_32_bits(u64_addr);
	} else if (src_fmt->yuv400) {
		u_addr = lower_32_bits(y_addr);
		v_addr = u_addr;
	}

	rk_rga_cmd_write(job, RK_RGA2_SRC_INFO_OFFSET, src_info);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE0_OFFSET,
			 lower_32_bits(y_addr));
	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE1_OFFSET, u_addr);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE2_OFFSET, v_addr);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE3_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_VIR_INFO_OFFSET, stride >> 2);
	rk_rga_cmd_write(job, RK_RGA2_SRC_ACT_INFO_OFFSET,
			 (active_w - 1) | ((active_h - 1) << 16));
	rk_rga_cmd_write(job, RK_RGA2_SRC_X_FACTOR_OFFSET, x_factor);
	rk_rga_cmd_write(job, RK_RGA2_SRC_Y_FACTOR_OFFSET, y_factor);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BG_COLOR_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_FG_COLOR_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_TR_COLOR0_OFFSET,
			 rk_rga2_task_uses_color_key(task) ?
			 task->color_key_min : 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_TR_COLOR1_OFFSET,
			 rk_rga2_task_uses_color_key(task) ?
			 task->color_key_max : 0);

	return 0;
}

static int rk_rga2_emit_dst(struct rk_rga_job *job,
			    const struct rga_req *task,
			    const struct rk_rga2_format_info *dst_fmt,
			    const struct rk_rga2_format_info *pat_fmt,
			    bool src1_a1555_alpha,
			    const struct rk_rga2_transform *transform)
{
	struct rga_img_info_t dst;
	u32 stride;
	u32 uv_stride;
	u32 y_offset;
	u32 uv_offset;
	u32 dst_info;
	u32 src1_stride = 0;
	u32 src1_uv_stride;
	__u64 y_addr;
	__u64 u_addr = 0;
	__u64 v_addr = 0;
	int ret;

	rk_rga2_normalized_dst(task, transform, &dst);

	ret = rk_rga2_stride(&dst, dst_fmt, &stride, &uv_stride);
	if (ret)
		return ret;
	ret = rk_rga2_image_offsets(&dst, dst_fmt, &y_offset, &uv_offset);
	if (ret)
		return ret;

	if (check_add_overflow(dst.yrgb_addr, (__u64)y_offset, &y_addr))
		return -EOVERFLOW;
	if (dst_fmt->plane_width) {
		if (check_add_overflow(dst.uv_addr, (__u64)uv_offset,
				       &u_addr))
			return -EOVERFLOW;
		if (check_add_overflow(dst.v_addr, (__u64)uv_offset,
				       &v_addr))
			return -EOVERFLOW;
	}
	ret = rk_rga2_select_dst_addresses(&dst, dst_fmt, transform, stride,
					   uv_stride, y_addr, u_addr, v_addr,
					   &y_addr, &u_addr, &v_addr);
	if (ret)
		return ret;

	dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, dst_fmt->hw_format) |
		   FIELD_PREP(RK_RGA2_DST_RB_SWAP, dst_fmt->rb_swap) |
		   FIELD_PREP(RK_RGA2_DST_ALPHA_SWAP, dst_fmt->alpha_swap) |
		   FIELD_PREP(RK_RGA2_DST_UV_SWAP, dst_fmt->uv_swap) |
		   FIELD_PREP(RK_RGA2_DST_CSC_MODE,
			      (task->yuv2rgb_mode >> 2) & 0x3) |
		   FIELD_PREP(RK_RGA2_DST_CSC_CLIP,
			      (task->yuv2rgb_mode >> 4) & 0x1);
	if (task->full_csc.flag & RK_RGA_FULL_CSC_ENABLE)
		dst_info |= RK_RGA2_DST_FULL_CSC_EN;
	if (dst_fmt->yuv400)
		dst_info |= RK_RGA2_DST_YUV400_EN;
	if (dst_fmt->y4)
		dst_info |= RK_RGA2_DST_Y4_EN;
	if (dst_fmt->y4_lut)
		dst_info |=
			FIELD_PREP(RK_RGA2_DST_DITHER_DOWN_EN,
				   task->alpha_rop_flag >> 5) |
			FIELD_PREP(RK_RGA2_DST_DITHER_MODE,
				   task->dither_mode);
	if (rk_rga2_task_uses_quantize(task))
		dst_info |= RK_RGA2_DST_NN_QUANTIZE_EN;
	if (pat_fmt) {
		ret = rk_rga2_stride(&task->pat, pat_fmt, &src1_stride,
				     &src1_uv_stride);
		if (ret)
			return ret;

		dst_info |= FIELD_PREP(RK_RGA2_DST_SRC1_FORMAT,
				       pat_fmt->hw_format) |
			    FIELD_PREP(RK_RGA2_DST_SRC1_RB_SWAP,
				       pat_fmt->rb_swap) |
			    FIELD_PREP(RK_RGA2_DST_SRC1_ALPHA_SWAP,
				       pat_fmt->alpha_swap);
		if (src1_a1555_alpha)
			dst_info |= RK_RGA2_DST_SRC1_A1555_ALPHA_EN;
	}

	rk_rga_cmd_write(job, RK_RGA2_DST_INFO_OFFSET, dst_info);
	rk_rga_cmd_write(job, RK_RGA2_DST_BASE0_OFFSET, lower_32_bits(y_addr));
	if (dst_fmt->planar_420 && !dst_fmt->uv_swap) {
		rk_rga_cmd_write(job, RK_RGA2_DST_BASE1_OFFSET,
				 lower_32_bits(v_addr));
		rk_rga_cmd_write(job, RK_RGA2_DST_BASE2_OFFSET,
				 lower_32_bits(u_addr));
	} else {
		rk_rga_cmd_write(job, RK_RGA2_DST_BASE1_OFFSET,
				 lower_32_bits(u_addr));
		rk_rga_cmd_write(job, RK_RGA2_DST_BASE2_OFFSET,
				 lower_32_bits(v_addr));
	}
	rk_rga_cmd_write(job, RK_RGA2_DST_VIR_INFO_OFFSET,
			 (stride >> 2) | ((src1_stride >> 2) << 16));
	rk_rga_cmd_write(job, RK_RGA2_DST_ACT_INFO_OFFSET,
			 ((u32)dst.act_w - 1) |
			 (((u32)dst.act_h - 1) << 16));
	if (dst_fmt->y4_lut) {
		rk_rga_cmd_write(job, RK_RGA2_DST_Y4MAP_LUT0_OFFSET,
				 (u16)task->gr_color.gr_x_r |
				 ((u32)(u16)task->gr_color.gr_x_g << 16));
		rk_rga_cmd_write(job, RK_RGA2_DST_Y4MAP_LUT1_OFFSET,
				 (u16)task->gr_color.gr_y_r |
				 ((u32)(u16)task->gr_color.gr_y_g << 16));
	}

	return 0;
}

static u32 rk_rga2_pack_nn_quantize(__s16 r, __s16 g, __s16 b)
{
	return ((u32)r & RK_RGA2_NN_QUANTIZE_MASK) |
	       (((u32)g & RK_RGA2_NN_QUANTIZE_MASK) << 10) |
	       (((u32)b & RK_RGA2_NN_QUANTIZE_MASK) << 20);
}

static int rk_rga2_palette_src_stride(const struct rga_req *task,
				      u32 *stride)
{
	u8 shift;
	u32 bytes;
	int ret;

	ret = rk_rga2_palette_shift(task, &shift);
	if (ret)
		return ret;
	bytes = (u32)task->src.vir_w >> shift;
	*stride = ALIGN(bytes, 4);
	if (!*stride)
		return -EINVAL;

	return 0;
}

static int rk_rga2_emit_color_palette(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	struct rk_rga2_palette_profile profile;
	struct rk_rga2_transform transform = {
		.dst_act_w = task->dst.act_w,
		.dst_act_h = task->dst.act_h,
	};
	u32 src_stride;
	u32 src_offset;
	__u64 src_addr;
	u32 src_info;
	int ret;

	ret = rk_rga2_validate_color_palette(task, &profile);
	if (ret)
		return ret;
	ret = rk_rga2_palette_src_stride(task, &src_stride);
	if (ret)
		return ret;
	if (check_mul_overflow((u32)task->src.y_offset, src_stride,
			       &src_offset))
		return -EOVERFLOW;
	if (check_add_overflow(src_offset,
			       (u32)task->src.x_offset >>
			       (3 - task->palette_mode),
			       &src_offset))
		return -EOVERFLOW;
	if (check_add_overflow(task->src.yrgb_addr, (__u64)src_offset,
			       &src_addr))
		return -EOVERFLOW;

	src_info = FIELD_PREP(RK_RGA2_SRC_FORMAT,
			      task->palette_mode | 0xc) |
		   FIELD_PREP(RK_RGA2_SRC_CP_ENDIAN,
			      task->endian_mode & 0x1);

	rk_rga_cmd_write(job, RK_RGA2_MODE_CTRL_OFFSET,
			 FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				    RK_RGA_RENDER_COLOR_PALETTE) |
			 RK_RGA2_MODE_INTR_CF_E);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE0_OFFSET,
			 lower_32_bits(src_addr));
	rk_rga_cmd_write(job, RK_RGA2_SRC_INFO_OFFSET, src_info);
	rk_rga_cmd_write(job, RK_RGA2_SRC_VIR_INFO_OFFSET, src_stride >> 2);
	rk_rga_cmd_write(job, RK_RGA2_SRC_ACT_INFO_OFFSET,
			 ((u32)task->src.act_w - 1) |
			 (((u32)task->src.act_h - 1) << 16));
	rk_rga_cmd_write(job, RK_RGA2_SRC_FG_COLOR_OFFSET, task->fg_color);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BG_COLOR_OFFSET, task->bg_color);

	ret = rk_rga2_emit_dst(job, task, &profile.dst_fmt, NULL, false,
			       &transform);
	if (ret)
		return ret;

	job->cmd_ready = true;

	return 0;
}

static int rk_rga2_emit_update_palette(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	int ret;

	ret = rk_rga2_validate_update_palette(task);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_MODE_CTRL_OFFSET,
			 FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				    RK_RGA2_HW_RENDER_UPDATE_PALETTE) |
			 RK_RGA2_MODE_INTR_CF_E);
	rk_rga_cmd_write(job, RK_RGA2_FADING_CTRL_OFFSET,
			 (u32)task->fading.g << 8);
	rk_rga_cmd_write(job, RK_RGA2_MASK_BASE_OFFSET,
			 lower_32_bits(task->pat.yrgb_addr));

	job->cmd_ready = true;

	return 0;
}

static u32 rk_rga2_alpha_bitmap_ctrl1(void)
{
	u32 color_ctrl;
	u32 alpha_ctrl;

	color_ctrl = FIELD_PREP(GENMASK(0, 0),
				RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
		     FIELD_PREP(GENMASK(1, 1),
				RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
		     FIELD_PREP(GENMASK(4, 2), RK_RGA2_ALPHA_ONE) |
		     FIELD_PREP(GENMASK(7, 5),
				RK_RGA2_ALPHA_OPPOSITE_INVERSE) |
		     FIELD_PREP(GENMASK(11, 10), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(13, 12), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(14, 14), RK_RGA2_ALPHA_STRAIGHT) |
		     FIELD_PREP(GENMASK(15, 15), RK_RGA2_ALPHA_STRAIGHT);
	alpha_ctrl = FIELD_PREP(GENMASK(2, 0), RK_RGA2_ALPHA_ONE) |
		     FIELD_PREP(GENMASK(5, 3),
				RK_RGA2_ALPHA_OPPOSITE_INVERSE) |
		     FIELD_PREP(GENMASK(9, 8), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(11, 10), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(12, 12), RK_RGA2_ALPHA_STRAIGHT) |
		     FIELD_PREP(GENMASK(13, 13), RK_RGA2_ALPHA_STRAIGHT);

	return color_ctrl | (alpha_ctrl << 16);
}

static u32 rk_rga2_osd_alpha_ctrl1(void)
{
	u32 color_ctrl;
	u32 alpha_ctrl;

	color_ctrl = FIELD_PREP(GENMASK(0, 0),
				RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
		     FIELD_PREP(GENMASK(1, 1), RK_RGA2_ALPHA_PRE_MULTIPLIED) |
		     FIELD_PREP(GENMASK(4, 2), RK_RGA2_ALPHA_ONE) |
		     FIELD_PREP(GENMASK(7, 5),
				RK_RGA2_ALPHA_OPPOSITE_INVERSE) |
		     FIELD_PREP(GENMASK(11, 10), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(13, 12), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(14, 14), RK_RGA2_ALPHA_STRAIGHT) |
		     FIELD_PREP(GENMASK(15, 15), RK_RGA2_ALPHA_STRAIGHT);
	alpha_ctrl = FIELD_PREP(GENMASK(2, 0), RK_RGA2_ALPHA_ONE) |
		     FIELD_PREP(GENMASK(5, 3),
				RK_RGA2_ALPHA_OPPOSITE_INVERSE) |
		     FIELD_PREP(GENMASK(9, 8), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(11, 10), RK_RGA2_ALPHA_PER_PIXEL) |
		     FIELD_PREP(GENMASK(12, 12), RK_RGA2_ALPHA_STRAIGHT) |
		     FIELD_PREP(GENMASK(13, 13), RK_RGA2_ALPHA_STRAIGHT);

	return color_ctrl | (alpha_ctrl << 16);
}

static u32
rk_rga2_color_key_alpha_ctrl1(const struct rk_rga2_bitblt_profile *profile)
{
	u32 src_blend = profile->src_fmt.alpha ? RK_RGA2_ALPHA_PER_PIXEL :
			RK_RGA2_ALPHA_GLOBAL;
	u32 dst_blend = profile->dst_fmt.alpha ? RK_RGA2_ALPHA_PER_PIXEL :
			RK_RGA2_ALPHA_GLOBAL;

	return FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_COLOR_M0,
			  RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_COLOR_M0,
			  RK_RGA2_ALPHA_NO_PRE_MULTIPLIED) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_FACTOR_M0,
			  RK_RGA2_ALPHA_ZERO) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M0,
			  RK_RGA2_ALPHA_ONE) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_ALPHA_CAL_M0, 0) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_CAL_M0, 0) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_BLEND_M0, dst_blend) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M0, src_blend) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_ALPHA_M0,
			  RK_RGA2_ALPHA_STRAIGHT) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_M0,
			  RK_RGA2_ALPHA_STRAIGHT) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_FACTOR_M1,
			  RK_RGA2_ALPHA_ZERO) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_FACTOR_M1,
			  RK_RGA2_ALPHA_ONE) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_ALPHA_CAL_M1, 0) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_CAL_M1, 0) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_BLEND_M1, dst_blend) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_BLEND_M1, src_blend) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_DST_ALPHA_M1,
			  RK_RGA2_ALPHA_STRAIGHT) |
	       FIELD_PREP(RK_RGA2_ALPHA_CTRL1_SRC_ALPHA_M1,
			  RK_RGA2_ALPHA_STRAIGHT);
}

static int rk_rga2_emit_src1_base(struct rk_rga_job *job,
				  const struct rga_img_info_t *img,
				  const struct rk_rga2_format_info *fmt)
{
	u32 stride;
	u32 uv_stride;
	u32 y_offset;
	u32 uv_offset;
	__u64 y_addr;
	int ret;

	ret = rk_rga2_stride(img, fmt, &stride, &uv_stride);
	if (ret)
		return ret;
	ret = rk_rga2_image_offsets(img, fmt, &y_offset, &uv_offset);
	if (ret)
		return ret;
	if (check_add_overflow(img->yrgb_addr, (__u64)y_offset, &y_addr))
		return -EOVERFLOW;

	rk_rga_cmd_write(job, RK_RGA2_SRC_BASE3_OFFSET,
			 lower_32_bits(y_addr));

	return 0;
}

static void
rk_rga2_emit_color_key(struct rk_rga_job *job, const struct rga_req *task,
		       const struct rk_rga2_bitblt_profile *profile)
{
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET,
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_0,
				    task->alpha_rop_flag) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_SEL,
				    task->alpha_rop_flag >> 1) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE,
				    task->alpha_rop_mode));
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET,
			 rk_rga2_color_key_alpha_ctrl1(profile));
}

static int rk_rga2_emit_alpha_bitmap(struct rk_rga_job *job,
				     const struct rga_req *task,
				     const struct rk_rga2_format_info *pat_fmt)
{
	int ret;

	ret = rk_rga2_emit_src1_base(job, &task->pat, pat_fmt);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_SRC_BG_COLOR_OFFSET,
			 (u32)task->rgba5551_alpha.alpha0 << 24);
	rk_rga_cmd_write(job, RK_RGA2_SRC_FG_COLOR_OFFSET,
			 (u32)task->rgba5551_alpha.alpha1 << 24);
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET,
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_0,
				    task->alpha_rop_flag) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_SEL,
				    task->alpha_rop_flag >> 1) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE,
				    task->alpha_rop_mode));
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET,
			 rk_rga2_alpha_bitmap_ctrl1());

	return 0;
}

static int rk_rga2_emit_osd(struct rk_rga_job *job,
			    const struct rga_req *task,
			    const struct rk_rga2_format_info *pat_fmt)
{
	const struct rga_osd_mode_ctrl *mode = &task->osd_info.mode_ctrl;
	const struct rga_osd_invert_factor *cal = &task->osd_info.cal_factor;
	u32 fix_width = mode->block_fix_width / 2 - 1;
	u32 ctrl0;
	u32 ctrl1;
	int ret;

	ret = rk_rga2_emit_src1_base(job, &task->pat, pat_fmt);
	if (ret)
		return ret;

	ctrl0 = FIELD_PREP(RK_RGA2_OSD_CTRL0_MODE, mode->mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL0_DIRECTION,
			   mode->direction_mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL0_WIDTH_MODE,
			   mode->width_mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL0_BLOCK_COUNT,
			   mode->block_num - 1) |
		FIELD_PREP(RK_RGA2_OSD_CTRL0_FLAGS_INDEX,
			   mode->flags_index) |
		FIELD_PREP(RK_RGA2_OSD_CTRL0_FIX_WIDTH, fix_width);
	ctrl1 = FIELD_PREP(RK_RGA2_OSD_CTRL1_COLOR_MODE,
			   mode->color_mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_FLAGS_MODE,
			   mode->invert_flags_mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_DEFAULT_COLOR,
			   mode->default_color_sel) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_MODE,
			   mode->invert_mode) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_THRESH,
			   mode->invert_thresh) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_A,
			   mode->invert_enable) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_Y,
			   mode->invert_enable >> 1) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_INVERT_C,
			   mode->invert_enable >> 2) |
		FIELD_PREP(RK_RGA2_OSD_CTRL1_UNFIX_INDEX,
			   mode->unfix_index);

	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET,
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_0,
				    task->alpha_rop_flag) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_SEL,
				    task->alpha_rop_flag >> 1) |
			 FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE,
				    task->alpha_rop_mode));
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET,
			 rk_rga2_osd_alpha_ctrl1());
	rk_rga_cmd_write(job, RK_RGA2_OSD_CTRL0_OFFSET, ctrl0);
	rk_rga_cmd_write(job, RK_RGA2_OSD_CTRL1_OFFSET, ctrl1);
	rk_rga_cmd_write(job, RK_RGA2_OSD_INVERSION_CAL0_OFFSET,
			 ((u32)cal->crb_max << 24) |
			 ((u32)cal->crb_min << 16) |
			 ((u32)cal->yg_max << 8) |
			 cal->yg_min);
	rk_rga_cmd_write(job, RK_RGA2_OSD_INVERSION_CAL1_OFFSET,
			 ((u32)cal->alpha_max << 8) | cal->alpha_min);
	rk_rga_cmd_write(job, RK_RGA2_OSD_LAST_FLAGS0_OFFSET,
			 task->osd_info.last_flags0);
	rk_rga_cmd_write(job, RK_RGA2_OSD_LAST_FLAGS1_OFFSET,
			 task->osd_info.last_flags1);

	if (mode->color_mode) {
		u32 color0 = task->osd_info.bpp2_info.color0.value & 0xffffff;
		u32 color1 = task->osd_info.bpp2_info.color1.value & 0xffffff;

		rk_rga_cmd_write(job, RK_RGA2_OSD_COLOR0_OFFSET, color0);
		rk_rga_cmd_write(job, RK_RGA2_OSD_COLOR1_OFFSET, color1);
	}

	return 0;
}

static int rk_rga2_emit_simple_bitblt(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	struct rk_rga2_bitblt_profile profile;
	u32 quant_offset;
	u32 quant_scale;
	u32 rop_ctrl;
	int ret;

	ret = rk_rga2_validate_bitblt(task, &profile);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_MODE_CTRL_OFFSET,
			 FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				    RK_RGA_RENDER_BITBLT) |
			 FIELD_PREP(RK_RGA2_MODE_BITBLT_MODE,
				    !!task->bsfilter_flag) |
			 RK_RGA2_MODE_INTR_CF_E |
			 FIELD_PREP(RK_RGA2_MODE_OSD_EN, profile.osd) |
			 FIELD_PREP(RK_RGA2_MODE_MOSAIC_EN,
				    !!task->mosaic_info.enable) |
			 FIELD_PREP(RK_RGA2_MODE_SRC_GAUSS_EN,
				    !!task->gauss_config.size));

	ret = rk_rga2_emit_src(job, task, &profile.src_fmt,
			       &profile.transform);
	if (ret)
		return ret;
	ret = rk_rga2_emit_dst(job, task, &profile.dst_fmt,
			       (profile.alpha_bitmap || profile.osd) ?
			       &profile.pat_fmt : NULL,
			       profile.alpha_bitmap, &profile.transform);
	if (ret)
		return ret;

	if (profile.alpha_bitmap) {
		ret = rk_rga2_emit_alpha_bitmap(job, task, &profile.pat_fmt);
		if (ret)
			return ret;
	} else if (profile.osd) {
		ret = rk_rga2_emit_osd(job, task, &profile.pat_fmt);
		if (ret)
			return ret;
	} else if (profile.color_key) {
		rk_rga2_emit_color_key(job, task, &profile);
	} else if (rk_rga2_task_uses_rop(task)) {
		ret = rk_rga2_rop_ctrl(task->rop_code, &rop_ctrl);
		if (ret)
			return ret;
		rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET,
				 FIELD_PREP(RK_RGA2_ALPHA_ROP_0,
					    task->alpha_rop_flag) |
				 FIELD_PREP(RK_RGA2_ALPHA_ROP_SEL,
					    task->alpha_rop_flag >> 1) |
				 FIELD_PREP(RK_RGA2_ALPHA_ROP_MODE,
					    task->alpha_rop_mode) |
				 FIELD_PREP(RK_RGA2_ALPHA_ROP_ENDIAN,
					    (task->endian_mode >> 1) & 0x1));
		rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET, 0);
		rk_rga_cmd_write(job, RK_RGA2_ROP_CTRL0_OFFSET, rop_ctrl);
		rk_rga_cmd_write(job, RK_RGA2_ROP_CTRL1_OFFSET, 0);
	} else {
		u32 src_global = 0;

		if (task->gauss_config.size && task->feature.global_alpha_en)
			src_global = task->fg_global_alpha;

		rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET,
				 FIELD_PREP(RK_RGA2_ALPHA_SRC_GLOBAL,
					    src_global));
		rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET, 0);
	}
	if (task->mosaic_info.enable)
		rk_rga_cmd_write(job, RK_RGA2_MOSAIC_MODE_OFFSET,
				 task->mosaic_info.mode & 0x7);
	if (task->gauss_config.size) {
		if (!job->gauss_coeffs)
			return -EINVAL;
		rk_rga_cmd_write(job, RK_RGA2_GAUSS_COE_OFFSET,
				 job->gauss_coeffs[job->current_task]);
	}
	if (rk_rga2_task_uses_quantize(task)) {
		quant_scale = rk_rga2_pack_nn_quantize(task->gr_color.gr_x_r,
						       task->gr_color.gr_x_g,
						       task->gr_color.gr_x_b);
		quant_offset = rk_rga2_pack_nn_quantize(task->gr_color.gr_y_r,
							task->gr_color.gr_y_g,
							task->gr_color.gr_y_b);
		rk_rga_cmd_write(job, RK_RGA2_DST_QUANTIZE_SCALE_OFFSET,
				 quant_scale);
		rk_rga_cmd_write(job, RK_RGA2_DST_QUANTIZE_OFFSET_OFFSET,
				 quant_offset);
	}

	job->cmd_ready = true;

	return 0;
}

static u32 rk_rga2_pack_gr(__s16 x, __s16 y)
{
	return (u16)x | ((u32)(u16)y << 16);
}

static int rk_rga2_emit_color_fill(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	struct rk_rga2_fill_profile profile;
	struct rk_rga2_transform transform = {
		.dst_act_w = task->dst.act_w,
		.dst_act_h = task->dst.act_h,
	};
	u32 mode;
	int ret;

	ret = rk_rga2_validate_color_fill(task, &profile);
	if (ret)
		return ret;

	mode = FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
			  RK_RGA_RENDER_COLOR_FILL) |
	       RK_RGA2_MODE_INTR_CF_E;

	rk_rga_cmd_write(job, RK_RGA2_MODE_CTRL_OFFSET, mode);
	rk_rga_cmd_write(job, RK_RGA2_SRC_FG_COLOR_OFFSET, task->fg_color);
	rk_rga_cmd_write(job, RK_RGA2_CF_GR_A_OFFSET,
			 rk_rga2_pack_gr(task->gr_color.gr_x_a,
					 task->gr_color.gr_y_a));
	rk_rga_cmd_write(job, RK_RGA2_CF_GR_B_OFFSET,
			 rk_rga2_pack_gr(task->gr_color.gr_x_b,
					 task->gr_color.gr_y_b));
	rk_rga_cmd_write(job, RK_RGA2_CF_GR_G_OFFSET,
			 rk_rga2_pack_gr(task->gr_color.gr_x_g,
					 task->gr_color.gr_y_g));
	rk_rga_cmd_write(job, RK_RGA2_CF_GR_R_OFFSET,
			 rk_rga2_pack_gr(task->gr_color.gr_x_r,
					 task->gr_color.gr_y_r));

	ret = rk_rga2_emit_dst(job, task, &profile.dst_fmt, NULL, false,
			       &transform);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET, 0);

	job->cmd_ready = true;

	return 0;
}

static u32 rk_rga3_alpha_global_ctrl(void)
{
	return FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, 0xff);
}

static u32 rk_rga3_alpha_pass_ctrl(void)
{
	return FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			  RK_RGA3_ALPHA_PER_PIXEL) |
	       FIELD_PREP(RK_RGA3_ALPHA_CAL_MODE,
			  RK_RGA3_ALPHA_NO_SATURATION);
}

static u32 rk_rga3_alpha_blend_mode(bool pixel_alpha, bool global_alpha)
{
	if (!pixel_alpha)
		return RK_RGA3_ALPHA_GLOBAL_BLEND;
	return global_alpha ? RK_RGA3_ALPHA_PER_PIXEL_GLOBAL :
			      RK_RGA3_ALPHA_PER_PIXEL;
}

static u32 rk_rga3_alpha_color_ctrl(bool pixel_alpha, bool global_alpha_en,
				    u8 global_alpha, bool pre_multiplied,
				    u32 factor)
{
	return FIELD_PREP(RK_RGA3_ALPHA_COLOR_MODE,
			  pre_multiplied ? RK_RGA3_ALPHA_PRE_MULTIPLIED :
					   RK_RGA3_ALPHA_NO_PRE_MULTIPLIED) |
	       FIELD_PREP(RK_RGA3_ALPHA_MODE, RK_RGA3_ALPHA_STRAIGHT) |
	       FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			  rk_rga3_alpha_blend_mode(pixel_alpha,
						   global_alpha_en)) |
	       FIELD_PREP(RK_RGA3_ALPHA_CAL_MODE,
			  RK_RGA3_ALPHA_SATURATION) |
	       FIELD_PREP(RK_RGA3_ALPHA_FACTOR, factor) |
	       FIELD_PREP(RK_RGA3_ALPHA_GLOBAL, global_alpha);
}

static u32 rk_rga3_alpha_factor_ctrl(bool pixel_alpha, bool global_alpha_en,
				     u32 factor)
{
	return FIELD_PREP(RK_RGA3_ALPHA_MODE, RK_RGA3_ALPHA_STRAIGHT) |
	       FIELD_PREP(RK_RGA3_ALPHA_BLEND_MODE,
			  rk_rga3_alpha_blend_mode(pixel_alpha,
						   global_alpha_en)) |
	       FIELD_PREP(RK_RGA3_ALPHA_CAL_MODE,
			  RK_RGA3_ALPHA_SATURATION) |
	       FIELD_PREP(RK_RGA3_ALPHA_FACTOR, factor);
}

static int rk_rga3_alpha_factors(u8 pd_mode, u32 *top_factor,
				 u32 *bottom_factor)
{
	switch (pd_mode) {
	case RK_RGA_ALPHA_BLEND_SRC:
		*top_factor = RK_RGA3_ALPHA_ONE;
		*bottom_factor = RK_RGA3_ALPHA_ZERO;
		return 0;
	case RK_RGA_ALPHA_BLEND_DST:
		*top_factor = RK_RGA3_ALPHA_ZERO;
		*bottom_factor = RK_RGA3_ALPHA_ONE;
		return 0;
	case RK_RGA_ALPHA_BLEND_SRC_OVER:
		*top_factor = RK_RGA3_ALPHA_ONE;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		return 0;
	case RK_RGA_ALPHA_BLEND_DST_OVER:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		*bottom_factor = RK_RGA3_ALPHA_ONE;
		return 0;
	case RK_RGA_ALPHA_BLEND_SRC_IN:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE;
		*bottom_factor = RK_RGA3_ALPHA_ZERO;
		return 0;
	case RK_RGA_ALPHA_BLEND_DST_IN:
		*top_factor = RK_RGA3_ALPHA_ZERO;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE;
		return 0;
	case RK_RGA_ALPHA_BLEND_SRC_OUT:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		*bottom_factor = RK_RGA3_ALPHA_ZERO;
		return 0;
	case RK_RGA_ALPHA_BLEND_DST_OUT:
		*top_factor = RK_RGA3_ALPHA_ZERO;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		return 0;
	case RK_RGA_ALPHA_BLEND_SRC_ATOP:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		return 0;
	case RK_RGA_ALPHA_BLEND_DST_ATOP:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE;
		return 0;
	case RK_RGA_ALPHA_BLEND_XOR:
		*top_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		*bottom_factor = RK_RGA3_ALPHA_OPPOSITE_INVERSE;
		return 0;
	case RK_RGA_ALPHA_BLEND_CLEAR:
		*top_factor = RK_RGA3_ALPHA_ZERO;
		*bottom_factor = RK_RGA3_ALPHA_ZERO;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static u32 rk_rga3_color_key_8_to_10(u32 color)
{
	return ((color & 0xff) << 22) |
	       (((color >> 8) & 0xff) << 2) |
	       (((color >> 16) & 0xff) << 12);
}

static int rk_rga3_emit_alpha_overlap(struct rk_rga_job *job,
				      const struct rga_req *task,
				      const struct rk_rga3_bitblt_profile *profile)
{
	bool fg_global_en = false;
	bool bg_global_en = false;
	bool pre_multiplied = !(task->alpha_rop_flag & BIT(9));
	u8 fg_global = 0xff;
	u8 bg_global = 0xff;
	u32 top_factor;
	u32 bottom_factor;
	u32 top_ctrl;
	u32 bottom_ctrl;
	u32 top_alpha;
	u32 bottom_alpha;
	u32 ovlp_off;
	u32 reg;
	int ret;

	ret = rk_rga3_alpha_factors(task->PD_mode, &top_factor,
				    &bottom_factor);
	if (ret)
		return ret;

	if (task->feature.global_alpha_en) {
		if (task->fg_global_alpha < 0xff) {
			fg_global_en = true;
			fg_global = task->fg_global_alpha;
		} else if (!profile->src_fmt.alpha) {
			fg_global_en = true;
		}

		if (task->bg_global_alpha < 0xff) {
			bg_global_en = true;
			bg_global = task->bg_global_alpha;
		} else if (!profile->bg_fmt.alpha) {
			bg_global_en = true;
		}
	}

	top_ctrl = rk_rga3_alpha_color_ctrl(profile->src_fmt.alpha,
					    fg_global_en, fg_global,
					    pre_multiplied, top_factor);
	bottom_ctrl = rk_rga3_alpha_color_ctrl(profile->bg_fmt.alpha,
					       bg_global_en, bg_global,
					       pre_multiplied,
					       bottom_factor);
	top_alpha = rk_rga3_alpha_factor_ctrl(profile->src_fmt.alpha,
					      fg_global_en, top_factor);
	bottom_alpha = rk_rga3_alpha_factor_ctrl(profile->bg_fmt.alpha,
						 bg_global_en,
						 bottom_factor);

	ret = rk_rga3_pack_pair(task->dst.x_offset, task->dst.y_offset,
				&ovlp_off);
	if (ret)
		return ret;

	reg = FIELD_PREP(RK_RGA3_OVLP_MODE, !profile->pattern_blend) |
	      RK_RGA3_OVLP_TOP_ALPHA_EN;
	if (profile->dst_fmt.yuv)
		reg |= RK_RGA3_OVLP_FIELD;
	if (profile->color_key)
		reg |= FIELD_PREP(RK_RGA3_OVLP_TOP_KEY_EN, 1);

	rk_rga_cmd_write(job, RK_RGA3_OVLP_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_OFF_OFFSET, ovlp_off);
	if (profile->color_key) {
		rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_KEY_MIN_OFFSET,
				 rk_rga3_color_key_8_to_10(task->color_key_min));
		rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_KEY_MAX_OFFSET,
				 rk_rga3_color_key_8_to_10(task->color_key_max));
	}
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_CTRL_OFFSET, top_ctrl);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_CTRL_OFFSET, bottom_ctrl);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_ALPHA_OFFSET, top_alpha);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_ALPHA_OFFSET, bottom_alpha);

	return 0;
}

static void rk_rga3_emit_overlap(struct rk_rga_job *job,
				 const struct rk_rga3_format_info *src_fmt,
				 const struct rk_rga3_format_info *dst_fmt)
{
	u32 bottom_ctrl;
	u32 bottom_alpha;
	u32 reg = 0;

	if (dst_fmt->yuv)
		reg |= RK_RGA3_OVLP_FIELD;

	if (src_fmt->alpha && dst_fmt->alpha) {
		bottom_ctrl = 0;
		bottom_alpha = rk_rga3_alpha_pass_ctrl();
	} else {
		bottom_ctrl = rk_rga3_alpha_global_ctrl();
		bottom_alpha = 0;
	}

	rk_rga_cmd_write(job, RK_RGA3_OVLP_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_OFF_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_CTRL_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_CTRL_OFFSET, bottom_ctrl);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_ALPHA_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_ALPHA_OFFSET, bottom_alpha);
}

static int rk_rga3_prepare_bitblt(struct rk_rga_job *job,
				  struct rk_rga3_bitblt_profile *profile)
{
	struct rga_req *task;

	if (!job->tasks)
		return -EINVAL;
	if (!job->task_count || job->current_task >= job->task_count)
		return -EINVAL;
	if (job->import_count < 2)
		return -EOPNOTSUPP;

	task = &job->tasks[job->current_task];

	return rk_rga3_validate_bitblt(task, profile);
}

static int rk_rga3_emit_alpha_bitblt(struct rk_rga_job *job,
				     const struct rga_req *task,
				     const struct rk_rga3_bitblt_profile *profile)
{
	const struct rga_img_info_t *bg;
	struct rga_img_info_t rotated_bg;
	u32 bg_dst_w = task->dst.act_w;
	u32 bg_dst_h = task->dst.act_h;
	int ret;

	bg = profile->pattern_blend ? &task->pat : &task->dst;
	if (!profile->pattern_blend &&
	    (profile->rotate_flags & RK_RGA3_ROT_BIT_ROT_90)) {
		rotated_bg = *bg;
		rotated_bg.act_w = task->dst.act_h;
		rotated_bg.act_h = task->dst.act_w;
		bg = &rotated_bg;
		bg_dst_w = task->dst.act_h;
		bg_dst_h = task->dst.act_w;
	}

	ret = rk_rga3_emit_read_window(job, bg, &profile->bg_fmt,
				       &profile->dst_fmt, profile->bg_mode, 0,
				       bg_dst_w, bg_dst_h,
				       RK_RGA3_WIN0_RD_CTRL_OFFSET, true,
				       task->yuv2rgb_mode, true);
	if (ret)
		return ret;
	ret = rk_rga3_emit_read_window(job, &task->src, &profile->src_fmt,
				       &profile->dst_fmt, profile->src_mode,
				       profile->rotate_flags, task->dst.act_w,
				       task->dst.act_h,
				       RK_RGA3_WIN1_RD_CTRL_OFFSET, false,
				       task->yuv2rgb_mode,
				       !profile->pattern_blend);
	if (ret)
		return ret;
	ret = rk_rga3_emit_alpha_overlap(job, task, profile);
	if (ret)
		return ret;

	return rk_rga3_emit_wr(job, task, &profile->dst_fmt,
			       profile->dst_mode, false);
}

static void rk_rga3_emit_no_blend_overlap(struct rk_rga_job *job,
					  const struct rga_req *task,
					  const struct rk_rga3_bitblt_profile *profile)
{
	u32 top_ctrl;
	u32 top_alpha;
	u32 reg = FIELD_PREP(RK_RGA3_OVLP_MODE, 1);
	u32 ovlp_off = (u32)task->dst.x_offset |
		       ((u32)task->dst.y_offset << 16);

	if (profile->dst_fmt.yuv)
		reg |= RK_RGA3_OVLP_FIELD;

	if (profile->src_fmt.alpha && profile->dst_fmt.alpha) {
		top_ctrl = 0;
		top_alpha = rk_rga3_alpha_pass_ctrl();
	} else {
		top_ctrl = rk_rga3_alpha_global_ctrl();
		top_alpha = 0;
	}

	rk_rga_cmd_write(job, RK_RGA3_OVLP_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_OFF_OFFSET, ovlp_off);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_CTRL_OFFSET, top_ctrl);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_CTRL_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_TOP_ALPHA_OFFSET, top_alpha);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_BOT_ALPHA_OFFSET,
			 rk_rga3_alpha_pass_ctrl());
}

static int rk_rga3_emit_overlap_bitblt(struct rk_rga_job *job,
				       const struct rga_req *task,
				       const struct rk_rga3_bitblt_profile *profile)
{
	int ret;

	ret = rk_rga3_emit_read_window(job, &task->dst, &profile->dst_fmt,
				       &profile->dst_fmt, profile->dst_mode, 0,
				       task->dst.act_w, task->dst.act_h,
				       RK_RGA3_WIN0_RD_CTRL_OFFSET, true,
				       task->yuv2rgb_mode, true);
	if (ret)
		return ret;

	ret = rk_rga3_emit_read_window(job, &task->src, &profile->src_fmt,
				       &profile->dst_fmt, profile->src_mode,
				       profile->rotate_flags, task->dst.act_w,
				       task->dst.act_h,
				       RK_RGA3_WIN1_RD_CTRL_OFFSET, false,
				       task->yuv2rgb_mode, true);
	if (ret)
		return ret;

	rk_rga3_emit_no_blend_overlap(job, task, profile);

	return rk_rga3_emit_wr(job, task, &profile->dst_fmt,
			       profile->dst_mode, false);
}

static int rk_rga3_emit_simple_bitblt(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	struct rk_rga3_bitblt_profile profile;
	int ret;

	ret = rk_rga3_prepare_bitblt(job, &profile);
	if (ret)
		return ret;

	if (profile.alpha_blend) {
		ret = rk_rga3_emit_alpha_bitblt(job, task, &profile);
		if (ret)
			return ret;
	} else if (profile.overlap_copy) {
		ret = rk_rga3_emit_overlap_bitblt(job, task, &profile);
		if (ret)
			return ret;
	} else {
		ret = rk_rga3_emit_win0(job, task, &profile.src_fmt,
					&profile.dst_fmt, profile.src_mode,
					profile.rotate_flags);
		if (ret)
			return ret;
		rk_rga3_emit_overlap(job, &profile.src_fmt,
				     &profile.dst_fmt);
		ret = rk_rga3_emit_wr(job, task, &profile.dst_fmt,
				      profile.dst_mode, true);
		if (ret)
			return ret;
	}

	job->cmd_ready = true;

	return 0;
}

static int rk_rga_job_emit_cmd(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	memset(job->cmd_vaddr, 0, job->cmd_size);
	job->cmd_ready = false;

	if (hw->type == RK_RGA_HW_RGA3)
		return rk_rga3_emit_simple_bitblt(job);
	if (hw->type == RK_RGA_HW_RGA2) {
		struct rga_req *task = &job->tasks[job->current_task];

		if (task->render_mode == RK_RGA_RENDER_BITBLT)
			return rk_rga2_emit_simple_bitblt(job);
		if (task->render_mode == RK_RGA_RENDER_COLOR_FILL)
			return rk_rga2_emit_color_fill(job);
		if (task->render_mode == RK_RGA_RENDER_COLOR_PALETTE)
			return rk_rga2_emit_color_palette(job);
		if (task->render_mode == RK_RGA_RENDER_UPDATE_PALETTE)
			return rk_rga2_emit_update_palette(job);
	}

	return -EOPNOTSUPP;
}

static int rk_rga_job_prepare_hw_mappings(struct rk_rga_hw *hw,
					  struct rk_rga_job *job)
{
	struct rga_req *task;
	int ret;

	if (!job->tasks || job->current_task >= job->task_count)
		return -EINVAL;

	task = &job->tasks[job->current_task];

	if (task->render_mode == RK_RGA_RENDER_BITBLT ||
	    task->render_mode == RK_RGA_RENDER_COLOR_PALETTE) {
		ret = rk_rga_job_rebase_img_to_hw(job, &task->src, hw->dev);
		if (ret)
			return ret;
		ret = rk_rga_job_rebase_img_to_hw(job, &task->dst, hw->dev);
		if (ret)
			return ret;
		if (rk_rga_img_has_addr(&task->pat))
			return rk_rga_job_rebase_img_to_hw(job, &task->pat,
							  hw->dev);
		return 0;
	}

	if (task->render_mode == RK_RGA_RENDER_COLOR_FILL)
		return rk_rga_job_rebase_img_to_hw(job, &task->dst, hw->dev);
	if (task->render_mode == RK_RGA_RENDER_UPDATE_PALETTE)
		return rk_rga_job_rebase_img_to_hw(job, &task->pat, hw->dev);

	return -EOPNOTSUPP;
}

static int rk_rga_task_core_valid(const struct rga_req *task)
{
	if (task->core & ~RK_RGA_CORE_MASK)
		return -EINVAL;

	return 0;
}

static bool rk_rga_task_core_allows_type(const struct rga_req *task,
					 enum rk_rga_hw_type type)
{
	u32 type_mask;

	if (!task->core)
		return true;

	if (type == RK_RGA_HW_RGA3)
		type_mask = RK_RGA_CORE_RGA3_MASK;
	else
		type_mask = RK_RGA_CORE_RGA2_MASK;

	return task->core & type_mask;
}

static bool rk_rga_job_current_task_allows_hw(struct rk_rga_job *job,
					      const struct rk_rga_hw *hw)
{
	struct rga_req *task;

	if (!job->tasks || job->current_task >= job->task_count)
		return false;

	task = &job->tasks[job->current_task];
	if (task->core && !(task->core & hw->core_mask))
		return false;

	return true;
}

static u32 rk_rga_hw_load(struct rk_rga_hw *hw)
{
	unsigned long flags;
	u32 load;

	spin_lock_irqsave(&hw->job_lock, flags);
	load = hw->queued_jobs;
	if (hw->active_job && load < U32_MAX)
		load++;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	return load;
}

static bool rk_rga_hw_tie_better(const struct rk_rga_hw *best,
				 const struct rk_rga_hw *hw, u32 rr_start)
{
	if (!best)
		return true;
	if (hw->type != best->type)
		return false;

	return rk_rga_core_distance(hw->core_mask, rr_start) <
	       rk_rga_core_distance(best->core_mask, rr_start);
}

static struct rk_rga_hw *
rk_rga_find_best_hw_for_job(struct list_head *hw_list, struct rk_rga_job *job,
			    u32 type_mask, u32 rr_start)
{
	struct rk_rga_hw *best = NULL;
	struct rk_rga_hw *hw;
	u32 best_load = U32_MAX;

	if (!type_mask)
		return NULL;

	list_for_each_entry(hw, hw_list, node) {
		u32 load;

		if (!rk_rga_hw_accepting_jobs(hw) ||
		    !(type_mask & BIT(hw->type)))
			continue;
		if (!rk_rga_job_current_task_allows_hw(job, hw))
			continue;

		load = rk_rga_hw_load(hw);
		if (best) {
			if (load > best_load)
				continue;
			if (load == best_load &&
			    !rk_rga_hw_tie_better(best, hw, rr_start))
				continue;
		}

		best = hw;
		best_load = load;
	}

	return best;
}

static u32 rk_rga_find_free_core_mask(struct list_head *hw_list,
				      enum rk_rga_hw_type type)
{
	struct rk_rga_hw *hw;
	u32 free_mask;

	if (type == RK_RGA_HW_RGA3)
		free_mask = RK_RGA_CORE_RGA3_MASK;
	else if (type == RK_RGA_HW_RGA2)
		free_mask = RK_RGA_CORE_RGA2_MASK;
	else
		return 0;

	list_for_each_entry(hw, hw_list, node) {
		if (hw->type == type)
			free_mask &= ~hw->core_mask;
	}
	if (!free_mask)
		return 0;

	return BIT(__ffs(free_mask));
}

static int rk_rga_select_default_hw_type(u32 type_mask,
					 enum rk_rga_hw_type *type)
{
	if (type_mask & RK_RGA_HW_TYPE_MASK_RGA3) {
		*type = RK_RGA_HW_RGA3;
		return 0;
	}
	if (type_mask & RK_RGA_HW_TYPE_MASK_RGA2) {
		*type = RK_RGA_HW_RGA2;
		return 0;
	}

	return -EOPNOTSUPP;
}

static int rk_rga_task_hw_type_mask(struct rk_rga_job *job, u32 task_index,
				    u32 *type_mask)
{
	struct rk_rga3_bitblt_profile profile;
	struct rk_rga2_bitblt_profile rga2_profile;
	struct rk_rga2_fill_profile fill_profile;
	struct rk_rga2_palette_profile palette_profile;
	struct rga_req *task;
	int ret;

	if (!job->task_count || !job->tasks || task_index >= job->task_count)
		return -EINVAL;

	*type_mask = 0;
	task = &job->tasks[task_index];

	switch (task->render_mode) {
	case RK_RGA_RENDER_BITBLT:
	{
		int rga3_ret;
		int rga2_ret;
		bool allow_rga3;
		bool allow_rga2;

		if (job->import_count < 2)
			return -EOPNOTSUPP;
		ret = rk_rga_task_core_valid(task);
		if (ret)
			return ret;
		allow_rga3 = rk_rga_task_core_allows_type(task,
							  RK_RGA_HW_RGA3);
		allow_rga2 = rk_rga_task_core_allows_type(task,
							  RK_RGA_HW_RGA2);

		rga3_ret = -EOPNOTSUPP;
		if (allow_rga3) {
			rga3_ret = rk_rga3_validate_bitblt(task, &profile);
			if (!rga3_ret)
				*type_mask |= RK_RGA_HW_TYPE_MASK_RGA3;
		}

		rga2_ret = -EOPNOTSUPP;
		if (allow_rga2) {
			rga2_ret = rk_rga2_validate_bitblt(task,
							   &rga2_profile);
			if (!rga2_ret)
				*type_mask |= RK_RGA_HW_TYPE_MASK_RGA2;
		}

		if (*type_mask)
			return 0;

		if (rga3_ret != -EOPNOTSUPP)
			return rga3_ret;
		if (rga2_ret != -EOPNOTSUPP)
			return rga2_ret;
		return -EOPNOTSUPP;
	}
	case RK_RGA_RENDER_COLOR_FILL:
		if (!job->import_count)
			return -EOPNOTSUPP;
		ret = rk_rga_task_core_valid(task);
		if (ret)
			return ret;
		if (!rk_rga_task_core_allows_type(task, RK_RGA_HW_RGA2))
			return -EOPNOTSUPP;
		ret = rk_rga2_validate_color_fill(task, &fill_profile);
		if (ret)
			return ret;
		*type_mask = RK_RGA_HW_TYPE_MASK_RGA2;
		return 0;
	case RK_RGA_RENDER_COLOR_PALETTE:
		if (job->import_count < 2)
			return -EOPNOTSUPP;
		ret = rk_rga_task_core_valid(task);
		if (ret)
			return ret;
		if (!rk_rga_task_core_allows_type(task, RK_RGA_HW_RGA2))
			return -EOPNOTSUPP;
		ret = rk_rga2_validate_color_palette(task, &palette_profile);
		if (ret)
			return ret;
		*type_mask = RK_RGA_HW_TYPE_MASK_RGA2;
		return 0;
	case RK_RGA_RENDER_UPDATE_PALETTE:
		if (!job->import_count)
			return -EOPNOTSUPP;
		ret = rk_rga_task_core_valid(task);
		if (ret)
			return ret;
		if (!rk_rga_task_core_allows_type(task, RK_RGA_HW_RGA2))
			return -EOPNOTSUPP;
		ret = rk_rga2_validate_update_palette(task);
		if (ret)
			return ret;
		*type_mask = RK_RGA_HW_TYPE_MASK_RGA2;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int __maybe_unused rk_rga_task_hw_type(struct rk_rga_job *job,
					      u32 task_index,
					      enum rk_rga_hw_type *type)
{
	u32 type_mask;
	int ret;

	ret = rk_rga_task_hw_type_mask(job, task_index, &type_mask);
	if (ret)
		return ret;

	return rk_rga_select_default_hw_type(type_mask, type);
}

static int rk_rga_job_validate_tasks(struct rk_rga_job *job)
{
	u32 type_mask;
	int ret;

	if (!job->task_count || !job->tasks)
		return -EINVAL;

	for (u32 i = 0; i < job->task_count; i++) {
		ret = rk_rga_task_hw_type_mask(job, i, &type_mask);
		if (ret)
			return ret;
	}

	return 0;
}

static int rk_rga_job_hw_type_mask(struct rk_rga_job *job, u32 *type_mask)
{
	int ret;

	ret = rk_rga_job_validate_tasks(job);
	if (ret)
		return ret;

	return rk_rga_task_hw_type_mask(job, job->current_task, type_mask);
}

static int __maybe_unused rk_rga_job_hw_type(struct rk_rga_job *job,
					     enum rk_rga_hw_type *type)
{
	u32 type_mask;
	int ret;

	ret = rk_rga_job_hw_type_mask(job, &type_mask);
	if (ret)
		return ret;

	return rk_rga_select_default_hw_type(type_mask, type);
}

static struct rk_rga_hw *rk_rga_hw_get_for_job(struct rk_rga_job *job,
					       int *error)
{
	u32 type_mask;
	u32 rr_start;
	struct rk_rga_hw *hw;
	int ret;

	ret = rk_rga_job_hw_type_mask(job, &type_mask);
	if (ret) {
		*error = ret;
		return NULL;
	}

	mutex_lock(&rk_rga.hw_lock);
	rr_start = rk_rga.core_select_seq;
	hw = rk_rga_find_best_hw_for_job(&rk_rga.hw_list, job, type_mask,
					 rr_start);
	if (hw) {
		refcount_inc(&hw->refs);
		rk_rga.core_select_seq =
			rk_rga_core_select_next(hw, rr_start);
		mutex_unlock(&rk_rga.hw_lock);
		*error = 0;

		return hw;
	}
	mutex_unlock(&rk_rga.hw_lock);

	*error = -ENODEV;

	return NULL;
}

static int rk_rga_backend_start(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	int ret;

	ret = rk_rga_job_prepare_hw_mappings(hw, job);
	if (ret)
		return ret;

	rk_rga_job_sync_userptr_for_device(job, hw->dev);

	ret = rk_rga_hw_power_on(hw);
	if (ret)
		return ret;

	ret = rk_rga_job_alloc_cmd(job, hw);
	if (ret) {
		rk_rga_hw_power_off(hw);
		return ret;
	}

	ret = rk_rga_job_emit_cmd(hw, job);
	if (ret) {
		atomic_inc(&rk_rga.unsupported_count);
		rk_rga_hw_power_off(hw);
		return ret;
	}
	if (!job->cmd_ready) {
		atomic_inc(&rk_rga.unsupported_count);
		rk_rga_hw_power_off(hw);
		return -EOPNOTSUPP;
	}

	rk_rga_hw_start(hw, job);

	return RK_RGA_BACKEND_QUEUED;
}

static irqreturn_t rk_rga_irq_handler(int irq, void *data)
{
	struct rk_rga_hw *hw = data;
	struct rk_rga_job *job;
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;

	/* Teardown may briefly balance a quarantined IRQ before freeing it. */
	if (unlikely(READ_ONCE(hw->recovery_failed)))
		return IRQ_HANDLED;

	spin_lock_irqsave(&hw->job_lock, flags);
	job = hw->active_job;
	if (job)
		ret = rk_rga_hw_irq_status(hw, job);
	else
		ret = rk_rga_hw_clear_spurious_irq(hw);
	spin_unlock_irqrestore(&hw->job_lock, flags);

	if (ret == IRQ_WAKE_THREAD)
		atomic_inc(&rk_rga.irq_count);

	return ret;
}

static irqreturn_t rk_rga_irq_thread(int irq, void *data)
{
	struct rk_rga_hw *hw = data;
	struct rk_rga_job *job;
	struct rk_rga_session *dispatch_session = NULL;
	int reset_ret = 0;
	int result;
	bool requeued = false;

	atomic_inc(&rk_rga.irq_thread_count);
	mutex_lock(&hw->run_lock);
	job = rk_rga_hw_take_active(hw);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return IRQ_HANDLED;
	}

	rk_rga_hw_cancel_timeout(hw);
	result = job->irq_result;
	rk_rga_job_note_hw_done(job);
	if (result) {
		reset_ret = rk_rga_hw_reset_for_recovery(hw);
		/* Prevent a failed reset from retriggering on powered-off MMIO. */
		if (reset_ret)
			rk_rga_hw_quarantine_irq(hw);
	}
	rk_rga_hw_power_off(hw);

	if (rk_rga_job_advance_task(job, result)) {
		dispatch_session = READ_ONCE(job->session);
		if (dispatch_session &&
		    rk_rga_session_begin_job_dispatch(dispatch_session)) {
			rk_rga_job_release_hw(job);
			requeued = true;
		} else {
			result = -EFAULT;
		}
	}

	mutex_unlock(&hw->run_lock);

	if (requeued) {
		rk_rga_job_queue_ref(job, false);
		rk_rga_session_end_job_dispatch(dispatch_session);
		rk_rga_hw_dispatch(hw);
		return IRQ_HANDLED;
	}

	rk_rga_job_complete_queued(job, result);
	rk_rga_hw_dispatch(hw);
	if (reset_ret)
		rk_rga_abort_pending_if_no_hw(-EIO);

	return IRQ_HANDLED;
}

static struct rk_rga_job *rk_rga_hw_take_active(struct rk_rga_hw *hw)
{
	struct rk_rga_job *job;
	unsigned long flags;

	spin_lock_irqsave(&hw->job_lock, flags);
	job = hw->active_job;
	hw->active_job = NULL;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	return job;
}

static bool rk_rga_hw_mark_iommu_fault(struct rk_rga_hw *hw)
{
	unsigned long flags;
	bool marked = false;

	spin_lock_irqsave(&hw->job_lock, flags);
	if (hw->active_job && hw->active_generation) {
		hw->iommu_fault_generation = hw->active_generation;
		marked = true;
	}
	spin_unlock_irqrestore(&hw->job_lock, flags);

	return marked;
}

static bool rk_rga_hw_iommu_fault_matches_locked(struct rk_rga_hw *hw)
{
	u64 generation;

	generation = hw->iommu_fault_generation;
	hw->iommu_fault_generation = 0;

	return generation && hw->active_job &&
	       generation == hw->active_generation;
}

static void rk_rga_hw_recover_active(struct rk_rga_hw *hw, bool iommu_fault,
				     struct rk_rga_job *timeout_job)
{
	struct rk_rga_job *job;
	unsigned long flags;
	bool irq_disabled;
	bool recover;
	int reset_ret;
	int result;

	irq_disabled = rk_rga_hw_disable_irq(hw);
	mutex_lock(&hw->run_lock);
	spin_lock_irqsave(&hw->job_lock, flags);
	job = hw->active_job;
	if (iommu_fault)
		recover = rk_rga_hw_iommu_fault_matches_locked(hw);
	else
		recover = job && job == timeout_job && !job->irq_seen;
	if (!recover) {
		spin_unlock_irqrestore(&hw->job_lock, flags);
		rk_rga_hw_enable_irq(hw, irq_disabled);
		mutex_unlock(&hw->run_lock);
		return;
	}

	if (hw->type == RK_RGA_HW_RGA3)
		rk_rga3_read_irq_status(hw, job);
	else
		rk_rga2_read_irq_status(hw, job);

	hw->active_job = NULL;
	spin_unlock_irqrestore(&hw->job_lock, flags);
	if (iommu_fault)
		rk_rga_hw_cancel_timeout(hw);

	if (iommu_fault) {
		result = -EIO;
		dev_err(hw->dev, "job failed on IOMMU fault\n");
	} else {
		result = -EBUSY;
		atomic_inc(&rk_rga.timeout_count);
	}

	rk_rga_job_note_hw_done(job);
	reset_ret = rk_rga_hw_reset_for_recovery(hw);
	rk_rga_hw_power_off(hw);
	rk_rga_job_complete_queued(job, result);
	rk_rga_hw_enable_irq(hw, irq_disabled);
	mutex_unlock(&hw->run_lock);
	rk_rga_hw_dispatch(hw);
	if (reset_ret)
		rk_rga_abort_pending_if_no_hw(-EIO);
}

static void rk_rga_hw_timeout_work(struct work_struct *work)
{
	struct delayed_work *delayed = to_delayed_work(work);
	struct rk_rga_hw *hw = container_of(delayed, struct rk_rga_hw,
					    timeout_work);
	struct rk_rga_job *job;

	job = rk_rga_hw_take_timeout_job(hw);
	if (!job)
		return;
	rk_rga_hw_recover_active(hw, false, job);
	rk_rga_job_put(job);
}

static void rk_rga_hw_iommu_fault_work(struct work_struct *work)
{
	struct rk_rga_hw *hw =
		container_of(work, struct rk_rga_hw, iommu_fault_work);

	rk_rga_hw_recover_active(hw, true, NULL);
}

static struct rk_rga_hw *
rk_rga_iommu_find_fault_hw(struct list_head *fault_hws,
			   struct iommu_domain *domain,
			   struct device *iommu_dev)
{
	struct rk_rga_hw *fallback = NULL;
	struct rk_rga_hw *match = NULL;
	struct rk_rga_hw *hw;

	list_for_each_entry(hw, fault_hws, fault_node) {
		if (hw->iommu_domain != domain)
			continue;

		if (!fallback)
			fallback = hw;
		if (iommu_dev &&
		    (hw->dev == iommu_dev ||
		     (iommu_dev->of_node &&
		      hw->iommu_node == iommu_dev->of_node))) {
			match = hw;
			break;
		}
	}

	/* A reported source must match exactly within a shared domain. */
	return iommu_dev ? match : fallback;
}

static int rk_rga_iommu_fault_handler(struct iommu_domain *domain,
				      struct device *iommu_dev,
				      unsigned long iova, int status,
				      void *arg)
{
	struct rk_rga_service *rga = arg;
	struct rk_rga_hw *match = NULL;
	unsigned long flags;

	atomic_inc(&rga->iommu_fault_count);

	spin_lock_irqsave(&rga->fault_lock, flags);
	match = rk_rga_iommu_find_fault_hw(&rga->fault_hws, domain,
					   iommu_dev);
	if (match) {
		if (rk_rga_hw_mark_iommu_fault(match))
			schedule_work(&match->iommu_fault_work);
		dev_err_ratelimited(match->dev,
				    "IOMMU fault iova %#lx status %#x\n",
				    iova, status);
	}
	spin_unlock_irqrestore(&rga->fault_lock, flags);

	if (!match) {
		pr_err_ratelimited("unmatched RGA IOMMU fault iova %#lx status %#x\n",
				   iova, status);
		return -ENODEV;
	}

	return 0;
}

static int rk_rga_iommu_register_fault_handler(struct rk_rga_hw *hw)
{
	unsigned long flags;
	int ret;

	hw->iommu_domain = iommu_get_domain_for_dev(hw->dev);
	if (!hw->iommu_domain)
		return 0;

	ret = rockchip_iommu_set_fault_handler(hw->dev,
					       rk_rga_iommu_fault_handler,
					       &rk_rga);
	if (ret)
		return dev_err_probe(hw->dev, ret,
				     "failed to register IOMMU fault handler\n");
	hw->iommu_fault_handler_registered = true;

	spin_lock_irqsave(&rk_rga.fault_lock, flags);
	list_add_tail(&hw->fault_node, &rk_rga.fault_hws);
	spin_unlock_irqrestore(&rk_rga.fault_lock, flags);

	return 0;
}

static void rk_rga_iommu_unregister_fault_handler(struct rk_rga_hw *hw)
{
	unsigned long flags;
	int ret;

	if (!hw->iommu_fault_handler_registered)
		return;

	spin_lock_irqsave(&rk_rga.fault_lock, flags);
	if (!list_empty(&hw->fault_node))
		list_del_init(&hw->fault_node);
	spin_unlock_irqrestore(&rk_rga.fault_lock, flags);

	/* Provider callbacks are per IOMMU, even when the DMA domain is shared. */
	ret = rockchip_iommu_set_fault_handler(hw->dev, NULL, NULL);
	if (ret)
		dev_warn(hw->dev, "failed to clear IOMMU fault handler: %pe\n",
			 ERR_PTR(ret));
	hw->iommu_fault_handler_registered = false;
}

static void rk_rga_hw_dispatch(struct rk_rga_hw *hw)
{
	for (;;) {
		struct rk_rga_job *job;
		unsigned long flags;
		bool recovery_failed;
		int ret;

		mutex_lock(&hw->run_lock);
		spin_lock_irqsave(&hw->job_lock, flags);
		if (hw->removing || hw->active_job ||
		    list_empty(&hw->job_queue)) {
			spin_unlock_irqrestore(&hw->job_lock, flags);
			mutex_unlock(&hw->run_lock);
			return;
		}

		job = list_first_entry(&hw->job_queue, struct rk_rga_job,
				       node);
		list_del_init(&job->node);
		job->queued = false;
		hw->queued_jobs--;
		recovery_failed = hw->recovery_failed;
		if (!recovery_failed) {
			hw->active_job = job;
			hw->active_generation++;
			if (!hw->active_generation)
				hw->active_generation++;
			hw->iommu_fault_generation = 0;
		}
		spin_unlock_irqrestore(&hw->job_lock, flags);
		if (recovery_failed) {
			rk_rga_job_complete_queued(job, -EIO);
			mutex_unlock(&hw->run_lock);
			continue;
		}

		atomic_inc(&rk_rga.dispatched_job_count);
		rk_rga_count_core(rk_rga.dispatched_core_count, hw);
		ret = rk_rga_backend_start(hw, job);
		if (ret == RK_RGA_BACKEND_QUEUED) {
			mutex_unlock(&hw->run_lock);
			return;
		}

		spin_lock_irqsave(&hw->job_lock, flags);
		if (hw->active_job == job)
			hw->active_job = NULL;
		spin_unlock_irqrestore(&hw->job_lock, flags);

		rk_rga_job_complete_queued(job, ret);
		mutex_unlock(&hw->run_lock);
	}
}

static void rk_rga_hw_age_jobs_after(struct rk_rga_hw *hw,
				     struct rk_rga_job *job)
{
	struct list_head *entry;

	for (entry = job->node.next; entry != &hw->job_queue;
	     entry = entry->next) {
		struct rk_rga_job *pos =
			list_entry(entry, struct rk_rga_job, node);

		if (pos->priority < RK_RGA_SCHED_PRIORITY_MAX)
			pos->priority++;
	}
}

static void rk_rga_hw_enqueue_job_locked(struct rk_rga_hw *hw,
					 struct rk_rga_job *job)
{
	struct rk_rga_job *pos;

	job->queued = true;
	if (job->priority) {
		list_for_each_entry(pos, &hw->job_queue, node) {
			if (job->priority > pos->priority) {
				list_add_tail(&job->node, &pos->node);
				rk_rga_hw_age_jobs_after(hw, job);
				goto queued;
			}
		}
	}

	list_add_tail(&job->node, &hw->job_queue);

queued:
	hw->queued_jobs++;
}

static int rk_rga_job_queue_ref(struct rk_rga_job *job, bool take_ref)
{
	struct rk_rga_hw *hw;
	int ret;

	hw = rk_rga_hw_get_for_job(job, &ret);
	if (!hw) {
		if (ret == -EOPNOTSUPP)
			atomic_inc(&rk_rga.unsupported_count);
		rk_rga_job_complete(job, ret);
		if (!take_ref)
			rk_rga_job_put(job);
		return ret;
	}

	return rk_rga_job_queue_on_hw(job, hw, take_ref);
}

static int rk_rga_job_queue_on_hw(struct rk_rga_job *job, struct rk_rga_hw *hw,
				  bool take_ref)
{
	unsigned long flags;
	int ret;

	job->hw = hw;
	if (take_ref)
		rk_rga_job_get(job);

	spin_lock_irqsave(&hw->job_lock, flags);
	if (hw->removing || hw->recovery_failed) {
		ret = hw->removing ? -ENODEV : -EIO;
		spin_unlock_irqrestore(&hw->job_lock, flags);
		rk_rga_job_complete_queued(job, ret);
		return ret;
	}

	rk_rga_hw_enqueue_job_locked(hw, job);
	atomic_inc(&rk_rga.scheduled_job_count);
	rk_rga_count_core(rk_rga.scheduled_core_count, hw);
	spin_unlock_irqrestore(&hw->job_lock, flags);

	rk_rga_hw_dispatch(hw);

	return 0;
}

static int rk_rga_job_queue(struct rk_rga_job *job)
{
	return rk_rga_job_queue_ref(job, true);
}

static int rk_rga_job_queue_and_wait(struct rk_rga_job *job)
{
	int ret;

	ret = rk_rga_job_queue(job);
	if (ret)
		return ret;

	/* Pairs with completion's release store before reading the result. */
	wait_event(job->wait, smp_load_acquire(&job->done));

	ret = READ_ONCE(job->result);

	return ret;
}

static void rk_rga_hw_abort_jobs(struct rk_rga_hw *hw, int result)
{
	struct rk_rga_job *job, *tmp;
	struct rk_rga_job *active;
	unsigned long flags;
	bool irq_disabled;
	LIST_HEAD(aborted);

	rk_rga_hw_cancel_timeout_sync(hw);
	irq_disabled = rk_rga_hw_disable_irq(hw);

	mutex_lock(&hw->run_lock);
	spin_lock_irqsave(&hw->job_lock, flags);
	active = hw->active_job;
	hw->active_job = NULL;
	list_for_each_entry_safe(job, tmp, &hw->job_queue, node) {
		list_del_init(&job->node);
		job->queued = false;
		list_add_tail(&job->node, &aborted);
	}
	hw->queued_jobs = 0;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	if (active) {
		rk_rga_job_note_hw_done(active);
		(void)rk_rga_hw_reset_for_recovery(hw);
		rk_rga_hw_power_off(hw);
	}
	rk_rga_hw_enable_irq(hw, irq_disabled);
	mutex_unlock(&hw->run_lock);
	rk_rga_hw_cancel_timeout_sync(hw);

	if (active)
		rk_rga_job_complete_queued(active, result);

	list_for_each_entry_safe(job, tmp, &aborted, node) {
		list_del_init(&job->node);
		rk_rga_job_complete_queued(job, result);
	}
}

static bool rk_rga_hw_abort_session_jobs(struct rk_rga_hw *hw,
					 struct rk_rga_session *session,
					 int result)
{
	struct rk_rga_job *job, *tmp;
	struct rk_rga_job *active = NULL;
	unsigned long flags;
	bool dispatch = false;
	bool irq_disabled;
	LIST_HEAD(aborted);

	irq_disabled = rk_rga_hw_disable_irq(hw);
	mutex_lock(&hw->run_lock);
	spin_lock_irqsave(&hw->job_lock, flags);
	if (hw->active_job && hw->active_job->session == session) {
		active = hw->active_job;
		hw->active_job = NULL;
	}

	list_for_each_entry_safe(job, tmp, &hw->job_queue, node) {
		if (job->session != session)
			continue;

		list_del_init(&job->node);
		job->queued = false;
		hw->queued_jobs--;
		list_add_tail(&job->node, &aborted);
	}
	spin_unlock_irqrestore(&hw->job_lock, flags);

	if (active) {
		rk_rga_hw_cancel_timeout(hw);
		rk_rga_job_note_hw_done(active);
		(void)rk_rga_hw_reset_for_recovery(hw);
		rk_rga_hw_power_off(hw);
	}
	rk_rga_hw_enable_irq(hw, irq_disabled);
	mutex_unlock(&hw->run_lock);

	if (active) {
		rk_rga_job_complete_queued(active, result);
		dispatch = true;
	}

	list_for_each_entry_safe(job, tmp, &aborted, node) {
		list_del_init(&job->node);
		rk_rga_job_complete_queued(job, result);
		dispatch = true;
	}

	return dispatch;
}

static void rk_rga_session_abort_hw_jobs(struct rk_rga_session *session,
					 int result)
{
	struct rk_rga_hw *hws[RK_RGA_CORE_COUNTER_COUNT];
	struct rk_rga_hw *hw;
	u32 count = 0;

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (count >= ARRAY_SIZE(hws))
			break;

		refcount_inc(&hw->refs);
		hws[count++] = hw;
	}
	mutex_unlock(&rk_rga.hw_lock);

	for (u32 i = 0; i < count; i++) {
		hw = hws[i];
		if (rk_rga_hw_abort_session_jobs(hw, session, result))
			rk_rga_hw_dispatch(hw);
		if (READ_ONCE(hw->recovery_failed))
			rk_rga_abort_pending_if_no_hw(-EIO);
		rk_rga_hw_put(hw);
	}
}

static void rk_rga_job_acquire_work(struct work_struct *work)
{
	struct rk_rga_job *job = container_of(work, struct rk_rga_job,
					      acquire_work);
	struct rk_rga_session *session = READ_ONCE(job->session);
	bool dispatching = false;
	int ret = READ_ONCE(job->result);

	if (!ret && session)
		dispatching =
			rk_rga_session_begin_acquire_dispatch(session, job);
	else
		rk_rga_job_set_waiting_acquire(job, false);
	if (!ret && !dispatching) {
		rk_rga_job_set_waiting_acquire(job, false);
		rk_rga_job_set_acquire_result(job, -EFAULT);
		ret = READ_ONCE(job->result);
	}

	if (ret)
		rk_rga_job_complete(job, ret);
	else
		rk_rga_job_queue(job);
	if (dispatching)
		rk_rga_session_end_job_dispatch(session);

	rk_rga_job_put(job);
}

static int rk_rga_job_submit(struct rk_rga_session *session,
			     struct rk_rga_job *job, int *release_fence_fd,
			     struct sync_file **release_sync_file)
{
	bool acquire_pending = false;
	int ret;

	if (release_fence_fd)
		*release_fence_fd = -1;
	if (release_sync_file)
		*release_sync_file = NULL;
	if (job->sync_mode == RGA_BLIT_ASYNC &&
	    (!release_fence_fd || !release_sync_file))
		return -EINVAL;

	ret = rk_rga_job_prepare_release_fence(job);
	if (ret)
		return ret;

	ret = rk_rga_session_track_job(session, job);
	if (ret)
		return ret;

	ret = rk_rga_job_acquire_status(job, &acquire_pending);
	if (ret) {
		rk_rga_job_complete(job, ret);
		return ret;
	}

	if (acquire_pending && job->sync_mode == RGA_BLIT_ASYNC) {
		struct sync_file *sync_file;

		ret = rk_rga_fence_create_fd(job->release_fence, &sync_file);
		if (ret < 0) {
			rk_rga_job_complete(job, ret);
			return ret;
		}

		job->release_fence_fd = ret;
		if (release_fence_fd)
			*release_fence_fd = ret;

		rk_rga_job_get(job);
		ret = rk_rga_job_arm_acquire_callbacks(job);
		if (ret) {
			rk_rga_job_put(job);
			rk_rga_fence_abort_fd(job->release_fence_fd,
					      sync_file);
			job->release_fence_fd = -1;
			if (release_fence_fd)
				*release_fence_fd = -1;
			rk_rga_job_complete(job, ret);
			return ret;
		}
		if (!rk_rga_has_available_hw())
			rk_rga_job_abort_pending_acquire(job, -ENODEV);

		*release_sync_file = sync_file;

		return 0;
	}

	if (acquire_pending) {
		ret = rk_rga_job_wait_acquire_fences(job);
		if (ret) {
			rk_rga_job_complete(job, ret);
			return ret;
		}
	}

	if (job->sync_mode == RGA_BLIT_ASYNC) {
		struct sync_file *sync_file;

		ret = rk_rga_fence_create_fd(job->release_fence, &sync_file);
		if (ret < 0) {
			rk_rga_job_complete(job, ret);
			return ret;
		}

		job->release_fence_fd = ret;
		if (release_fence_fd)
			*release_fence_fd = ret;

		ret = rk_rga_job_queue(job);
		if (ret) {
			rk_rga_fence_abort_fd(job->release_fence_fd,
					      sync_file);
			job->release_fence_fd = -1;
			if (release_fence_fd)
				*release_fence_fd = -1;
			return ret;
		}

		*release_sync_file = sync_file;

		return 0;
	}

	ret = rk_rga_job_queue_and_wait(job);

	return ret;
}

static int rk_rga_copy_user_tasks(const struct rga_user_request *user,
				  struct rga_req **tasks_out)
{
	struct rga_req *tasks;
	size_t bytes;
	u64 task_ptr;

	bytes = array_size(user->task_num, sizeof(*tasks));
	if (bytes == SIZE_MAX)
		return -EOVERFLOW;

	task_ptr = user->task_ptr;
	tasks = memdup_user(u64_to_user_ptr(task_ptr), bytes);
	if (IS_ERR(tasks))
		return PTR_ERR(tasks);

	*tasks_out = tasks;

	return 0;
}

static int rk_rga_request_config(struct rk_rga_session *session,
				 const struct rga_user_request *user,
				 struct rk_rga_job **job_out)
{
	struct rk_rga_request *request;
	struct rk_rga_acquire_fd *acquire_fds;
	struct dma_fence **fences = NULL;
	struct rk_rga_import **imports = NULL;
	struct rga_req *tasks = NULL;
	u32 *gauss_coeffs = NULL;
	u32 acquire_fd_count = 0;
	u32 fence_count = 0;
	u32 import_count = 0;
	bool close_acquire_fds = false;
	int ret;

	ret = rk_rga_copy_user_tasks(user, &tasks);
	if (ret)
		return ret;

	ret = rk_rga_copy_gauss_coeffs(tasks, user->task_num, &gauss_coeffs);
	if (ret) {
		kfree(tasks);
		return ret;
	}

	acquire_fds = kcalloc(RGA_TASK_NUM_MAX + 1, sizeof(*acquire_fds),
			      GFP_KERNEL);
	if (!acquire_fds) {
		kfree(gauss_coeffs);
		kfree(tasks);
		return -ENOMEM;
	}

	mutex_lock(&session->lock);
	request = idr_find(&session->requests, user->id);
	if (!request) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = rk_rga_prepare_tasks_locked(session, tasks, user->task_num,
					  user->acquire_fence_fd,
					  &imports, &import_count,
					  &fences, &fence_count,
					  acquire_fds, &acquire_fd_count);
	if (ret)
		goto out_unlock;

	rk_rga_request_clear_fences(request);
	rk_rga_request_clear_imports(request);
	rk_rga_request_clear_gauss(request);
	kfree(request->tasks);
	request->tasks = tasks;
	request->imports = imports;
	request->acquire_fences = fences;
	request->gauss_coeffs = gauss_coeffs;
	request->import_count = import_count;
	request->acquire_fence_count = fence_count;
	request->task_count = user->task_num;
	request->sync_mode = user->sync_mode;
	request->mpi_config_flags = user->mpi_config_flags;
	request->acquire_fence_fd = user->acquire_fence_fd;
	request->release_fence_fd = -1;
	request->configured = true;
	tasks = NULL;
	imports = NULL;
	fences = NULL;
	gauss_coeffs = NULL;
	import_count = 0;
	fence_count = 0;
	close_acquire_fds = true;

	if (job_out) {
		ret = rk_rga_job_clone_request_locked(request, job_out);
		if (ret)
			goto out_unlock;
	}

out_unlock:
	mutex_unlock(&session->lock);
	if (close_acquire_fds)
		rk_rga_close_kernel_acquire_fds(acquire_fds,
						acquire_fd_count);
	rk_rga_put_import_array(imports, import_count);
	rk_rga_put_fence_array(fences, fence_count);
	kfree(gauss_coeffs);
	kfree(tasks);
	kfree(acquire_fds);

	return ret;
}

static int rk_rga_import_buffer_size(const struct rga_external_buffer *buffer,
				     size_t *size)
{
	struct rk_rga_img_layout layout;
	struct rga_img_info_t img = {};
	int ret;

	if (buffer->memory_parm.size) {
		*size = buffer->memory_parm.size;
		return 0;
	}

	img.vir_w = buffer->memory_parm.width;
	img.vir_h = buffer->memory_parm.height;
	img.format = buffer->memory_parm.format;

	ret = rk_rga_img_layout(&img, &layout);
	if (ret)
		return ret;

	*size = layout.total_size;

	return 0;
}

static int rk_rga_map_userptr_sgt_iommu(struct rk_rga_import *import,
					struct device *dev,
					struct sg_table *sgt,
					dma_addr_t *iova_out,
					struct iommu_domain **domain_out,
					size_t *iova_size_out)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	struct sg_table aligned_sgt;
	size_t data_size;
	size_t map_size;
	dma_addr_t iova;
	ssize_t mapped;
	int prot;
	int ret;

	if (!domain || !(domain->type & __IOMMU_DOMAIN_PAGING))
		return -EOPNOTSUPP;

	memset(&aligned_sgt, 0, sizeof(aligned_sgt));
	ret = rk_rga_alloc_aligned_sgt(sgt, &aligned_sgt, &data_size,
				       &map_size);
	if (ret)
		return ret;

	ret = rk_rga_alloc_iommu_iova(domain, dev, map_size, &iova);
	if (ret)
		goto err_free_aligned_sgt;

	prot = rk_rga_iommu_prot(dev, DMA_BIDIRECTIONAL);
	if (!prot) {
		ret = -EINVAL;
		goto err_free_iova;
	}

	mapped = iommu_map_sg(domain, iova, aligned_sgt.sgl,
			      aligned_sgt.orig_nents, prot, GFP_KERNEL);
	if (mapped < 0) {
		ret = mapped;
		goto err_free_iova;
	}
	if ((size_t)mapped < map_size) {
		if (mapped)
			iommu_unmap(domain, iova, mapped);
		ret = -EIO;
		goto err_free_iova;
	}

	*iova_out = iova + import->page_offset;
	ret = rk_rga_check_iova_span(*iova_out, data_size,
				     "driver-owned userptr IOMMU", true);
	if (ret)
		goto err_unmap_iova;

	sg_free_table(&aligned_sgt);
	*domain_out = domain;
	*iova_size_out = map_size;

	return 0;

err_unmap_iova:
	iommu_unmap(domain, iova, map_size);
err_free_iova:
	rk_rga_free_iommu_iova(domain, iova, map_size);
err_free_aligned_sgt:
	sg_free_table(&aligned_sgt);
	return ret;
}

static int rk_rga_map_userptr_sgt(struct rk_rga_import *import,
				  struct device *dev,
				  struct sg_table **sgt_out,
				  dma_addr_t *iova_out,
				  struct iommu_domain **domain_out,
				  size_t *iova_size_out,
				  bool *iommu_mapped_out)
{
	struct sg_table *sgt;
	bool force_iommu;
	int ret;

	*sgt_out = NULL;
	*domain_out = NULL;
	*iova_size_out = 0;
	*iommu_mapped_out = false;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	ret = sg_alloc_table_from_pages(sgt, import->pages,
					import->page_count,
					import->page_offset,
					import->size, GFP_KERNEL);
	if (ret)
		goto err_free_sgt;

	ret = dma_map_sgtable(dev, sgt, DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto err_free_table;

	force_iommu = READ_ONCE(rk_rga.route_b_force_remap);
	ret = rk_rga_check_dma_sgt(sgt, "userptr", import->size, iova_out,
				   false);
	if (!ret && !force_iommu) {
		*sgt_out = sgt;
		return 0;
	}

	dma_unmap_sgtable(dev, sgt, DMA_BIDIRECTIONAL, 0);
	rk_rga_reset_sgt_dma_state(sgt);
	if (ret && ret != -EOPNOTSUPP && ret != -EOVERFLOW)
		goto err_free_table;

	atomic_inc(&rk_rga.route_b_attempt_count);
	ret = rk_rga_map_userptr_sgt_iommu(import, dev, sgt, iova_out,
					   domain_out, iova_size_out);
	if (ret)
		goto err_free_table;

	atomic_inc(&rk_rga.route_b_ok_count);
	atomic_inc(&rk_rga.route_b_active_count);

	*sgt_out = sgt;
	*iommu_mapped_out = true;

	return 0;

err_free_table:
	sg_free_table(sgt);
err_free_sgt:
	kfree(sgt);
	return ret;
}

static int rk_rga_import_dmabuf(struct rga_external_buffer *buffer,
				struct rk_rga_import **import_out)
{
	struct rk_rga_import *import;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct device *dev;
	dma_addr_t iova;
	int fd;
	int ret;

	ret = rk_rga_import_dmabuf_fd(buffer, &fd);
	if (ret)
		return ret;

	dev = rk_rga_get_map_dev();
	if (!dev)
		return -ENODEV;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		put_device(dev);
		return PTR_ERR(dmabuf);
	}

	attach = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attach)) {
		dma_buf_put(dmabuf);
		put_device(dev);
		return PTR_ERR(attach);
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return PTR_ERR(sgt);
	}

	ret = rk_rga_check_dma_sgt(sgt, "dma-buf", dmabuf->size, &iova,
				   true);
	if (ret) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return ret;
	}

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	if (!import) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return -ENOMEM;
	}

	import->type = RK_RGA_IMPORT_DMABUF;
	import->fd = fd;
	refcount_set(&import->refs, 1);
	import->dev = dev;
	import->dmabuf = dmabuf;
	import->attach = attach;
	import->sgt = sgt;
	import->iova = iova;
	import->size = dmabuf->size;

	*import_out = import;

	return 0;
}

static int rk_rga_import_userptr(struct rga_external_buffer *buffer,
				 struct rk_rga_import **import_out)
{
	struct rk_rga_import *import;
	struct device *dev;
	struct page **pages;
	unsigned long start;
	unsigned long addr;
	unsigned int page_count;
	unsigned int page_offset;
	size_t nr_pages;
	size_t span;
	size_t size;
	int pinned;
	int ret;

	if ((u64)(unsigned long)buffer->memory != buffer->memory)
		return -EINVAL;

	ret = rk_rga_import_buffer_size(buffer, &size);
	if (ret)
		return ret;
	if (!size)
		return -EINVAL;
	if (!access_ok(u64_to_user_ptr(buffer->memory), size))
		return -EFAULT;

	addr = (unsigned long)buffer->memory;
	start = addr & PAGE_MASK;
	page_offset = offset_in_page(addr);
	if (check_add_overflow(size, (size_t)page_offset, &span))
		return -EOVERFLOW;
	nr_pages = DIV_ROUND_UP(span, (size_t)PAGE_SIZE);
	if (!nr_pages || nr_pages > INT_MAX)
		return -EOVERFLOW;
	page_count = nr_pages;

	dev = rk_rga_get_map_dev();
	if (!dev)
		return -ENODEV;

	pages = kcalloc(page_count, sizeof(*pages), GFP_KERNEL);
	if (!pages) {
		put_device(dev);
		return -ENOMEM;
	}

	pinned = pin_user_pages_fast(start, page_count,
				     FOLL_WRITE | FOLL_LONGTERM, pages);
	if (pinned != page_count) {
		if (pinned > 0)
			unpin_user_pages(pages, pinned);
		kfree(pages);
		put_device(dev);
		return pinned < 0 ? pinned : -EFAULT;
	}

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	if (!import) {
		unpin_user_pages(pages, page_count);
		kfree(pages);
		put_device(dev);
		return -ENOMEM;
	}

	import->type = RK_RGA_IMPORT_USERPTR;
	import->fd = -1;
	refcount_set(&import->refs, 1);
	import->dev = dev;
	import->pages = pages;
	import->size = size;
	import->page_count = page_count;
	import->pinned_pages = page_count;
	import->page_offset = page_offset;

	ret = rk_rga_map_userptr_sgt(import, dev, &import->sgt,
				     &import->iova, &import->domain,
				     &import->iova_size,
				     &import->iommu_mapped);
	if (ret) {
		rk_rga_import_put(import);
		return ret;
	}

	*import_out = import;

	return 0;
}

static int rk_rga_import_one(struct rk_rga_session *session,
			     struct rga_external_buffer *buffer)
{
	struct rk_rga_import *import;
	int handle;
	int ret;

	switch (buffer->type) {
	case RGA_DMA_BUFFER:
		ret = rk_rga_import_dmabuf(buffer, &import);
		break;
	case RGA_VIRTUAL_ADDRESS:
		ret = rk_rga_import_userptr(buffer, &import);
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (ret)
		return ret;

	mutex_lock(&session->lock);
	handle = idr_alloc(&session->imports, import, 1, 0, GFP_KERNEL);
	if (handle >= 0) {
		import->counted = true;
		atomic_inc(&rk_rga.import_count);
	}
	mutex_unlock(&session->lock);
	if (handle < 0) {
		rk_rga_import_put(import);
		return handle;
	}

	buffer->handle = handle;

	return handle;
}

static long rk_rga_ioctl_import_buffer(unsigned long arg,
				       struct rk_rga_session *session)
{
	struct rga_buffer_pool pool;
	struct rga_external_buffer *buffers;
	int imported[RGA_BUFFER_POOL_SIZE_MAX];
	u32 imported_count = 0;
	size_t bytes;
	int ret = 0;

	if (copy_from_user(&pool, (void __user *)arg, sizeof(pool)))
		return -EFAULT;
	if (pool.size > RGA_BUFFER_POOL_SIZE_MAX)
		return -EFBIG;
	if (!pool.buffers_ptr)
		return -EFAULT;

	bytes = sizeof(*buffers) * pool.size;
	buffers = memdup_user(u64_to_user_ptr(pool.buffers_ptr), bytes);
	if (IS_ERR(buffers))
		return PTR_ERR(buffers);

	for (u32 i = 0; i < pool.size; i++) {
		ret = rk_rga_import_one(session, &buffers[i]);
		if (ret < 0)
			goto rollback;
		imported[imported_count++] = ret;
	}

	if (copy_to_user(u64_to_user_ptr(pool.buffers_ptr), buffers, bytes)) {
		ret = -EFAULT;
		goto rollback;
	}

out:
	kfree(buffers);
	return ret;

rollback:
	while (imported_count) {
		void *ptr;

		imported_count--;
		mutex_lock(&session->lock);
		ptr = idr_remove(&session->imports, imported[imported_count]);
		mutex_unlock(&session->lock);
		if (ptr)
			rk_rga_import_put(ptr);
	}
	goto out;
}

static long rk_rga_ioctl_release_buffer(unsigned long arg,
					struct rk_rga_session *session)
{
	struct rga_buffer_pool pool;
	struct rga_external_buffer *buffers;
	size_t bytes;
	int ret = 0;

	if (copy_from_user(&pool, (void __user *)arg, sizeof(pool)))
		return -EFAULT;
	if (pool.size > RGA_BUFFER_POOL_SIZE_MAX)
		return -EFBIG;
	if (!pool.buffers_ptr)
		return -EFAULT;

	bytes = sizeof(*buffers) * pool.size;
	buffers = memdup_user(u64_to_user_ptr(pool.buffers_ptr), bytes);
	if (IS_ERR(buffers))
		return PTR_ERR(buffers);

	for (u32 i = 0; i < pool.size; i++) {
		void *ptr;

		mutex_lock(&session->lock);
		ptr = idr_remove(&session->imports, buffers[i].handle);
		mutex_unlock(&session->lock);
		if (!ptr) {
			ret = -EINVAL;
			goto out;
		}
		rk_rga_import_put(ptr);
	}

out:
	kfree(buffers);
	return ret;
}

static long rk_rga_ioctl_request_create(unsigned long arg,
					struct rk_rga_session *session)
{
	struct rk_rga_request *request;
	__u32 flags;
	int id;

	if (copy_from_user(&flags, (void __user *)arg, sizeof(flags)))
		return -EFAULT;

	request = kzalloc(sizeof(*request), GFP_KERNEL);
	if (!request)
		return -ENOMEM;
	request->flags = flags;

	mutex_lock(&session->lock);
	id = idr_alloc(&session->requests, request, 1, 0, GFP_KERNEL);
	mutex_unlock(&session->lock);
	if (id < 0) {
		kfree(request);
		return id;
	}

	if (copy_to_user((void __user *)arg, &id, sizeof(id))) {
		rk_rga_request_remove_free(session, id);
		return -EFAULT;
	}

	return 0;
}

static int rk_rga_request_check(const struct rga_user_request *user)
{
	if (!user->id)
		return -EINVAL;
	if (!user->task_ptr)
		return -EINVAL;
	if (!user->task_num)
		return -EINVAL;
	if (user->task_num > RGA_TASK_NUM_MAX)
		return -EFBIG;

	return 0;
}

static int rk_rga_request_ioctl_ret(int ret)
{
	return ret ? -EFAULT : 0;
}

static long rk_rga_ioctl_request_submit(unsigned long arg,
					struct rk_rga_session *session,
					bool run)
{
	struct rk_rga_job *job = NULL;
	struct sync_file *release_sync_file = NULL;
	struct rga_user_request user;
	int release_fence_fd = -1;
	int ret;

	if (copy_from_user(&user, (void __user *)arg, sizeof(user)))
		return -EFAULT;

	ret = rk_rga_request_check(&user);
	if (ret)
		return ret;

	ret = rk_rga_request_config(session, &user, run ? &job : NULL);
	if (ret)
		return rk_rga_request_ioctl_ret(ret);

	if (run) {
		ret = rk_rga_job_submit(session, job, &release_fence_fd,
					user.sync_mode == RGA_BLIT_ASYNC ?
					&release_sync_file : NULL);
		if (ret) {
			ret = rk_rga_request_ioctl_ret(ret);
		} else if (user.sync_mode == RGA_BLIT_ASYNC) {
			if (release_fence_fd < 0 || !release_sync_file) {
				ret = -EFAULT;
			} else {
				user.release_fence_fd = release_fence_fd;
				if (copy_to_user((void __user *)arg, &user,
						 sizeof(user))) {
					rk_rga_job_abort_fd(job, release_sync_file);
					ret = -EFAULT;
				} else {
					rk_rga_job_install_fd(job, release_sync_file);
				}
			}
		}
		rk_rga_job_put(job);
		rk_rga_request_remove_free(session, user.id);
		return ret;
	}

	return 0;
}

static long rk_rga_ioctl_request_cancel(unsigned long arg,
					struct rk_rga_session *session)
{
	__u32 id;

	if (copy_from_user(&id, (void __user *)arg, sizeof(id)))
		return -EFAULT;

	if (!rk_rga_request_remove_free(session, id))
		return -EINVAL;

	return 0;
}

static long rk_rga_ioctl_get_version(unsigned long arg)
{
	char version[RGA_VERSION_SIZE] = "0.00";

	mutex_lock(&rk_rga.hw_lock);
	if (rk_rga.hw_versions.size) {
		struct rga_version_t *first = &rk_rga.hw_versions.version[0];

		snprintf(version, sizeof(version), "%x.%02x",
			 first->major, first->minor);
	}
	mutex_unlock(&rk_rga.hw_lock);

	if (copy_to_user((void __user *)arg, version, sizeof(version)))
		return -EFAULT;

	return 0;
}

static long rk_rga_ioctl_get_rga2_version(unsigned long arg)
{
	struct rga_version_t version = {};
	struct rk_rga_hw *hw;
	bool found = false;

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (hw->type == RK_RGA_HW_RGA2) {
			rk_rga_set_version(&version,
					   hw->version.major,
					   hw->version.minor,
					   hw->version.revision);
			found = true;
			break;
		}
	}
	mutex_unlock(&rk_rga.hw_lock);

	if (!found)
		return -EFAULT;
	if (copy_to_user((void __user *)arg, version.str, sizeof(version.str)))
		return -EFAULT;

	/* BSP/librga compatibility: success is reported as a positive result. */
	return true;
}

static long rk_rga_ioctl_get_hw_versions(unsigned long arg)
{
	struct rga_hw_versions_t versions;

	mutex_lock(&rk_rga.hw_lock);
	versions = rk_rga.hw_versions;
	mutex_unlock(&rk_rga.hw_lock);

	if (copy_to_user((void __user *)arg, &versions, sizeof(versions)))
		return -EFAULT;

	return true;
}

static long rk_rga_ioctl_get_driver_version(unsigned long arg)
{
	struct rga_version_t version = {
		.major = DRIVER_MAJOR_VERISON,
		.minor = DRIVER_MINOR_VERSION,
		.revision = DRIVER_REVISION_VERSION,
	};

	strscpy((char *)version.str, DRIVER_VERSION, sizeof(version.str));

	if (copy_to_user((void __user *)arg, &version, sizeof(version)))
		return -EFAULT;

	return true;
}

static long rk_rga_ioctl_blit(unsigned long arg, struct rk_rga_session *session,
			      __u32 sync_mode)
{
	struct rk_rga_job *job = NULL;
	struct rk_rga_acquire_fd *acquire_fds;
	struct dma_fence **fences = NULL;
	struct rk_rga_import **imports = NULL;
	struct sync_file *release_sync_file = NULL;
	struct rga_req *task;
	u32 *gauss_coeffs = NULL;
	int release_fence_fd = -1;
	u32 acquire_fd_count = 0;
	u32 fence_count = 0;
	u32 import_count = 0;
	bool close_acquire_fds = false;
	int ret;

	task = memdup_user((void __user *)arg, sizeof(*task));
	if (IS_ERR(task))
		return PTR_ERR(task);

	ret = rk_rga_copy_gauss_coeffs(task, 1, &gauss_coeffs);
	if (ret) {
		kfree(task);
		return ret;
	}

	acquire_fds = kcalloc(RGA_TASK_NUM_MAX + 1, sizeof(*acquire_fds),
			      GFP_KERNEL);
	if (!acquire_fds) {
		kfree(gauss_coeffs);
		kfree(task);
		return -ENOMEM;
	}

	mutex_lock(&session->lock);
	ret = rk_rga_prepare_tasks_locked(session, task, 1, -1,
					  &imports, &import_count,
					  &fences, &fence_count,
					  acquire_fds, &acquire_fd_count);
	if (!ret) {
		close_acquire_fds = true;
		ret = rk_rga_job_take_prepared(task, 1, sync_mode, imports,
					       import_count, fences, fence_count,
					       gauss_coeffs,
					       &job);
		if (!ret) {
			task = NULL;
			imports = NULL;
			fences = NULL;
			gauss_coeffs = NULL;
			import_count = 0;
			fence_count = 0;
		}
	}
	mutex_unlock(&session->lock);
	if (close_acquire_fds)
		rk_rga_close_kernel_acquire_fds(acquire_fds,
						acquire_fd_count);
	rk_rga_put_import_array(imports, import_count);
	rk_rga_put_fence_array(fences, fence_count);
	kfree(gauss_coeffs);
	kfree(task);
	if (ret) {
		kfree(acquire_fds);
		return ret;
	}

	ret = rk_rga_job_submit(session, job, &release_fence_fd,
				sync_mode == RGA_BLIT_ASYNC ?
				&release_sync_file : NULL);
	if (!ret && sync_mode == RGA_BLIT_ASYNC) {
		if (release_fence_fd < 0 || !release_sync_file) {
			ret = -EIO;
		} else {
			job->tasks[0].out_fence_fd = release_fence_fd;
			if (copy_to_user((void __user *)arg, job->tasks,
					 sizeof(*job->tasks))) {
				rk_rga_job_abort_fd(job, release_sync_file);
				ret = -EFAULT;
			} else {
				rk_rga_job_install_fd(job, release_sync_file);
			}
		}
	}
	rk_rga_job_put(job);
	kfree(acquire_fds);

	return ret;
}

static long rk_rga_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rk_rga_session *session = file->private_data;

	if (!session)
		return -EINVAL;

	atomic_inc(&rk_rga.ioctl_count);

	switch (cmd) {
	case RGA_BLIT_SYNC:
	case RGA_BLIT_ASYNC:
		return rk_rga_ioctl_blit(arg, session, cmd);
	case RGA_CACHE_FLUSH:
	case RGA_FLUSH:
	case RGA2_FLUSH:
	case RGA_GET_RESULT:
	case RGA2_GET_RESULT:
		return 0;
	case RGA_GET_VERSION:
		rk_rga_refresh_hw_versions();
		return rk_rga_ioctl_get_version(arg);
	case RGA2_GET_VERSION:
		rk_rga_refresh_hw_versions();
		return rk_rga_ioctl_get_rga2_version(arg);
	case RGA_IOC_GET_HW_VERSION:
		rk_rga_refresh_hw_versions();
		return rk_rga_ioctl_get_hw_versions(arg);
	case RGA_IOC_GET_DRVIER_VERSION:
		return rk_rga_ioctl_get_driver_version(arg);
	case RGA_IOC_IMPORT_BUFFER:
		return rk_rga_ioctl_import_buffer(arg, session);
	case RGA_IOC_RELEASE_BUFFER:
		return rk_rga_ioctl_release_buffer(arg, session);
	case RGA_IOC_REQUEST_CREATE:
		return rk_rga_ioctl_request_create(arg, session);
	case RGA_IOC_REQUEST_SUBMIT:
		return rk_rga_ioctl_request_submit(arg, session, true);
	case RGA_IOC_REQUEST_CONFIG:
		return rk_rga_ioctl_request_submit(arg, session, false);
	case RGA_IOC_REQUEST_CANCEL:
		return rk_rga_ioctl_request_cancel(arg, session);
	case RGA_IMPORT_DMA:
	case RGA_RELEASE_DMA:
	default:
		return -EINVAL;
	}
}

static const struct rk_rga_hw_match rk_rga2_match = {
	.type = RK_RGA_HW_RGA2,
	.name = "rga2",
	.version_major = 3,
	.version_minor = 2,
	.version_revision = 0x63318,
	.min_reg_size = RK_RGA2_MIN_REG_SIZE,
};

static const struct rk_rga_hw_match rk_rga3_match = {
	.type = RK_RGA_HW_RGA3,
	.name = "rga3",
	.version_major = 3,
	.version_minor = 0,
	.version_revision = 0x76831,
	.min_reg_size = RK_RGA3_MIN_REG_SIZE,
};

static const struct of_device_id rk_rga_of_match[] = {
	{ .compatible = "rockchip,rga2", .data = &rk_rga2_match },
	{ .compatible = "rockchip,rga2_core0", .data = &rk_rga2_match },
	{ .compatible = "rockchip,rk3588-rga", .data = &rk_rga2_match },
	{ .compatible = "rockchip,rk3288-rga", .data = &rk_rga2_match },
	{ .compatible = "rockchip,rga3", .data = &rk_rga3_match },
	{ .compatible = "rockchip,rga3_core0", .data = &rk_rga3_match },
	{ .compatible = "rockchip,rga3_core1", .data = &rk_rga3_match },
	{ .compatible = "rockchip,rk3588-rga3", .data = &rk_rga3_match },
	{ }
};
MODULE_DEVICE_TABLE(of, rk_rga_of_match);

static int rk_rga_hw_detect_version(struct rk_rga_hw *hw, u32 *raw_version)
{
	int ret;

	ret = rk_rga_hw_power_on_internal(hw, false);
	if (ret)
		return ret;

	*raw_version = rk_rga_read(hw,
				   hw->type == RK_RGA_HW_RGA3 ?
				   RK_RGA3_VERSION_NUM : RK_RGA2_VERSION_NUM);
	rk_rga_hw_power_off(hw);

	return rk_rga_validate_hw_version(hw->match, *raw_version,
					  &hw->version);
}

static int rk_rga_hw_probe(struct platform_device *pdev)
{
	const struct rk_rga_hw_match *match;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct rk_rga_hw *hw;
	u32 raw_version = 0;
	int irq;
	int ret;

	match = of_device_get_match_data(dev);
	if (!match)
		return -EINVAL;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ret = rk_rga_validate_mmio_size(match->min_reg_size,
					res ? resource_size(res) : 0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "%s register window is missing or too small\n",
				     match->name);

	hw->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hw->regs))
		return PTR_ERR(hw->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	hw->irq = irq;

	ret = rk_rga_validate_clock_count(devm_clk_bulk_get_all(dev,
								&hw->clks));
	if (ret < 0)
		return ret;
	hw->num_clks = ret;

	hw->resets = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(hw->resets))
		return PTR_ERR(hw->resets);

	hw->dev = dev;
	hw->type = match->type;
	hw->match = match;
	ret = rk_rga_hw_reset_controls(hw);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to reset %s hardware\n",
				     match->name);

	ret = dma_set_mask(dev, hw->type == RK_RGA_HW_RGA3 ?
			   DMA_BIT_MASK(40) : DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (hw->type == RK_RGA_HW_RGA3)
		rk_rga_set_iommu_dma_limit(dev);

	refcount_set(&hw->refs, 1);
	atomic_set(&hw->irq_disable_depth, 0);
	init_waitqueue_head(&hw->idle);
	spin_lock_init(&hw->job_lock);
	mutex_init(&hw->run_lock);
	INIT_DELAYED_WORK(&hw->timeout_work, rk_rga_hw_timeout_work);
	INIT_WORK(&hw->iommu_fault_work, rk_rga_hw_iommu_fault_work);
	INIT_LIST_HEAD(&hw->fault_node);
	INIT_LIST_HEAD(&hw->job_queue);
	platform_set_drvdata(pdev, hw);

	hw->iommu_node = of_parse_phandle(dev->of_node, "iommus", 0);
	if (hw->iommu_node) {
		ret = devm_add_action_or_reset(dev, rk_rga_of_node_put,
					       hw->iommu_node);
		if (ret)
			return ret;
	}

	ret = devm_request_threaded_irq(dev, hw->irq, rk_rga_irq_handler,
					rk_rga_irq_thread, IRQF_ONESHOT,
					dev_name(dev), hw);
	if (ret)
		return ret;
	hw->irq_registered = true;

	pm_runtime_enable(dev);
	ret = rk_rga_hw_detect_version(hw, &raw_version);
	if (ret) {
		pm_runtime_disable(dev);
		if (ret == -ENODEV)
			return dev_err_probe(dev, ret,
				"unsupported %s hardware version %#010x (%s)\n",
				match->name, raw_version,
				(char *)hw->version.str);
		return dev_err_probe(dev, ret,
				     "failed to read %s hardware version\n",
				     match->name);
	}

	ret = rk_rga_iommu_register_fault_handler(hw);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	mutex_lock(&rk_rga.hw_lock);
	hw->core_mask =
		rk_rga_find_free_core_mask(&rk_rga.hw_list, hw->type);
	if (!hw->core_mask) {
		mutex_unlock(&rk_rga.hw_lock);
		ret = -ENOSPC;
		goto err_unregister_fault_handler;
	}
	hw->index = __ffs(hw->core_mask);
	list_add_tail(&hw->node, &rk_rga.hw_list);
	rk_rga_refresh_hw_versions_locked();
	mutex_unlock(&rk_rga.hw_lock);

	dev_info(dev,
		 "registered %s %s core %d mask %#x irq %d clocks %d\n",
		 match->name, (char *)hw->version.str, hw->index,
		 hw->core_mask, hw->irq, hw->num_clks);

	return 0;

err_unregister_fault_handler:
	rk_rga_iommu_unregister_fault_handler(hw);
	cancel_work_sync(&hw->iommu_fault_work);
	pm_runtime_disable(dev);

	return dev_err_probe(dev, ret, "no free public core slot\n");
}

static void rk_rga_hw_remove(struct platform_device *pdev)
{
	struct rk_rga_hw *hw = platform_get_drvdata(pdev);
	unsigned long flags;
	bool no_hw_left;

	mutex_lock(&rk_rga.hw_lock);
	spin_lock_irqsave(&hw->job_lock, flags);
	hw->removing = true;
	spin_unlock_irqrestore(&hw->job_lock, flags);
	list_del_init(&hw->node);
	rk_rga_refresh_hw_versions_locked();
	no_hw_left = !rk_rga.hw_count;
	mutex_unlock(&rk_rga.hw_lock);

	if (no_hw_left)
		rk_rga_abort_all_pending_acquire_jobs(-ENODEV);
	rk_rga_iommu_unregister_fault_handler(hw);
	cancel_work_sync(&hw->iommu_fault_work);
	rk_rga_hw_abort_jobs(hw, -ENODEV);
	wait_event(hw->idle, refcount_read(&hw->refs) == 1);
	/* The fail-fast handler makes balancing safe before devres frees the IRQ. */
	rk_rga_hw_restore_irq_depth(hw);
	devm_free_irq(&pdev->dev, hw->irq, hw);
	hw->irq_registered = false;
	pm_runtime_disable(&pdev->dev);
}

static struct platform_driver rk_rga_platform_driver = {
	.probe = rk_rga_hw_probe,
	.remove = rk_rga_hw_remove,
	.driver = {
		.name = "rockchip-rga-rewrite",
		.of_match_table = rk_rga_of_match,
	},
};

static int rk_rga_open(struct inode *inode, struct file *file)
{
	struct rk_rga_session *session;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	rk_rga_session_init(session);
	rk_rga_session_link(session);
	file->private_data = session;

	return nonseekable_open(inode, file);
}

static bool rk_rga_session_jobs_empty(struct rk_rga_session *session)
{
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&session->job_lock, flags);
	empty = list_empty(&session->jobs);
	spin_unlock_irqrestore(&session->job_lock, flags);

	return empty;
}

static int rk_rga_release(struct inode *inode, struct file *file)
{
	struct rk_rga_session *session = file->private_data;
	struct rk_rga_import *import;
	struct rk_rga_request *request;
	int id;

	if (!session)
		return 0;

	rk_rga_session_unlink(session);
	rk_rga_session_mark_closing(session);
	wait_event(session->job_wait,
		   rk_rga_session_dispatches_idle(session));

	rk_rga_session_abort_pending_acquire_jobs(session, -EFAULT);
	rk_rga_session_abort_hw_jobs(session, -EFAULT);
	wait_event(session->job_wait, rk_rga_session_jobs_empty(session));

	idr_for_each_entry(&session->imports, import, id)
		rk_rga_import_put(import);
	idr_for_each_entry(&session->requests, request, id)
		rk_rga_request_free(request);
	idr_destroy(&session->imports);
	idr_destroy(&session->requests);
	kfree(session);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations rk_rga_fops = {
	.owner		= THIS_MODULE,
	.open		= rk_rga_open,
	.release	= rk_rga_release,
	.unlocked_ioctl	= rk_rga_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= rk_rga_ioctl,
#endif
	.llseek		= noop_llseek,
};

static void
rk_rga_debugfs_create_core_counts(const char *prefix,
				  atomic_t counters[RK_RGA_CORE_COUNTER_COUNT])
{
	static const char * const core_names[RK_RGA_CORE_COUNTER_COUNT] = {
		"rga3_core0",
		"rga3_core1",
		"rga2_core0",
		"rga2_core1",
	};
	char name[48];

	for (u32 i = 0; i < ARRAY_SIZE(core_names); i++) {
		snprintf(name, sizeof(name), "%s_%s_count", prefix,
			 core_names[i]);
		debugfs_create_atomic_t(name, 0444, rk_rga.debugfs_root,
					&counters[i]);
	}
}

static void
rk_rga_debugfs_create_core_times(const char *prefix,
				 atomic64_t counters[RK_RGA_CORE_COUNTER_COUNT])
{
	static const char * const core_names[RK_RGA_CORE_COUNTER_COUNT] = {
		"rga3_core0",
		"rga3_core1",
		"rga2_core0",
		"rga2_core1",
	};
	char name[48];

	for (u32 i = 0; i < ARRAY_SIZE(core_names); i++) {
		snprintf(name, sizeof(name), "%s_%s", prefix,
			 core_names[i]);
		rk_rga_debugfs_create_atomic64(name, &counters[i]);
	}
}

static void rk_rga_debugfs_create_route_b(void)
{
	struct dentry *dir;

	dir = debugfs_create_dir("route_b", rk_rga.debugfs_root);
	debugfs_create_atomic_t("attempt", 0444, dir,
				&rk_rga.route_b_attempt_count);
	debugfs_create_atomic_t("ok", 0444, dir, &rk_rga.route_b_ok_count);
	debugfs_create_atomic_t("active", 0444, dir,
				&rk_rga.route_b_active_count);
	debugfs_create_bool("force_remap", 0600, dir, &rk_rga.route_b_force_remap);
}

static int __init rk_rga_init(void)
{
	int ret;

	mutex_init(&rk_rga.hw_lock);
	mutex_init(&rk_rga.session_lock);
	spin_lock_init(&rk_rga.fault_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
	INIT_LIST_HEAD(&rk_rga.sessions);
	INIT_LIST_HEAD(&rk_rga.fault_hws);
	spin_lock_init(&rk_rga.fence_lock);
	rk_rga.fence_context = dma_fence_context_alloc(1);

	ret = platform_driver_register(&rk_rga_platform_driver);
	if (ret)
		return ret;

	rk_rga.miscdev.minor = MISC_DYNAMIC_MINOR;
	rk_rga.miscdev.name = "rga";
	rk_rga.miscdev.fops = &rk_rga_fops;

	ret = misc_register(&rk_rga.miscdev);
	if (ret)
		goto err_unregister_platform;

	rk_rga.debugfs_root = debugfs_create_dir("rk_rga_rewrite", NULL);
	debugfs_create_u32("hw_count", 0444, rk_rga.debugfs_root,
			   &rk_rga.hw_count);
	debugfs_create_atomic_t("ioctl_count", 0444, rk_rga.debugfs_root,
				&rk_rga.ioctl_count);
	debugfs_create_atomic_t("import_count", 0444, rk_rga.debugfs_root,
				&rk_rga.import_count);
	debugfs_create_atomic_t("prepared_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.prepared_job_count);
	debugfs_create_atomic_t("release_fence_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.release_fence_count);
	debugfs_create_atomic_t("completed_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.completed_job_count);
	debugfs_create_atomic_t("scheduled_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.scheduled_job_count);
	rk_rga_debugfs_create_core_counts("scheduled",
					  rk_rga.scheduled_core_count);
	debugfs_create_atomic_t("dispatched_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.dispatched_job_count);
	rk_rga_debugfs_create_core_counts("dispatched",
					  rk_rga.dispatched_core_count);
	debugfs_create_atomic_t("started_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.started_job_count);
	rk_rga_debugfs_create_core_counts("started",
					  rk_rga.started_core_count);
	rk_rga_debugfs_create_atomic64("hw_total_ns", &rk_rga.hw_total_ns);
	rk_rga_debugfs_create_atomic64("hw_max_ns", &rk_rga.hw_max_ns);
	rk_rga_debugfs_create_core_times("hw_total_ns",
					 rk_rga.hw_total_core_ns);
	rk_rga_debugfs_create_core_times("hw_max_ns",
					 rk_rga.hw_max_core_ns);
	debugfs_create_atomic_t("cmd_alloc_count", 0444, rk_rga.debugfs_root,
				&rk_rga.cmd_alloc_count);
	debugfs_create_atomic_t("power_cycle_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.power_cycle_count);
	debugfs_create_atomic_t("irq_count", 0444, rk_rga.debugfs_root,
				&rk_rga.irq_count);
	debugfs_create_atomic_t("irq_thread_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.irq_thread_count);
	debugfs_create_atomic_t("irq_error_count", 0444, rk_rga.debugfs_root,
				&rk_rga.irq_error_count);
	debugfs_create_atomic_t("irq_spurious_count", 0444, rk_rga.debugfs_root,
				&rk_rga.irq_spurious_count);
	debugfs_create_atomic_t("timeout_count", 0444, rk_rga.debugfs_root,
				&rk_rga.timeout_count);
	debugfs_create_atomic_t("iommu_fault_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.iommu_fault_count);
	debugfs_create_atomic_t("iommu_refresh_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.iommu_refresh_count);
	debugfs_create_atomic_t("recovery_failure_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.recovery_failure_count);
	rk_rga_debugfs_create_route_b();
	debugfs_create_atomic_t("unsupported_count", 0444, rk_rga.debugfs_root,
				&rk_rga.unsupported_count);

	pr_info("registered /dev/rga (%s), hw_count=%u\n",
		RK_RGA_REWRITE_VERSION, rk_rga.hw_versions.size);

	return 0;

err_unregister_platform:
	platform_driver_unregister(&rk_rga_platform_driver);

	return ret;
}

static void __exit rk_rga_exit(void)
{
	debugfs_remove_recursive(rk_rga.debugfs_root);
	misc_deregister(&rk_rga.miscdev);
	platform_driver_unregister(&rk_rga_platform_driver);
}

module_init(rk_rga_init);
module_exit(rk_rga_exit);

MODULE_IMPORT_NS("DMA_BUF");
MODULE_DESCRIPTION("Minimal Rockchip RGA compatibility rewrite");
MODULE_AUTHOR("OpenAI");
MODULE_LICENSE("GPL");
MODULE_VERSION(RK_RGA_REWRITE_VERSION);
