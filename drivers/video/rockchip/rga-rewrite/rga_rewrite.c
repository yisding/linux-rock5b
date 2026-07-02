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
#include <linux/dma-mapping.h>
#include <linux/fdtable.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#if IS_ENABLED(CONFIG_ROCKCHIP_RGA_REWRITE_KUNIT_TEST)
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
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#ifndef kzalloc_obj
#define kzalloc_obj(obj, flags)	kzalloc(sizeof(obj), flags)
#endif

#define RK_RGA_REWRITE_VERSION		"rk3588-rga-rewrite-0.1"
#define RK_RGA2_CMD_REG_COUNT		32
#define RK_RGA3_CMD_REG_COUNT		48
#define RK_RGA_JOB_TIMEOUT_MS		1000
#define RK_RGA_RESET_TIMEOUT_US		1000
#define RK_RGA_FULL_CSC_ENABLE		BIT(0)

#define RK_RGA2_SYS_CTRL	0x000
#define RK_RGA2_CMD_CTRL	0x004
#define RK_RGA2_CMD_BASE	0x008
#define RK_RGA2_STATUS1		0x00c
#define RK_RGA2_INT		0x010
#define RK_RGA2_STATUS2		0x01c
#define RK_RGA2_WORK_CNT	0x020

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
#define RK_RGA2_CF_GR_G_OFFSET			0x060
#define RK_RGA2_CF_GR_R_OFFSET			0x064
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

#define RK_RGA2_SRC_FORMAT			GENMASK(3, 0)
#define RK_RGA2_SRC_RB_SWAP			BIT(4)
#define RK_RGA2_SRC_ALPHA_SWAP			BIT(5)
#define RK_RGA2_SRC_UV_SWAP			BIT(6)
#define RK_RGA2_SRC_CSC_MODE			GENMASK(9, 8)
#define RK_RGA2_SRC_ROT_MODE			GENMASK(11, 10)
#define RK_RGA2_SRC_MIR_MODE			GENMASK(13, 12)
#define RK_RGA2_SRC_HSCL_MODE			GENMASK(15, 14)
#define RK_RGA2_SRC_VSCL_MODE			GENMASK(17, 16)
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
#define RK_RGA2_DST_CSC_MODE			GENMASK(17, 16)
#define RK_RGA2_DST_CSC_CLIP			BIT(18)
#define RK_RGA2_DST_FULL_CSC_EN		BIT(19)
#define RK_RGA2_DST_YUV400_EN			BIT(24)
#define RK_RGA2_DST_Y4_EN			BIT(25)

#define RK_RGA2_SCALE_BYPASS			0
#define RK_RGA2_SCALE_DOWN			1
#define RK_RGA2_SCALE_UP			2
#define RK_RGA2_SCALE_FORCE_TILE		3
#define RK_RGA2_BILINEAR_PREC			12
#define RK_RGA2_INTERP_DEFAULT			0
#define RK_RGA2_INTERP_LINEAR			1
#define RK_RGA2_INTERP_AVERAGE			3

#define RK_RGA3_SYS_CTRL	0x000
#define RK_RGA3_CMD_CTRL	0x004
#define RK_RGA3_CMD_ADDR	0x008
#define RK_RGA3_INT_EN		0x020
#define RK_RGA3_INT_RAW		0x024
#define RK_RGA3_INT_CLR		0x02c
#define RK_RGA3_RO_SRST		0x030
#define RK_RGA3_STATUS0		0x034
#define RK_RGA3_CMD_STATE	0x040

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

struct rk_rga_import {
	refcount_t refs;
	enum rk_rga_import_type type;
	int fd;
	struct device *dev;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct page **pages;
	dma_addr_t iova;
	size_t size;
	unsigned int page_count;
	unsigned int pinned_pages;
	unsigned int page_offset;
};

struct rk_rga_job_mapping {
	struct rk_rga_import *import;
	struct device *dev;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
	bool userptr;
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
	struct rk_rga_hw *hw;
	struct work_struct acquire_work;
	refcount_t refs;
	struct rga_req *tasks;
	struct rk_rga_import **imports;
	struct rk_rga_job_mapping *mappings;
	struct dma_fence **acquire_fences;
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
	__u32 sync_mode;
	int release_fence_fd;
	int irq_result;
	int result;
	bool queued;
	bool cmd_ready;
	bool irq_seen;
	bool done;
};

struct rk_rga_hw_match {
	enum rk_rga_hw_type type;
	const char *name;
	__u32 version_major;
	__u32 version_minor;
	__u32 version_revision;
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
	u32 core_mask;
	refcount_t refs;
	wait_queue_head_t idle;
	spinlock_t job_lock;
	struct mutex run_lock; /* serializes start, timeout, IRQ, and remove */
	struct delayed_work timeout_work;
	struct list_head job_queue;
	struct rk_rga_job *active_job;
	u32 queued_jobs;
	atomic_t iommu_fault_pending;
	bool removing;
};

struct rk_rga_session {
	struct mutex lock;
	struct idr imports;
	struct idr requests;
};

struct rk_rga_service {
	struct miscdevice miscdev;
	struct dentry *debugfs_root;
	struct mutex hw_lock;
	spinlock_t fault_lock; /* protects fault_hws in fault handler context */
	struct list_head hw_list;
	struct list_head fault_hws;
	struct rga_hw_versions_t hw_versions;
	u32 hw_count;
	u64 fence_context;
	u32 fence_seqno;
	spinlock_t fence_lock;
	atomic_t ioctl_count;
	atomic_t import_count;
	atomic_t prepared_job_count;
	atomic_t release_fence_count;
	atomic_t completed_job_count;
	atomic_t scheduled_job_count;
	atomic_t dispatched_job_count;
	atomic_t started_job_count;
	atomic_t cmd_alloc_count;
	atomic_t power_cycle_count;
	atomic_t irq_count;
	atomic_t irq_thread_count;
	atomic_t irq_error_count;
	atomic_t irq_spurious_count;
	atomic_t timeout_count;
	atomic_t iommu_fault_count;
	atomic_t unsupported_count;
};

static struct rk_rga_service rk_rga;

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

static void rk_rga_add_hw_match_version(struct rga_hw_versions_t *versions,
					const struct rk_rga_hw_match *match)
{
	rk_rga_add_hw_version(versions, match->version_major,
			      match->version_minor, match->version_revision);
}

static void rk_rga_refresh_hw_versions_locked(void)
{
	struct rga_hw_versions_t versions = {};
	struct rk_rga_hw *hw;

	list_for_each_entry(hw, &rk_rga.hw_list, node)
		rk_rga_add_hw_match_version(&versions, hw->match);

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

static int rk_rga_hw_power_on(struct rk_rga_hw *hw)
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

	atomic_inc(&rk_rga.power_cycle_count);

	return 0;
}

static void rk_rga_hw_power_off(struct rk_rga_hw *hw)
{
	clk_bulk_disable_unprepare(hw->num_clks, hw->clks);
	pm_runtime_put_sync(hw->dev);
}

static struct device *rk_rga_get_map_dev(void)
{
	struct rk_rga_hw *hw;
	struct device *dev = NULL;

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (hw->removing)
			continue;
		if (hw->type == RK_RGA_HW_RGA3) {
			dev = get_device(hw->dev);
			break;
		}
	}
	list_for_each_entry(hw, &rk_rga.hw_list, node) {
		if (dev)
			break;
		if (hw->removing)
			continue;
		dev = get_device(hw->dev);
		break;
	}
	mutex_unlock(&rk_rga.hw_lock);

	return dev;
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
		if (import->sgt) {
			dma_unmap_sgtable(import->dev, import->sgt,
					   DMA_BIDIRECTIONAL, 0);
			sg_free_table(import->sgt);
			kfree(import->sgt);
		}
		if (import->pages) {
			unpin_user_pages_dirty_lock(import->pages,
						    import->pinned_pages,
						    true);
			kfree(import->pages);
		}
	}
	if (import->dev)
		put_device(import->dev);
	kfree(import);
}

static void rk_rga_import_put(struct rk_rga_import *import)
{
	if (refcount_dec_and_test(&import->refs))
		rk_rga_import_destroy(import);
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

static void rk_rga_request_free(void *ptr)
{
	struct rk_rga_request *request = ptr;

	rk_rga_request_clear_imports(request);
	rk_rga_request_clear_fences(request);
	kfree(request->tasks);
	kfree(request);
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
	case RK_RGA_FORMAT_YCBCR_400:
	case RK_RGA_FORMAT_A8:
	case RK_RGA_FORMAT_Y8:
		layout->yrgb_size = pixels;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP4:
	case RK_RGA_FORMAT_Y4:
		layout->yrgb_size = pixels >> 1;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP2:
		layout->yrgb_size = pixels >> 2;
		ret = 0;
		break;
	case RK_RGA_FORMAT_BPP1:
		layout->yrgb_size = pixels >> 3;
		ret = 0;
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
	if (img->rd_mode == RK_RGA_FBC_MODE) {
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

static int rk_rga_resolve_direct_img(struct rga_req *task,
				     struct rga_img_info_t *img,
				     struct rk_rga_import **imports,
				     u32 *import_count,
				     bool required)
{
	struct rga_external_buffer buffer = {};
	struct rk_rga_import *import;
	int ret;

	if (!rk_rga_direct_img_uses_mmu(task, img)) {
		if (!img->yrgb_addr && !img->uv_addr && !img->v_addr)
			return required ? -EINVAL : 0;
		return -EOPNOTSUPP;
	}

	buffer.memory_parm.width = img->vir_w;
	buffer.memory_parm.height = img->vir_h;
	buffer.memory_parm.format = img->format;

	if (img->yrgb_addr && img->yrgb_addr <= INT_MAX) {
		buffer.type = RGA_DMA_BUFFER;
		buffer.memory = img->yrgb_addr;
		ret = rk_rga_import_dmabuf(&buffer, &import);
	} else {
		buffer.type = RGA_VIRTUAL_ADDRESS;
		buffer.memory = img->yrgb_addr ? img->yrgb_addr : img->uv_addr;
		if (!buffer.memory)
			return required ? -EINVAL : 0;
		ret = rk_rga_import_userptr(&buffer, &import);
	}
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

	if (img->rd_mode == RK_RGA_FBC_MODE && (uv_handle || v_handle))
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
	if (img->rd_mode == RK_RGA_FBC_MODE) {
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
			dma_unmap_sgtable(mapping->dev, mapping->sgt,
					   DMA_BIDIRECTIONAL, 0);
			sg_free_table(mapping->sgt);
			kfree(mapping->sgt);
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
	if (job->cmd_vaddr)
		return 0;

	job->cmd_size = rk_rga_cmd_size(hw);
	job->cmd_dev = get_device(hw->dev);
	job->cmd_vaddr = dma_alloc_coherent(hw->dev, job->cmd_size,
					    &job->cmd_dma, GFP_KERNEL);
	if (!job->cmd_vaddr) {
		put_device(job->cmd_dev);
		job->cmd_dev = NULL;
		job->cmd_size = 0;
		return -ENOMEM;
	}

	memset(job->cmd_vaddr, 0, job->cmd_size);
	atomic_inc(&rk_rga.cmd_alloc_count);

	return 0;
}

static int rk_rga_map_userptr_sgt(struct rk_rga_import *import,
				  struct device *dev,
				  struct sg_table **sgt_out,
				  dma_addr_t *iova_out);
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

		ret = rk_rga_map_userptr_sgt(import, dev, &sgt,
					     &mapped_iova);
		if (ret)
			return ret;

		job->mappings[job->mapping_count] = (struct rk_rga_job_mapping) {
			.import = import,
			.dev = get_device(dev),
			.sgt = sgt,
			.iova = mapped_iova,
			.userptr = true,
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

	job->mappings[job->mapping_count] = (struct rk_rga_job_mapping) {
		.import = import,
		.dev = get_device(dev),
		.attach = attach,
		.sgt = sgt,
		.iova = sg_dma_address(sgt->sgl),
	};
	*iova = job->mappings[job->mapping_count].iova;
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
	if (img->rd_mode == RK_RGA_FBC_MODE) {
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
	rk_rga_job_free_cmd(job);
	rk_rga_job_clear_mappings(job);
	rk_rga_put_import_array(job->imports, job->import_count);
	rk_rga_put_fence_array(job->acquire_fences, job->acquire_fence_count);
	kfree(job->acquire_waiters);
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

static void rk_rga_job_acquire_work(struct work_struct *work);

static void rk_rga_job_init(struct rk_rga_job *job)
{
	INIT_LIST_HEAD(&job->node);
	INIT_WORK(&job->acquire_work, rk_rga_job_acquire_work);
	refcount_set(&job->refs, 1);
	init_waitqueue_head(&job->wait);
	atomic_set(&job->pending_acquire_count, 0);
	atomic_set(&job->acquire_work_queued, 0);
	job->release_fence_fd = -1;
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

static void rk_rga_job_queue_acquire_work(struct rk_rga_job *job)
{
	if (atomic_cmpxchg(&job->acquire_work_queued, 0, 1) == 0)
		queue_work(system_wq, &job->acquire_work);
}

static void rk_rga_job_acquire_cb(struct dma_fence *fence,
				  struct dma_fence_cb *cb)
{
	struct rk_rga_fence_waiter *waiter =
		container_of(cb, struct rk_rga_fence_waiter, cb);
	struct rk_rga_job *job = waiter->job;
	int status;

	status = dma_fence_get_status(fence);
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
			status = dma_fence_get_status(fence);
			if (status < 0)
				rk_rga_job_set_acquire_result(job, status);
			if (atomic_dec_and_test(&job->pending_acquire_count))
				rk_rga_job_queue_acquire_work(job);
		} else if (ret) {
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

	rk_rga_job_sync_userptr_for_cpu(job);
	job->result = result;
	job->done = true;
	job->hw = NULL;
	rk_rga_fence_signal(job->release_fence, result);
	wake_up_all(&job->wait);
	atomic_inc(&rk_rga.completed_job_count);
	rk_rga_hw_put(hw);
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

static void rk_rga_hw_dispatch(struct rk_rga_hw *hw);
static struct rk_rga_job *rk_rga_hw_take_active(struct rk_rga_hw *hw);
static void rk_rga_hw_timeout_work(struct work_struct *work);

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

static void rk_rga_hw_reset_for_recovery(struct rk_rga_hw *hw)
{
	int ret;

	if (hw->type == RK_RGA_HW_RGA3)
		ret = rk_rga3_soft_reset(hw);
	else
		ret = rk_rga2_soft_reset(hw);

	if (!ret)
		return;

	dev_warn(hw->dev, "%s soft reset during recovery failed: %d\n",
		 hw->match->name, ret);

	if (!hw->resets)
		return;

	ret = reset_control_reset(hw->resets);
	if (ret)
		dev_warn(hw->dev,
			 "%s reset-control recovery fallback failed: %d\n",
			 hw->match->name, ret);
}

static void rk_rga2_start_hw(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	u32 sys_ctrl = RK_RGA2_SYS_CTRL_AUTO_CKG |
		       RK_RGA2_SYS_CTRL_AUTO_RST |
		       RK_RGA2_SYS_CTRL_CMD_MODE;

	rk_rga2_clear_irq(hw);
	rk_rga2_write_full_csc(hw, &job->tasks[job->current_task]);
	rk_rga_write(hw, rk_rga_read(hw, RK_RGA2_INT) |
		     RK_RGA2_INT_ENABLE_MASK, RK_RGA2_INT);
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

static void rk_rga_hw_start(struct rk_rga_hw *hw, struct rk_rga_job *job)
{
	job->irq_result = 0;
	job->irq_seen = false;

	if (hw->type == RK_RGA_HW_RGA3)
		rk_rga3_start_hw(hw, job);
	else
		rk_rga2_start_hw(hw, job);

	atomic_inc(&rk_rga.started_job_count);
	schedule_delayed_work(&hw->timeout_work,
			      msecs_to_jiffies(RK_RGA_JOB_TIMEOUT_MS));
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
		job->irq_result = done ? 0 : rk_rga3_irq_result(job);
		job->irq_seen = true;
		rk_rga3_clear_irq(hw);
	} else {
		rk_rga2_read_irq_status(hw, job);
		done = job->intr_status & RK_RGA2_INT_DONE_MASK;
		error = job->intr_status & RK_RGA2_INT_ERROR_MASK;
		if (!done && !error)
			return IRQ_NONE;
		job->irq_result = error ? rk_rga2_irq_result(job) : 0;
		job->irq_seen = true;
		rk_rga2_clear_irq(hw);
	}

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
		if (!(intr & (RK_RGA2_INT_DONE_MASK | RK_RGA2_INT_ERROR_MASK)))
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
	struct rk_rga2_format_info dst_fmt;
	struct rk_rga2_transform transform;
};

struct rk_rga2_fill_profile {
	u8 dst_format;
	u8 pixel_width;
	bool rb_swap;
	bool alpha_swap;
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

static int rk_rga2_format_info(u32 format, bool write,
			       struct rk_rga2_format_info *info)
{
	memset(info, 0, sizeof(*info));
	info->pixel_width = 1;
	info->x_div = 1;
	info->y_div = 1;
	info->yuv = rk_rga_format_is_yuv(format);

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
	case RK_RGA_FORMAT_YCBCR_400:
		info->hw_format = 0x8;
		info->yuv400 = true;
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
		profile->dst_format = 0x0;
		profile->pixel_width = 4;
		return 0;
	case RK_RGA_FORMAT_BGRA_8888:
		profile->dst_format = 0x0;
		profile->pixel_width = 4;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBX_8888:
		profile->dst_format = 0x1;
		profile->pixel_width = 4;
		return 0;
	case RK_RGA_FORMAT_BGRX_8888:
		profile->dst_format = 0x1;
		profile->pixel_width = 4;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGB_888:
		profile->dst_format = 0x2;
		profile->pixel_width = 3;
		return 0;
	case RK_RGA_FORMAT_BGR_888:
		profile->dst_format = 0x2;
		profile->pixel_width = 3;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGB_565:
		profile->dst_format = 0x4;
		profile->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGR_565:
		profile->dst_format = 0x4;
		profile->pixel_width = 2;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBA_5551:
		profile->dst_format = 0x5;
		profile->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGRA_5551:
		profile->dst_format = 0x5;
		profile->pixel_width = 2;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_RGBA_4444:
		profile->dst_format = 0x6;
		profile->pixel_width = 2;
		return 0;
	case RK_RGA_FORMAT_BGRA_4444:
		profile->dst_format = 0x6;
		profile->pixel_width = 2;
		profile->rb_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_8888:
		profile->dst_format = 0x0;
		profile->pixel_width = 4;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ABGR_8888:
		profile->dst_format = 0x0;
		profile->pixel_width = 4;
		profile->rb_swap = true;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_XRGB_8888:
		profile->dst_format = 0x1;
		profile->pixel_width = 4;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_XBGR_8888:
		profile->dst_format = 0x1;
		profile->pixel_width = 4;
		profile->rb_swap = true;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_5551:
		profile->dst_format = 0x5;
		profile->pixel_width = 2;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ABGR_5551:
		profile->dst_format = 0x5;
		profile->pixel_width = 2;
		profile->rb_swap = true;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ARGB_4444:
		profile->dst_format = 0x6;
		profile->pixel_width = 2;
		profile->alpha_swap = true;
		return 0;
	case RK_RGA_FORMAT_ABGR_4444:
		profile->dst_format = 0x6;
		profile->pixel_width = 2;
		profile->rb_swap = true;
		profile->alpha_swap = true;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
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
			return -EOPNOTSUPP;
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
		return -EOPNOTSUPP;
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
		return -EOPNOTSUPP;
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
			return -EOPNOTSUPP;
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
		return -EOPNOTSUPP;
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
		return -EOPNOTSUPP;
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

#if IS_ENABLED(CONFIG_ROCKCHIP_RGA_REWRITE_KUNIT_TEST)
static int rk_rga2_select_dst_addresses(const struct rga_img_info_t *dst,
					const struct rk_rga2_format_info *fmt,
					const struct rk_rga2_transform *transform,
					u32 stride, u32 uv_stride,
					__u64 y_lt, __u64 u_lt, __u64 v_lt,
					__u64 *y_addr, __u64 *u_addr,
					__u64 *v_addr);
static int rk_rga_job_hw_type(struct rk_rga_job *job,
			      enum rk_rga_hw_type *type);
static int rk_rga2_emit_simple_bitblt(struct rk_rga_job *job);
static int rk_rga3_emit_simple_bitblt(struct rk_rga_job *job);
static int rk_rga_request_check(const struct rga_user_request *user);
static int rk_rga_request_ioctl_ret(int ret);
static struct rk_rga_hw *
rk_rga_find_best_hw_for_job(struct list_head *hw_list, struct rk_rga_job *job,
			    enum rk_rga_hw_type type);

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

	KUNIT_EXPECT_EQ(test, rk_rga2_decode_transform(&bad, &transform),
			-EOPNOTSUPP);
	bad.sina = 0;
	bad.cosa = 65536;
	bad.src.rotate_mode = 1;
	KUNIT_EXPECT_EQ(test, rk_rga2_decode_transform(&bad, &transform),
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

static void rk_rga_mixed_task_hw_type_kunit(struct kunit *test)
{
	enum rk_rga_hw_type type = 0;
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

	KUNIT_EXPECT_EQ(test, rk_rga_job_hw_type(&job, &type),
			-EOPNOTSUPP);
}

static void rk_rga_find_best_hw_for_job_kunit(struct kunit *test)
{
	struct rga_req task =
		rk_rga_ffmpeg_bitblt_task(RK_RGA_FORMAT_RGBA_8888,
					  RK_RGA_FORMAT_BGRA_8888);
	struct rk_rga_job active = { };
	struct rk_rga_job job = {
		.tasks = &task,
		.task_count = 1,
	};
	struct rk_rga_hw busy = {
		.type = RK_RGA_HW_RGA3,
		.core_mask = BIT(0),
		.queued_jobs = 2,
	};
	struct rk_rga_hw idle = {
		.type = RK_RGA_HW_RGA3,
		.core_mask = BIT(1),
	};
	LIST_HEAD(hw_list);

	spin_lock_init(&busy.job_lock);
	spin_lock_init(&idle.job_lock);
	list_add_tail(&busy.node, &hw_list);
	list_add_tail(&idle.node, &hw_list);

	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, &job,
							RK_RGA_HW_RGA3),
			    &idle);

	task.core = BIT(0);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, &job,
							RK_RGA_HW_RGA3),
			    &busy);

	task.core = BIT(1);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, &job,
							RK_RGA_HW_RGA3),
			    &idle);

	task.core = 0;
	idle.active_job = &active;
	idle.queued_jobs = 3;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, &job,
							RK_RGA_HW_RGA3),
			    &busy);

	task.core = BIT(0);
	busy.removing = true;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_rga_find_best_hw_for_job(&hw_list, &job,
							RK_RGA_HW_RGA3),
			    NULL);
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

	task.src.rd_mode = BIT(4);
	task.dst.rd_mode = RK_RGA_RASTER_MODE;
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

static struct kunit_case rk_rga_rewrite_test_cases[] = {
	KUNIT_CASE(rk_rga2_decode_transform_kunit),
	KUNIT_CASE(rk_rga2_dst_corner_kunit),
	KUNIT_CASE(rk_rga_fill_hw_type_kunit),
	KUNIT_CASE(rk_rga_request_check_kunit),
	KUNIT_CASE(rk_rga_request_ioctl_ret_kunit),
	KUNIT_CASE(rk_rga_job_free_release_fence_kunit),
	KUNIT_CASE(rk_rga_mixed_task_hw_type_kunit),
	KUNIT_CASE(rk_rga_find_best_hw_for_job_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_rga3_profiles_kunit),
	KUNIT_CASE(rk_rga2_compact_10bit_profile_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_fbc_profiles_kunit),
	KUNIT_CASE(rk_rga_ffmpeg_alpha_overlay_kunit),
	KUNIT_CASE(rk_rga3_alpha_rotate_emit_kunit),
	{ }
};

static struct kunit_suite rk_rga_rewrite_test_suite = {
	.name = "rockchip-rga-rewrite",
	.test_cases = rk_rga_rewrite_test_cases,
};

kunit_test_suite(rk_rga_rewrite_test_suite);
#endif

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
	    task->osd_info.enable || task->pre_intr_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;
	if (task->dst.rd_mode && task->dst.rd_mode != RK_RGA_RASTER_MODE)
		return -EOPNOTSUPP;

	ret = rk_rga3_validate_image(&task->dst);
	if (ret)
		return ret;

	return rk_rga2_fill_format_info(task->dst.format, profile);
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
	int ret;

	if (task->render_mode != RK_RGA_RENDER_BITBLT)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr || task->bsfilter_flag ||
	    task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->pat.yrgb_addr || task->pat.uv_addr || task->pat.v_addr)
		return -EOPNOTSUPP;
	if (task->src.yrgb_addr == task->dst.yrgb_addr)
		return -EOPNOTSUPP;
	if (task->alpha_rop_flag || task->PD_mode ||
	    task->feature.global_alpha_en)
		return -EOPNOTSUPP;
	ret = rk_rga2_decode_transform(task, &profile->transform);
	if (ret)
		return ret;
	if (task->mosaic_info.enable || task->osd_info.enable ||
	    task->pre_intr_info.enable || task->gauss_config.size)
		return -EOPNOTSUPP;
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

	return 0;
}

static int rk_rga3_validate_bitblt(const struct rga_req *task,
				   struct rk_rga3_bitblt_profile *profile)
{
	bool has_pat = rk_rga_img_has_addr(&task->pat);
	int ret;

	if (task->render_mode != RK_RGA_RENDER_BITBLT)
		return -EOPNOTSUPP;
	if (task->rop_mask_addr || task->LUT_addr ||
	    task->color_key_min || task->color_key_max)
		return -EOPNOTSUPP;
	if (task->bsfilter_flag != has_pat)
		return -EOPNOTSUPP;
	if (task->src.yrgb_addr == task->dst.yrgb_addr)
		return -EOPNOTSUPP;
	if (task->full_csc.flag || task->mosaic_info.enable ||
	    task->osd_info.enable || task->pre_intr_info.enable ||
	    task->gauss_config.size)
		return -EOPNOTSUPP;

	ret = rk_rga3_validate_alpha_blend(task);
	if (ret)
		return ret;

	profile->alpha_blend = rk_rga3_task_uses_alpha_blend(task);
	profile->pattern_blend = has_pat;
	if (profile->pattern_blend && !profile->alpha_blend)
		return -EOPNOTSUPP;

	ret = rk_rga3_rotate_flags(task, &profile->rotate_flags);
	if (ret)
		return ret;

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
	if (profile->dst_mode == 1 &&
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
		if (task->pat.x_offset || task->pat.y_offset ||
		    task->pat.act_w != task->dst.act_w ||
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
	ret = rk_rga3_format_info(task->dst.format, true,
				  &profile->dst_fmt);
	if (ret)
		return ret;
	if (profile->dst_mode == 1 &&
	    !rk_rga3_fbc_format_supported(task->dst.format, true))
		return -EOPNOTSUPP;
	if (profile->dst_fmt.yuv10 &&
	    (task->dst.x_offset || task->dst.y_offset))
		return -EOPNOTSUPP;
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
	 * RGB/RGBA pattern over an RGB destination, or the default RKMPP
	 * overlay path where an 8-bit YUV main image stays in a YUV destination
	 * and the pattern read window converts RGB/RGBA to that write domain.
	 */
	if (!profile->bg_fmt.rgb)
		return -EOPNOTSUPP;
	if (!profile->dst_fmt.rgb &&
	    !(profile->dst_fmt.yuv && profile->src_fmt.yuv &&
	      !profile->dst_fmt.yuv10 && !profile->src_fmt.yuv10))
		return -EOPNOTSUPP;

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
		ret = rk_rga3_stride(task->dst.vir_w, dst_fmt->pixel_width,
				     &stride);
		if (ret)
			return ret;

		uv_stride = dst_fmt->yuv_sp ?
			    ALIGN((u32)task->dst.vir_w, 16) >> 2 : stride;
		y_stride_bytes = stride << 2;
		uv_stride_bytes = uv_stride << 2;

		if (apply_dst_offset &&
		    (task->dst.x_offset || task->dst.y_offset)) {
			u32 x_offset = task->dst.x_offset;
			u32 y_plane_offset;

			if (dst_fmt->yuv_sp && (x_offset & 1))
				return -EINVAL;
			if (dst_fmt->yuv420_sp && (task->dst.y_offset & 1))
				return -EINVAL;

			if (check_mul_overflow((u32)task->dst.y_offset,
					       y_stride_bytes, &y_offset))
				return -EOVERFLOW;
			if (dst_fmt->yuv_sp) {
				if (check_add_overflow(y_offset, x_offset,
						       &y_offset))
					return -EOVERFLOW;
			} else {
				u32 x_offset_bytes;

				if (check_mul_overflow(x_offset,
						       (u32)dst_fmt->pixel_width,
						       &x_offset_bytes))
					return -EOVERFLOW;
				if (check_add_overflow(y_offset, x_offset_bytes,
						       &y_offset))
					return -EOVERFLOW;
			}

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
				if (check_add_overflow(y_plane_offset, x_offset,
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

	if (fmt->packed_yuv422) {
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

static int rk_rga2_scale_factor(u32 src, u32 dst, u8 interp, u32 *mode,
				u32 *factor, bool *filter)
{
	u32 param;

	*filter = false;
	if (src == dst) {
		*mode = RK_RGA2_SCALE_BYPASS;
		*factor = 0;
		return 0;
	}

	if (src > dst) {
		*mode = RK_RGA2_SCALE_DOWN;
		if (interp == RK_RGA2_INTERP_LINEAR) {
			param = (src << RK_RGA2_BILINEAR_PREC) / dst;
			if (param > 0xffff)
				return -EOPNOTSUPP;
			*factor = param |
				  (((1 << RK_RGA2_BILINEAR_PREC) >> 1) << 16);
			*filter = true;
			return 0;
		}

		param = (dst << 16) / src;
		while (param && (u64)param * (src - 1) > (u64)dst << 16)
			param--;
		*factor = param;
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
				   &h_filter);
	if (ret)
		return ret;
	ret = rk_rga2_scale_factor(task->src.act_h, dst_h,
				   task->interp.verti, &v_mode, &y_factor,
				   &v_filter);
	if (ret)
		return ret;
	if (rk_rga2_needs_force_tile(task, transform, dst_w, dst_h)) {
		h_mode = RK_RGA2_SCALE_FORCE_TILE;
		v_mode = RK_RGA2_SCALE_FORCE_TILE;
		x_factor = 0;
		y_factor = 0;
		h_filter = false;
		v_filter = false;
	}

	src_info = FIELD_PREP(RK_RGA2_SRC_FORMAT, src_fmt->hw_format) |
		   FIELD_PREP(RK_RGA2_SRC_RB_SWAP, src_fmt->rb_swap) |
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

	if (check_add_overflow(task->src.yrgb_addr, (__u64)y_offset,
			       &y_addr))
		return -EOVERFLOW;
	if (src_fmt->plane_width) {
		__u64 u64_addr;

		if (check_add_overflow(task->src.uv_addr, (__u64)uv_offset,
				       &u64_addr))
			return -EOVERFLOW;
		u_addr = lower_32_bits(u64_addr);
		if (check_add_overflow(task->src.v_addr, (__u64)uv_offset,
				       &u64_addr))
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
			 ((u32)task->src.act_w - 1) |
			 (((u32)task->src.act_h - 1) << 16));
	rk_rga_cmd_write(job, RK_RGA2_SRC_X_FACTOR_OFFSET, x_factor);
	rk_rga_cmd_write(job, RK_RGA2_SRC_Y_FACTOR_OFFSET, y_factor);
	rk_rga_cmd_write(job, RK_RGA2_SRC_BG_COLOR_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_FG_COLOR_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_TR_COLOR0_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_SRC_TR_COLOR1_OFFSET, 0);

	return 0;
}

static int rk_rga2_emit_dst(struct rk_rga_job *job,
			    const struct rga_req *task,
			    const struct rk_rga2_format_info *dst_fmt,
			    const struct rk_rga2_transform *transform)
{
	struct rga_img_info_t dst;
	u32 stride;
	u32 uv_stride;
	u32 y_offset;
	u32 uv_offset;
	u32 dst_info;
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
	rk_rga_cmd_write(job, RK_RGA2_DST_VIR_INFO_OFFSET, stride >> 2);
	rk_rga_cmd_write(job, RK_RGA2_DST_ACT_INFO_OFFSET,
			 ((u32)dst.act_w - 1) |
			 (((u32)dst.act_h - 1) << 16));

	return 0;
}

static int rk_rga2_emit_simple_bitblt(struct rk_rga_job *job)
{
	struct rga_req *task = &job->tasks[job->current_task];
	struct rk_rga2_bitblt_profile profile;
	int ret;

	ret = rk_rga2_validate_bitblt(task, &profile);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_MODE_CTRL_OFFSET,
			 FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
				    RK_RGA_RENDER_BITBLT) |
			 RK_RGA2_MODE_INTR_CF_E);

	ret = rk_rga2_emit_src(job, task, &profile.src_fmt,
			       &profile.transform);
	if (ret)
		return ret;
	ret = rk_rga2_emit_dst(job, task, &profile.dst_fmt,
			       &profile.transform);
	if (ret)
		return ret;

	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL0_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_ALPHA_CTRL1_OFFSET, 0);

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
	u32 stride_bytes;
	u32 stride_words;
	u32 x_offset_bytes;
	u32 y_offset_bytes;
	u64 y_addr;
	u32 mode;
	u32 dst_info;
	u32 act_info;
	int ret;

	ret = rk_rga2_validate_color_fill(task, &profile);
	if (ret)
		return ret;

	if (check_mul_overflow((u32)task->dst.vir_w,
			       (u32)profile.pixel_width, &stride_bytes))
		return -EOVERFLOW;
	stride_bytes = ALIGN(stride_bytes, 4);
	stride_words = stride_bytes >> 2;

	if (check_mul_overflow((u32)task->dst.x_offset,
			       (u32)profile.pixel_width, &x_offset_bytes))
		return -EOVERFLOW;
	if (check_mul_overflow((u32)task->dst.y_offset, stride_bytes,
			       &y_offset_bytes))
		return -EOVERFLOW;
	if (check_add_overflow(y_offset_bytes, x_offset_bytes,
			       &y_offset_bytes))
		return -EOVERFLOW;
	if (check_add_overflow(task->dst.yrgb_addr, (u64)y_offset_bytes,
			       &y_addr))
		return -EOVERFLOW;

	mode = FIELD_PREP(RK_RGA2_MODE_RENDER_MODE,
			  RK_RGA_RENDER_COLOR_FILL) |
	       RK_RGA2_MODE_INTR_CF_E;
	dst_info = FIELD_PREP(RK_RGA2_DST_FORMAT, profile.dst_format);
	if (profile.rb_swap)
		dst_info |= RK_RGA2_DST_RB_SWAP;
	if (profile.alpha_swap)
		dst_info |= RK_RGA2_DST_ALPHA_SWAP;

	act_info = ((u32)task->dst.act_w - 1) |
		   (((u32)task->dst.act_h - 1) << 16);

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
	rk_rga_cmd_write(job, RK_RGA2_DST_INFO_OFFSET, dst_info);
	rk_rga_cmd_write(job, RK_RGA2_DST_BASE0_OFFSET,
			 lower_32_bits(y_addr));
	rk_rga_cmd_write(job, RK_RGA2_DST_BASE1_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_DST_BASE2_OFFSET, 0);
	rk_rga_cmd_write(job, RK_RGA2_DST_VIR_INFO_OFFSET, stride_words);
	rk_rga_cmd_write(job, RK_RGA2_DST_ACT_INFO_OFFSET, act_info);
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

	reg = FIELD_PREP(RK_RGA3_OVLP_MODE, 1) |
	      RK_RGA3_OVLP_TOP_ALPHA_EN;
	if (profile->dst_fmt.yuv)
		reg |= RK_RGA3_OVLP_FIELD;

	rk_rga_cmd_write(job, RK_RGA3_OVLP_CTRL_OFFSET, reg);
	rk_rga_cmd_write(job, RK_RGA3_OVLP_OFF_OFFSET, ovlp_off);
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

	if (task->render_mode == RK_RGA_RENDER_BITBLT) {
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

static bool rk_rga_job_core_allows_hw(struct rk_rga_job *job,
				      const struct rk_rga_hw *hw)
{
	for (u32 i = 0; i < job->task_count; i++) {
		if (job->tasks[i].core && !(job->tasks[i].core & hw->core_mask))
			return false;
	}

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

static struct rk_rga_hw *
rk_rga_find_best_hw_for_job(struct list_head *hw_list, struct rk_rga_job *job,
			    enum rk_rga_hw_type type)
{
	struct rk_rga_hw *best = NULL;
	struct rk_rga_hw *hw;
	u32 best_load = U32_MAX;

	list_for_each_entry(hw, hw_list, node) {
		u32 load;

		if (hw->removing || hw->type != type)
			continue;
		if (!rk_rga_job_core_allows_hw(job, hw))
			continue;

		load = rk_rga_hw_load(hw);
		if (best && load >= best_load)
			continue;

		best = hw;
		best_load = load;
		if (!load)
			break;
	}

	return best;
}

static u32 rk_rga_hw_core_mask(enum rk_rga_hw_type type, u32 core_index)
{
	if (core_index >= 2)
		return 0;
	if (type == RK_RGA_HW_RGA3)
		return BIT(core_index);
	if (type == RK_RGA_HW_RGA2)
		return BIT(core_index + 2);

	return 0;
}

static int rk_rga_job_hw_type(struct rk_rga_job *job,
			      enum rk_rga_hw_type *type)
{
	bool type_valid = false;
	struct rk_rga3_bitblt_profile profile;
	struct rk_rga2_bitblt_profile rga2_profile;
	struct rk_rga2_fill_profile fill_profile;
	int ret;

	if (!job->task_count || !job->tasks)
		return -EINVAL;

	for (u32 i = 0; i < job->task_count; i++) {
		enum rk_rga_hw_type task_type;

		switch (job->tasks[i].render_mode) {
		case RK_RGA_RENDER_BITBLT:
		{
			int rga3_ret;
			bool allow_rga3;
			bool allow_rga2;

			if (job->import_count < 2)
				return -EOPNOTSUPP;
			ret = rk_rga_task_core_valid(&job->tasks[i]);
			if (ret)
				return ret;
			allow_rga3 = rk_rga_task_core_allows_type(&job->tasks[i],
								  RK_RGA_HW_RGA3);
			allow_rga2 = rk_rga_task_core_allows_type(&job->tasks[i],
								  RK_RGA_HW_RGA2);

			rga3_ret = -EOPNOTSUPP;
			if (allow_rga3) {
				rga3_ret = rk_rga3_validate_bitblt(&job->tasks[i],
								   &profile);
				if (!rga3_ret) {
					task_type = RK_RGA_HW_RGA3;
					break;
				}
			}

			if (!allow_rga2) {
				if (rga3_ret != -EOPNOTSUPP)
					return rga3_ret;
				return -EOPNOTSUPP;
			}

			ret = rk_rga2_validate_bitblt(&job->tasks[i],
						      &rga2_profile);
			if (!ret) {
				task_type = RK_RGA_HW_RGA2;
				break;
			}
			if (rga3_ret != -EOPNOTSUPP)
				return rga3_ret;
			return ret;
		}
		case RK_RGA_RENDER_COLOR_FILL:
			if (!job->import_count)
				return -EOPNOTSUPP;
			ret = rk_rga_task_core_valid(&job->tasks[i]);
			if (ret)
				return ret;
			if (!rk_rga_task_core_allows_type(&job->tasks[i],
							  RK_RGA_HW_RGA2))
				return -EOPNOTSUPP;
			ret = rk_rga2_validate_color_fill(&job->tasks[i],
							  &fill_profile);
			if (ret)
				return ret;
			task_type = RK_RGA_HW_RGA2;
			break;
		default:
			return -EOPNOTSUPP;
		}

		if (!type_valid) {
			*type = task_type;
			type_valid = true;
		} else if (*type != task_type) {
			return -EOPNOTSUPP;
		}
	}

	return 0;
}

static struct rk_rga_hw *rk_rga_hw_get_for_job(struct rk_rga_job *job,
					       int *error)
{
	enum rk_rga_hw_type type;
	struct rk_rga_hw *hw;
	int ret;

	ret = rk_rga_job_hw_type(job, &type);
	if (ret) {
		*error = ret;
		return NULL;
	}

	mutex_lock(&rk_rga.hw_lock);
	hw = rk_rga_find_best_hw_for_job(&rk_rga.hw_list, job, type);
	if (hw) {
		refcount_inc(&hw->refs);
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

	atomic_set(&hw->iommu_fault_pending, 0);

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
	int result;
	int ret;

	atomic_inc(&rk_rga.irq_thread_count);
	mutex_lock(&hw->run_lock);
	job = rk_rga_hw_take_active(hw);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return IRQ_HANDLED;
	}

	cancel_delayed_work(&hw->timeout_work);
	result = job->irq_result;
	rk_rga_hw_power_off(hw);

	if (rk_rga_job_advance_task(job, result)) {
		unsigned long flags;

		spin_lock_irqsave(&hw->job_lock, flags);
		if (!hw->removing && !hw->active_job) {
			hw->active_job = job;
			spin_unlock_irqrestore(&hw->job_lock, flags);

			ret = rk_rga_backend_start(hw, job);
			if (ret == RK_RGA_BACKEND_QUEUED) {
				mutex_unlock(&hw->run_lock);
				return IRQ_HANDLED;
			}

			spin_lock_irqsave(&hw->job_lock, flags);
			if (hw->active_job == job)
				hw->active_job = NULL;
			spin_unlock_irqrestore(&hw->job_lock, flags);

			result = ret;
		} else {
			spin_unlock_irqrestore(&hw->job_lock, flags);
			result = -ENODEV;
		}
	}

	rk_rga_job_complete_queued(job, result);
	mutex_unlock(&hw->run_lock);
	rk_rga_hw_dispatch(hw);

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

static void rk_rga_hw_timeout_work(struct work_struct *work)
{
	struct delayed_work *delayed = to_delayed_work(work);
	struct rk_rga_hw *hw = container_of(delayed, struct rk_rga_hw,
					    timeout_work);
	struct rk_rga_job *job;
	unsigned long flags;
	bool iommu_fault;
	int result;

	mutex_lock(&hw->run_lock);
	spin_lock_irqsave(&hw->job_lock, flags);
	job = hw->active_job;
	iommu_fault = atomic_xchg(&hw->iommu_fault_pending, 0);
	if (!job || (job->irq_seen && !iommu_fault)) {
		spin_unlock_irqrestore(&hw->job_lock, flags);
		mutex_unlock(&hw->run_lock);
		return;
	}

	if (hw->type == RK_RGA_HW_RGA3)
		rk_rga3_read_irq_status(hw, job);
	else
		rk_rga2_read_irq_status(hw, job);

	hw->active_job = NULL;
	spin_unlock_irqrestore(&hw->job_lock, flags);

	if (iommu_fault) {
		result = -EIO;
		dev_err(hw->dev, "job failed on IOMMU fault\n");
	} else {
		result = -EBUSY;
		atomic_inc(&rk_rga.timeout_count);
	}

	rk_rga_hw_reset_for_recovery(hw);
	rk_rga_hw_power_off(hw);
	rk_rga_job_complete_queued(job, result);
	mutex_unlock(&hw->run_lock);
	rk_rga_hw_dispatch(hw);
}

static int rk_rga_iommu_fault_handler(struct iommu_domain *domain,
				      struct device *iommu_dev,
				      unsigned long iova, int status,
				      void *arg)
{
	struct rk_rga_service *rga = arg;
	struct rk_rga_hw *fallback = NULL;
	struct rk_rga_hw *match = NULL;
	struct rk_rga_hw *hw;
	unsigned long flags;

	atomic_inc(&rga->iommu_fault_count);

	spin_lock_irqsave(&rga->fault_lock, flags);
	list_for_each_entry(hw, &rga->fault_hws, fault_node) {
		if (hw->iommu_domain != domain)
			continue;

		if (!fallback)
			fallback = hw;
		if (iommu_dev && hw->iommu_node == iommu_dev->of_node) {
			match = hw;
			break;
		}
	}
	if (!match)
		match = fallback;

	if (match) {
		atomic_set(&match->iommu_fault_pending, 1);
		mod_delayed_work(system_wq, &match->timeout_work, 0);
		dev_err_ratelimited(match->dev,
				    "IOMMU fault iova %#lx status %#x\n",
				    iova, status);
	}
	spin_unlock_irqrestore(&rga->fault_lock, flags);

	if (!match)
		pr_err_ratelimited("unmatched RGA IOMMU fault iova %#lx status %#x\n",
				   iova, status);

	return 0;
}

static void rk_rga_iommu_register_fault_handler(struct rk_rga_hw *hw)
{
	unsigned long flags;

	hw->iommu_domain = iommu_get_domain_for_dev(hw->dev);
	if (!hw->iommu_domain)
		return;

	spin_lock_irqsave(&rk_rga.fault_lock, flags);
	list_add_tail(&hw->fault_node, &rk_rga.fault_hws);
	spin_unlock_irqrestore(&rk_rga.fault_lock, flags);

	iommu_set_fault_handler(hw->iommu_domain,
				rk_rga_iommu_fault_handler, &rk_rga);
}

static void rk_rga_iommu_unregister_fault_handler(struct rk_rga_hw *hw)
{
	struct rk_rga_hw *other;
	unsigned long flags;
	bool clear = true;

	if (!hw->iommu_domain)
		return;

	spin_lock_irqsave(&rk_rga.fault_lock, flags);
	if (!list_empty(&hw->fault_node))
		list_del_init(&hw->fault_node);
	list_for_each_entry(other, &rk_rga.fault_hws, fault_node) {
		if (other->iommu_domain == hw->iommu_domain) {
			clear = false;
			break;
		}
	}
	spin_unlock_irqrestore(&rk_rga.fault_lock, flags);

	if (clear)
		iommu_set_fault_handler(hw->iommu_domain, NULL, NULL);
}

static void rk_rga_hw_dispatch(struct rk_rga_hw *hw)
{
	for (;;) {
		struct rk_rga_job *job;
		unsigned long flags;
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
		hw->active_job = job;
		spin_unlock_irqrestore(&hw->job_lock, flags);

		atomic_inc(&rk_rga.dispatched_job_count);
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

static int rk_rga_job_queue(struct rk_rga_job *job)
{
	struct rk_rga_hw *hw;
	unsigned long flags;
	int ret;

	hw = rk_rga_hw_get_for_job(job, &ret);
	if (!hw) {
		if (ret == -EOPNOTSUPP)
			atomic_inc(&rk_rga.unsupported_count);
		rk_rga_job_complete(job, ret);
		return ret;
	}

	job->hw = hw;
	rk_rga_job_get(job);

	spin_lock_irqsave(&hw->job_lock, flags);
	if (hw->removing) {
		spin_unlock_irqrestore(&hw->job_lock, flags);
		rk_rga_job_complete_queued(job, -ENODEV);
		return -ENODEV;
	}

	job->queued = true;
	list_add_tail(&job->node, &hw->job_queue);
	hw->queued_jobs++;
	atomic_inc(&rk_rga.scheduled_job_count);
	spin_unlock_irqrestore(&hw->job_lock, flags);

	rk_rga_hw_dispatch(hw);

	return 0;
}

static int rk_rga_job_queue_and_wait(struct rk_rga_job *job)
{
	int ret;

	ret = rk_rga_job_queue(job);
	if (ret)
		return ret;

	wait_event(job->wait, job->done);

	ret = job->result;

	return ret;
}

static void rk_rga_hw_abort_jobs(struct rk_rga_hw *hw, int result)
{
	struct rk_rga_job *job, *tmp;
	struct rk_rga_job *active;
	unsigned long flags;
	LIST_HEAD(aborted);

	cancel_delayed_work_sync(&hw->timeout_work);

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
		rk_rga_hw_reset_for_recovery(hw);
		rk_rga_hw_power_off(hw);
	}
	mutex_unlock(&hw->run_lock);
	cancel_delayed_work_sync(&hw->timeout_work);

	if (active)
		rk_rga_job_complete_queued(active, result);

	list_for_each_entry_safe(job, tmp, &aborted, node) {
		list_del_init(&job->node);
		rk_rga_job_complete_queued(job, result);
	}
}

static void rk_rga_job_acquire_work(struct work_struct *work)
{
	struct rk_rga_job *job = container_of(work, struct rk_rga_job,
					      acquire_work);
	int ret = READ_ONCE(job->result);

	if (ret)
		rk_rga_job_complete(job, ret);
	else
		rk_rga_job_queue(job);

	rk_rga_job_put(job);
}

static int rk_rga_job_submit(struct rk_rga_job *job, int *release_fence_fd,
			     bool *deferred)
{
	bool acquire_pending = false;
	int ret;

	if (release_fence_fd)
		*release_fence_fd = -1;
	if (deferred)
		*deferred = false;

	ret = rk_rga_job_prepare_release_fence(job);
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

		rk_rga_fence_install_fd(job->release_fence_fd, sync_file);
		if (deferred)
			*deferred = true;

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

		rk_rga_fence_install_fd(job->release_fence_fd, sync_file);
		if (deferred)
			*deferred = true;

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

	bytes = array_size(user->task_num, sizeof(*tasks));
	if (bytes == SIZE_MAX)
		return -EOVERFLOW;

	tasks = memdup_user(u64_to_user_ptr(user->task_ptr), bytes);
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
	u32 acquire_fd_count = 0;
	u32 fence_count = 0;
	u32 import_count = 0;
	bool close_acquire_fds = false;
	int ret;

	ret = rk_rga_copy_user_tasks(user, &tasks);
	if (ret)
		return ret;

	acquire_fds = kcalloc(RGA_TASK_NUM_MAX + 1, sizeof(*acquire_fds),
			      GFP_KERNEL);
	if (!acquire_fds) {
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
	kfree(request->tasks);
	request->tasks = tasks;
	request->imports = imports;
	request->acquire_fences = fences;
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
	import_count = 0;
	fence_count = 0;

	if (job_out) {
		ret = rk_rga_job_clone_request_locked(request, job_out);
		if (ret)
			goto out_unlock;
		close_acquire_fds = true;
	}

out_unlock:
	mutex_unlock(&session->lock);
	if (close_acquire_fds)
		rk_rga_close_kernel_acquire_fds(acquire_fds,
						acquire_fd_count);
	rk_rga_put_import_array(imports, import_count);
	rk_rga_put_fence_array(fences, fence_count);
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

static int rk_rga_map_userptr_sgt(struct rk_rga_import *import,
				  struct device *dev,
				  struct sg_table **sgt_out,
				  dma_addr_t *iova_out)
{
	struct sg_table *sgt;
	int ret;

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

	*sgt_out = sgt;
	*iova_out = sg_dma_address(sgt->sgl);

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

	dev = rk_rga_get_map_dev();
	if (!dev)
		return -ENODEV;

	dmabuf = dma_buf_get((int)buffer->memory);
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

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	if (!import) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return -ENOMEM;
	}

	import->type = RK_RGA_IMPORT_DMABUF;
	import->fd = (int)buffer->memory;
	refcount_set(&import->refs, 1);
	import->dev = dev;
	import->dmabuf = dmabuf;
	import->attach = attach;
	import->sgt = sgt;
	import->iova = sg_dma_address(sgt->sgl);
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

	ret = rk_rga_map_userptr_sgt(import, dev, &import->sgt, &import->iova);
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
	mutex_unlock(&session->lock);
	if (handle < 0) {
		rk_rga_import_put(import);
		return handle;
	}

	buffer->handle = handle;
	atomic_inc(&rk_rga.import_count);

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
		mutex_lock(&session->lock);
		idr_remove(&session->requests, id);
		mutex_unlock(&session->lock);
		rk_rga_request_free(request);
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
		ret = rk_rga_job_submit(job, &release_fence_fd, NULL);
		if (ret) {
			ret = rk_rga_request_ioctl_ret(ret);
		} else if (user.sync_mode == RGA_BLIT_ASYNC) {
			if (release_fence_fd < 0) {
				ret = -EFAULT;
			} else {
				user.release_fence_fd = release_fence_fd;
				if (copy_to_user((void __user *)arg, &user,
						 sizeof(user))) {
					close_fd(release_fence_fd);
					ret = -EFAULT;
				}
			}
		}
		rk_rga_job_put(job);
		return ret;
	}

	return 0;
}

static long rk_rga_ioctl_request_cancel(unsigned long arg,
					struct rk_rga_session *session)
{
	struct rk_rga_request *request;
	__u32 id;

	if (copy_from_user(&id, (void __user *)arg, sizeof(id)))
		return -EFAULT;

	mutex_lock(&session->lock);
	request = idr_remove(&session->requests, id);
	mutex_unlock(&session->lock);
	if (!request)
		return -EINVAL;

	rk_rga_request_free(request);

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
					   hw->match->version_major,
					   hw->match->version_minor,
					   hw->match->version_revision);
			found = true;
			break;
		}
	}
	mutex_unlock(&rk_rga.hw_lock);

	if (!found)
		return -EFAULT;
	if (copy_to_user((void __user *)arg, version.str, sizeof(version.str)))
		return -EFAULT;

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
	struct rga_req *task;
	int release_fence_fd = -1;
	u32 acquire_fd_count = 0;
	u32 fence_count = 0;
	u32 import_count = 0;
	bool close_acquire_fds = false;
	int ret;

	task = memdup_user((void __user *)arg, sizeof(*task));
	if (IS_ERR(task))
		return PTR_ERR(task);

	acquire_fds = kcalloc(RGA_TASK_NUM_MAX + 1, sizeof(*acquire_fds),
			      GFP_KERNEL);
	if (!acquire_fds) {
		kfree(task);
		return -ENOMEM;
	}

	mutex_lock(&session->lock);
	ret = rk_rga_prepare_tasks_locked(session, task, 1, -1,
					  &imports, &import_count,
					  &fences, &fence_count,
					  acquire_fds, &acquire_fd_count);
	if (!ret) {
		ret = rk_rga_job_take_prepared(task, 1, sync_mode, imports,
					       import_count, fences, fence_count,
					       &job);
		if (!ret) {
			close_acquire_fds = true;
			task = NULL;
			imports = NULL;
			fences = NULL;
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
	kfree(task);
	if (ret) {
		kfree(acquire_fds);
		return ret;
	}

	ret = rk_rga_job_submit(job, &release_fence_fd, NULL);
	if (!ret && sync_mode == RGA_BLIT_ASYNC) {
		if (release_fence_fd < 0) {
			ret = -EIO;
		} else {
			job->tasks[0].out_fence_fd = release_fence_fd;
			if (copy_to_user((void __user *)arg, job->tasks,
					 sizeof(*job->tasks))) {
				close_fd(release_fence_fd);
				ret = -EFAULT;
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
};

static const struct rk_rga_hw_match rk_rga3_match = {
	.type = RK_RGA_HW_RGA3,
	.name = "rga3",
	.version_major = 3,
	.version_minor = 0,
	.version_revision = 0x76831,
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

static int rk_rga_hw_probe(struct platform_device *pdev)
{
	const struct rk_rga_hw_match *match;
	struct device *dev = &pdev->dev;
	struct rk_rga_hw *hw;
	struct rk_rga_hw *iter;
	u32 core_index = 0;
	int irq;
	int ret;

	match = of_device_get_match_data(dev);
	if (!match)
		return -EINVAL;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	hw->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hw->regs))
		return PTR_ERR(hw->regs);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;
	hw->irq = irq < 0 ? -1 : irq;

	ret = devm_clk_bulk_get_all(dev, &hw->clks);
	if (ret < 0)
		return ret;
	hw->num_clks = ret;

	hw->resets = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(hw->resets))
		return PTR_ERR(hw->resets);

	hw->dev = dev;
	hw->type = match->type;
	hw->match = match;
	refcount_set(&hw->refs, 1);
	init_waitqueue_head(&hw->idle);
	spin_lock_init(&hw->job_lock);
	mutex_init(&hw->run_lock);
	INIT_DELAYED_WORK(&hw->timeout_work, rk_rga_hw_timeout_work);
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

	if (hw->irq >= 0) {
		ret = devm_request_threaded_irq(dev, hw->irq,
						rk_rga_irq_handler,
						rk_rga_irq_thread,
						IRQF_ONESHOT,
						dev_name(dev), hw);
		if (ret)
			return ret;
	}

	pm_runtime_enable(dev);

	rk_rga_iommu_register_fault_handler(hw);

	mutex_lock(&rk_rga.hw_lock);
	list_for_each_entry(iter, &rk_rga.hw_list, node) {
		if (iter->type == hw->type)
			core_index++;
	}
	hw->core_mask = rk_rga_hw_core_mask(hw->type, core_index);
	hw->index = rk_rga.hw_count;
	list_add_tail(&hw->node, &rk_rga.hw_list);
	rk_rga_refresh_hw_versions_locked();
	mutex_unlock(&rk_rga.hw_lock);

	dev_info(dev, "registered %s core %d mask %#x irq %d clocks %d\n",
		 match->name, hw->index, hw->core_mask, hw->irq,
		 hw->num_clks);

	return 0;
}

static void rk_rga_hw_remove(struct platform_device *pdev)
{
	struct rk_rga_hw *hw = platform_get_drvdata(pdev);

	mutex_lock(&rk_rga.hw_lock);
	hw->removing = true;
	list_del_init(&hw->node);
	rk_rga_refresh_hw_versions_locked();
	mutex_unlock(&rk_rga.hw_lock);

	rk_rga_iommu_unregister_fault_handler(hw);
	rk_rga_hw_abort_jobs(hw, -ENODEV);
	wait_event(hw->idle, refcount_read(&hw->refs) == 1);
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

	mutex_init(&session->lock);
	idr_init(&session->imports);
	idr_init(&session->requests);
	file->private_data = session;

	return nonseekable_open(inode, file);
}

static int rk_rga_release(struct inode *inode, struct file *file)
{
	struct rk_rga_session *session = file->private_data;
	struct rk_rga_import *import;
	struct rk_rga_request *request;
	int id;

	if (!session)
		return 0;

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

static int __init rk_rga_init(void)
{
	int ret;

	mutex_init(&rk_rga.hw_lock);
	spin_lock_init(&rk_rga.fault_lock);
	INIT_LIST_HEAD(&rk_rga.hw_list);
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
	debugfs_create_atomic_t("dispatched_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.dispatched_job_count);
	debugfs_create_atomic_t("started_job_count", 0444,
				rk_rga.debugfs_root,
				&rk_rga.started_job_count);
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
