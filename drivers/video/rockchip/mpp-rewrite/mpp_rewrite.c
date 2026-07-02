// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Rockchip MPP service compatibility rewrite.
 *
 * This is an ABI-first char device for /dev/mpp_service.  It deliberately
 * keeps the BSP userspace ABI while rebuilding the RK3588 execution paths on
 * top of public DMA/IOMMU APIs.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/property.h>
#include <linux/refcount.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <uapi/linux/rk-mpp.h>

#if IS_ENABLED(CONFIG_ROCKCHIP_MPP_REWRITE_KUNIT_TEST)
#include <kunit/test.h>
#endif

#define RK_MPP_REWRITE_VERSION		"rk3588-mpp-rewrite-0.1"
#define RK_MPP_MAX_MSG_NUM		16
#define RK_MPP_MAX_REG_TRANS_NUM	80
#define RK_MPP_MAX_HW_REGS		4
#define RK_MPP_MAX_JOB_PAYLOAD		(128 * 1024)
#define RK_MPP_MAX_REG_IMAGE_BYTES	RK_MPP_MAX_JOB_PAYLOAD
#define RK_MPP_MAX_RCB_ELEMS		16
#define RK_MPP_RKVENC_MAX_RCB_ELEMS	4
#define RK_MPP_RKVENC_MAX_SLICE_FIFO	256
#define RK_MPP_RKVENC_MAX_DCHS_CORES	4
#define RK_MPP_RKVENC_MAX_DCHS_ID	4
#define RK_MPP_RKVDEC_PERF_SEL_NUM	64
#define RK_MPP_WORK_TIMEOUT_MS		500
#define RK_MPP_CODEC_INFO_MAX		11
#define RK_MPP_ENC_INFO_BUTT		RK_MPP_CODEC_INFO_MAX
#define RK_MPP_DEC_INFO_WIDTH		1
#define RK_MPP_DEC_INFO_HEIGHT		2
#define RK_MPP_DEC_INFO_BITDEPTH	4
#define RK_MPP_DEC_INFO_BUTT		6
#define RK_MPP_CODEC_INFO_FLAG_NULL	0
#define RK_MPP_CODEC_INFO_FLAG_BUTT	3

enum rk_mpp_device_type {
	RK_MPP_DEVICE_AV1DEC	= 4,
	RK_MPP_DEVICE_RKVDEC	= 9,
	RK_MPP_DEVICE_RKVENC	= 16,
	RK_MPP_DEVICE_BUTT	= 30,
};

struct rk_mpp_msg_v1 {
	__u32 cmd;
	__u32 flags;
	__u32 size;
	__u32 offset;
	__u64 data_ptr;
};

struct rk_mpp_import {
	struct list_head link;
	int fd;
	struct device *dev;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
	refcount_t refs;
};

struct rk_mpp_rcb_desc {
	u32 index;
	u32 size;
};

struct rk_mpp_codec_info_elem {
	__u32 type;
	__u32 flag;
	__u64 data;
};

struct rk_mpp_codec_info_state {
	u32 flag;
	u64 val;
};

enum rk_mpp_job_state {
	RK_MPP_JOB_STAGED,
	RK_MPP_JOB_ACTIVE,
	RK_MPP_JOB_DONE,
};

struct rk_mpp_backend_ops;
struct rk_mpp_job;

struct rk_mpp_rkvenc_dchs_entry {
	struct rk_mpp_job *job;
	u64 val;
	u8 txid_orig;
	u8 rxid_orig;
};

struct rk_mpp_hw_match {
	const char *name;
	const char *alias;
	enum rk_mpp_device_type type;
	bool contributes_support;
	const struct rk_mpp_backend_ops *ops;
};

struct rk_mpp_hw {
	struct list_head link;
	struct device *dev;
	const struct rk_mpp_hw_match *match;
	void __iomem *regs[RK_MPP_MAX_HW_REGS];
	resource_size_t reg_size[RK_MPP_MAX_HW_REGS];
	struct clk_bulk_data *clks;
	struct reset_control *resets;
	struct delayed_work timeout_work;
	struct mutex run_lock; /* serializes start, abort, timeout, and completion */
	spinlock_t lock;
	struct rk_mpp_job *active_job;
	struct device_node *ccu_node;
	u32 taskqueue_node;
	u32 task_capacity;
	u32 core_mask;
	u32 irq_status;
	u32 hw_id;
	void *rcb_vaddr;
	dma_addr_t rcb_iova;
	size_t rcb_size;
	u32 rcb_min_width;
	refcount_t refs;
	atomic_t queued_job_count;
	struct completion released;
	int num_regs;
	int num_clks;
	int irq;
	int core_id;
	bool online;
};

struct rk_mpp_service {
	struct miscdevice miscdev;
	struct dentry *debugfs_root;
	struct proc_dir_entry *procfs_root;
	struct mutex hw_lock;
	struct mutex sched_lock; /* protects queued_jobs */
	spinlock_t rkvenc_dchs_lock;
	struct list_head hw_list;
	struct list_head queued_jobs;
	struct work_struct sched_work;
	atomic_t ioctl_count;
	atomic_t unsupported_count;
	atomic_t import_count;
	atomic_t submitted_job_count;
	atomic_t queued_job_count;
	atomic_t timeout_count;
	atomic_t next_session_id;
	struct rk_mpp_rkvenc_dchs_entry rkvenc_dchs[RK_MPP_RKVENC_MAX_DCHS_CORES];
	u32 hw_support;
	u32 bound_hw_count;
};

struct rk_mpp_session {
	struct rk_mpp_service *srv;
	struct mutex lock;
	struct list_head imports;
	struct list_head active_jobs;
	wait_queue_head_t wait;
	refcount_t refs;
	u32 client_type;
	u16 trans_table[RK_MPP_MAX_REG_TRANS_NUM];
	u32 trans_count;
	u32 id;
	u32 next_job_id;
	u32 active_job_count;
	struct rk_mpp_rcb_desc rcb_descs[RK_MPP_MAX_RCB_ELEMS];
	u32 rcb_count;
	struct rk_mpp_codec_info_state codec_info[RK_MPP_CODEC_INFO_MAX];
	bool initialized;
};

struct rk_mpp_job_req {
	struct mpp_request req;
	void *payload;
};

struct rk_mpp_reg_offset {
	u32 index;
	u32 offset;
};

struct rk_mpp_reg_image {
	u32 *regs;
	u32 reg_words;
	u32 reg_bytes;
	struct mpp_request read_reqs[RK_MPP_MAX_MSG_NUM];
	u32 read_req_count;
	struct rk_mpp_reg_offset offsets[RK_MPP_MAX_REG_TRANS_NUM];
	u32 offset_count;
	struct rk_mpp_rcb_desc rcb_descs[RK_MPP_MAX_RCB_ELEMS];
	u32 rcb_count;
	u32 rkvdec_perf_sel[RK_MPP_RKVDEC_PERF_SEL_NUM];
	bool translated;
};

struct rk_mpp_trans_table {
	const u16 *regs;
	u32 count;
};

struct rk_mpp_backend_ops {
	int (*validate)(struct rk_mpp_job *job);
	int (*submit)(struct rk_mpp_job *job);
	irqreturn_t (*irq)(struct rk_mpp_hw *hw);
	irqreturn_t (*thread)(struct rk_mpp_hw *hw);
};

struct rk_mpp_job {
	struct list_head link;
	struct list_head session_link;
	struct list_head sched_link;
	struct rk_mpp_session *session;
	enum rk_mpp_job_state state;
	struct rk_mpp_hw *hw;
	refcount_t refs;
	u32 id;
	u32 req_cnt;
	u32 set_cnt;
	u32 poll_cnt;
	u32 flags;
	int result;
	u32 rkvdec_stream_addr;
	u32 rkvenc_dchs_core_id;
	bool poll_irq;
	bool canceled;
	bool rkvenc_dchs_active;
	bool rkvenc_slice_mode;
	bool rkvenc_slice_done;
	bool rkvenc_slice_overflow;
	spinlock_t rkvenc_slice_lock;
	DECLARE_KFIFO(rkvenc_slice_fifo, u32, RK_MPP_RKVENC_MAX_SLICE_FIFO);
	struct mpp_request poll_req;
	struct rk_mpp_reg_image reg_image;
	struct rk_mpp_import *imports[RK_MPP_MAX_REG_TRANS_NUM];
	u32 import_count;
	struct rk_mpp_job_req reqs[RK_MPP_MAX_MSG_NUM];
};

struct rk_mpp_batch_state {
	struct list_head jobs;
	struct rk_mpp_job *cur_job;
	u32 req_cnt;
};

static const struct file_operations rk_mpp_fops;
static const struct rk_mpp_backend_ops rk_mpp_unsupported_backend_ops;
static const struct rk_mpp_backend_ops rk_mpp_rkvenc2_backend_ops;
static const struct rk_mpp_backend_ops rk_mpp_rkvdec2_backend_ops;
static void rk_mpp_job_activate(struct rk_mpp_job *job);
static void rk_mpp_job_get(struct rk_mpp_job *job);
static void rk_mpp_job_put(struct rk_mpp_job *job);
static bool rk_mpp_job_rkvenc_slice_mode(struct rk_mpp_job *job);
static bool rk_mpp_job_rkvenc_slice_ready(struct rk_mpp_job *job);
static bool rk_mpp_job_rkvenc_slice_done(struct rk_mpp_job *job);
static struct rk_mpp_service rk_mpp_srv;

static const struct rk_mpp_hw_match rk_mpp_rkvenc2_core = {
	.name = "rkvenc2",
	.alias = "rkvenc",
	.type = RK_MPP_DEVICE_RKVENC,
	.contributes_support = true,
	.ops = &rk_mpp_rkvenc2_backend_ops,
};

static const struct rk_mpp_hw_match rk_mpp_rkvdec2_core = {
	.name = "rkvdec2",
	.alias = "rkvdec",
	.type = RK_MPP_DEVICE_RKVDEC,
	.contributes_support = true,
	.ops = &rk_mpp_rkvdec2_backend_ops,
};

static const struct rk_mpp_hw_match rk_mpp_rkvenc2_ccu = {
	.name = "rkvenc2-ccu",
	.type = RK_MPP_DEVICE_BUTT,
};

static const struct rk_mpp_hw_match rk_mpp_rkvdec2_ccu = {
	.name = "rkvdec2-ccu",
	.type = RK_MPP_DEVICE_BUTT,
};

static const struct of_device_id rk_mpp_hw_of_match[] = {
	{ .compatible = "rockchip,rkv-encoder-v2-core", .data = &rk_mpp_rkvenc2_core },
	{ .compatible = "rockchip,rkv-decoder-v2", .data = &rk_mpp_rkvdec2_core },
	{ .compatible = "rockchip,rkv-encoder-v2-ccu", .data = &rk_mpp_rkvenc2_ccu },
	{ .compatible = "rockchip,rkv-decoder-v2-ccu", .data = &rk_mpp_rkvdec2_ccu },
	{ }
};
MODULE_DEVICE_TABLE(of, rk_mpp_hw_of_match);

#define RK_MPP_RKVDEC_REG_FMT			9
#define RK_MPP_RKVDEC_EN_MODE_WORD		13
#define RK_MPP_RKVDEC_CORE_CTRL_WORD		28
#define RK_MPP_RKVDEC_TIMEOUT_THRESHOLD_WORD	32
#define RK_MPP_RKVDEC_FILM_IDX_MASK		GENMASK(25, 16)
#define RK_MPP_RKVDEC_FILM_IDX_SHIFT		16
#define RK_MPP_RKVDEC_CCU_TIMEOUT_DISABLE	BIT(1)
#define RK_MPP_RKVENC_PIC_BASE_WORDS		(0x0280 / sizeof(u32))
#define RK_MPP_RKVENC_OSD_BASE_WORDS		(0x3000 / sizeof(u32))
#define RK_MPP_RKVENC_ENC_PIC_WORD		(RK_MPP_RKVENC_PIC_BASE_WORDS + 32)
#define RK_MPP_RKVENC_SLI_SPLIT_WORD		(RK_MPP_RKVENC_PIC_BASE_WORDS + 56)
#define RK_MPP_RKVENC_FMT_WORD			(0x0300 / sizeof(u32))
#define RK_MPP_RKVENC_FMT_MASK			0x1
#define RK_MPP_RKVENC_DCHS_WORD		(0x0304 / sizeof(u32))
#define RK_MPP_RKVENC_START_BASE		0x0010
#define RK_MPP_RKVENC_CLR_BASE			0x0014
#define RK_MPP_RKVENC_INT_MASK_BASE		0x0024
#define RK_MPP_RKVENC_INT_CLR_BASE		0x0028
#define RK_MPP_RKVENC_INT_STA_BASE		0x002c
#define RK_MPP_RKVENC_COUNTER_CLR_BASE		0x5300
#define RK_MPP_RKVENC_SLICE_NUM_BASE		0x4034
#define RK_MPP_RKVENC_SLICE_LEN_BASE		0x4038
#define RK_MPP_RKVENC_INT_DONE			BIT(0)
#define RK_MPP_RKVENC_INT_SLICE_DONE		BIT(3)
#define RK_MPP_RKVENC_INT_ERROR			(BIT(5) | BIT(6) | BIT(7) | BIT(8))
#define RK_MPP_RKVENC_INT_WATCHDOG		BIT(8)
#define RK_MPP_RKVENC_ENC_PIC_SLEN_FIFO	BIT(30)
#define RK_MPP_RKVENC_SLI_SPLIT_EN		BIT(0)
#define RK_MPP_RKVENC_SLICE_NUM_MASK		GENMASK(5, 0)
#define RK_MPP_RKVENC_SLICE_LAST		BIT(31)
#define RK_MPP_RKVENC_DCHS_TXID_MASK		GENMASK(1, 0)
#define RK_MPP_RKVENC_DCHS_RXID_MASK		GENMASK(3, 2)
#define RK_MPP_RKVENC_DCHS_TXID_SHIFT		0
#define RK_MPP_RKVENC_DCHS_RXID_SHIFT		2
#define RK_MPP_RKVENC_DCHS_TXE			BIT(4)
#define RK_MPP_RKVENC_DCHS_RXE			BIT(5)

struct rk_mpp_rkvenc_poll_slice_cfg {
	s32 poll_type;
	s32 poll_ret;
	s32 count_max;
	s32 count_ret;
};

#define RK_MPP_RKVDEC_START_BASE		0x0028
#define RK_MPP_RKVDEC_START_EN			BIT(0)
#define RK_MPP_RKVDEC_REG_EN_WORD		(RK_MPP_RKVDEC_START_BASE / sizeof(u32))
#define RK_MPP_RKVDEC_RLC_BASE			0x0200
#define RK_MPP_RKVDEC_RLC_WORD			(RK_MPP_RKVDEC_RLC_BASE / sizeof(u32))
#define RK_MPP_RKVDEC_INT_STA_BASE		0x0380
#define RK_MPP_RKVDEC_INT_STA_WORD		(RK_MPP_RKVDEC_INT_STA_BASE / sizeof(u32))
#define RK_MPP_RKVDEC_IRQ_RAW			BIT(1)
#define RK_MPP_RKVDEC_PERF_SEL_OFFSET		0x20000
#define RK_MPP_RKVDEC_PERF_SEL_BASE		0x0424
#define RK_MPP_RKVDEC_SEL_VAL0_BASE		0x0428
#define RK_MPP_RKVDEC_SEL_VAL1_BASE		0x042c
#define RK_MPP_RKVDEC_SEL_VAL2_BASE		0x0430
#define RK_MPP_RKVDEC_SET_PERF_SEL(a, b, c)	((a) | ((b) << 8) | ((c) << 16))
#define RK_MPP_RKVDEC_1080P_PIXELS		(1920 * 1080)
#define RK_MPP_RKVDEC_4K_PIXELS		(4096 * 2304)
#define RK_MPP_RKVDEC_CCU_TIMEOUT_20MS		0x00efffff
#define RK_MPP_RKVDEC_CCU_TIMEOUT_50MS		0x02cfffff
#define RK_MPP_RKVDEC_CCU_TIMEOUT_100MS		0x04ffffff
#define RK_MPP_RKVDEC_CACHE0_SIZE_BASE		0x051c
#define RK_MPP_RKVDEC_CACHE1_SIZE_BASE		0x055c
#define RK_MPP_RKVDEC_CACHE2_SIZE_BASE		0x059c
#define RK_MPP_RKVDEC_CLR_CACHE0_BASE		0x0510
#define RK_MPP_RKVDEC_CLR_CACHE1_BASE		0x0550
#define RK_MPP_RKVDEC_CLR_CACHE2_BASE		0x0590
#define RK_MPP_RKVDEC_CACHE_CFG			(BIT(0) | BIT(1) | BIT(4))

enum rk_mpp_rkvdec_fmt {
	RK_MPP_RKVDEC_FMT_H265D	= 0,
	RK_MPP_RKVDEC_FMT_H264D	= 1,
	RK_MPP_RKVDEC_FMT_VP9D	= 2,
	RK_MPP_RKVDEC_FMT_AVS2	= 3,
};

enum rk_mpp_rkvenc_fmt {
	RK_MPP_RKVENC_FMT_H264E	= 0,
	RK_MPP_RKVENC_FMT_H265E	= 1,
};

static const u16 rk_mpp_rkvdec_h264d_regs[] = {
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
	141, 142, 161, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198,
	199,
};

static const u16 rk_mpp_rkvdec_h265d_regs[] = {
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
	141, 142, 161, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198,
	199,
};

static const u16 rk_mpp_rkvdec_vp9d_regs[] = {
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
	141, 142, 160, 162, 164, 165, 166, 167, 168, 169, 170, 171, 172,
	180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192,
	193, 194, 195, 196, 197, 198, 199,
};

static const u16 rk_mpp_rkvdec_avs2d_regs[] = {
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
	141, 142, 161, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198,
	199,
};

static const struct rk_mpp_trans_table rk_mpp_rkvdec_tables[] = {
	[RK_MPP_RKVDEC_FMT_H265D] = {
		.regs = rk_mpp_rkvdec_h265d_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvdec_h265d_regs),
	},
	[RK_MPP_RKVDEC_FMT_H264D] = {
		.regs = rk_mpp_rkvdec_h264d_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvdec_h264d_regs),
	},
	[RK_MPP_RKVDEC_FMT_VP9D] = {
		.regs = rk_mpp_rkvdec_vp9d_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvdec_vp9d_regs),
	},
	[RK_MPP_RKVDEC_FMT_AVS2] = {
		.regs = rk_mpp_rkvdec_avs2d_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvdec_avs2d_regs),
	},
};

static const u16 rk_mpp_rkvenc_pic_regs[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
	12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
};

static const u16 rk_mpp_rkvenc_osd_regs[] = {
	20, 21, 22, 23, 24, 25, 26, 27,
};

static const struct rk_mpp_trans_table rk_mpp_rkvenc_pic_tables[] = {
	[RK_MPP_RKVENC_FMT_H264E] = {
		.regs = rk_mpp_rkvenc_pic_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvenc_pic_regs),
	},
	[RK_MPP_RKVENC_FMT_H265E] = {
		.regs = rk_mpp_rkvenc_pic_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvenc_pic_regs),
	},
};

static const struct rk_mpp_trans_table rk_mpp_rkvenc_osd_tables[] = {
	[RK_MPP_RKVENC_FMT_H264E] = {
		.regs = rk_mpp_rkvenc_osd_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvenc_osd_regs),
	},
	[RK_MPP_RKVENC_FMT_H265E] = {
		.regs = rk_mpp_rkvenc_osd_regs,
		.count = ARRAY_SIZE(rk_mpp_rkvenc_osd_regs),
	},
};

static void rk_mpp_session_get(struct rk_mpp_session *session)
{
	refcount_inc(&session->refs);
}

static void rk_mpp_session_put(struct rk_mpp_session *session)
{
	if (session && refcount_dec_and_test(&session->refs))
		kfree(session);
}

static bool rk_mpp_hw_ccu_online_locked(struct rk_mpp_service *srv,
					struct rk_mpp_hw *core)
{
	struct rk_mpp_hw *hw;

	if (!core->ccu_node)
		return true;

	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->dev->of_node == core->ccu_node && hw->online)
			return true;
	}

	return false;
}

static bool rk_mpp_hw_ccu_online(struct rk_mpp_service *srv,
				 struct rk_mpp_hw *core)
{
	bool online;

	mutex_lock(&srv->hw_lock);
	online = rk_mpp_hw_ccu_online_locked(srv, core);
	mutex_unlock(&srv->hw_lock);

	return online;
}

static void rk_mpp_refresh_hw_support_locked(struct rk_mpp_service *srv)
{
	struct rk_mpp_hw *hw;
	u32 support = 0;
	u32 count = 0;

	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->match->contributes_support &&
		    rk_mpp_hw_ccu_online_locked(srv, hw) &&
		    hw->match->type < RK_MPP_DEVICE_BUTT) {
			support |= BIT(hw->match->type);
			count++;
		}
	}

	srv->hw_support = support;
	srv->bound_hw_count = count;
}

static u32 rk_mpp_get_hw_support(struct rk_mpp_service *srv)
{
	u32 support;

	mutex_lock(&srv->hw_lock);
	support = srv->hw_support;
	mutex_unlock(&srv->hw_lock);

	return support;
}

static void rk_mpp_hw_put(struct rk_mpp_hw *hw)
{
	if (hw && refcount_dec_and_test(&hw->refs))
		complete(&hw->released);
}

static bool rk_mpp_hw_is_idle(struct rk_mpp_hw *hw)
{
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&hw->lock, flags);
	idle = !hw->active_job;
	spin_unlock_irqrestore(&hw->lock, flags);

	return idle;
}

static u32 rk_mpp_hw_load(struct rk_mpp_hw *hw)
{
	return (rk_mpp_hw_is_idle(hw) ? 0 : 1) +
	       atomic_read(&hw->queued_job_count);
}

static struct rk_mpp_hw *rk_mpp_hw_get_for_session(struct rk_mpp_session *session,
						   bool prefer_idle)
{
	struct rk_mpp_service *srv = session->srv;
	struct rk_mpp_hw *hw;
	struct rk_mpp_hw *selected = NULL;
	u32 selected_load = U32_MAX;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		u32 load;

		if (!hw->online || !hw->match->contributes_support ||
		    hw->match->type != session->client_type)
			continue;
		if (!rk_mpp_hw_ccu_online_locked(srv, hw))
			continue;

		if (!prefer_idle) {
			selected = hw;
			break;
		}

		load = rk_mpp_hw_load(hw);
		if (!selected || load < selected_load) {
			selected = hw;
			selected_load = load;
			if (!load)
				break;
		}
	}
	if (selected)
		refcount_inc(&selected->refs);
	mutex_unlock(&srv->hw_lock);

	return selected;
}

static u32 rk_mpp_get_hw_id(struct rk_mpp_service *srv, u32 client_type)
{
	struct rk_mpp_hw *hw;
	u32 hw_id = 0;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->match->contributes_support &&
		    rk_mpp_hw_ccu_online_locked(srv, hw) &&
		    hw->match->type == client_type) {
			hw_id = hw->hw_id;
			break;
		}
	}
	mutex_unlock(&srv->hw_lock);

	return hw_id;
}

static int rk_mpp_next_core_id_locked(struct rk_mpp_service *srv,
				      const struct rk_mpp_hw_match *match)
{
	struct rk_mpp_hw *hw;
	int count = 0;

	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->match->type == match->type)
			count++;
	}

	return count;
}

static struct device *rk_mpp_get_map_dev(struct rk_mpp_session *session)
{
	struct rk_mpp_hw *hw;
	struct device *dev = NULL;

	hw = rk_mpp_hw_get_for_session(session, false);
	if (!hw)
		return NULL;

	dev = get_device(hw->dev);
	rk_mpp_hw_put(hw);

	return dev;
}

static int rk_mpp_check_cmd_v1(__u32 cmd)
{
	bool found;

	found = cmd < MPP_CMD_QUERY_BUTT;
	found = (cmd >= MPP_CMD_INIT_BASE && cmd < MPP_CMD_INIT_BUTT) ? true : found;
	found = (cmd >= MPP_CMD_SEND_BASE && cmd < MPP_CMD_SEND_BUTT) ? true : found;
	found = (cmd >= MPP_CMD_POLL_BASE && cmd < MPP_CMD_POLL_BUTT) ? true : found;
	found = (cmd >= MPP_CMD_CONTROL_BASE && cmd < MPP_CMD_CONTROL_BUTT) ? true : found;

	return found ? 0 : -EINVAL;
}

static __u32 rk_mpp_get_cmd_butt(__u32 cmd)
{
	switch (cmd) {
	case MPP_CMD_QUERY_BASE:
		return MPP_CMD_QUERY_BUTT;
	case MPP_CMD_INIT_BASE:
		return MPP_CMD_INIT_BUTT;
	case MPP_CMD_SEND_BASE:
		return MPP_CMD_SEND_BUTT;
	case MPP_CMD_POLL_BASE:
		return MPP_CMD_POLL_BUTT;
	case MPP_CMD_CONTROL_BASE:
		return MPP_CMD_CONTROL_BUTT;
	default:
		return 0;
	}
}

static int rk_mpp_support_cmd_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "QUERY_HW_SUPPORT\n");
	seq_puts(s, "QUERY_HW_ID\n");
	seq_puts(s, "QUERY_CMD_SUPPORT\n");
	seq_puts(s, "QUERY_BUTT\n");
	seq_puts(s, "INIT_CLIENT_TYPE\n");
	seq_puts(s, "INIT_DRIVER_DATA\n");
	seq_puts(s, "INIT_TRANS_TABLE\n");
	seq_puts(s, "INIT_BUTT\n");
	seq_puts(s, "SET_REG_WRITE\n");
	seq_puts(s, "SET_REG_READ\n");
	seq_puts(s, "SET_REG_ADDR_OFFSET\n");
	seq_puts(s, "SET_RCB_INFO\n");
	seq_puts(s, "SEND_BUTT\n");
	seq_puts(s, "POLL_HW_FINISH\n");
	seq_puts(s, "POLL_HW_IRQ\n");
	seq_puts(s, "POLL_BUTT\n");
	seq_puts(s, "RESET_SESSION\n");
	seq_puts(s, "TRANS_FD_TO_IOVA\n");
	seq_puts(s, "RELEASE_FD\n");
	seq_puts(s, "SEND_CODEC_INFO\n");
	seq_puts(s, "SET_ERR_REF_HACK\n");
	seq_puts(s, "CONTROL_BUTT\n");

	return 0;
}

static int rk_mpp_create_procfs(struct rk_mpp_service *srv)
{
	srv->procfs_root = proc_mkdir("mpp_service", NULL);
	if (IS_ERR_OR_NULL(srv->procfs_root)) {
		int ret = srv->procfs_root ? PTR_ERR(srv->procfs_root) : -ENOMEM;

		srv->procfs_root = NULL;
		return ret;
	}

	if (!proc_create_single("supports-cmd", 0444, srv->procfs_root,
				rk_mpp_support_cmd_show) ||
	    !proc_create_single("support_cmd", 0444, srv->procfs_root,
				rk_mpp_support_cmd_show)) {
		proc_remove(srv->procfs_root);
		srv->procfs_root = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void rk_mpp_remove_procfs(struct rk_mpp_service *srv)
{
	proc_remove(srv->procfs_root);
	srv->procfs_root = NULL;
}

static void rk_mpp_import_release(struct rk_mpp_import *import)
{
	if (import->sgt)
		dma_buf_unmap_attachment(import->attach, import->sgt,
					 DMA_BIDIRECTIONAL);
	if (import->attach)
		dma_buf_detach(import->dmabuf, import->attach);
	if (import->dmabuf)
		dma_buf_put(import->dmabuf);
	if (import->dev)
		put_device(import->dev);
	kfree(import);
}

static void rk_mpp_import_put(struct rk_mpp_import *import)
{
	if (import && refcount_dec_and_test(&import->refs))
		rk_mpp_import_release(import);
}

static void rk_mpp_session_release_imports(struct rk_mpp_session *session)
{
	struct rk_mpp_import *import, *tmp;

	mutex_lock(&session->lock);
	list_for_each_entry_safe(import, tmp, &session->imports, link) {
		list_del_init(&import->link);
		rk_mpp_import_put(import);
	}
	mutex_unlock(&session->lock);
}

static struct rk_mpp_import *
rk_mpp_find_import_locked(struct rk_mpp_session *session, int fd,
			  struct device *dev)
{
	struct rk_mpp_import *import;

	list_for_each_entry(import, &session->imports, link) {
		if (import->fd == fd && import->dev == dev)
			return import;
	}

	return NULL;
}

static struct rk_mpp_import *rk_mpp_import_fd(struct rk_mpp_session *session,
					      int fd, struct device *map_dev)
{
	struct rk_mpp_import *import;
	struct rk_mpp_import *existing;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct device *dev;

	if (map_dev)
		dev = get_device(map_dev);
	else
		dev = rk_mpp_get_map_dev(session);
	if (!dev)
		return ERR_PTR(-ENODEV);

	mutex_lock(&session->lock);
	import = rk_mpp_find_import_locked(session, fd, dev);
	if (import) {
		refcount_inc(&import->refs);
		mutex_unlock(&session->lock);
		put_device(dev);
		return import;
	}
	mutex_unlock(&session->lock);

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		put_device(dev);
		return ERR_CAST(dmabuf);
	}

	attach = dma_buf_attach(dmabuf, dev);
	if (IS_ERR(attach)) {
		dma_buf_put(dmabuf);
		put_device(dev);
		return ERR_CAST(attach);
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return ERR_CAST(sgt);
	}

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	if (!import) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, attach);
		dma_buf_put(dmabuf);
		put_device(dev);
		return ERR_PTR(-ENOMEM);
	}

	import->fd = fd;
	import->dev = dev;
	import->dmabuf = dmabuf;
	import->attach = attach;
	import->sgt = sgt;
	import->iova = sg_dma_address(sgt->sgl);
	refcount_set(&import->refs, 2);
	INIT_LIST_HEAD(&import->link);

	mutex_lock(&session->lock);
	existing = rk_mpp_find_import_locked(session, fd, dev);
	if (existing) {
		refcount_inc(&existing->refs);
		mutex_unlock(&session->lock);
		rk_mpp_import_release(import);
		return existing;
	}
	list_add_tail(&import->link, &session->imports);
	mutex_unlock(&session->lock);
	atomic_inc(&session->srv->import_count);

	return import;
}

static int rk_mpp_release_fd(struct rk_mpp_session *session, int fd)
{
	struct rk_mpp_import *import, *tmp;
	LIST_HEAD(release_list);
	bool found = false;

	mutex_lock(&session->lock);
	list_for_each_entry_safe(import, tmp, &session->imports, link) {
		if (import->fd != fd)
			continue;
		list_move_tail(&import->link, &release_list);
		found = true;
	}
	mutex_unlock(&session->lock);

	if (!found)
		return -EINVAL;

	list_for_each_entry_safe(import, tmp, &release_list, link) {
		list_del_init(&import->link);
		rk_mpp_import_put(import);
	}

	return 0;
}

static int rk_mpp_trans_fd_to_iova(struct rk_mpp_session *session,
				   struct mpp_request *req)
{
	u32 data[RK_MPP_MAX_REG_TRANS_NUM];
	u32 count;
	u32 i;

	if (!session->initialized)
		return -EINVAL;
	if (!req->size || req->size > sizeof(data))
		return -EINVAL;

	memset(data, 0, sizeof(data));
	if (copy_from_user(data, req->data, req->size))
		return -EINVAL;

	count = req->size / sizeof(u32);
	for (i = 0; i < count; i++) {
		struct rk_mpp_import *import;

		import = rk_mpp_import_fd(session, data[i], NULL);
		if (IS_ERR(import))
			return -EINVAL;
		data[i] = lower_32_bits(import->iova);
		rk_mpp_import_put(import);
	}

	if (copy_to_user(req->data, data, req->size))
		return -EINVAL;

	return 0;
}

static int rk_mpp_release_fds(struct rk_mpp_session *session,
			      struct mpp_request *req)
{
	u32 data[RK_MPP_MAX_REG_TRANS_NUM];
	u32 count;
	u32 i;
	int ret;

	if (!req->size || req->size > sizeof(data))
		return -EINVAL;

	memset(data, 0, sizeof(data));
	if (copy_from_user(data, req->data, req->size))
		return -EINVAL;

	count = req->size / sizeof(u32);
	for (i = 0; i < count; i++) {
		ret = rk_mpp_release_fd(session, data[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int rk_mpp_copy_in_discard(struct mpp_request *req)
{
	u8 stack_buf[128];
	void *buf = stack_buf;
	int ret = 0;

	if (!req->size)
		return 0;
	if (req->size > PAGE_SIZE)
		return -ENOMEM;
	if (req->size > sizeof(stack_buf)) {
		buf = kmalloc(req->size, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
	}

	if (copy_from_user(buf, req->data, req->size))
		ret = -EINVAL;

	if (buf != stack_buf)
		kfree(buf);

	return ret;
}

static u32 rk_mpp_session_rcb_limit(const struct rk_mpp_session *session)
{
	if (session->client_type == RK_MPP_DEVICE_RKVENC)
		return RK_MPP_RKVENC_MAX_RCB_ELEMS;

	return RK_MPP_MAX_RCB_ELEMS;
}

static u32 rk_mpp_session_codec_info_limit(const struct rk_mpp_session *session)
{
	switch (session->client_type) {
	case RK_MPP_DEVICE_RKVDEC:
		return RK_MPP_DEC_INFO_BUTT;
	case RK_MPP_DEVICE_RKVENC:
		return RK_MPP_ENC_INFO_BUTT;
	default:
		return RK_MPP_CODEC_INFO_MAX;
	}
}

static int rk_mpp_store_codec_info(struct rk_mpp_session *session,
				   const struct mpp_request *req)
{
	struct rk_mpp_codec_info_elem elems[RK_MPP_CODEC_INFO_MAX];
	u32 limit = rk_mpp_session_codec_info_limit(session);
	u32 count;
	u32 i;

	if (!req->size)
		return 0;
	if (!req->data)
		return -EINVAL;

	count = req->size / sizeof(elems[0]);
	if (count > ARRAY_SIZE(elems))
		count = ARRAY_SIZE(elems);
	if (count > limit)
		count = limit;
	if (!count)
		return 0;

	if (copy_from_user(elems, req->data, count * sizeof(elems[0])))
		return -EINVAL;

	mutex_lock(&session->lock);
	for (i = 0; i < count; i++) {
		u32 type = elems[i].type;
		u32 flag = elems[i].flag;

		if (type > 0 && type < limit &&
		    flag > RK_MPP_CODEC_INFO_FLAG_NULL &&
		    flag < RK_MPP_CODEC_INFO_FLAG_BUTT) {
			session->codec_info[type].flag = flag;
			session->codec_info[type].val = elems[i].data;
		}
	}
	mutex_unlock(&session->lock);

	return 0;
}

static bool rk_mpp_cmd_copies_payload(u32 cmd)
{
	switch (cmd) {
	case MPP_CMD_SET_REG_WRITE:
	case MPP_CMD_SET_REG_ADDR_OFFSET:
	case MPP_CMD_SET_RCB_INFO:
		return true;
	default:
		return false;
	}
}

static int rk_mpp_request_check_reg_span(const struct mpp_request *req)
{
	if (!req->size)
		return 0;
	if (req->offset > RK_MPP_MAX_REG_IMAGE_BYTES ||
	    req->size > RK_MPP_MAX_REG_IMAGE_BYTES - req->offset)
		return -ENOMEM;

	return 0;
}

static bool rk_mpp_job_is_rkvdec_perf_read(struct rk_mpp_job *job,
					   const struct mpp_request *req)
{
	return job->session->client_type == RK_MPP_DEVICE_RKVDEC &&
	       req->offset >= RK_MPP_RKVDEC_PERF_SEL_OFFSET;
}

static int rk_mpp_request_check_rkvdec_perf_span(const struct mpp_request *req)
{
	u32 max_size = RK_MPP_RKVDEC_PERF_SEL_NUM * sizeof(u32);
	u32 offset;

	if (!req->size)
		return 0;
	if (req->offset < RK_MPP_RKVDEC_PERF_SEL_OFFSET)
		return -EINVAL;
	if (req->offset % sizeof(u32) || req->size % sizeof(u32))
		return -EINVAL;

	offset = req->offset - RK_MPP_RKVDEC_PERF_SEL_OFFSET;
	if (offset > max_size || req->size > max_size - offset)
		return -ENOMEM;

	return 0;
}

static u32 rk_mpp_rkvdec2_ccu_timeout_threshold(u32 width, u32 height,
						u32 bitdepth)
{
	u64 adjusted_width = width;
	u64 pixels;

	if (bitdepth > 8)
		adjusted_width = DIV_ROUND_UP_ULL((u64)width * bitdepth, 8);

	pixels = adjusted_width * height;
	if (pixels < RK_MPP_RKVDEC_1080P_PIXELS)
		return RK_MPP_RKVDEC_CCU_TIMEOUT_20MS;
	if (pixels < RK_MPP_RKVDEC_4K_PIXELS)
		return RK_MPP_RKVDEC_CCU_TIMEOUT_50MS;

	return RK_MPP_RKVDEC_CCU_TIMEOUT_100MS;
}

static int rk_mpp_poll_irq_check_size(s32 count_max, u32 req_size)
{
	size_t slice_bytes;
	size_t needed;

	if (req_size < sizeof(struct rk_mpp_rkvenc_poll_slice_cfg))
		return -EINVAL;
	if (count_max <= 0)
		return -EINVAL;
	if (check_mul_overflow((size_t)count_max, sizeof(u32),
			       &slice_bytes) ||
	    check_add_overflow(sizeof(struct rk_mpp_rkvenc_poll_slice_cfg),
			       slice_bytes, &needed) ||
	    req_size < needed)
		return -EINVAL;

	return 0;
}

#if IS_ENABLED(CONFIG_ROCKCHIP_MPP_REWRITE_KUNIT_TEST)
static void rk_mpp_check_cmd_v1_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_QUERY_HW_SUPPORT), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_INIT_CLIENT_TYPE), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_SET_REG_WRITE), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_POLL_HW_FINISH), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_POLL_HW_IRQ), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_RESET_SESSION), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_SET_ERR_REF_HACK), 0);

	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_QUERY_BUTT), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_INIT_BUTT), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_SEND_BUTT), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_POLL_BUTT), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(MPP_CMD_CONTROL_BUTT), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_check_cmd_v1(U32_MAX), -EINVAL);
}

static void rk_mpp_get_cmd_butt_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_QUERY_BASE),
			(__u32)MPP_CMD_QUERY_BUTT);
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_INIT_BASE),
			(__u32)MPP_CMD_INIT_BUTT);
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_SEND_BASE),
			(__u32)MPP_CMD_SEND_BUTT);
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_POLL_BASE),
			(__u32)MPP_CMD_POLL_BUTT);
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_CONTROL_BASE),
			(__u32)MPP_CMD_CONTROL_BUTT);
	KUNIT_EXPECT_EQ(test, rk_mpp_get_cmd_butt(MPP_CMD_QUERY_HW_SUPPORT),
			(__u32)0);
}

static void rk_mpp_cmd_copies_payload_kunit(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
			  rk_mpp_cmd_copies_payload(MPP_CMD_SET_REG_WRITE));
	KUNIT_EXPECT_TRUE(test,
			  rk_mpp_cmd_copies_payload(MPP_CMD_SET_REG_ADDR_OFFSET));
	KUNIT_EXPECT_TRUE(test,
			  rk_mpp_cmd_copies_payload(MPP_CMD_SET_RCB_INFO));

	KUNIT_EXPECT_FALSE(test,
			   rk_mpp_cmd_copies_payload(MPP_CMD_SET_REG_READ));
	KUNIT_EXPECT_FALSE(test,
			   rk_mpp_cmd_copies_payload(MPP_CMD_POLL_HW_FINISH));
	KUNIT_EXPECT_FALSE(test,
			   rk_mpp_cmd_copies_payload(MPP_CMD_TRANS_FD_TO_IOVA));
}

static void rk_mpp_request_check_reg_span_kunit(struct kunit *test)
{
	struct mpp_request req = {};

	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), 0);

	req.offset = RK_MPP_MAX_REG_IMAGE_BYTES + 1;
	req.size = 0;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), 0);

	req.offset = 0;
	req.size = RK_MPP_MAX_REG_IMAGE_BYTES;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), 0);

	req.offset = RK_MPP_MAX_REG_IMAGE_BYTES - 4;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), 0);

	req.offset = RK_MPP_MAX_REG_IMAGE_BYTES - 4;
	req.size = 5;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), -ENOMEM);

	req.offset = RK_MPP_MAX_REG_IMAGE_BYTES;
	req.size = 1;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), -ENOMEM);

	req.offset = RK_MPP_MAX_REG_IMAGE_BYTES + 1;
	req.size = 1;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_reg_span(&req), -ENOMEM);
}

static void rk_mpp_request_check_rkvdec_perf_span_kunit(struct kunit *test)
{
	struct mpp_request req = {};
	u32 max_size = RK_MPP_RKVDEC_PERF_SEL_NUM * sizeof(u32);

	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req), 0);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET - 4;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req),
			-EINVAL);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req), 0);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET + max_size - 4;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req), 0);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET + max_size;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req),
			-ENOMEM);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET + 1;
	req.size = 4;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req),
			-EINVAL);

	req.offset = RK_MPP_RKVDEC_PERF_SEL_OFFSET;
	req.size = 5;
	KUNIT_EXPECT_EQ(test, rk_mpp_request_check_rkvdec_perf_span(&req),
			-EINVAL);
}

static void rk_mpp_rkvdec2_ccu_timeout_threshold_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(0, 0, 0),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_20MS);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(1919, 1080, 8),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_20MS);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(1920, 1080, 8),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_50MS);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(4095, 2304, 8),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_50MS);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(4096, 2304, 8),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_100MS);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_timeout_threshold(1280, 720, 10),
			(u32)RK_MPP_RKVDEC_CCU_TIMEOUT_20MS);
}

static void rk_mpp_poll_irq_check_size_kunit(struct kunit *test)
{
	u32 base = sizeof(struct rk_mpp_rkvenc_poll_slice_cfg);

	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(1, base + 4), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(8, base + 32), 0);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(1, base + 8), 0);

	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(0, base), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(-1, base), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(1, base - 1), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(1, base), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_poll_irq_check_size(S32_MAX, base),
			-EINVAL);
}

static void rk_mpp_rkvenc_slice_mode_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {
		.client_type = RK_MPP_DEVICE_RKVENC,
	};
	u32 regs[RK_MPP_RKVENC_SLI_SPLIT_WORD + 1] = {};
	struct rk_mpp_job job = {
		.session = &session,
		.reg_image = {
			.regs = regs,
			.reg_words = ARRAY_SIZE(regs),
		},
	};

	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_mode(&job));

	regs[RK_MPP_RKVENC_ENC_PIC_WORD] = RK_MPP_RKVENC_ENC_PIC_SLEN_FIFO;
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_mode(&job));

	regs[RK_MPP_RKVENC_SLI_SPLIT_WORD] = RK_MPP_RKVENC_SLI_SPLIT_EN;
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_mode(&job));

	session.client_type = RK_MPP_DEVICE_RKVDEC;
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_mode(&job));
}

static struct kunit_case rk_mpp_rewrite_test_cases[] = {
	KUNIT_CASE(rk_mpp_check_cmd_v1_kunit),
	KUNIT_CASE(rk_mpp_get_cmd_butt_kunit),
	KUNIT_CASE(rk_mpp_cmd_copies_payload_kunit),
	KUNIT_CASE(rk_mpp_request_check_reg_span_kunit),
	KUNIT_CASE(rk_mpp_request_check_rkvdec_perf_span_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_timeout_threshold_kunit),
	KUNIT_CASE(rk_mpp_poll_irq_check_size_kunit),
	KUNIT_CASE(rk_mpp_rkvenc_slice_mode_kunit),
	{}
};

static struct kunit_suite rk_mpp_rewrite_test_suite = {
	.name = "rk_mpp_rewrite",
	.test_cases = rk_mpp_rewrite_test_cases,
};

kunit_test_suite(rk_mpp_rewrite_test_suite);
#endif

static int rk_mpp_job_ensure_reg_bytes(struct rk_mpp_job *job, u32 reg_bytes)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	u32 old_words = image->reg_words;
	u32 new_words;
	size_t old_bytes;
	size_t new_bytes;
	u32 *regs;

	if (reg_bytes <= image->reg_bytes)
		return 0;

	new_words = DIV_ROUND_UP(reg_bytes, sizeof(*image->regs));
	if (new_words > RK_MPP_MAX_REG_IMAGE_BYTES / sizeof(*image->regs))
		return -ENOMEM;

	new_bytes = (size_t)new_words * sizeof(*image->regs);
	old_bytes = (size_t)old_words * sizeof(*image->regs);

	regs = krealloc(image->regs, new_bytes, GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	if (new_bytes > old_bytes)
		memset((u8 *)regs + old_bytes, 0, new_bytes - old_bytes);

	image->regs = regs;
	image->reg_words = new_words;
	image->reg_bytes = reg_bytes;

	return 0;
}

static int rk_mpp_job_store_reg_write(struct rk_mpp_job *job,
				      const struct rk_mpp_job_req *job_req)
{
	const struct mpp_request *req = &job_req->req;
	u32 reg_end;
	int ret;

	if (!req->size)
		return 0;
	if (!job_req->payload)
		return -EINVAL;

	ret = rk_mpp_request_check_reg_span(req);
	if (ret)
		return ret;

	reg_end = req->offset + req->size;
	ret = rk_mpp_job_ensure_reg_bytes(job, reg_end);
	if (ret)
		return ret;

	memcpy((u8 *)job->reg_image.regs + req->offset, job_req->payload,
	       req->size);

	return 0;
}

static int rk_mpp_job_store_reg_read(struct rk_mpp_job *job,
				     const struct mpp_request *req)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	u32 reg_end;
	int ret;

	if (!req->size)
		return 0;

	if (image->read_req_count >= ARRAY_SIZE(image->read_reqs))
		return -EINVAL;

	if (rk_mpp_job_is_rkvdec_perf_read(job, req)) {
		ret = rk_mpp_request_check_rkvdec_perf_span(req);
		if (ret)
			return ret;
		image->read_reqs[image->read_req_count++] = *req;
		return 0;
	}

	ret = rk_mpp_request_check_reg_span(req);
	if (ret)
		return ret;

	reg_end = req->offset + req->size;
	ret = rk_mpp_job_ensure_reg_bytes(job, reg_end);
	if (ret)
		return ret;

	image->read_reqs[image->read_req_count++] = *req;

	return 0;
}

static int rk_mpp_job_store_reg_offsets(struct rk_mpp_job *job,
					const struct rk_mpp_job_req *job_req)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	const struct mpp_request *req = &job_req->req;
	u32 count;

	if (!req->size)
		return 0;
	if (!job_req->payload)
		return -EINVAL;
	if (req->size % sizeof(image->offsets[0]))
		return -EINVAL;

	count = req->size / sizeof(image->offsets[0]);
	if (count > ARRAY_SIZE(image->offsets) - image->offset_count)
		return -EINVAL;

	memcpy(&image->offsets[image->offset_count], job_req->payload,
	       req->size);
	image->offset_count += count;

	return 0;
}

static int rk_mpp_job_store_rcb_info(struct rk_mpp_job *job,
				     const struct rk_mpp_job_req *job_req)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_session *session = job->session;
	const struct mpp_request *req = &job_req->req;
	u32 count;
	u32 limit;

	if (!req->size)
		return 0;
	if (!job_req->payload)
		return -EINVAL;
	if (req->size % sizeof(image->rcb_descs[0]))
		return -EINVAL;

	limit = rk_mpp_session_rcb_limit(session);
	count = req->size / sizeof(image->rcb_descs[0]);
	if (count > limit)
		return -EINVAL;

	memcpy(image->rcb_descs, job_req->payload, req->size);
	image->rcb_count = count;

	mutex_lock(&session->lock);
	memcpy(session->rcb_descs, image->rcb_descs, req->size);
	session->rcb_count = count;
	mutex_unlock(&session->lock);

	return 0;
}

static int rk_mpp_job_ensure_reg_word(struct rk_mpp_job *job, u32 index)
{
	if (index >= RK_MPP_MAX_REG_IMAGE_BYTES / sizeof(u32))
		return -ENOMEM;

	return rk_mpp_job_ensure_reg_bytes(job, (index + 1) * sizeof(u32));
}

static int rk_mpp_job_hold_import(struct rk_mpp_job *job,
				  struct rk_mpp_import *import)
{
	u32 i;

	for (i = 0; i < job->import_count; i++) {
		if (job->imports[i] == import) {
			rk_mpp_import_put(import);
			return 0;
		}
	}

	if (job->import_count >= ARRAY_SIZE(job->imports)) {
		rk_mpp_import_put(import);
		return -EINVAL;
	}

	job->imports[job->import_count++] = import;

	return 0;
}

static int rk_mpp_job_translate_reg(struct rk_mpp_job *job, u32 index)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_import *import;
	u32 embedded_offset;
	dma_addr_t iova;
	u32 raw;
	int fd;
	int ret;

	if (index >= image->reg_words)
		return 0;

	raw = image->regs[index];
	if (job->flags & MPP_FLAGS_REG_NO_OFFSET) {
		if (raw > INT_MAX)
			return -EINVAL;
		fd = raw;
		embedded_offset = 0;
	} else {
		fd = raw & 0x3ff;
		embedded_offset = raw >> 10;
	}

	if (!fd)
		return 0;

	if (!job->hw)
		return -ENODEV;

	import = rk_mpp_import_fd(job->session, fd, job->hw->dev);
	if (IS_ERR(import))
		return PTR_ERR(import);

	iova = import->iova + embedded_offset;
	ret = rk_mpp_job_hold_import(job, import);
	if (ret)
		return ret;

	image->regs[index] = lower_32_bits(iova);

	return 0;
}

static int rk_mpp_job_translate_table(struct rk_mpp_job *job,
				      const struct rk_mpp_trans_table *table,
				      u32 base_words)
{
	u32 max_words = RK_MPP_MAX_REG_IMAGE_BYTES / sizeof(u32);
	u32 i;
	int ret;

	if (!table || !table->regs)
		return -EINVAL;

	for (i = 0; i < table->count; i++) {
		u32 index = table->regs[i];

		if (index >= max_words || base_words > max_words - index)
			return -ENOMEM;

		ret = rk_mpp_job_translate_reg(job, base_words + index);
		if (ret)
			return ret;
	}

	return 0;
}

static int rk_mpp_job_apply_reg_offsets(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	int ret;
	u32 i;

	for (i = 0; i < image->offset_count; i++) {
		u32 index = image->offsets[i].index;

		ret = rk_mpp_job_ensure_reg_word(job, index);
		if (ret)
			return ret;

		image->regs[index] += image->offsets[i].offset;
	}

	return 0;
}

static void rk_mpp_job_snapshot_rcb_info(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_session *session = job->session;

	if (image->rcb_count)
		return;

	mutex_lock(&session->lock);
	if (session->rcb_count) {
		image->rcb_count = session->rcb_count;
		memcpy(image->rcb_descs, session->rcb_descs,
		       image->rcb_count * sizeof(image->rcb_descs[0]));
	}
	mutex_unlock(&session->lock);
}

static bool rk_mpp_job_rkvdec_rcb_enabled(struct rk_mpp_job *job)
{
	struct rk_mpp_session *session = job->session;
	u64 width;

	if (session->client_type != RK_MPP_DEVICE_RKVDEC ||
	    !job->hw->rcb_min_width)
		return true;

	mutex_lock(&session->lock);
	width = session->codec_info[RK_MPP_DEC_INFO_WIDTH].val;
	mutex_unlock(&session->lock);

	return width >= job->hw->rcb_min_width;
}

static int rk_mpp_job_apply_rcb_info(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	dma_addr_t rcb_iova;
	u32 rcb_offset = 0;
	u32 i;
	int ret;

	if (!job->hw || !job->hw->rcb_iova || !job->hw->rcb_size)
		return 0;

	rk_mpp_job_snapshot_rcb_info(job);
	if (!image->rcb_count)
		return 0;
	if (!rk_mpp_job_rkvdec_rcb_enabled(job))
		return 0;

	for (i = 0; i < image->rcb_count; i++) {
		const struct rk_mpp_rcb_desc *desc = &image->rcb_descs[i];
		u32 next_offset;

		if (check_add_overflow(rcb_offset, desc->size, &next_offset) ||
		    next_offset > job->hw->rcb_size)
			continue;
		if (check_add_overflow(job->hw->rcb_iova,
				       (dma_addr_t)rcb_offset, &rcb_iova))
			return -EOVERFLOW;

		ret = rk_mpp_job_ensure_reg_word(job, desc->index);
		if (ret)
			return ret;

		image->regs[desc->index] = lower_32_bits(rcb_iova);
		rcb_offset = next_offset;
	}

	return 0;
}

static int rk_mpp_job_translate_custom_table(struct rk_mpp_job *job)
{
	struct rk_mpp_session *session = job->session;
	struct rk_mpp_trans_table table = {
		.regs = session->trans_table,
		.count = session->trans_count,
	};

	return rk_mpp_job_translate_table(job, &table, 0);
}

static int rk_mpp_job_translate_rkvdec(struct rk_mpp_job *job)
{
	const struct rk_mpp_reg_image *image = &job->reg_image;
	u32 fmt = 0;

	if (RK_MPP_RKVDEC_REG_FMT < image->reg_words)
		fmt = image->regs[RK_MPP_RKVDEC_REG_FMT] & 0x3ff;
	if (fmt >= ARRAY_SIZE(rk_mpp_rkvdec_tables))
		return -EINVAL;

	return rk_mpp_job_translate_table(job, &rk_mpp_rkvdec_tables[fmt], 0);
}

static int rk_mpp_job_translate_rkvenc(struct rk_mpp_job *job)
{
	const struct rk_mpp_reg_image *image = &job->reg_image;
	u32 fmt;
	int ret;

	if (RK_MPP_RKVENC_FMT_WORD >= image->reg_words)
		return -EINVAL;

	fmt = image->regs[RK_MPP_RKVENC_FMT_WORD] & RK_MPP_RKVENC_FMT_MASK;
	if (fmt >= ARRAY_SIZE(rk_mpp_rkvenc_pic_tables))
		return -EINVAL;

	ret = rk_mpp_job_translate_table(job, &rk_mpp_rkvenc_pic_tables[fmt],
					RK_MPP_RKVENC_PIC_BASE_WORDS);
	if (ret)
		return ret;

	return rk_mpp_job_translate_table(job, &rk_mpp_rkvenc_osd_tables[fmt],
					 RK_MPP_RKVENC_OSD_BASE_WORDS);
}

static bool rk_mpp_job_rkvenc_slice_mode(struct rk_mpp_job *job)
{
	const struct rk_mpp_reg_image *image = &job->reg_image;
	u32 enc_pic;
	u32 sli_split;

	if (job->session->client_type != RK_MPP_DEVICE_RKVENC)
		return false;
	if (RK_MPP_RKVENC_ENC_PIC_WORD >= image->reg_words ||
	    RK_MPP_RKVENC_SLI_SPLIT_WORD >= image->reg_words)
		return false;

	enc_pic = image->regs[RK_MPP_RKVENC_ENC_PIC_WORD];
	sli_split = image->regs[RK_MPP_RKVENC_SLI_SPLIT_WORD];

	return (enc_pic & RK_MPP_RKVENC_ENC_PIC_SLEN_FIFO) &&
	       (sli_split & RK_MPP_RKVENC_SLI_SPLIT_EN);
}

static u32 rk_mpp_rkvenc_dchs_txid(u32 val)
{
	return (val & RK_MPP_RKVENC_DCHS_TXID_MASK) >>
	       RK_MPP_RKVENC_DCHS_TXID_SHIFT;
}

static u32 rk_mpp_rkvenc_dchs_rxid(u32 val)
{
	return (val & RK_MPP_RKVENC_DCHS_RXID_MASK) >>
	       RK_MPP_RKVENC_DCHS_RXID_SHIFT;
}

static u32 rk_mpp_rkvenc_dchs_set_txid(u32 val, u32 id)
{
	val &= ~RK_MPP_RKVENC_DCHS_TXID_MASK;
	val |= (id << RK_MPP_RKVENC_DCHS_TXID_SHIFT) &
	       RK_MPP_RKVENC_DCHS_TXID_MASK;

	return val;
}

static u32 rk_mpp_rkvenc_dchs_set_rxid(u32 val, u32 id)
{
	val &= ~RK_MPP_RKVENC_DCHS_RXID_MASK;
	val |= (id << RK_MPP_RKVENC_DCHS_RXID_SHIFT) &
	       RK_MPP_RKVENC_DCHS_RXID_MASK;

	return val;
}

static int rk_mpp_rkvenc_dchs_find_id(u32 valid)
{
	u32 id;

	for (id = 0; id < RK_MPP_RKVENC_MAX_DCHS_ID; id++) {
		if (valid & BIT(id))
			return id;
	}

	return -ENOSPC;
}

static void rk_mpp_rkvenc2_dchs_patch(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_service *srv = job->session->srv;
	struct rk_mpp_hw *hw = job->hw;
	struct rk_mpp_rkvenc_dchs_entry *entry;
	unsigned long flags;
	u32 id_valid = GENMASK(RK_MPP_RKVENC_MAX_DCHS_ID - 1, 0);
	u32 low;
	u32 patched;
	u32 core_id;
	u32 txid_orig;
	u32 rxid_orig;
	int txid_map;
	int rxid_map = -1;
	bool rxe_map;
	u32 i;

	if (job->session->client_type != RK_MPP_DEVICE_RKVENC || !hw)
		return;
	if (image->reg_words <= RK_MPP_RKVENC_DCHS_WORD)
		return;

	low = image->regs[RK_MPP_RKVENC_DCHS_WORD] | RK_MPP_RKVENC_DCHS_TXE;
	image->regs[RK_MPP_RKVENC_DCHS_WORD] = low;

	if (!hw->ccu_node)
		return;

	core_id = hw->core_id;
	if (core_id >= RK_MPP_RKVENC_MAX_DCHS_CORES) {
		dev_err(hw->dev, "invalid RKVENC2 DCHS core id %u\n", core_id);
		return;
	}

	txid_orig = rk_mpp_rkvenc_dchs_txid(low);
	rxid_orig = rk_mpp_rkvenc_dchs_rxid(low);
	rxe_map = low & RK_MPP_RKVENC_DCHS_RXE;

	spin_lock_irqsave(&srv->rkvenc_dchs_lock, flags);

	entry = &srv->rkvenc_dchs[core_id];
	if (entry->job) {
		spin_unlock_irqrestore(&srv->rkvenc_dchs_lock, flags);
		dev_err(hw->dev, "RKVENC2 DCHS core %u is still active\n",
			core_id);
		return;
	}

	for (i = 0; i < RK_MPP_RKVENC_MAX_DCHS_CORES; i++) {
		u32 busy = lower_32_bits(srv->rkvenc_dchs[i].val);

		if (!srv->rkvenc_dchs[i].job)
			continue;

		id_valid &= ~BIT(rk_mpp_rkvenc_dchs_txid(busy));
		id_valid &= ~BIT(rk_mpp_rkvenc_dchs_rxid(busy));
	}

	if (low & RK_MPP_RKVENC_DCHS_RXE) {
		for (i = 0; i < RK_MPP_RKVENC_MAX_DCHS_CORES; i++) {
			struct rk_mpp_rkvenc_dchs_entry *busy_entry =
				&srv->rkvenc_dchs[i];
			u32 busy = lower_32_bits(busy_entry->val);

			if (!busy_entry->job || i == core_id)
				continue;
			if ((u32)(busy_entry->val >> 32) != job->session->id)
				continue;
			if (rxid_orig != busy_entry->txid_orig)
				continue;

			rxid_map = rk_mpp_rkvenc_dchs_txid(busy);
			break;
		}
	}

	txid_map = rk_mpp_rkvenc_dchs_find_id(id_valid);
	if (txid_map < 0) {
		spin_unlock_irqrestore(&srv->rkvenc_dchs_lock, flags);
		dev_err(hw->dev, "job %u session %u failed to allocate DCHS tx id\n",
			job->id, job->session->id);
		return;
	}

	id_valid &= ~BIT(txid_map);

	if (rxid_map < 0) {
		rxid_map = rk_mpp_rkvenc_dchs_find_id(id_valid);
		if (rxid_map < 0) {
			spin_unlock_irqrestore(&srv->rkvenc_dchs_lock, flags);
			dev_err(hw->dev, "job %u session %u failed to allocate DCHS rx id\n",
				job->id, job->session->id);
			return;
		}

		rxe_map = false;
	}

	patched = rk_mpp_rkvenc_dchs_set_txid(low, txid_map);
	patched = rk_mpp_rkvenc_dchs_set_rxid(patched, rxid_map);
	if (rxe_map)
		patched |= RK_MPP_RKVENC_DCHS_RXE;
	else
		patched &= ~RK_MPP_RKVENC_DCHS_RXE;

	entry->job = job;
	entry->val = ((u64)job->session->id << 32) | patched;
	entry->txid_orig = txid_orig;
	entry->rxid_orig = rxid_orig;
	job->rkvenc_dchs_core_id = core_id;
	job->rkvenc_dchs_active = true;
	image->regs[RK_MPP_RKVENC_DCHS_WORD] = patched;

	spin_unlock_irqrestore(&srv->rkvenc_dchs_lock, flags);
}

static void rk_mpp_rkvenc2_dchs_release(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;
	unsigned long flags;
	u32 core_id = job->rkvenc_dchs_core_id;

	spin_lock_irqsave(&srv->rkvenc_dchs_lock, flags);
	if (job->rkvenc_dchs_active) {
		if (core_id < RK_MPP_RKVENC_MAX_DCHS_CORES &&
		    srv->rkvenc_dchs[core_id].job == job)
			memset(&srv->rkvenc_dchs[core_id], 0,
			       sizeof(srv->rkvenc_dchs[core_id]));
		job->rkvenc_dchs_active = false;
	}
	spin_unlock_irqrestore(&srv->rkvenc_dchs_lock, flags);
}

static int rk_mpp_job_translate_reg_image(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	int ret = 0;

	if (image->translated)
		return 0;
	if (!image->reg_words && !image->offset_count)
		return 0;
	if (job->flags & MPP_FLAGS_REG_FD_NO_TRANS)
		return 0;

	if (image->reg_words) {
		if (job->session->trans_count) {
			ret = rk_mpp_job_translate_custom_table(job);
		} else {
			switch (job->session->client_type) {
			case RK_MPP_DEVICE_RKVDEC:
				ret = rk_mpp_job_translate_rkvdec(job);
				break;
			case RK_MPP_DEVICE_RKVENC:
				ret = rk_mpp_job_translate_rkvenc(job);
				break;
			default:
				ret = -EINVAL;
				break;
			}
		}
		if (ret)
			return ret;
	}

	ret = rk_mpp_job_apply_reg_offsets(job);
	if (ret)
		return ret;

	image->translated = true;

	return 0;
}

static int rk_mpp_job_select_hw(struct rk_mpp_job *job)
{
	bool prefer_idle = !(job->flags & MPP_FLAGS_REG_FD_NO_TRANS);

	if (job->hw)
		return 0;

	job->hw = rk_mpp_hw_get_for_session(job->session, prefer_idle);
	if (!job->hw)
		return -ENODEV;

	return 0;
}

static int rk_mpp_backend_submit_unsupported(struct rk_mpp_job *job)
{
	return -EOPNOTSUPP;
}

static const struct rk_mpp_backend_ops rk_mpp_unsupported_backend_ops = {
	.submit = rk_mpp_backend_submit_unsupported,
};

static int rk_mpp_job_submit(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;
	const struct rk_mpp_backend_ops *ops;

	if (!job->hw)
		return -ENODEV;

	ops = job->hw->match->ops;
	if (!ops || !ops->submit)
		return -EOPNOTSUPP;
	if (ops->validate) {
		int ret = ops->validate(job);

		if (ret)
			return ret;
	}

	job->rkvenc_slice_mode = rk_mpp_job_rkvenc_slice_mode(job);
	rk_mpp_job_activate(job);
	rk_mpp_job_get(job);

	mutex_lock(&srv->sched_lock);
	list_add_tail(&job->sched_link, &srv->queued_jobs);
	atomic_inc(&job->hw->queued_job_count);
	atomic_inc(&srv->queued_job_count);
	mutex_unlock(&srv->sched_lock);

	schedule_work(&srv->sched_work);

	return 0;
}

static int rk_mpp_job_materialize_request(struct rk_mpp_job *job,
					  const struct rk_mpp_job_req *job_req)
{
	switch (job_req->req.cmd) {
	case MPP_CMD_SET_REG_WRITE:
		return rk_mpp_job_store_reg_write(job, job_req);
	case MPP_CMD_SET_REG_READ:
		return rk_mpp_job_store_reg_read(job, &job_req->req);
	case MPP_CMD_SET_REG_ADDR_OFFSET:
		return rk_mpp_job_store_reg_offsets(job, job_req);
	case MPP_CMD_SET_RCB_INFO:
		return rk_mpp_job_store_rcb_info(job, job_req);
	default:
		return 0;
	}
}

static int rk_mpp_job_copy_readback(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	u32 i;

	for (i = 0; i < image->read_req_count; i++) {
		const struct mpp_request *req = &image->read_reqs[i];

		if (rk_mpp_job_is_rkvdec_perf_read(job, req)) {
			u32 offset = req->offset - RK_MPP_RKVDEC_PERF_SEL_OFFSET;

			if (rk_mpp_request_check_rkvdec_perf_span(req))
				return -EINVAL;
			if (copy_to_user(req->data,
					 (u8 *)image->rkvdec_perf_sel + offset,
					 req->size))
				return -EFAULT;
			continue;
		}

		if (req->offset > image->reg_bytes ||
		    req->size > image->reg_bytes - req->offset)
			return -EINVAL;
		if (copy_to_user(req->data, (u8 *)image->regs + req->offset,
				 req->size))
			return -EFAULT;
	}

	return 0;
}

static void rk_mpp_job_get(struct rk_mpp_job *job)
{
	refcount_inc(&job->refs);
}

static void rk_mpp_job_release(struct rk_mpp_job *job)
{
	u32 i;

	for (i = 0; i < job->req_cnt; i++)
		kfree(job->reqs[i].payload);
	for (i = 0; i < job->import_count; i++)
		rk_mpp_import_put(job->imports[i]);
	rk_mpp_hw_put(job->hw);
	kfree(job->reg_image.regs);
	rk_mpp_session_put(job->session);
	kfree(job);
}

static void rk_mpp_job_put(struct rk_mpp_job *job)
{
	if (job && refcount_dec_and_test(&job->refs))
		rk_mpp_job_release(job);
}

static void rk_mpp_job_drop_hw(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = xchg(&job->hw, NULL);

	rk_mpp_hw_put(hw);
}

static void rk_mpp_batch_release_jobs(struct rk_mpp_batch_state *batch)
{
	struct rk_mpp_job *job, *tmp;

	list_for_each_entry_safe(job, tmp, &batch->jobs, link) {
		list_del(&job->link);
		rk_mpp_job_put(job);
	}
	batch->cur_job = NULL;
}

static struct rk_mpp_job *
rk_mpp_batch_get_job(struct rk_mpp_batch_state *batch,
		     struct rk_mpp_session *session)
{
	struct rk_mpp_job *job = batch->cur_job;

	if (job && job->session == session)
		return job;

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);

	job->session = session;
	job->state = RK_MPP_JOB_STAGED;
	refcount_set(&job->refs, 1);
	spin_lock_init(&job->rkvenc_slice_lock);
	INIT_KFIFO(job->rkvenc_slice_fifo);
	rk_mpp_session_get(session);
	INIT_LIST_HEAD(&job->link);
	INIT_LIST_HEAD(&job->session_link);
	INIT_LIST_HEAD(&job->sched_link);
	list_add_tail(&job->link, &batch->jobs);
	batch->cur_job = job;

	return job;
}

static void rk_mpp_job_activate(struct rk_mpp_job *job)
{
	struct rk_mpp_session *session = job->session;

	mutex_lock(&session->lock);
	job->id = ++session->next_job_id;
	job->state = RK_MPP_JOB_ACTIVE;
	job->result = -EINPROGRESS;
	rk_mpp_job_get(job);
	list_add_tail(&job->session_link, &session->active_jobs);
	session->active_job_count++;
	mutex_unlock(&session->lock);

	atomic_inc(&session->srv->submitted_job_count);
}

static void rk_mpp_job_complete(struct rk_mpp_job *job, int result)
{
	struct rk_mpp_session *session = job->session;

	mutex_lock(&session->lock);
	job->result = result;
	job->state = RK_MPP_JOB_DONE;
	mutex_unlock(&session->lock);
	rk_mpp_rkvenc2_dchs_release(job);
	rk_mpp_job_drop_hw(job);
	wake_up_all(&session->wait);
	schedule_work(&session->srv->sched_work);
}

static void rk_mpp_job_unqueue_locked(struct rk_mpp_job *job)
{
	list_del_init(&job->sched_link);
	atomic_dec(&job->hw->queued_job_count);
	atomic_dec(&job->session->srv->queued_job_count);
}

static bool rk_mpp_job_dequeue(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;
	bool removed = false;

	mutex_lock(&srv->sched_lock);
	if (!list_empty(&job->sched_link)) {
		rk_mpp_job_unqueue_locked(job);
		removed = true;
	}
	mutex_unlock(&srv->sched_lock);

	if (removed)
		rk_mpp_job_put(job);

	return removed;
}

static void rk_mpp_hw_abort_queued(struct rk_mpp_hw *hw, int result)
{
	struct rk_mpp_service *srv = &rk_mpp_srv;
	struct rk_mpp_job *job, *tmp;
	LIST_HEAD(aborted);

	mutex_lock(&srv->sched_lock);
	list_for_each_entry_safe(job, tmp, &srv->queued_jobs, sched_link) {
		if (job->hw != hw)
			continue;

		WRITE_ONCE(job->canceled, true);
		rk_mpp_job_unqueue_locked(job);
		list_add_tail(&job->sched_link, &aborted);
	}
	mutex_unlock(&srv->sched_lock);

	list_for_each_entry_safe(job, tmp, &aborted, sched_link) {
		list_del_init(&job->sched_link);
		rk_mpp_job_complete(job, result);
		rk_mpp_job_put(job);
	}
}

static struct rk_mpp_job *
rk_mpp_scheduler_take_job(struct rk_mpp_service *srv)
{
	struct rk_mpp_job *job;

	mutex_lock(&srv->sched_lock);
	list_for_each_entry(job, &srv->queued_jobs, sched_link) {
		if (!READ_ONCE(job->canceled) && READ_ONCE(job->hw->online) &&
		    rk_mpp_hw_is_idle(job->hw)) {
			rk_mpp_job_unqueue_locked(job);
			mutex_unlock(&srv->sched_lock);
			return job;
		}
	}
	mutex_unlock(&srv->sched_lock);

	return NULL;
}

static void rk_mpp_scheduler_work(struct work_struct *work)
{
	struct rk_mpp_service *srv =
		container_of(work, struct rk_mpp_service, sched_work);
	struct rk_mpp_job *job;

	while ((job = rk_mpp_scheduler_take_job(srv))) {
		const struct rk_mpp_backend_ops *ops = job->hw->match->ops;
		int ret;

		if (READ_ONCE(job->canceled))
			ret = -ECANCELED;
		else if (!ops || !ops->submit)
			ret = -EOPNOTSUPP;
		else
			ret = ops->submit(job);

		if (ret) {
			if (ret == -EOPNOTSUPP)
				atomic_inc(&srv->unsupported_count);
			rk_mpp_job_complete(job, ret);
		}

		rk_mpp_job_put(job);
	}
}

static bool rk_mpp_hw_reg_range_valid(struct rk_mpp_hw *hw, u32 region,
				      u32 offset, u32 size)
{
	if (region >= RK_MPP_MAX_HW_REGS || !hw->regs[region])
		return false;
	if (offset > hw->reg_size[region] ||
	    size > hw->reg_size[region] - offset)
		return false;

	return true;
}

static int rk_mpp_hw_power_on(struct rk_mpp_hw *hw)
{
	int ret;

	ret = pm_runtime_resume_and_get(hw->dev);
	if (ret < 0)
		return ret;

	ret = reset_control_deassert(hw->resets);
	if (ret)
		goto err_pm_put;

	ret = clk_bulk_prepare_enable(hw->num_clks, hw->clks);
	if (ret)
		goto err_pm_put;

	return 0;

err_pm_put:
	pm_runtime_put_sync_suspend(hw->dev);
	return ret;
}

static void rk_mpp_hw_power_off(struct rk_mpp_hw *hw)
{
	clk_bulk_disable_unprepare(hw->num_clks, hw->clks);
	pm_runtime_mark_last_busy(hw->dev);
	pm_runtime_put_autosuspend(hw->dev);
}

static void rk_mpp_hw_reset_active(struct rk_mpp_hw *hw)
{
	if (!hw->resets)
		return;

	reset_control_assert(hw->resets);
	udelay(10);
	reset_control_deassert(hw->resets);
}

static int rk_mpp_hw_begin_active_job(struct rk_mpp_hw *hw,
				      struct rk_mpp_job *job)
{
	unsigned long flags;
	int ret = 0;

	if (!rk_mpp_hw_ccu_online(job->session->srv, hw))
		return -ENODEV;

	spin_lock_irqsave(&hw->lock, flags);
	if (!READ_ONCE(hw->online)) {
		ret = -ENODEV;
	} else if (READ_ONCE(job->canceled)) {
		ret = -ECANCELED;
	} else if (hw->active_job) {
		ret = -EBUSY;
	} else {
		rk_mpp_job_get(job);
		hw->active_job = job;
		hw->irq_status = 0;
	}
	spin_unlock_irqrestore(&hw->lock, flags);

	return ret;
}

static bool rk_mpp_hw_clear_active_job(struct rk_mpp_hw *hw,
				       struct rk_mpp_job *job, u32 *irq_status)
{
	unsigned long flags;
	bool cleared = false;

	spin_lock_irqsave(&hw->lock, flags);
	if (hw->active_job == job) {
		if (irq_status)
			*irq_status = hw->irq_status;
		hw->active_job = NULL;
		hw->irq_status = 0;
		cleared = true;
	}
	spin_unlock_irqrestore(&hw->lock, flags);

	if (cleared) {
		cancel_delayed_work(&hw->timeout_work);
		rk_mpp_job_put(job);
	}

	return cleared;
}

static struct rk_mpp_job *rk_mpp_hw_take_active_job(struct rk_mpp_hw *hw,
						    u32 *irq_status)
{
	struct rk_mpp_job *job;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);
	job = hw->active_job;
	if (job) {
		if (irq_status)
			*irq_status = hw->irq_status;
		hw->active_job = NULL;
		hw->irq_status = 0;
	}
	spin_unlock_irqrestore(&hw->lock, flags);

	return job;
}

static struct rk_mpp_job *rk_mpp_hw_get_active_job(struct rk_mpp_hw *hw)
{
	struct rk_mpp_job *job;
	unsigned long flags;

	spin_lock_irqsave(&hw->lock, flags);
	job = hw->active_job;
	if (job)
		rk_mpp_job_get(job);
	spin_unlock_irqrestore(&hw->lock, flags);

	return job;
}

static void rk_mpp_hw_abort_job(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;

	if (!hw)
		return;

	cancel_delayed_work_sync(&hw->timeout_work);
	mutex_lock(&hw->run_lock);
	if (rk_mpp_hw_clear_active_job(hw, job, NULL)) {
		rk_mpp_rkvenc2_dchs_release(job);
		rk_mpp_hw_reset_active(hw);
		rk_mpp_hw_power_off(hw);
	}
	mutex_unlock(&hw->run_lock);
}

static void rk_mpp_hw_schedule_timeout(struct rk_mpp_hw *hw)
{
	schedule_delayed_work(&hw->timeout_work,
			      msecs_to_jiffies(RK_MPP_WORK_TIMEOUT_MS));
}

static void rk_mpp_hw_timeout_work(struct work_struct *work)
{
	struct rk_mpp_hw *hw =
		container_of(to_delayed_work(work), struct rk_mpp_hw,
			     timeout_work);
	struct rk_mpp_job *job;

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, NULL);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return;
	}

	atomic_inc(&job->session->srv->timeout_count);
	dev_err(hw->dev, "session client %u job %u timed out\n",
		job->session->client_type, job->id);

	rk_mpp_hw_reset_active(hw);
	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, -ETIMEDOUT);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);
}

static void rk_mpp_hw_abort_active(struct rk_mpp_hw *hw, int result)
{
	struct rk_mpp_job *job;

	cancel_delayed_work_sync(&hw->timeout_work);

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, NULL);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return;
	}

	rk_mpp_hw_reset_active(hw);
	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, result);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);
}

static struct rk_mpp_hw **
rk_mpp_hw_collect_ccu_dependents(struct rk_mpp_hw *ccu, u32 *count)
{
	struct rk_mpp_service *srv = &rk_mpp_srv;
	struct rk_mpp_hw **deps;
	struct rk_mpp_hw *hw;
	u32 i = 0;
	u32 n = 0;

	*count = 0;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->online && hw->ccu_node == ccu->dev->of_node)
			n++;
	}
	mutex_unlock(&srv->hw_lock);
	if (!n)
		return NULL;

	deps = kcalloc(n, sizeof(*deps), GFP_KERNEL);
	if (!deps)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (!hw->online || hw->ccu_node != ccu->dev->of_node)
			continue;
		if (i >= n)
			break;

		refcount_inc(&hw->refs);
		deps[i++] = hw;
	}
	mutex_unlock(&srv->hw_lock);

	*count = i;

	return deps;
}

static void rk_mpp_hw_abort_ccu_dependents(struct rk_mpp_hw *ccu)
{
	struct rk_mpp_hw **deps;
	u32 count;
	u32 i;

	deps = rk_mpp_hw_collect_ccu_dependents(ccu, &count);
	if (IS_ERR(deps)) {
		dev_warn(ccu->dev, "failed to collect CCU dependents: %pe\n",
			 deps);
		return;
	}

	for (i = 0; i < count; i++) {
		rk_mpp_hw_abort_queued(deps[i], -ENODEV);
		rk_mpp_hw_abort_active(deps[i], -ENODEV);
		rk_mpp_hw_put(deps[i]);
	}

	kfree(deps);
}

static int rk_mpp_job_store_reg_word(struct rk_mpp_job *job, u32 offset,
				     u32 value)
{
	int ret;

	if (offset % sizeof(u32))
		return -EINVAL;

	ret = rk_mpp_job_ensure_reg_word(job, offset / sizeof(u32));
	if (ret)
		return ret;

	job->reg_image.regs[offset / sizeof(u32)] = value;
	return 0;
}

static int rk_mpp_job_validate_readbacks(struct rk_mpp_job *job,
					 struct rk_mpp_hw *hw)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	int ret;
	u32 i;

	for (i = 0; i < image->read_req_count; i++) {
		const struct mpp_request *req = &image->read_reqs[i];

		if (rk_mpp_job_is_rkvdec_perf_read(job, req)) {
			ret = rk_mpp_request_check_rkvdec_perf_span(req);
			if (ret)
				return ret;
			if (!rk_mpp_hw_reg_range_valid(hw, 0,
						       RK_MPP_RKVDEC_PERF_SEL_BASE,
						       4 * sizeof(u32)))
				return -EOPNOTSUPP;
			continue;
		}

		if (req->offset % sizeof(u32) || req->size % sizeof(u32))
			return -EINVAL;
		if (!rk_mpp_hw_reg_range_valid(hw, 0, req->offset, req->size))
			return -EOPNOTSUPP;
	}

	return 0;
}

static int rk_mpp_job_validate_write_regs(struct rk_mpp_job *job,
					  u32 start_offset)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_hw *hw = job->hw;
	bool start_seen = false;
	u32 i;

	for (i = 0; i < job->req_cnt; i++) {
		const struct mpp_request *req = &job->reqs[i].req;
		u32 offset;
		u32 end;

		if (req->cmd != MPP_CMD_SET_REG_WRITE || !req->size)
			continue;
		if (req->offset % sizeof(u32) || req->size % sizeof(u32))
			return -EINVAL;
		if (req->offset > image->reg_bytes ||
		    req->size > image->reg_bytes - req->offset)
			return -EINVAL;
		if (!rk_mpp_hw_reg_range_valid(hw, 0, req->offset, req->size))
			return -EINVAL;

		end = req->offset + req->size;
		for (offset = req->offset; offset < end; offset += sizeof(u32)) {
			if (offset == start_offset) {
				start_seen = true;
				break;
			}
		}
	}

	return start_seen ? 0 : -EINVAL;
}

static int rk_mpp_job_write_regs(struct rk_mpp_job *job, u32 start_offset,
				 u32 *start_value, bool *start_seen)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_hw *hw = job->hw;
	u32 i;

	*start_seen = false;

	for (i = 0; i < job->req_cnt; i++) {
		const struct mpp_request *req = &job->reqs[i].req;
		u32 offset;
		u32 end;

		if (req->cmd != MPP_CMD_SET_REG_WRITE || !req->size)
			continue;
		if (req->offset % sizeof(u32) || req->size % sizeof(u32))
			return -EINVAL;
		if (req->offset > image->reg_bytes ||
		    req->size > image->reg_bytes - req->offset)
			return -EINVAL;
		if (!rk_mpp_hw_reg_range_valid(hw, 0, req->offset, req->size))
			return -EINVAL;

		end = req->offset + req->size;
		for (offset = req->offset; offset < end; offset += sizeof(u32)) {
			u32 value = image->regs[offset / sizeof(u32)];

			if (offset == start_offset) {
				*start_value = value;
				*start_seen = true;
				continue;
			}

			writel_relaxed(value, hw->regs[0] + offset);
		}
	}

	return 0;
}

static void rk_mpp_rkvdec2_read_perf_sel(struct rk_mpp_job *job,
					 const struct mpp_request *req)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_hw *hw = job->hw;
	u32 start = (req->offset - RK_MPP_RKVDEC_PERF_SEL_OFFSET) / sizeof(u32);
	u32 end = start + req->size / sizeof(u32);
	u32 i;

	for (i = start; i < end; i += 3) {
		u32 sel0 = i;
		u32 sel1 = i + 1 < end ? i + 1 : 0;
		u32 sel2 = i + 2 < end ? i + 2 : 0;
		u32 val = RK_MPP_RKVDEC_SET_PERF_SEL(sel0, sel1, sel2);

		writel_relaxed(val, hw->regs[0] + RK_MPP_RKVDEC_PERF_SEL_BASE);
		image->rkvdec_perf_sel[sel0] =
			readl_relaxed(hw->regs[0] + RK_MPP_RKVDEC_SEL_VAL0_BASE);
		if (sel1)
			image->rkvdec_perf_sel[sel1] =
				readl_relaxed(hw->regs[0] +
					      RK_MPP_RKVDEC_SEL_VAL1_BASE);
		if (sel2)
			image->rkvdec_perf_sel[sel2] =
				readl_relaxed(hw->regs[0] +
					      RK_MPP_RKVDEC_SEL_VAL2_BASE);
	}
}

static int rk_mpp_job_read_regs(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_hw *hw = job->hw;
	u32 i;

	for (i = 0; i < image->read_req_count; i++) {
		const struct mpp_request *req = &image->read_reqs[i];
		u32 offset;
		u32 end;

		if (rk_mpp_job_is_rkvdec_perf_read(job, req)) {
			int ret = rk_mpp_request_check_rkvdec_perf_span(req);

			if (ret)
				return ret;
			if (!rk_mpp_hw_reg_range_valid(hw, 0,
						       RK_MPP_RKVDEC_PERF_SEL_BASE,
						       4 * sizeof(u32)))
				return -EOPNOTSUPP;
			rk_mpp_rkvdec2_read_perf_sel(job, req);
			continue;
		}

		if (req->offset % sizeof(u32) || req->size % sizeof(u32))
			return -EINVAL;
		if (req->offset > image->reg_bytes ||
		    req->size > image->reg_bytes - req->offset)
			return -EINVAL;
		if (!rk_mpp_hw_reg_range_valid(hw, 0, req->offset, req->size))
			return -EOPNOTSUPP;

		end = req->offset + req->size;
		for (offset = req->offset; offset < end; offset += sizeof(u32)) {
			image->regs[offset / sizeof(u32)] =
				readl_relaxed(hw->regs[0] + offset);
		}
	}

	return 0;
}

static void rk_mpp_rkvdec2_prepare_ccu_regs(struct rk_mpp_job *job)
{
	struct rk_mpp_reg_image *image = &job->reg_image;
	struct rk_mpp_session *session = job->session;
	u32 width;
	u32 height;
	u32 bitdepth;
	u32 timeout;
	u32 session_id;

	if (session->client_type != RK_MPP_DEVICE_RKVDEC || !job->hw ||
	    !job->hw->ccu_node)
		return;

	session_id = session->id & (RK_MPP_RKVDEC_FILM_IDX_MASK >>
				    RK_MPP_RKVDEC_FILM_IDX_SHIFT);
	if (image->reg_words > RK_MPP_RKVDEC_CORE_CTRL_WORD) {
		u32 val = image->regs[RK_MPP_RKVDEC_CORE_CTRL_WORD];

		val &= ~RK_MPP_RKVDEC_FILM_IDX_MASK;
		val |= (session_id << RK_MPP_RKVDEC_FILM_IDX_SHIFT) &
		       RK_MPP_RKVDEC_FILM_IDX_MASK;
		image->regs[RK_MPP_RKVDEC_CORE_CTRL_WORD] = val;
	}

	if (image->reg_words > RK_MPP_RKVDEC_EN_MODE_WORD)
		image->regs[RK_MPP_RKVDEC_EN_MODE_WORD] |=
			RK_MPP_RKVDEC_CCU_TIMEOUT_DISABLE;

	if (image->reg_words <= RK_MPP_RKVDEC_TIMEOUT_THRESHOLD_WORD)
		return;

	mutex_lock(&session->lock);
	width = lower_32_bits(session->codec_info[RK_MPP_DEC_INFO_WIDTH].val);
	height = lower_32_bits(session->codec_info[RK_MPP_DEC_INFO_HEIGHT].val);
	bitdepth = lower_32_bits(session->codec_info[RK_MPP_DEC_INFO_BITDEPTH].val);
	mutex_unlock(&session->lock);

	timeout = rk_mpp_rkvdec2_ccu_timeout_threshold(width, height, bitdepth);
	image->regs[RK_MPP_RKVDEC_TIMEOUT_THRESHOLD_WORD] = timeout;
}

static int rk_mpp_rkvenc2_validate(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	int ret;

	if (hw->irq < 0 || !hw->regs[0])
		return -ENODEV;
	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_START_BASE,
				       sizeof(u32)))
		return -ENODEV;

	ret = rk_mpp_job_validate_readbacks(job, hw);
	if (ret)
		return ret;

	return rk_mpp_job_validate_write_regs(job, RK_MPP_RKVENC_START_BASE);
}

static int rk_mpp_rkvenc2_submit(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	u32 start_value = 0;
	bool start_seen;
	int ret;

	if (hw->irq < 0 || !hw->regs[0])
		return -ENODEV;
	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_START_BASE,
				       sizeof(u32)))
		return -ENODEV;

	ret = rk_mpp_job_validate_readbacks(job, hw);
	if (ret)
		return ret;

	mutex_lock(&hw->run_lock);
	ret = rk_mpp_hw_begin_active_job(hw, job);
	if (ret)
		goto err_unlock;

	ret = rk_mpp_hw_power_on(hw);
	if (ret)
		goto err_clear_active;
	if (!READ_ONCE(hw->online) || READ_ONCE(job->canceled)) {
		ret = READ_ONCE(job->canceled) ? -ECANCELED : -ENODEV;
		goto err_power_off;
	}

	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_CLR_BASE, sizeof(u32))) {
		writel_relaxed(0x2, hw->regs[0] + RK_MPP_RKVENC_CLR_BASE);
		udelay(5);
		writel_relaxed(0x0, hw->regs[0] + RK_MPP_RKVENC_CLR_BASE);
	}
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_COUNTER_CLR_BASE,
				      sizeof(u32)))
		writel_relaxed(0x2, hw->regs[0] + RK_MPP_RKVENC_COUNTER_CLR_BASE);

	rk_mpp_rkvenc2_dchs_patch(job);

	ret = rk_mpp_job_write_regs(job, RK_MPP_RKVENC_START_BASE,
				    &start_value, &start_seen);
	if (ret)
		goto err_power_off;
	if (!start_seen) {
		ret = -EINVAL;
		goto err_power_off;
	}
	if (!READ_ONCE(hw->online) || READ_ONCE(job->canceled)) {
		ret = READ_ONCE(job->canceled) ? -ECANCELED : -ENODEV;
		goto err_power_off;
	}

	rk_mpp_hw_schedule_timeout(hw);
	wmb();
	writel(start_value, hw->regs[0] + RK_MPP_RKVENC_START_BASE);
	mutex_unlock(&hw->run_lock);

	return 0;

err_power_off:
	rk_mpp_rkvenc2_dchs_release(job);
	rk_mpp_hw_power_off(hw);
err_clear_active:
	rk_mpp_hw_clear_active_job(hw, job, NULL);
err_unlock:
	mutex_unlock(&hw->run_lock);
	return ret;
}

static void rk_mpp_job_push_rkvenc_slice(struct rk_mpp_job *job, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&job->rkvenc_slice_lock, flags);
	if (value & RK_MPP_RKVENC_SLICE_LAST)
		job->rkvenc_slice_done = true;
	if (kfifo_avail(&job->rkvenc_slice_fifo))
		kfifo_in(&job->rkvenc_slice_fifo, &value, 1);
	else
		job->rkvenc_slice_overflow = true;
	spin_unlock_irqrestore(&job->rkvenc_slice_lock, flags);
}

static bool rk_mpp_rkvenc2_read_slice_len(struct rk_mpp_hw *hw,
					  struct rk_mpp_job *job,
					  u32 *irq_status)
{
	u32 sli_num;
	u32 new_irq_status;
	bool last;
	bool queued = false;
	u32 i;

	if (!job->rkvenc_slice_mode)
		return false;
	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_SLICE_NUM_BASE,
				       sizeof(u32)) ||
	    !rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_SLICE_LEN_BASE,
				       sizeof(u32)))
		return false;

	sli_num = readl_relaxed(hw->regs[0] + RK_MPP_RKVENC_SLICE_NUM_BASE) &
		  RK_MPP_RKVENC_SLICE_NUM_MASK;
	new_irq_status = readl_relaxed(hw->regs[0] + RK_MPP_RKVENC_INT_STA_BASE);
	if (new_irq_status != *irq_status &&
	    (new_irq_status & RK_MPP_RKVENC_INT_DONE)) {
		*irq_status |= new_irq_status;
		sli_num = readl_relaxed(hw->regs[0] +
					RK_MPP_RKVENC_SLICE_NUM_BASE) &
			  RK_MPP_RKVENC_SLICE_NUM_MASK;
		if (rk_mpp_hw_reg_range_valid(hw, 0,
					       RK_MPP_RKVENC_INT_CLR_BASE,
					       sizeof(u32)))
			writel(new_irq_status,
			       hw->regs[0] + RK_MPP_RKVENC_INT_CLR_BASE);
	}

	last = *irq_status & RK_MPP_RKVENC_INT_DONE;
	for (i = 0; i < sli_num; i++) {
		u32 value = readl_relaxed(hw->regs[0] +
					  RK_MPP_RKVENC_SLICE_LEN_BASE);

		last |= value & RK_MPP_RKVENC_SLICE_LAST;
		if (last && i == sli_num - 1)
			value |= RK_MPP_RKVENC_SLICE_LAST;

		rk_mpp_job_push_rkvenc_slice(job, value);
		queued = true;
	}

	if (last && !rk_mpp_job_rkvenc_slice_done(job)) {
		u32 value = RK_MPP_RKVENC_SLICE_LAST;

		rk_mpp_job_push_rkvenc_slice(job, value);
		queued = true;
	}

	return queued || rk_mpp_job_rkvenc_slice_ready(job);
}

static irqreturn_t rk_mpp_rkvenc2_irq(struct rk_mpp_hw *hw)
{
	struct rk_mpp_job *job;
	unsigned long flags;
	u32 status;
	bool slice_ready = false;

	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_INT_STA_BASE,
				       sizeof(u32)))
		return IRQ_NONE;

	status = readl_relaxed(hw->regs[0] + RK_MPP_RKVENC_INT_STA_BASE);
	if (!status)
		return IRQ_NONE;

	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_INT_CLR_BASE,
				      sizeof(u32)))
		writel(status, hw->regs[0] + RK_MPP_RKVENC_INT_CLR_BASE);
	if ((status & RK_MPP_RKVENC_INT_WATCHDOG) &&
	    rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVENC_INT_MASK_BASE,
				      sizeof(u32)))
		writel(RK_MPP_RKVENC_INT_WATCHDOG,
		       hw->regs[0] + RK_MPP_RKVENC_INT_MASK_BASE);

	job = rk_mpp_hw_get_active_job(hw);
	if (job) {
		if (status & (RK_MPP_RKVENC_INT_SLICE_DONE |
			      RK_MPP_RKVENC_INT_DONE))
			slice_ready = rk_mpp_rkvenc2_read_slice_len(hw, job,
								   &status);
		if (slice_ready)
			wake_up_all(&job->session->wait);
		rk_mpp_job_put(job);
	}

	spin_lock_irqsave(&hw->lock, flags);
	hw->irq_status |= status;
	spin_unlock_irqrestore(&hw->lock, flags);
	if (status & (RK_MPP_RKVENC_INT_DONE | RK_MPP_RKVENC_INT_ERROR))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

static irqreturn_t rk_mpp_rkvenc2_thread(struct rk_mpp_hw *hw)
{
	struct rk_mpp_job *job;
	u32 irq_status = 0;
	int ret;

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, &irq_status);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return IRQ_HANDLED;
	}
	cancel_delayed_work(&hw->timeout_work);

	ret = rk_mpp_job_read_regs(job);
	if (!ret)
		ret = rk_mpp_job_store_reg_word(job, RK_MPP_RKVENC_INT_STA_BASE,
						irq_status);

	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, ret);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);

	return IRQ_HANDLED;
}

static int rk_mpp_rkvdec2_submit(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	u32 start_value = 0;
	bool start_seen;
	int ret;

	if (hw->irq < 0 || !hw->regs[0])
		return -ENODEV;
	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_START_BASE,
				       sizeof(u32)) ||
	    !rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_INT_STA_BASE,
				       sizeof(u32)))
		return -ENODEV;

	ret = rk_mpp_job_validate_readbacks(job, hw);
	if (ret)
		return ret;

	if (RK_MPP_RKVDEC_RLC_WORD < job->reg_image.reg_words)
		job->rkvdec_stream_addr =
			job->reg_image.regs[RK_MPP_RKVDEC_RLC_WORD];

	mutex_lock(&hw->run_lock);
	ret = rk_mpp_hw_begin_active_job(hw, job);
	if (ret)
		goto err_unlock;

	ret = rk_mpp_hw_power_on(hw);
	if (ret)
		goto err_clear_active;
	if (!READ_ONCE(hw->online) || READ_ONCE(job->canceled)) {
		ret = READ_ONCE(job->canceled) ? -ECANCELED : -ENODEV;
		goto err_power_off;
	}

	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CACHE0_SIZE_BASE,
				      sizeof(u32)))
		writel_relaxed(RK_MPP_RKVDEC_CACHE_CFG,
			       hw->regs[0] + RK_MPP_RKVDEC_CACHE0_SIZE_BASE);
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CACHE1_SIZE_BASE,
				      sizeof(u32)))
		writel_relaxed(RK_MPP_RKVDEC_CACHE_CFG,
			       hw->regs[0] + RK_MPP_RKVDEC_CACHE1_SIZE_BASE);
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CACHE2_SIZE_BASE,
				      sizeof(u32)))
		writel_relaxed(RK_MPP_RKVDEC_CACHE_CFG,
			       hw->regs[0] + RK_MPP_RKVDEC_CACHE2_SIZE_BASE);
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CLR_CACHE0_BASE,
				      sizeof(u32)))
		writel_relaxed(1, hw->regs[0] + RK_MPP_RKVDEC_CLR_CACHE0_BASE);
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CLR_CACHE1_BASE,
				      sizeof(u32)))
		writel_relaxed(1, hw->regs[0] + RK_MPP_RKVDEC_CLR_CACHE1_BASE);
	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_CLR_CACHE2_BASE,
				      sizeof(u32)))
		writel_relaxed(1, hw->regs[0] + RK_MPP_RKVDEC_CLR_CACHE2_BASE);

	rk_mpp_rkvdec2_prepare_ccu_regs(job);

	ret = rk_mpp_job_write_regs(job, RK_MPP_RKVDEC_START_BASE,
				    &start_value, &start_seen);
	if (ret)
		goto err_power_off;
	if (!start_seen) {
		ret = -EINVAL;
		goto err_power_off;
	}
	if (!READ_ONCE(hw->online) || READ_ONCE(job->canceled)) {
		ret = READ_ONCE(job->canceled) ? -ECANCELED : -ENODEV;
		goto err_power_off;
	}

	rk_mpp_hw_schedule_timeout(hw);
	wmb();
	writel(start_value | RK_MPP_RKVDEC_START_EN,
	       hw->regs[0] + RK_MPP_RKVDEC_START_BASE);
	mutex_unlock(&hw->run_lock);

	return 0;

err_power_off:
	rk_mpp_hw_power_off(hw);
err_clear_active:
	rk_mpp_hw_clear_active_job(hw, job, NULL);
err_unlock:
	mutex_unlock(&hw->run_lock);
	return ret;
}

static int rk_mpp_rkvdec2_validate(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	int ret;

	if (hw->irq < 0 || !hw->regs[0])
		return -ENODEV;
	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_START_BASE,
				       sizeof(u32)) ||
	    !rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_INT_STA_BASE,
				       sizeof(u32)))
		return -ENODEV;

	ret = rk_mpp_job_validate_readbacks(job, hw);
	if (ret)
		return ret;

	return rk_mpp_job_validate_write_regs(job, RK_MPP_RKVDEC_START_BASE);
}

static irqreturn_t rk_mpp_rkvdec2_irq(struct rk_mpp_hw *hw)
{
	unsigned long flags;
	u32 status;

	if (!rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_INT_STA_BASE,
				       sizeof(u32)))
		return IRQ_NONE;

	status = readl_relaxed(hw->regs[0] + RK_MPP_RKVDEC_INT_STA_BASE);
	if (!(status & RK_MPP_RKVDEC_IRQ_RAW))
		return IRQ_NONE;

	writel(0, hw->regs[0] + RK_MPP_RKVDEC_INT_STA_BASE);
	spin_lock_irqsave(&hw->lock, flags);
	hw->irq_status |= status;
	spin_unlock_irqrestore(&hw->lock, flags);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rk_mpp_rkvdec2_thread(struct rk_mpp_hw *hw)
{
	struct rk_mpp_job *job;
	u32 irq_status = 0;
	int ret;

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, &irq_status);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return IRQ_HANDLED;
	}
	cancel_delayed_work(&hw->timeout_work);

	ret = rk_mpp_job_read_regs(job);
	if (!ret)
		ret = rk_mpp_job_store_reg_word(job, RK_MPP_RKVDEC_INT_STA_BASE,
						irq_status);
	if (!ret && RK_MPP_RKVDEC_RLC_WORD < job->reg_image.reg_words) {
		u32 dec_get = readl_relaxed(hw->regs[0] + RK_MPP_RKVDEC_RLC_BASE);
		s32 dec_length = (s32)(dec_get - job->rkvdec_stream_addr);

		job->reg_image.regs[RK_MPP_RKVDEC_RLC_WORD] = dec_length << 10;
	}

	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, ret);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);

	return IRQ_HANDLED;
}

static const struct rk_mpp_backend_ops rk_mpp_rkvenc2_backend_ops = {
	.validate = rk_mpp_rkvenc2_validate,
	.submit = rk_mpp_rkvenc2_submit,
	.irq = rk_mpp_rkvenc2_irq,
	.thread = rk_mpp_rkvenc2_thread,
};

static const struct rk_mpp_backend_ops rk_mpp_rkvdec2_backend_ops = {
	.validate = rk_mpp_rkvdec2_validate,
	.submit = rk_mpp_rkvdec2_submit,
	.irq = rk_mpp_rkvdec2_irq,
	.thread = rk_mpp_rkvdec2_thread,
};

static irqreturn_t rk_mpp_hw_irq(int irq, void *data)
{
	struct rk_mpp_hw *hw = data;

	if (!hw->match->ops || !hw->match->ops->irq)
		return IRQ_NONE;

	return hw->match->ops->irq(hw);
}

static irqreturn_t rk_mpp_hw_irq_thread(int irq, void *data)
{
	struct rk_mpp_hw *hw = data;

	if (!hw->match->ops || !hw->match->ops->thread)
		return IRQ_HANDLED;

	return hw->match->ops->thread(hw);
}

static void rk_mpp_session_abort_jobs(struct rk_mpp_session *session)
{
	struct rk_mpp_job *job, *tmp;
	LIST_HEAD(aborted);

	mutex_lock(&session->lock);
	list_for_each_entry_safe(job, tmp, &session->active_jobs, session_link) {
		WRITE_ONCE(job->canceled, true);
		list_move_tail(&job->session_link, &aborted);
		job->result = -ECANCELED;
		job->state = RK_MPP_JOB_DONE;
	}
	session->active_job_count = 0;
	mutex_unlock(&session->lock);
	wake_up_all(&session->wait);

	list_for_each_entry_safe(job, tmp, &aborted, session_link) {
		list_del_init(&job->session_link);
		rk_mpp_job_dequeue(job);
		rk_mpp_hw_abort_job(job);
		rk_mpp_job_put(job);
	}
}

static struct rk_mpp_job *
rk_mpp_session_first_job_get(struct rk_mpp_session *session)
{
	struct rk_mpp_job *job;

	mutex_lock(&session->lock);
	job = list_first_entry_or_null(&session->active_jobs,
				       struct rk_mpp_job, session_link);
	if (job)
		rk_mpp_job_get(job);
	mutex_unlock(&session->lock);

	return job;
}

static bool rk_mpp_job_is_done(struct rk_mpp_job *job)
{
	struct rk_mpp_session *session = job->session;
	bool done;

	mutex_lock(&session->lock);
	done = job->state == RK_MPP_JOB_DONE;
	mutex_unlock(&session->lock);

	return done;
}

static bool rk_mpp_job_rkvenc_slice_ready(struct rk_mpp_job *job)
{
	unsigned long flags;
	bool ready;

	spin_lock_irqsave(&job->rkvenc_slice_lock, flags);
	ready = job->rkvenc_slice_overflow ||
		!kfifo_is_empty(&job->rkvenc_slice_fifo);
	spin_unlock_irqrestore(&job->rkvenc_slice_lock, flags);

	return ready;
}

static bool rk_mpp_job_rkvenc_slice_done(struct rk_mpp_job *job)
{
	unsigned long flags;
	bool done;

	spin_lock_irqsave(&job->rkvenc_slice_lock, flags);
	done = job->rkvenc_slice_done;
	spin_unlock_irqrestore(&job->rkvenc_slice_lock, flags);

	return done;
}

static int rk_mpp_job_pop_rkvenc_slice(struct rk_mpp_job *job, u32 *value)
{
	unsigned long flags;
	int ret = -EAGAIN;

	spin_lock_irqsave(&job->rkvenc_slice_lock, flags);
	if (job->rkvenc_slice_overflow) {
		ret = -EOVERFLOW;
	} else if (kfifo_out(&job->rkvenc_slice_fifo, value, 1) == 1) {
		ret = 0;
	}
	spin_unlock_irqrestore(&job->rkvenc_slice_lock, flags);

	return ret;
}

static bool rk_mpp_session_irq_poll_ready(struct rk_mpp_session *session)
{
	struct rk_mpp_job *job;
	bool ready;

	mutex_lock(&session->lock);
	job = list_first_entry_or_null(&session->active_jobs,
				       struct rk_mpp_job, session_link);
	if (!job) {
		ready = true;
	} else if (job->state == RK_MPP_JOB_DONE || !job->rkvenc_slice_mode) {
		ready = true;
	} else {
		ready = rk_mpp_job_rkvenc_slice_ready(job) ||
			rk_mpp_job_rkvenc_slice_done(job);
	}
	mutex_unlock(&session->lock);

	return ready;
}

static bool rk_mpp_session_done_or_empty(struct rk_mpp_session *session)
{
	struct rk_mpp_job *job;
	bool ready;

	mutex_lock(&session->lock);
	job = list_first_entry_or_null(&session->active_jobs,
				       struct rk_mpp_job, session_link);
	ready = !job || job->state == RK_MPP_JOB_DONE;
	mutex_unlock(&session->lock);

	return ready;
}

static int rk_mpp_session_poll_job(struct rk_mpp_session *session, u32 flags)
{
	struct rk_mpp_job *job;
	int ret;

	for (;;) {
		mutex_lock(&session->lock);
		job = list_first_entry_or_null(&session->active_jobs,
					       struct rk_mpp_job, session_link);
		if (!job) {
			mutex_unlock(&session->lock);
			return -EIO;
		}
		if (job->state == RK_MPP_JOB_DONE) {
			list_del_init(&job->session_link);
			if (session->active_job_count)
				session->active_job_count--;
			ret = job->result;
			mutex_unlock(&session->lock);
			if (!ret)
				ret = rk_mpp_job_copy_readback(job);
			rk_mpp_job_put(job);
			return ret;
		}
		mutex_unlock(&session->lock);

		if (flags & MPP_FLAGS_POLL_NON_BLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(session->wait,
					       rk_mpp_session_done_or_empty(session));
		if (ret)
			return ret;
	}
}

static int rk_mpp_poll_irq_validate_req(const struct mpp_request *req,
					struct rk_mpp_rkvenc_poll_slice_cfg *cfg,
					bool *copy_slices)
{
	int ret;

	*copy_slices = req && req->size && req->data;
	if (!*copy_slices)
		return 0;
	if (req->size < sizeof(*cfg))
		return -EINVAL;
	if (copy_from_user(cfg, req->data, sizeof(*cfg)))
		return -EFAULT;

	ret = rk_mpp_poll_irq_check_size(cfg->count_max, req->size);
	if (ret)
		return ret;

	cfg->count_ret = 0;

	return 0;
}

static int rk_mpp_poll_irq_put_slice(const struct mpp_request *req,
				     s32 count_ret, u32 value)
{
	struct rk_mpp_rkvenc_poll_slice_cfg __user *ucfg = req->data;
	u32 __user *dst = (u32 __user *)(ucfg + 1);

	if (put_user(value, dst + count_ret))
		return -EFAULT;
	if (put_user(count_ret + 1, &ucfg->count_ret))
		return -EFAULT;

	return 0;
}

static int rk_mpp_session_poll_irq(struct rk_mpp_session *session,
				   const struct mpp_request *req,
				   u32 flags)
{
	struct rk_mpp_rkvenc_poll_slice_cfg cfg = {};
	bool copy_slices;
	int ret;

	ret = rk_mpp_poll_irq_validate_req(req, &cfg, &copy_slices);
	if (ret)
		return ret;

	for (;;) {
		struct rk_mpp_job *job;
		u32 slice_info = 0;

		job = rk_mpp_session_first_job_get(session);
		if (!job)
			return -EIO;

		if (!job->rkvenc_slice_mode) {
			rk_mpp_job_put(job);
			return rk_mpp_session_poll_job(session, flags);
		}

		ret = rk_mpp_job_pop_rkvenc_slice(job, &slice_info);
		if (!ret) {
			bool last = slice_info & RK_MPP_RKVENC_SLICE_LAST;

			if (copy_slices && cfg.count_ret < cfg.count_max) {
				ret = rk_mpp_poll_irq_put_slice(req,
							       cfg.count_ret,
							       slice_info);
				if (ret) {
					rk_mpp_job_put(job);
					return ret;
				}
				cfg.count_ret++;
			}

			rk_mpp_job_put(job);

			if (last)
				return rk_mpp_session_poll_job(session, flags);
			if (copy_slices && cfg.count_ret >= cfg.count_max)
				return 0;
			continue;
		}

		if (ret != -EAGAIN) {
			rk_mpp_job_put(job);
			return ret;
		}

		if (rk_mpp_job_is_done(job) ||
		    rk_mpp_job_rkvenc_slice_done(job)) {
			rk_mpp_job_put(job);
			return rk_mpp_session_poll_job(session, flags);
		}
		rk_mpp_job_put(job);

		if (flags & MPP_FLAGS_POLL_NON_BLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(session->wait,
					       rk_mpp_session_irq_poll_ready(session));
		if (ret)
			return ret;
	}
}

static int rk_mpp_job_add_request(struct rk_mpp_session *session,
				  struct mpp_request *req,
				  struct rk_mpp_batch_state *batch)
{
	struct rk_mpp_job_req *job_req;
	struct rk_mpp_job *job;
	bool copy_payload;
	int ret;

	job = rk_mpp_batch_get_job(batch, session);
	if (IS_ERR(job))
		return PTR_ERR(job);

	if (job->req_cnt >= RK_MPP_MAX_MSG_NUM)
		return -EINVAL;
	if (req->size > RK_MPP_MAX_JOB_PAYLOAD)
		return -ENOMEM;
	if (req->size && !req->data)
		return -EINVAL;

	copy_payload = rk_mpp_cmd_copies_payload(req->cmd);
	if (req->size && !copy_payload && !access_ok(req->data, req->size))
		return -EFAULT;

	job_req = &job->reqs[job->req_cnt];
	job_req->req = *req;
	if (copy_payload && req->size) {
		job_req->payload = memdup_user(req->data, req->size);
		if (IS_ERR(job_req->payload)) {
			void *payload = job_req->payload;

			job_req->payload = NULL;
			memset(job_req, 0, sizeof(*job_req));
			return PTR_ERR(payload);
		}
	}

	ret = rk_mpp_job_materialize_request(job, job_req);
	if (ret) {
		kfree(job_req->payload);
		memset(job_req, 0, sizeof(*job_req));
		return ret;
	}

	job->req_cnt++;
	job->flags |= req->flags;

	switch (req->cmd) {
	case MPP_CMD_SET_REG_WRITE:
	case MPP_CMD_SET_REG_READ:
	case MPP_CMD_SET_REG_ADDR_OFFSET:
	case MPP_CMD_SET_RCB_INFO:
		job->set_cnt++;
		break;
	case MPP_CMD_POLL_HW_FINISH:
	case MPP_CMD_POLL_HW_IRQ:
		job->poll_cnt++;
		job->poll_irq = req->cmd == MPP_CMD_POLL_HW_IRQ;
		job->poll_req = *req;
		break;
	default:
		break;
	}

	return 0;
}

static int rk_mpp_execute_jobs(struct rk_mpp_batch_state *batch)
{
	struct rk_mpp_job *job;
	int ret;

	list_for_each_entry(job, &batch->jobs, link) {
		if (job->set_cnt) {
			if (!job->session->initialized)
				return -EINVAL;
			ret = rk_mpp_job_select_hw(job);
			if (ret)
				return ret;
			ret = rk_mpp_job_translate_reg_image(job);
			if (ret)
				return ret;
			ret = rk_mpp_job_apply_rcb_info(job);
			if (ret)
				return ret;
			ret = rk_mpp_job_submit(job);
			if (ret) {
				if (ret == -EOPNOTSUPP)
					atomic_inc(&job->session->srv->unsupported_count);
				return ret;
			}
		}
	}

	ret = 0;
	list_for_each_entry(job, &batch->jobs, link) {
		if (job->poll_cnt) {
			int poll_ret;

			if (!job->session->initialized)
				return -EINVAL;
			if (job->poll_irq)
				poll_ret = rk_mpp_session_poll_irq(job->session,
								   &job->poll_req,
								   job->flags);
			else
				poll_ret = rk_mpp_session_poll_job(job->session,
								   job->flags);
			if (poll_ret && !ret)
				ret = poll_ret;
		}
	}

	return ret;
}

static int rk_mpp_process_request(struct rk_mpp_session *session,
				  struct mpp_request *req,
				  struct rk_mpp_batch_state *batch)
{
	u32 value;

	switch (req->cmd) {
	case MPP_CMD_QUERY_HW_SUPPORT:
		session->srv->hw_support = rk_mpp_get_hw_support(session->srv);
		if (put_user(session->srv->hw_support, (u32 __user *)req->data))
			return -EFAULT;
		return 0;
	case MPP_CMD_QUERY_HW_ID:
		if (session->initialized) {
			value = session->client_type;
		} else if (get_user(value, (u32 __user *)req->data)) {
			return -EFAULT;
		}
		if (value >= RK_MPP_DEVICE_BUTT ||
		    !(session->srv->hw_support & BIT(value)))
			return -EINVAL;
		value = rk_mpp_get_hw_id(session->srv, value);
		if (put_user(value, (u32 __user *)req->data))
			return -EFAULT;
		return 0;
	case MPP_CMD_QUERY_CMD_SUPPORT:
		if (get_user(value, (u32 __user *)req->data))
			return -EINVAL;
		value = rk_mpp_get_cmd_butt(value);
		if (put_user(value, (u32 __user *)req->data))
			return -EFAULT;
		return 0;
	case MPP_CMD_INIT_CLIENT_TYPE:
		if (get_user(value, (u32 __user *)req->data))
			return -EFAULT;
		if (value >= RK_MPP_DEVICE_BUTT ||
		    !(session->srv->hw_support & BIT(value)))
			return -EINVAL;
		session->client_type = value;
		session->initialized = true;
		return 0;
	case MPP_CMD_INIT_DRIVER_DATA:
		if (!session->initialized)
			return -EINVAL;
		if (get_user(value, (u32 __user *)req->data))
			return -EFAULT;
		return 0;
	case MPP_CMD_INIT_TRANS_TABLE:
		if (req->size > sizeof(session->trans_table))
			return -ENOMEM;
		if (req->size &&
		    copy_from_user(session->trans_table, req->data, req->size))
			return -EINVAL;
		session->trans_count = req->size / sizeof(session->trans_table[0]);
		return 0;
	case MPP_CMD_SET_REG_WRITE:
	case MPP_CMD_SET_REG_READ:
	case MPP_CMD_SET_REG_ADDR_OFFSET:
	case MPP_CMD_SET_RCB_INFO:
	case MPP_CMD_POLL_HW_FINISH:
	case MPP_CMD_POLL_HW_IRQ:
		return rk_mpp_job_add_request(session, req, batch);
	case MPP_CMD_RESET_SESSION:
		if (!session->initialized)
			return -EINVAL;
		rk_mpp_session_abort_jobs(session);
		rk_mpp_session_release_imports(session);
		return 0;
	case MPP_CMD_TRANS_FD_TO_IOVA:
		return rk_mpp_trans_fd_to_iova(session, req);
	case MPP_CMD_RELEASE_FD:
		return rk_mpp_release_fds(session, req);
	case MPP_CMD_SEND_CODEC_INFO:
		if (!session->initialized)
			return -EINVAL;
		return rk_mpp_store_codec_info(session, req);
	case MPP_CMD_SET_ERR_REF_HACK:
		if (!session->initialized)
			return -EINVAL;
		return rk_mpp_copy_in_discard(req);
	default:
		return -EINVAL;
	}
}

static int rk_mpp_switch_session(struct rk_mpp_session **session,
				 struct fd *held_fd,
				 const struct rk_mpp_msg_v1 *msg)
{
	struct mpp_bat_msg bat_msg;
	struct mpp_bat_msg __user *ubatch;
	struct fd f;
	int ret = 0;

	ubatch = (struct mpp_bat_msg __user *)(uintptr_t)msg->data_ptr;
	if (copy_from_user(&bat_msg, ubatch, sizeof(bat_msg)))
		return -EFAULT;

	if (bat_msg.flag & MPP_BAT_MSG_DONE)
		return 0;

	f = fdget(bat_msg.fd);
	if (!fd_file(f)) {
		ret = -EBADF;
		if (copy_to_user(&ubatch->ret, &ret, sizeof(ubatch->ret)))
			pr_debug("failed to write bad-fd result\n");
		return 0;
	}

	if (fd_file(f)->f_op != &rk_mpp_fops || !fd_file(f)->private_data) {
		fdput(f);
		return -EINVAL;
	}

	if (fd_file(*held_fd))
		fdput(*held_fd);
	*held_fd = f;
	*session = fd_file(f)->private_data;

	return 0;
}

static int rk_mpp_collect_msgs(struct rk_mpp_session *session,
			       unsigned int cmd, void __user *arg)
{
	struct rk_mpp_batch_state batch;
	struct fd held_fd = {};
	int ret = 0;

	if (cmd != MPP_IOC_CFG_V1)
		return -EINVAL;

	memset(&batch, 0, sizeof(batch));
	INIT_LIST_HEAD(&batch.jobs);

	for (;;) {
		struct rk_mpp_msg_v1 msg;
		struct mpp_request req;
		bool last;

		if (copy_from_user(&msg, arg, sizeof(msg))) {
			ret = -EFAULT;
			break;
		}
		arg += sizeof(msg);

		if (rk_mpp_check_cmd_v1(msg.cmd)) {
			ret = -EFAULT;
			break;
		}

		last = !(msg.flags & MPP_FLAGS_MULTI_MSG) ||
		       (msg.flags & MPP_FLAGS_LAST_MSG);

		if (msg.cmd == MPP_CMD_SET_SESSION_FD) {
			ret = rk_mpp_switch_session(&session, &held_fd, &msg);
			batch.cur_job = NULL;
			if (ret || last)
				break;
			continue;
		}

		if (batch.req_cnt >= RK_MPP_MAX_MSG_NUM) {
			ret = -EINVAL;
			break;
		}
		batch.req_cnt++;

		req.cmd = msg.cmd;
		req.flags = msg.flags;
		req.size = msg.size;
		req.offset = msg.offset;
		req.data = (void __user *)(uintptr_t)msg.data_ptr;

		ret = rk_mpp_process_request(session, &req, &batch);
		if (ret || last)
			break;
	}

	if (!ret)
		ret = rk_mpp_execute_jobs(&batch);

	rk_mpp_batch_release_jobs(&batch);

	if (fd_file(held_fd))
		fdput(held_fd);

	return ret;
}

static long rk_mpp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rk_mpp_session *session = filp->private_data;

	if (!session)
		return -EINVAL;

	atomic_inc(&session->srv->ioctl_count);

	return rk_mpp_collect_msgs(session, cmd, (void __user *)arg);
}

static int rk_mpp_open(struct inode *inode, struct file *filp)
{
	struct rk_mpp_session *session;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return -ENOMEM;

	session->srv = &rk_mpp_srv;
	session->id = (u32)atomic_inc_return(&rk_mpp_srv.next_session_id);
	session->client_type = RK_MPP_DEVICE_BUTT;
	mutex_init(&session->lock);
	init_waitqueue_head(&session->wait);
	refcount_set(&session->refs, 1);
	INIT_LIST_HEAD(&session->imports);
	INIT_LIST_HEAD(&session->active_jobs);
	filp->private_data = session;

	return nonseekable_open(inode, filp);
}

static int rk_mpp_release(struct inode *inode, struct file *filp)
{
	struct rk_mpp_session *session = filp->private_data;

	if (!session)
		return 0;

	rk_mpp_session_abort_jobs(session);
	rk_mpp_session_release_imports(session);
	rk_mpp_session_put(session);
	filp->private_data = NULL;

	return 0;
}

static const struct file_operations rk_mpp_fops = {
	.owner		= THIS_MODULE,
	.open		= rk_mpp_open,
	.release	= rk_mpp_release,
	.unlocked_ioctl	= rk_mpp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= rk_mpp_ioctl,
#endif
	.llseek		= noop_llseek,
};

static void rk_mpp_hw_pm_disable(void *data)
{
	struct device *dev = data;

	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
}

static void rk_mpp_of_node_put(void *data)
{
	of_node_put(data);
}

static int rk_mpp_hw_alloc_rcb(struct rk_mpp_hw *hw)
{
	struct device *dev = hw->dev;
	u32 vals[2];
	size_t size;
	int ret;

	if (!hw->match->contributes_support)
		return 0;

	ret = device_property_read_u32_array(dev, "rockchip,rcb-iova",
					     vals, ARRAY_SIZE(vals));
	if (ret)
		return 0;

	size = PAGE_ALIGN(vals[1]);
	if (!size)
		return 0;

	hw->rcb_vaddr = dmam_alloc_coherent(dev, size, &hw->rcb_iova,
					    GFP_KERNEL);
	if (!hw->rcb_vaddr)
		return -ENOMEM;
	hw->rcb_size = size;

	of_property_read_u32(dev->of_node, "rockchip,rcb-min-width",
			     &hw->rcb_min_width);

	dev_info(dev, "rcb scratch dma %pad size %zu min_width %u\n",
		 &hw->rcb_iova, hw->rcb_size, hw->rcb_min_width);

	return 0;
}

static int rk_mpp_hw_read_id(struct rk_mpp_hw *hw)
{
	int ret;

	if (!hw->match->contributes_support || !hw->regs[0])
		return 0;

	ret = rk_mpp_hw_power_on(hw);
	if (ret)
		return ret;

	hw->hw_id = readl(hw->regs[0]);
	rk_mpp_hw_power_off(hw);

	return 0;
}

static int rk_mpp_hw_probe(struct platform_device *pdev)
{
	const struct rk_mpp_hw_match *match;
	struct rk_mpp_hw *hw;
	struct device *dev = &pdev->dev;
	int alias_id;
	int ret;
	int i;

	match = of_device_get_match_data(dev);
	if (!match)
		return -ENODEV;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	hw->dev = dev;
	hw->match = match;
	hw->irq = -1;
	hw->taskqueue_node = U32_MAX;
	refcount_set(&hw->refs, 1);
	init_completion(&hw->released);
	mutex_init(&hw->run_lock);
	spin_lock_init(&hw->lock);
	INIT_DELAYED_WORK(&hw->timeout_work, rk_mpp_hw_timeout_work);

	if (of_find_property(dev->of_node, "rockchip,ccu", NULL)) {
		hw->ccu_node = of_parse_phandle(dev->of_node, "rockchip,ccu", 0);
		if (!hw->ccu_node)
			return -EINVAL;

		ret = devm_add_action_or_reset(dev, rk_mpp_of_node_put,
					       hw->ccu_node);
		if (ret)
			return ret;
	}

	for (i = 0; i < RK_MPP_MAX_HW_REGS; i++) {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			break;

		hw->regs[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(hw->regs[i]))
			return PTR_ERR(hw->regs[i]);

		hw->reg_size[i] = resource_size(res);
		hw->num_regs++;
	}

	ret = platform_get_irq_optional(pdev, 0);
	if (ret < 0 && ret != -ENXIO)
		return ret;
	if (ret >= 0)
		hw->irq = ret;

	ret = devm_clk_bulk_get_all(dev, &hw->clks);
	if (ret < 0)
		return ret;
	hw->num_clks = ret;

	hw->resets = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(hw->resets))
		return PTR_ERR(hw->resets);

	ret = rk_mpp_hw_alloc_rcb(hw);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 200);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, rk_mpp_hw_pm_disable, dev);
	if (ret)
		return ret;

	ret = rk_mpp_hw_read_id(hw);
	if (ret)
		return ret;

	if (match->ops && match->ops->irq) {
		if (hw->irq < 0)
			return -ENODEV;
		ret = devm_request_threaded_irq(dev, hw->irq, rk_mpp_hw_irq,
						rk_mpp_hw_irq_thread,
						IRQF_ONESHOT, dev_name(dev), hw);
		if (ret)
			return ret;
	}

	of_property_read_u32(dev->of_node, "rockchip,taskqueue-node",
			     &hw->taskqueue_node);
	of_property_read_u32(dev->of_node, "rockchip,task-capacity",
			     &hw->task_capacity);
	of_property_read_u32(dev->of_node, "rockchip,core-mask",
			     &hw->core_mask);

	mutex_lock(&rk_mpp_srv.hw_lock);
	if (match->alias) {
		alias_id = of_alias_get_id(dev->of_node, match->alias);
		hw->core_id = alias_id >= 0 ? alias_id :
			rk_mpp_next_core_id_locked(&rk_mpp_srv, match);
	} else {
		hw->core_id = rk_mpp_next_core_id_locked(&rk_mpp_srv, match);
	}
	hw->online = true;
	list_add_tail(&hw->link, &rk_mpp_srv.hw_list);
	rk_mpp_refresh_hw_support_locked(&rk_mpp_srv);
	mutex_unlock(&rk_mpp_srv.hw_lock);

	platform_set_drvdata(pdev, hw);

	dev_info(dev, "bound %s core %d hw_id %#x irq %d regs %d clocks %d%s\n",
		 match->name, hw->core_id, hw->hw_id, hw->irq,
		 hw->num_regs, hw->num_clks,
		 hw->ccu_node ? " ccu-gated" : "");

	return 0;
}

static void rk_mpp_hw_remove(struct platform_device *pdev)
{
	struct rk_mpp_hw *hw = platform_get_drvdata(pdev);

	mutex_lock(&rk_mpp_srv.hw_lock);
	WRITE_ONCE(hw->online, false);
	list_del_init(&hw->link);
	rk_mpp_refresh_hw_support_locked(&rk_mpp_srv);
	mutex_unlock(&rk_mpp_srv.hw_lock);

	if (!hw->match->contributes_support)
		rk_mpp_hw_abort_ccu_dependents(hw);
	rk_mpp_hw_abort_queued(hw, -ENODEV);
	rk_mpp_hw_abort_active(hw, -ENODEV);
	rk_mpp_hw_put(hw);
	wait_for_completion(&hw->released);
}

static struct platform_driver rk_mpp_hw_driver = {
	.probe = rk_mpp_hw_probe,
	.remove = rk_mpp_hw_remove,
	.driver = {
		.name = "rk-mpp-rewrite-hw",
		.of_match_table = rk_mpp_hw_of_match,
	},
};

static int __init rk_mpp_init(void)
{
	int ret;

	mutex_init(&rk_mpp_srv.hw_lock);
	mutex_init(&rk_mpp_srv.sched_lock);
	spin_lock_init(&rk_mpp_srv.rkvenc_dchs_lock);
	INIT_LIST_HEAD(&rk_mpp_srv.hw_list);
	INIT_LIST_HEAD(&rk_mpp_srv.queued_jobs);
	INIT_WORK(&rk_mpp_srv.sched_work, rk_mpp_scheduler_work);

	ret = platform_driver_register(&rk_mpp_hw_driver);
	if (ret)
		return ret;

	rk_mpp_srv.miscdev.minor = MISC_DYNAMIC_MINOR;
	rk_mpp_srv.miscdev.name = "mpp_service";
	rk_mpp_srv.miscdev.fops = &rk_mpp_fops;

	ret = misc_register(&rk_mpp_srv.miscdev);
	if (ret)
		goto err_unregister_hw;

	ret = rk_mpp_create_procfs(&rk_mpp_srv);
	if (ret)
		goto err_deregister_misc;

	rk_mpp_srv.debugfs_root = debugfs_create_dir("rk_mpp_rewrite", NULL);
	debugfs_create_u32("hw_support", 0444, rk_mpp_srv.debugfs_root,
			   &rk_mpp_srv.hw_support);
	debugfs_create_u32("bound_hw_count", 0444, rk_mpp_srv.debugfs_root,
			   &rk_mpp_srv.bound_hw_count);
	debugfs_create_atomic_t("ioctl_count", 0444, rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.ioctl_count);
	debugfs_create_atomic_t("unsupported_count", 0444, rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.unsupported_count);
	debugfs_create_atomic_t("import_count", 0444, rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.import_count);
	debugfs_create_atomic_t("submitted_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.submitted_job_count);
	debugfs_create_atomic_t("queued_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.queued_job_count);
	debugfs_create_atomic_t("timeout_count", 0444, rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.timeout_count);

	pr_info("registered /dev/mpp_service (%s), hw_support=0x%08x\n",
		RK_MPP_REWRITE_VERSION, rk_mpp_srv.hw_support);

	return 0;

err_deregister_misc:
	misc_deregister(&rk_mpp_srv.miscdev);
err_unregister_hw:
	platform_driver_unregister(&rk_mpp_hw_driver);
	return ret;
}

static void __exit rk_mpp_exit(void)
{
	rk_mpp_remove_procfs(&rk_mpp_srv);
	debugfs_remove_recursive(rk_mpp_srv.debugfs_root);
	misc_deregister(&rk_mpp_srv.miscdev);
	platform_driver_unregister(&rk_mpp_hw_driver);
	flush_work(&rk_mpp_srv.sched_work);
}

module_init(rk_mpp_init);
module_exit(rk_mpp_exit);

MODULE_IMPORT_NS("DMA_BUF");
MODULE_DESCRIPTION("Minimal Rockchip MPP service compatibility rewrite");
MODULE_AUTHOR("OpenAI");
MODULE_LICENSE("GPL");
MODULE_VERSION(RK_MPP_REWRITE_VERSION);
