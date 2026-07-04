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
#include <linux/bitmap.h>
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
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/ktime.h>
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
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <soc/rockchip/rockchip_iommu.h>
#include <uapi/linux/rk-mpp.h>

#if IS_ENABLED(CONFIG_ROCKCHIP_MPP_REWRITE_KUNIT_TEST)
#include <linux/mm.h>
#include <linux/mman.h>
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
#define RK_MPP_CORE_COUNTER_COUNT	4
#define RK_MPP_RKVDEC_MAX_CCU_CORES	4
#define RK_MPP_RKVDEC_PERF_SEL_NUM	64
#define RK_MPP_RKVDEC_LINK_REGION	1
#define RK_MPP_RKVDEC_LINK_NODE_ALIGN	256
#define RK_MPP_RKVDEC_LINK_WRITE_PARTS	3
#define RK_MPP_RKVDEC_LINK_READ_PARTS	2
#define RK_MPP_RKVDEC_LINK_ADD_CFG_NUM	1
#define RK_MPP_RKVDEC_LINK_IRQ_RAW	BIT(9)
#define RK_MPP_RKVDEC_LINK_IP_TIMEOUT	0x007fffff
#define RK_MPP_RKVDEC_LINK_CORE_WORK_MODE	BIT(16)
#define RK_MPP_RKVDEC_LINK_CCU_WORK_MODE	BIT(17)
#define RK_MPP_RKVDEC_LINK_FIX_RCB	BIT(20)
#define RK_MPP_RKVDEC_CCU_MODE_SOFT	1
#define RK_MPP_RKVDEC_CCU_MODE_HARD	2
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

#define RK_MPP_MSG_V1_ABI_SIZE			24
#define RK_MPP_MSG_V1_DATA_PTR_ABI_OFFSET	16
#define RK_MPP_BAT_MSG_ABI_SIZE		16
#define RK_MPP_BAT_MSG_RET_ABI_OFFSET		12

static_assert(sizeof(struct rk_mpp_msg_v1) == RK_MPP_MSG_V1_ABI_SIZE);
static_assert(offsetof(struct rk_mpp_msg_v1, data_ptr) ==
	      RK_MPP_MSG_V1_DATA_PTR_ABI_OFFSET);
static_assert(sizeof(struct mpp_bat_msg) == RK_MPP_BAT_MSG_ABI_SIZE);
static_assert(offsetof(struct mpp_bat_msg, ret) ==
	      RK_MPP_BAT_MSG_RET_ABI_OFFSET);
static_assert(_IOC_TYPE(MPP_IOC_CFG_V1) == MPP_IOC_MAGIC);
static_assert(_IOC_NR(MPP_IOC_CFG_V1) == 1);
static_assert(_IOC_DIR(MPP_IOC_CFG_V1) == _IOC_WRITE);
static_assert(_IOC_SIZE(MPP_IOC_CFG_V1) == sizeof(unsigned int));

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

struct rk_mpp_rkvdec2_link_part {
	u16 table_word;
	u16 reg_word;
	u16 word_count;
};

struct rk_mpp_rkvdec2_link_info {
	u16 table_words;
	u16 next_word;
	u16 readback_word;
	s16 debug_word;
	s16 seg0_word;
	s16 seg1_word;
	s16 seg2_word;
	s16 second_en_word;
	u16 irq_status_word;
	u16 cycle_word;
	u8 write_part_count;
	u8 read_part_count;
	struct rk_mpp_rkvdec2_link_part write_parts[RK_MPP_RKVDEC_LINK_WRITE_PARTS];
	struct rk_mpp_rkvdec2_link_part read_parts[RK_MPP_RKVDEC_LINK_READ_PARTS];
	u32 next_addr_base;
	u32 ip_reset_base;
	u32 ip_reset_en;
	u32 irq_base;
	u32 irq_mask;
	u32 status_base;
	u32 status_mask;
	u32 err_mask;
	u32 ip_reset_mask;
	u32 ip_time_base;
	u32 en_base;
	u32 ip_en_base;
	u32 ip_en_val;
	bool sw_iommu_zap;
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
	struct list_head fault_link;
	struct device *dev;
	const struct rk_mpp_hw_match *match;
	struct device_node *iommu_node;
	struct iommu_domain *iommu_domain;
	void __iomem *regs[RK_MPP_MAX_HW_REGS];
	resource_size_t reg_size[RK_MPP_MAX_HW_REGS];
	struct clk_bulk_data *clks;
	u32 *normal_rates;
	struct reset_control *resets;
	struct delayed_work timeout_work;
	struct mutex run_lock; /* serializes start, abort, timeout, and completion */
	spinlock_t lock;
	struct rk_mpp_job *active_job;
	struct device_node *ccu_node;
	struct list_head rkvdec_ccu_jobs;
	struct list_head rkvdec_link_jobs;
	u32 taskqueue_node;
	u32 task_capacity;
	u32 core_mask;
	u32 rkvdec_ccu_mode;
	u32 irq_status;
	u32 hw_id;
	void *rcb_vaddr;
	dma_addr_t rcb_iova;
	size_t rcb_size;
	struct rk_mpp_rcb_desc rcb_descs[RK_MPP_MAX_RCB_ELEMS];
	u32 rcb_count;
	void *rkvdec_link_vaddr;
	dma_addr_t rkvdec_link_iova;
	size_t rkvdec_link_size;
	unsigned long *rkvdec_link_used;
	u32 rkvdec_link_node_size;
	u32 rkvdec_link_capacity;
	u32 rcb_min_width;
	refcount_t refs;
	atomic_t queued_job_count;
	atomic_t iommu_fault_pending;
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
	spinlock_t fault_lock; /* protects fault_hws in fault handler context */
	spinlock_t rkvenc_dchs_lock;
	struct list_head hw_list;
	struct list_head fault_hws;
	struct list_head queued_jobs;
	struct work_struct sched_work;
	atomic_t ioctl_count;
	atomic_t unsupported_count;
	atomic_t import_count;
	atomic_t submitted_job_count;
	atomic_t scheduled_job_count;
	atomic_t scheduled_rkvenc_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t scheduled_rkvdec_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t dispatched_job_count;
	atomic_t dispatched_rkvenc_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t dispatched_rkvdec_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t started_job_count;
	atomic_t started_rkvenc_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t started_rkvdec_core_count[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t hw_total_ns;
	atomic64_t hw_max_ns;
	atomic64_t hw_total_rkvenc_core_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t hw_total_rkvdec_core_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t hw_max_rkvenc_core_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t hw_max_rkvdec_core_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t queued_job_count;
	atomic_t timeout_count;
	atomic_t iommu_fault_count;
	atomic_t iommu_refresh_count;
	atomic_t next_session_id;
	struct rk_mpp_rkvenc_dchs_entry rkvenc_dchs[RK_MPP_RKVENC_MAX_DCHS_CORES];
	u32 hw_support;
	u32 bound_hw_count;
	u32 core_select_seq;
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
	struct list_head rkvdec_ccu_node;
	struct list_head rkvdec_link_node;
	struct rk_mpp_session *session;
	enum rk_mpp_job_state state;
	struct rk_mpp_hw *hw;
	struct rk_mpp_hw *rkvdec_ccu;
	refcount_t refs;
	u32 id;
	u32 req_cnt;
	u32 set_cnt;
	u32 poll_cnt;
	u32 flags;
	int result;
	u32 rkvdec_stream_addr;
	void *rkvdec_link_vaddr;
	dma_addr_t rkvdec_link_iova;
	u32 rkvdec_link_index;
	u32 rkvdec_ccu_core_work;
	u32 rkvdec_ccu_cfg_addr;
	u32 rkvdec_ccu_link_mode;
	u32 rkvdec_ccu_ctrl;
	u32 rkvdec_ccu_work;
	u32 rkvdec_ccu_cfg_done;
	u32 rkvdec_link_irq_mode;
	u32 rkvenc_dchs_core_id;
	struct rk_mpp_hw *rkvdec_ccu_powered_cores[RK_MPP_RKVDEC_MAX_CCU_CORES];
	u32 rkvdec_ccu_powered_core_count;
	bool poll_irq;
	bool canceled;
	bool rkvdec_link_active;
	bool rkvdec_link_listed;
	bool rkvdec_ccu_listed;
	bool rkvdec_ccu_desc_valid;
	bool rkvdec_ccu_powered;
	bool rkvdec_ccu_started;
	bool rkvenc_dchs_active;
	bool rkvenc_slice_mode;
	bool rkvenc_slice_done;
	bool rkvenc_slice_overflow;
	u64 hw_start_ns;
	u64 hw_elapsed_ns;
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
static void rk_mpp_batch_release_jobs(struct rk_mpp_batch_state *batch);
static struct rk_mpp_job *
rk_mpp_batch_get_job(struct rk_mpp_batch_state *batch,
		     struct rk_mpp_session *session);
static void rk_mpp_job_activate(struct rk_mpp_job *job);
static void rk_mpp_job_get(struct rk_mpp_job *job);
static void rk_mpp_job_put(struct rk_mpp_job *job);
static bool rk_mpp_hw_take_active_if(struct rk_mpp_hw *hw,
				     struct rk_mpp_job *match,
				     u32 *irq_status);
static void rk_mpp_hw_timeout_work(struct work_struct *work);
static void rk_mpp_rkvdec2_force_stop_ccu(struct rk_mpp_hw *ccu);
static void rk_mpp_hw_abort_ccu_dependents(struct rk_mpp_hw *ccu);
static void
rk_mpp_hw_abort_ccu_active_dependents(struct rk_mpp_hw *ccu,
				      struct rk_mpp_hw *skip, int result);
static bool rk_mpp_job_rkvenc_slice_mode(struct rk_mpp_job *job);
static bool rk_mpp_job_rkvenc_slice_ready(struct rk_mpp_job *job);
static bool rk_mpp_job_rkvenc_slice_done(struct rk_mpp_job *job);
static void rk_mpp_job_push_rkvenc_slice(struct rk_mpp_job *job, u32 value);
static int rk_mpp_job_pop_rkvenc_slice(struct rk_mpp_job *job, u32 *value);
static int rk_mpp_job_apply_rcb_info(struct rk_mpp_job *job);
static void rk_mpp_scheduler_work(struct work_struct *work);
static void rk_mpp_session_abort_jobs(struct rk_mpp_session *session);
static int rk_mpp_session_poll_job(struct rk_mpp_session *session, u32 flags);
static int rk_mpp_process_request(struct rk_mpp_session *session,
				  struct mpp_request *req,
				  struct rk_mpp_batch_state *batch);
static int rk_mpp_release(struct inode *inode, struct file *filp);
static struct rk_mpp_hw *
rk_mpp_iommu_find_fault_hw(struct list_head *fault_hws,
			   struct iommu_domain *domain,
			   struct device *iommu_dev);
static struct rk_mpp_service rk_mpp_srv;

static int rk_mpp_core_counter_index(const struct rk_mpp_hw *hw)
{
	if (!hw || !hw->match || !hw->match->contributes_support)
		return -EINVAL;
	if (hw->match->type != RK_MPP_DEVICE_RKVENC &&
	    hw->match->type != RK_MPP_DEVICE_RKVDEC)
		return -EINVAL;
	if (hw->core_id < 0 || hw->core_id >= RK_MPP_CORE_COUNTER_COUNT)
		return -EINVAL;

	return hw->core_id;
}

static void
rk_mpp_count_core(atomic_t rkvenc_counters[RK_MPP_CORE_COUNTER_COUNT],
		  atomic_t rkvdec_counters[RK_MPP_CORE_COUNTER_COUNT],
		  const struct rk_mpp_hw *hw)
{
	int index = rk_mpp_core_counter_index(hw);

	if (index < 0)
		return;

	if (hw->match->type == RK_MPP_DEVICE_RKVENC)
		atomic_inc(&rkvenc_counters[index]);
	else
		atomic_inc(&rkvdec_counters[index]);
}

static void rk_mpp_atomic64_max(atomic64_t *counter, u64 value)
{
	s64 old = atomic64_read(counter);

	while ((u64)old < value) {
		s64 prev = atomic64_cmpxchg(counter, old, value);

		if (prev == old)
			break;
		old = prev;
	}
}

static int rk_mpp_debugfs_atomic64_get(void *data, u64 *val)
{
	*val = atomic64_read(data);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(rk_mpp_debugfs_atomic64_fops,
			 rk_mpp_debugfs_atomic64_get, NULL, "%llu\n");

static void rk_mpp_debugfs_create_atomic64(const char *name, atomic64_t *value)
{
	debugfs_create_file(name, 0444, rk_mpp_srv.debugfs_root, value,
			    &rk_mpp_debugfs_atomic64_fops);
}

static void
rk_mpp_count_core_ns(atomic64_t rkvenc_counters[RK_MPP_CORE_COUNTER_COUNT],
		     atomic64_t rkvdec_counters[RK_MPP_CORE_COUNTER_COUNT],
		     const struct rk_mpp_hw *hw, u64 value, bool max)
{
	int index = rk_mpp_core_counter_index(hw);
	atomic64_t *counter;

	if (index < 0)
		return;

	if (hw->match->type == RK_MPP_DEVICE_RKVENC)
		counter = &rkvenc_counters[index];
	else
		counter = &rkvdec_counters[index];

	if (max)
		rk_mpp_atomic64_max(counter, value);
	else
		atomic64_add(value, counter);
}

static void rk_mpp_job_note_hw_done(struct rk_mpp_job *job)
{
	u64 elapsed;
	u64 start = job->hw_start_ns;

	if (!start)
		return;

	elapsed = ktime_get_ns() - start;
	job->hw_elapsed_ns += elapsed;
	job->hw_start_ns = 0;
	rk_mpp_count_core_ns(job->session->srv->hw_total_rkvenc_core_ns,
			     job->session->srv->hw_total_rkvdec_core_ns,
			     job->hw, elapsed, false);
	rk_mpp_count_core_ns(job->session->srv->hw_max_rkvenc_core_ns,
			     job->session->srv->hw_max_rkvdec_core_ns,
			     job->hw, elapsed, true);
}

static void rk_mpp_job_record_hw_stats(struct rk_mpp_job *job)
{
	u64 elapsed = job->hw_elapsed_ns;

	if (!elapsed)
		return;

	atomic64_add(elapsed, &job->session->srv->hw_total_ns);
	rk_mpp_atomic64_max(&job->session->srv->hw_max_ns, elapsed);
	job->hw_elapsed_ns = 0;
}

static void rk_mpp_count_scheduled_core(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;

	atomic_inc(&srv->scheduled_job_count);
	rk_mpp_count_core(srv->scheduled_rkvenc_core_count,
			  srv->scheduled_rkvdec_core_count, job->hw);
}

static void rk_mpp_count_dispatched_core(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;

	atomic_inc(&srv->dispatched_job_count);
	rk_mpp_count_core(srv->dispatched_rkvenc_core_count,
			  srv->dispatched_rkvdec_core_count, job->hw);
}

static void rk_mpp_count_started_core(struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job->session->srv;

	rk_mpp_job_note_hw_done(job);
	job->hw_start_ns = ktime_get_ns();
	atomic_inc(&srv->started_job_count);
	rk_mpp_count_core(srv->started_rkvenc_core_count,
			  srv->started_rkvdec_core_count, job->hw);
}

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
#define RK_MPP_RKVDEC_LINK_STATUS_WORD		15
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
#define RK_MPP_RKVDEC_MAX_READS_BASE		0x0518
#define RK_MPP_RKVDEC_MAX_READS			0x1c
#define RK_MPP_RKVDEC_CACHE0_SIZE_BASE		0x051c
#define RK_MPP_RKVDEC_CACHE1_SIZE_BASE		0x055c
#define RK_MPP_RKVDEC_CACHE2_SIZE_BASE		0x059c
#define RK_MPP_RKVDEC_CLR_CACHE0_BASE		0x0510
#define RK_MPP_RKVDEC_CLR_CACHE1_BASE		0x0550
#define RK_MPP_RKVDEC_CLR_CACHE2_BASE		0x0590
#define RK_MPP_RKVDEC_CACHE_CFG			(BIT(0) | BIT(1) | BIT(4))
#define RK_MPP_RKVDEC_CCU_CTRL_BASE		0x0000
#define RK_MPP_RKVDEC_CCU_AUTOGATE		BIT(0)
#define RK_MPP_RKVDEC_CCU_CFG_ADDR_BASE		0x0004
#define RK_MPP_RKVDEC_CCU_LINK_MODE_BASE	0x0008
#define RK_MPP_RKVDEC_CCU_ADD_MODE		BIT(31)
#define RK_MPP_RKVDEC_CCU_CFG_DONE_BASE		0x000c
#define RK_MPP_RKVDEC_CCU_CFG_DONE		BIT(0)
#define RK_MPP_RKVDEC_CCU_WORK_BASE		0x0018
#define RK_MPP_RKVDEC_CCU_WORK_EN		BIT(0)
#define RK_MPP_RKVDEC_CCU_WORK_MODE_BASE	0x0040
#define RK_MPP_RKVDEC_CCU_WORK_MODE		BIT(0)
#define RK_MPP_RKVDEC_CCU_CORE_WORK_BASE	0x0044
#define RK_MPP_RKVDEC_CCU_CORE_STA_BASE		0x0048

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

static const struct rk_mpp_rkvdec2_link_info rk_mpp_rkvdec2_vdpu383_link_info = {
	.table_words = 256,
	.next_word = 0,
	.readback_word = 1,
	.debug_word = 2,
	.seg0_word = 3,
	.seg1_word = 4,
	.seg2_word = 5,
	.second_en_word = -1,
	.irq_status_word = 16,
	.cycle_word = 27,
	.write_part_count = 3,
	.read_part_count = 2,
	.write_parts = {
		{ .table_word = 80, .reg_word = 8, .word_count = 24 },
		{ .table_word = 104, .reg_word = 64, .word_count = 44 },
		{ .table_word = 148, .reg_word = 128, .word_count = 108 },
	},
	.read_parts = {
		{ .table_word = 16, .reg_word = 15, .word_count = 1 },
		{ .table_word = 20, .reg_word = 320, .word_count = 40 },
	},
	.next_addr_base = 0x20,
	.ip_reset_base = 0x44,
	.ip_reset_en = BIT(0),
	.irq_base = 0x48,
	.irq_mask = 0x30000,
	.status_base = 0x4c,
	.status_mask = 0x3ff0000,
	.err_mask = 0x3fe,
	.ip_reset_mask = 0x8000000,
	.ip_time_base = 0x54,
	.en_base = 0x40,
	.ip_en_base = 0x58,
	.ip_en_val = 0x01000000,
	.sw_iommu_zap = true,
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

static bool rk_mpp_rkvdec2_ccu_mode_valid(u32 mode)
{
	return mode == RK_MPP_RKVDEC_CCU_MODE_SOFT ||
	       mode == RK_MPP_RKVDEC_CCU_MODE_HARD;
}

static u32 rk_mpp_rkvdec2_normalize_ccu_mode(u32 mode)
{
	if (rk_mpp_rkvdec2_ccu_mode_valid(mode))
		return mode;

	return RK_MPP_RKVDEC_CCU_MODE_SOFT;
}

static bool rk_mpp_rkvdec2_soft_ccu_enabled(const struct rk_mpp_hw *hw)
{
	return hw && hw->ccu_node &&
	       hw->rkvdec_ccu_mode == RK_MPP_RKVDEC_CCU_MODE_SOFT;
}

static bool rk_mpp_rkvdec2_hard_ccu_enabled(const struct rk_mpp_hw *hw)
{
	return hw && hw->ccu_node &&
	       hw->rkvdec_ccu_mode == RK_MPP_RKVDEC_CCU_MODE_HARD;
}

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

static struct rk_mpp_hw *rk_mpp_hw_get_ccu_for_core(struct rk_mpp_service *srv,
						    struct rk_mpp_hw *core)
{
	struct rk_mpp_hw *hw;
	struct rk_mpp_hw *ccu = NULL;

	if (!core->ccu_node)
		return NULL;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->dev->of_node == core->ccu_node && hw->online) {
			refcount_inc(&hw->refs);
			ccu = hw;
			break;
		}
	}
	mutex_unlock(&srv->hw_lock);

	return ccu;
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

static void rk_mpp_hw_get(struct rk_mpp_hw *hw)
{
	if (hw)
		refcount_inc(&hw->refs);
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

static u32 rk_mpp_hw_core_distance(const struct rk_mpp_hw *hw, u32 start)
{
	u32 core_id;

	if (hw->core_id < 0 || hw->core_id >= RK_MPP_CORE_COUNTER_COUNT)
		return U32_MAX;

	core_id = hw->core_id;
	return (core_id + RK_MPP_CORE_COUNTER_COUNT -
		(start % RK_MPP_CORE_COUNTER_COUNT)) %
	       RK_MPP_CORE_COUNTER_COUNT;
}

static bool rk_mpp_hw_tie_better(struct rk_mpp_hw *selected,
				 struct rk_mpp_hw *candidate, u32 start)
{
	if (!selected)
		return true;

	return rk_mpp_hw_core_distance(candidate, start) <
	       rk_mpp_hw_core_distance(selected, start);
}

static struct rk_mpp_hw *rk_mpp_hw_get_for_session(struct rk_mpp_session *session,
						   bool prefer_idle)
{
	struct rk_mpp_service *srv = session->srv;
	struct rk_mpp_hw *hw;
	struct rk_mpp_hw *selected = NULL;
	u32 selected_load = U32_MAX;
	u32 rr_start = srv->core_select_seq;

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
		if (!selected || load < selected_load ||
		    (load == selected_load &&
		     rk_mpp_hw_tie_better(selected, hw, rr_start))) {
			selected = hw;
			selected_load = load;
		}
	}
	if (selected) {
		refcount_inc(&selected->refs);
		if (prefer_idle && selected->core_id >= 0 &&
		    selected->core_id < RK_MPP_CORE_COUNTER_COUNT)
			srv->core_select_seq = selected->core_id + 1;
	}
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

struct rk_mpp_support_cmd {
	const char *name;
	__u32 cmd;
};

static const struct rk_mpp_support_cmd rk_mpp_support_cmds[] = {
	{ "QUERY_HW_SUPPORT:", MPP_CMD_QUERY_HW_SUPPORT },
	{ "QUERY_HW_ID:", MPP_CMD_QUERY_HW_ID },
	{ "QUERY_CMD_SUPPORT:", MPP_CMD_QUERY_CMD_SUPPORT },
	{ "QUERY_BUTT:", MPP_CMD_QUERY_BUTT },
	{ NULL, 0 },
	{ "INIT_CLIENT_TYPE:", MPP_CMD_INIT_CLIENT_TYPE },
	{ "INIT_DRIVER_DATA:", MPP_CMD_INIT_DRIVER_DATA },
	{ "INIT_TRANS_TABLE:", MPP_CMD_INIT_TRANS_TABLE },
	{ "INIT_BUTT:", MPP_CMD_INIT_BUTT },
	{ NULL, 0 },
	{ "SET_REG_WRITE:", MPP_CMD_SET_REG_WRITE },
	{ "SET_REG_READ:", MPP_CMD_SET_REG_READ },
	{ "SET_REG_ADDR_OFFSET:", MPP_CMD_SET_REG_ADDR_OFFSET },
	{ "SET_RCB_INFO:", MPP_CMD_SET_RCB_INFO },
	{ "SET_SESSION_FD:", MPP_CMD_SET_SESSION_FD },
	{ "SEND_BUTT:", MPP_CMD_SEND_BUTT },
	{ NULL, 0 },
	{ "POLL_HW_FINISH:", MPP_CMD_POLL_HW_FINISH },
	{ "POLL_HW_IRQ:", MPP_CMD_POLL_HW_IRQ },
	{ "POLL_BUTT:", MPP_CMD_POLL_BUTT },
	{ NULL, 0 },
	{ "RESET_SESSION:", MPP_CMD_RESET_SESSION },
	{ "TRANS_FD_TO_IOVA:", MPP_CMD_TRANS_FD_TO_IOVA },
	{ "RELEASE_FD:", MPP_CMD_RELEASE_FD },
	{ "SEND_CODEC_INFO:", MPP_CMD_SEND_CODEC_INFO },
	{ "SET_ERR_REF_HACK:", MPP_CMD_SET_ERR_REF_HACK },
	{ "CONTROL_BUTT:", MPP_CMD_CONTROL_BUTT },
};

static void rk_mpp_msg_v1_to_request(const struct rk_mpp_msg_v1 *msg,
				     struct mpp_request *req)
{
	req->cmd = msg->cmd;
	req->flags = msg->flags;
	req->size = msg->size;
	req->offset = msg->offset;
	req->data = (void __user *)(uintptr_t)msg->data_ptr;
}

static int rk_mpp_support_cmd_show(struct seq_file *s, void *unused)
{
	__u32 i;

	seq_puts(s, "------------- SUPPORT CMD -------------\n");
	for (i = 0; i < ARRAY_SIZE(rk_mpp_support_cmds); i++) {
		const struct rk_mpp_support_cmd *cmd = &rk_mpp_support_cmds[i];

		if (!cmd->name)
			seq_puts(s, "----\n");
		else
			seq_printf(s, "%-22s0x%08x\n", cmd->name, cmd->cmd);
	}

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

static size_t
rk_mpp_rkvdec2_link_node_size(const struct rk_mpp_rkvdec2_link_info *info)
{
	return ALIGN(info->table_words * sizeof(u32),
		     RK_MPP_RKVDEC_LINK_NODE_ALIGN);
}

static bool
rk_mpp_rkvdec2_link_irq_decode(const struct rk_mpp_rkvdec2_link_info *info,
			       u32 irq_val, u32 status_val, u32 *irq_status)
{
	u32 irq_bits = info->irq_mask >> 16;
	u32 status_bits = info->status_mask >> 16;

	if (!(irq_val & (irq_bits | RK_MPP_RKVDEC_LINK_IRQ_RAW)))
		return false;

	*irq_status = status_val;

	return !!(status_val & status_bits) ||
	       !!(irq_val & RK_MPP_RKVDEC_LINK_IRQ_RAW);
}

static int
rk_mpp_rkvdec2_link_part_check(const struct rk_mpp_rkvdec2_link_part *part,
			       u32 table_words)
{
	if (part->table_word > table_words ||
	    part->word_count > table_words - part->table_word)
		return -EOVERFLOW;

	return 0;
}

static int
rk_mpp_rkvdec2_link_reg_part_check(const struct rk_mpp_rkvdec2_link_part *part,
				   u32 table_words, u32 reg_words)
{
	int ret;

	ret = rk_mpp_rkvdec2_link_part_check(part, table_words);
	if (ret)
		return ret;
	if (part->reg_word > reg_words ||
	    part->word_count > reg_words - part->reg_word)
		return -EINVAL;

	return 0;
}

static int
rk_mpp_rkvdec2_fill_link_table(const struct rk_mpp_reg_image *image,
			       const struct rk_mpp_rkvdec2_link_info *info,
			       u32 *table, dma_addr_t table_iova,
			       dma_addr_t next_iova)
{
	u32 i;
	int ret;

	if (!table)
		return -EINVAL;

	memset(table, 0, rk_mpp_rkvdec2_link_node_size(info));

	for (i = 0; i < info->write_part_count; i++) {
		const struct rk_mpp_rkvdec2_link_part *part =
			&info->write_parts[i];

		ret = rk_mpp_rkvdec2_link_reg_part_check(part,
							 info->table_words,
							 image->reg_words);
		if (ret)
			return ret;
		memcpy(&table[part->table_word], &image->regs[part->reg_word],
		       part->word_count * sizeof(u32));
	}

	for (i = 0; i < info->read_part_count; i++) {
		const struct rk_mpp_rkvdec2_link_part *part =
			&info->read_parts[i];

		ret = rk_mpp_rkvdec2_link_part_check(part, info->table_words);
		if (ret)
			return ret;
		memset(&table[part->table_word], 0,
		       part->word_count * sizeof(u32));
	}

	table[info->next_word] = lower_32_bits(next_iova);
	table[info->readback_word] = lower_32_bits(table_iova +
		info->read_parts[0].table_word * sizeof(u32));
	if (info->debug_word >= 0)
		table[info->debug_word] = table[info->readback_word];
	if (info->seg0_word >= 0)
		table[info->seg0_word] = lower_32_bits(table_iova +
			info->write_parts[0].table_word * sizeof(u32));
	if (info->seg1_word >= 0)
		table[info->seg1_word] = lower_32_bits(table_iova +
			info->write_parts[1].table_word * sizeof(u32));
	if (info->seg2_word >= 0)
		table[info->seg2_word] = lower_32_bits(table_iova +
			info->write_parts[2].table_word * sizeof(u32));

	return 0;
}

static int __maybe_unused
rk_mpp_rkvdec2_read_link_table(struct rk_mpp_reg_image *image,
			       const struct rk_mpp_rkvdec2_link_info *info,
			       const u32 *table, u32 irq_status)
{
	u32 i;
	int ret;

	if (!table)
		return -EINVAL;

	for (i = 0; i < info->read_part_count; i++) {
		const struct rk_mpp_rkvdec2_link_part *part =
			&info->read_parts[i];

		ret = rk_mpp_rkvdec2_link_reg_part_check(part,
							 info->table_words,
							 image->reg_words);
		if (ret)
			return ret;
		memcpy(&image->regs[part->reg_word], &table[part->table_word],
		       part->word_count * sizeof(u32));
	}

	if (image->reg_words <= RK_MPP_RKVDEC_LINK_STATUS_WORD)
		return -EINVAL;

	image->regs[RK_MPP_RKVDEC_LINK_STATUS_WORD] = irq_status;

	return 0;
}

static u32
rk_mpp_rkvdec2_link_table_irq_status(const struct rk_mpp_rkvdec2_link_info *info,
				     const u32 *table, u32 fallback)
{
	if (!table || info->irq_status_word >= info->table_words)
		return fallback;

	return table[info->irq_status_word] ?: fallback;
}

static int rk_mpp_rkvdec2_read_ccu_link_table(struct rk_mpp_job *job,
					      const struct rk_mpp_rkvdec2_link_info *info,
					      u32 fallback_irq_status)
{
	const u32 *table = job->rkvdec_link_vaddr;
	u32 irq_status;

	irq_status =
		rk_mpp_rkvdec2_link_table_irq_status(info, table,
						     fallback_irq_status);

	return rk_mpp_rkvdec2_read_link_table(&job->reg_image, info, table,
					      irq_status);
}

static bool
rk_mpp_rkvdec2_ccu_job_error(const struct rk_mpp_job *job,
			     const struct rk_mpp_rkvdec2_link_info *info)
{
	const struct rk_mpp_reg_image *image = &job->reg_image;

	if (!job->rkvdec_ccu_started ||
	    image->reg_words <= RK_MPP_RKVDEC_LINK_STATUS_WORD)
		return false;

	return image->regs[RK_MPP_RKVDEC_LINK_STATUS_WORD] & info->err_mask;
}

static bool rk_mpp_rkvdec2_ccu_regs_ready(struct rk_mpp_hw *ccu);
static u32 rk_mpp_rkvdec2_ccu_core_mask(struct rk_mpp_service *srv,
					struct rk_mpp_hw *ccu);
static bool
rk_mpp_rkvdec2_link_regs_ready(struct rk_mpp_hw *hw,
			       const struct rk_mpp_rkvdec2_link_info *info);
static bool rk_mpp_hw_reg_range_valid(struct rk_mpp_hw *hw, u32 region,
				      u32 offset, u32 size);
static int rk_mpp_hw_power_on(struct rk_mpp_hw *hw);
static void rk_mpp_hw_power_off(struct rk_mpp_hw *hw);
static void rk_mpp_job_get(struct rk_mpp_job *job);
static void rk_mpp_job_put(struct rk_mpp_job *job);

static void rk_mpp_rkvdec2_power_off_ccu_cores(struct rk_mpp_job *job)
{
	u32 i;

	for (i = 0; i < job->rkvdec_ccu_powered_core_count; i++) {
		struct rk_mpp_hw *hw = job->rkvdec_ccu_powered_cores[i];

		rk_mpp_hw_power_off(hw);
		rk_mpp_hw_put(hw);
		job->rkvdec_ccu_powered_cores[i] = NULL;
	}
	job->rkvdec_ccu_powered_core_count = 0;
}

static bool rk_mpp_rkvdec2_move_powered_ccu_cores(struct rk_mpp_job *from,
						  struct rk_mpp_job *to)
{
	u32 i;

	if (!from || !to || from == to ||
	    !from->rkvdec_ccu_powered_core_count ||
	    to->rkvdec_ccu_powered_core_count)
		return false;

	for (i = 0; i < from->rkvdec_ccu_powered_core_count; i++) {
		to->rkvdec_ccu_powered_cores[i] =
			from->rkvdec_ccu_powered_cores[i];
		from->rkvdec_ccu_powered_cores[i] = NULL;
	}
	to->rkvdec_ccu_powered_core_count =
		from->rkvdec_ccu_powered_core_count;
	from->rkvdec_ccu_powered_core_count = 0;

	return true;
}

static int rk_mpp_rkvdec2_power_on_ccu_cores(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *cores[RK_MPP_RKVDEC_MAX_CCU_CORES];
	struct rk_mpp_service *srv = job->session->srv;
	struct rk_mpp_hw *ccu = job->rkvdec_ccu;
	struct rk_mpp_hw *hw;
	u32 count = 0;
	u32 i;
	int ret;

	if (!srv || !ccu || !ccu->dev)
		return 0;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (!hw->online || hw == job->hw ||
		    hw->ccu_node != ccu->dev->of_node)
			continue;
		if (count >= ARRAY_SIZE(cores)) {
			ret = -EOPNOTSUPP;
			goto err_put_locked;
		}
		rk_mpp_hw_get(hw);
		cores[count++] = hw;
	}
	mutex_unlock(&srv->hw_lock);

	for (i = 0; i < count; i++) {
		ret = rk_mpp_hw_power_on(cores[i]);
		if (ret)
			goto err_power_off;
		job->rkvdec_ccu_powered_cores[i] = cores[i];
		job->rkvdec_ccu_powered_core_count++;
	}

	return 0;

err_power_off:
	while (i--) {
		rk_mpp_hw_power_off(cores[i]);
		rk_mpp_hw_put(cores[i]);
	}
	for (i = job->rkvdec_ccu_powered_core_count; i < count; i++)
		rk_mpp_hw_put(cores[i]);
	job->rkvdec_ccu_powered_core_count = 0;
	return ret;

err_put_locked:
	while (count--)
		rk_mpp_hw_put(cores[count]);
	mutex_unlock(&srv->hw_lock);
	return ret;
}

static void rk_mpp_rkvdec2_fix_core_rcb_regs(struct rk_mpp_hw *hw)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	void __iomem *link;
	dma_addr_t rcb_iova;
	u32 rcb_offset = 0;
	u32 irq_val;
	u32 i;

	if (!hw || !hw->rcb_iova || !hw->rcb_size || !hw->rcb_count ||
	    !rk_mpp_rkvdec2_link_regs_ready(hw, info))
		return;

	link = hw->regs[RK_MPP_RKVDEC_LINK_REGION];
	irq_val = readl_relaxed(link + info->irq_base);
	if (irq_val & RK_MPP_RKVDEC_LINK_FIX_RCB)
		return;

	for (i = 0; i < hw->rcb_count; i++) {
		const struct rk_mpp_rcb_desc *desc = &hw->rcb_descs[i];
		u32 reg_offset;
		u32 next_offset;

		if (check_add_overflow(rcb_offset, desc->size,
				       &next_offset) ||
		    next_offset > hw->rcb_size)
			return;
		if (check_mul_overflow(desc->index, (u32)sizeof(u32),
				       &reg_offset) ||
		    !rk_mpp_hw_reg_range_valid(hw, 0, reg_offset,
					       sizeof(u32)))
			return;
		if (check_add_overflow(hw->rcb_iova,
				       (dma_addr_t)rcb_offset, &rcb_iova))
			return;

		writel_relaxed(lower_32_bits(rcb_iova),
			       hw->regs[0] + reg_offset);
		rcb_offset = next_offset;
	}

	irq_val = readl_relaxed(link + info->irq_base);
	irq_val |= RK_MPP_RKVDEC_LINK_FIX_RCB;
	writel_relaxed(irq_val, link + info->irq_base);
}

static void rk_mpp_rkvdec2_prepare_core_for_ccu(struct rk_mpp_hw *hw)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	void __iomem *link;
	u32 irq_val;

	if (!rk_mpp_rkvdec2_link_regs_ready(hw, info))
		return;

	rk_mpp_rkvdec2_fix_core_rcb_regs(hw);

	link = hw->regs[RK_MPP_RKVDEC_LINK_REGION];
	irq_val = readl_relaxed(link + info->irq_base);
	irq_val |= RK_MPP_RKVDEC_LINK_CCU_WORK_MODE;
	writel_relaxed(irq_val, link + info->irq_base);
}

static dma_addr_t rk_mpp_rkvdec2_next_unused_link_iova(struct rk_mpp_hw *hw)
{
	unsigned long index;

	if (!hw->rkvdec_link_used || !hw->rkvdec_link_capacity)
		return 0;

	index = find_first_zero_bit(hw->rkvdec_link_used,
				    hw->rkvdec_link_capacity);
	if (index >= hw->rkvdec_link_capacity)
		return 0;

	return hw->rkvdec_link_iova + index * hw->rkvdec_link_node_size;
}

static void rk_mpp_rkvdec2_relink_tables_locked(struct rk_mpp_hw *hw)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;

	list_for_each_entry(job, &hw->rkvdec_link_jobs, rkvdec_link_node) {
		struct rk_mpp_job *next;
		dma_addr_t next_iova;
		u32 *table = job->rkvdec_link_vaddr;

		if (!table)
			continue;

		if (list_is_last(&job->rkvdec_link_node, &hw->rkvdec_link_jobs)) {
			next_iova = rk_mpp_rkvdec2_next_unused_link_iova(hw);
		} else {
			next = list_next_entry(job, rkvdec_link_node);
			next_iova = next->rkvdec_link_iova;
		}

		table[info->next_word] = lower_32_bits(next_iova);
	}
}

static void rk_mpp_rkvdec2_link_table_list_add(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	unsigned long flags;

	if (!hw || job->rkvdec_link_listed)
		return;

	spin_lock_irqsave(&hw->lock, flags);
	list_add_tail(&job->rkvdec_link_node, &hw->rkvdec_link_jobs);
	job->rkvdec_link_listed = true;
	rk_mpp_rkvdec2_relink_tables_locked(hw);
	spin_unlock_irqrestore(&hw->lock, flags);
}

static void rk_mpp_rkvdec2_ccu_relink_tables_locked(struct rk_mpp_hw *ccu)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;

	list_for_each_entry(job, &ccu->rkvdec_ccu_jobs, rkvdec_ccu_node) {
		struct rk_mpp_job *next;
		dma_addr_t next_iova;
		u32 *table = job->rkvdec_link_vaddr;

		if (!table)
			continue;

		if (list_is_last(&job->rkvdec_ccu_node,
				 &ccu->rkvdec_ccu_jobs)) {
			next_iova = rk_mpp_rkvdec2_next_unused_link_iova(job->hw);
		} else {
			next = list_next_entry(job, rkvdec_ccu_node);
			next_iova = next->rkvdec_link_iova;
		}

		table[info->next_word] = lower_32_bits(next_iova);
	}
}

static bool rk_mpp_rkvdec2_ccu_has_jobs(struct rk_mpp_hw *ccu)
{
	unsigned long flags;
	bool has_jobs;

	if (!ccu)
		return false;

	spin_lock_irqsave(&ccu->lock, flags);
	has_jobs = !list_empty(&ccu->rkvdec_ccu_jobs);
	spin_unlock_irqrestore(&ccu->lock, flags);

	return has_jobs;
}

static bool
rk_mpp_rkvdec2_ccu_job_done(const struct rk_mpp_job *job,
			    const struct rk_mpp_rkvdec2_link_info *info)
{
	const u32 *table;

	if (!job || !info)
		return false;
	table = job->rkvdec_link_vaddr;
	if (!table || info->irq_status_word >= info->table_words)
		return false;

	return table[info->irq_status_word];
}

static u32
rk_mpp_rkvdec2_ccu_relink_unfinished_locked(struct rk_mpp_hw *ccu)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;
	struct rk_mpp_job *prev = NULL;
	u32 count = 0;

	list_for_each_entry(job, &ccu->rkvdec_ccu_jobs, rkvdec_ccu_node) {
		u32 *table = job->rkvdec_link_vaddr;

		if (!table || rk_mpp_rkvdec2_ccu_job_done(job, info))
			continue;

		if (prev) {
			u32 *prev_table = prev->rkvdec_link_vaddr;

			prev_table[info->next_word] =
				lower_32_bits(job->rkvdec_link_iova);
		}
		prev = job;
		count++;
	}

	if (prev) {
		u32 *table = prev->rkvdec_link_vaddr;
		dma_addr_t next_iova = prev->hw ?
			rk_mpp_rkvdec2_next_unused_link_iova(prev->hw) : 0;

		table[info->next_word] = lower_32_bits(next_iova);
	}

	return count;
}

static u32 rk_mpp_rkvdec2_ccu_prepare_resend_chain(struct rk_mpp_hw *ccu)
{
	unsigned long flags;
	u32 count;

	if (!ccu)
		return 0;

	spin_lock_irqsave(&ccu->lock, flags);
	count = rk_mpp_rkvdec2_ccu_relink_unfinished_locked(ccu);
	spin_unlock_irqrestore(&ccu->lock, flags);

	return count;
}

static int
rk_mpp_rkvdec2_collect_unfinished_ccu_jobs(struct rk_mpp_hw *ccu,
					   struct rk_mpp_job ***jobs_out,
					   u32 *count_out)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job **jobs;
	struct rk_mpp_job *job;
	unsigned long flags;
	u32 count = 0;
	u32 i = 0;

	*jobs_out = NULL;
	*count_out = 0;

	if (!ccu)
		return 0;

	spin_lock_irqsave(&ccu->lock, flags);
	list_for_each_entry(job, &ccu->rkvdec_ccu_jobs, rkvdec_ccu_node) {
		if (!rk_mpp_rkvdec2_ccu_job_done(job, info))
			count++;
	}
	spin_unlock_irqrestore(&ccu->lock, flags);
	if (!count)
		return 0;

	jobs = kcalloc(count, sizeof(*jobs), GFP_KERNEL);
	if (!jobs)
		return -ENOMEM;

	spin_lock_irqsave(&ccu->lock, flags);
	list_for_each_entry(job, &ccu->rkvdec_ccu_jobs, rkvdec_ccu_node) {
		if (rk_mpp_rkvdec2_ccu_job_done(job, info))
			continue;
		if (i >= count)
			break;
		rk_mpp_job_get(job);
		jobs[i++] = job;
	}
	spin_unlock_irqrestore(&ccu->lock, flags);

	if (!i) {
		kfree(jobs);
		return 0;
	}

	*jobs_out = jobs;
	*count_out = i;

	return 0;
}

static void rk_mpp_rkvdec2_ccu_job_add(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *ccu = job->rkvdec_ccu;
	unsigned long flags;

	if (!ccu || job->rkvdec_ccu_listed)
		return;

	spin_lock_irqsave(&ccu->lock, flags);
	list_add_tail(&job->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	job->rkvdec_ccu_listed = true;
	rk_mpp_rkvdec2_ccu_relink_tables_locked(ccu);
	spin_unlock_irqrestore(&ccu->lock, flags);
}

static bool
rk_mpp_rkvdec2_ccu_job_del(struct rk_mpp_job *job, struct rk_mpp_hw *ccu)
{
	unsigned long flags;
	bool empty = true;

	if (!ccu || !job->rkvdec_ccu_listed)
		return true;

	spin_lock_irqsave(&ccu->lock, flags);
	list_del_init(&job->rkvdec_ccu_node);
	job->rkvdec_ccu_listed = false;
	empty = list_empty(&ccu->rkvdec_ccu_jobs);
	if (!empty)
		rk_mpp_rkvdec2_ccu_relink_tables_locked(ccu);
	spin_unlock_irqrestore(&ccu->lock, flags);

	return empty;
}

static void rk_mpp_rkvdec2_transfer_powered_ccu_cores(struct rk_mpp_job *from,
						      struct rk_mpp_hw *ccu)
{
	struct rk_mpp_job *to;
	unsigned long flags;

	if (!ccu || !from->rkvdec_ccu_powered_core_count)
		return;

	spin_lock_irqsave(&ccu->lock, flags);
	to = list_first_entry_or_null(&ccu->rkvdec_ccu_jobs,
				      struct rk_mpp_job, rkvdec_ccu_node);
	if (to)
		rk_mpp_rkvdec2_move_powered_ccu_cores(from, to);
	spin_unlock_irqrestore(&ccu->lock, flags);
}

static struct rk_mpp_job *rk_mpp_rkvdec2_ccu_first_done_job(struct rk_mpp_hw *ccu)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;
	unsigned long flags;

	if (!ccu)
		return NULL;

	spin_lock_irqsave(&ccu->lock, flags);
	list_for_each_entry(job, &ccu->rkvdec_ccu_jobs, rkvdec_ccu_node) {
		if (rk_mpp_rkvdec2_ccu_job_done(job, info)) {
			rk_mpp_job_get(job);
			spin_unlock_irqrestore(&ccu->lock, flags);
			return job;
		}
	}
	spin_unlock_irqrestore(&ccu->lock, flags);

	return NULL;
}

static struct rk_mpp_job *
rk_mpp_rkvdec2_ccu_done_active_job(struct rk_mpp_job *active)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *done = NULL;
	unsigned long flags;

	if (!active || !active->rkvdec_ccu_started)
		return NULL;

	ccu = active->rkvdec_ccu;
	if (!ccu)
		return NULL;

	spin_lock_irqsave(&ccu->lock, flags);
	if (active->rkvdec_ccu_listed &&
	    rk_mpp_rkvdec2_ccu_job_done(active, info)) {
		rk_mpp_job_get(active);
		done = active;
	}
	spin_unlock_irqrestore(&ccu->lock, flags);

	return done;
}

static u32 rk_mpp_rkvdec2_ccu_link_mode(struct rk_mpp_job *job, bool add_mode)
{
	u32 link_mode = job->rkvdec_ccu_link_mode;

	if (add_mode)
		link_mode |= RK_MPP_RKVDEC_CCU_ADD_MODE;

	return link_mode;
}

static void rk_mpp_rkvdec2_release_link_table(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	struct rk_mpp_hw *ccu = job->rkvdec_ccu;
	unsigned long flags;
	bool ccu_empty = true;

	job->rkvdec_ccu = NULL;

	if (job->rkvdec_ccu_started && ccu) {
		mutex_lock(&ccu->run_lock);
		ccu_empty = rk_mpp_rkvdec2_ccu_job_del(job, ccu);
		if (!ccu_empty)
			rk_mpp_rkvdec2_transfer_powered_ccu_cores(job, ccu);
		if (ccu_empty && rk_mpp_rkvdec2_ccu_regs_ready(ccu))
			writel_relaxed(0,
				       ccu->regs[0] +
				       RK_MPP_RKVDEC_CCU_WORK_BASE);
		mutex_unlock(&ccu->run_lock);
	} else {
		rk_mpp_rkvdec2_ccu_job_del(job, ccu);
	}

	if (job->rkvdec_ccu_powered && ccu)
		rk_mpp_hw_power_off(ccu);
	rk_mpp_rkvdec2_power_off_ccu_cores(job);

	if (job->rkvdec_link_active && hw && hw->rkvdec_link_used) {
		spin_lock_irqsave(&hw->lock, flags);
		if (job->rkvdec_link_index < hw->rkvdec_link_capacity)
			clear_bit(job->rkvdec_link_index, hw->rkvdec_link_used);
		if (job->rkvdec_link_listed) {
			list_del_init(&job->rkvdec_link_node);
			job->rkvdec_link_listed = false;
		}
		rk_mpp_rkvdec2_relink_tables_locked(hw);
		spin_unlock_irqrestore(&hw->lock, flags);
	}

	job->rkvdec_link_vaddr = NULL;
	job->rkvdec_link_iova = 0;
	job->rkvdec_link_index = 0;
	job->rkvdec_link_active = false;
	job->rkvdec_link_listed = false;
	job->rkvdec_ccu_listed = false;
	job->rkvdec_ccu_core_work = 0;
	job->rkvdec_ccu_cfg_addr = 0;
	job->rkvdec_ccu_link_mode = 0;
	job->rkvdec_ccu_ctrl = 0;
	job->rkvdec_ccu_work = 0;
	job->rkvdec_ccu_cfg_done = 0;
	job->rkvdec_link_irq_mode = 0;
	job->rkvdec_ccu_desc_valid = false;
	job->rkvdec_ccu_powered = false;
	job->rkvdec_ccu_started = false;

	rk_mpp_hw_put(ccu);
}

static int rk_mpp_rkvdec2_reserve_link_table(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	unsigned long flags;
	unsigned long index;

	if (!hw || !hw->rkvdec_link_vaddr || !hw->rkvdec_link_used ||
	    !hw->rkvdec_link_capacity)
		return -EOPNOTSUPP;
	if (job->rkvdec_link_active)
		return 0;
	if (rk_mpp_rkvdec2_hard_ccu_enabled(hw) && !job->rkvdec_ccu) {
		job->rkvdec_ccu =
			rk_mpp_hw_get_ccu_for_core(job->session->srv, hw);
		if (!job->rkvdec_ccu)
			return -ENODEV;
	}

	spin_lock_irqsave(&hw->lock, flags);
	index = find_first_zero_bit(hw->rkvdec_link_used,
				    hw->rkvdec_link_capacity);
	if (index < hw->rkvdec_link_capacity)
		set_bit(index, hw->rkvdec_link_used);
	spin_unlock_irqrestore(&hw->lock, flags);

	if (index >= hw->rkvdec_link_capacity) {
		rk_mpp_rkvdec2_release_link_table(job);
		return -ENOSPC;
	}

	job->rkvdec_link_index = index;
	job->rkvdec_link_vaddr = (u8 *)hw->rkvdec_link_vaddr +
		index * hw->rkvdec_link_node_size;
	job->rkvdec_link_iova = hw->rkvdec_link_iova +
		index * hw->rkvdec_link_node_size;
	job->rkvdec_link_active = true;

	return 0;
}

static int rk_mpp_rkvdec2_fill_ccu_descriptor(struct rk_mpp_job *job,
					      u32 core_work)
{
	if (!job->rkvdec_link_active || !job->rkvdec_ccu)
		return -EINVAL;
	if (!core_work)
		return -EINVAL;

	job->rkvdec_ccu_core_work = core_work;
	job->rkvdec_ccu_cfg_addr = lower_32_bits(job->rkvdec_link_iova);
	job->rkvdec_ccu_link_mode = RK_MPP_RKVDEC_LINK_ADD_CFG_NUM;
	job->rkvdec_ccu_ctrl = RK_MPP_RKVDEC_CCU_AUTOGATE;
	job->rkvdec_ccu_work = RK_MPP_RKVDEC_CCU_WORK_EN;
	job->rkvdec_ccu_cfg_done = RK_MPP_RKVDEC_CCU_CFG_DONE;
	job->rkvdec_link_irq_mode = RK_MPP_RKVDEC_LINK_CCU_WORK_MODE;
	job->rkvdec_ccu_desc_valid = true;

	return 0;
}

static int rk_mpp_rkvdec2_prepare_ccu_descriptor(struct rk_mpp_job *job)
{
	u32 core_work;

	if (!job->rkvdec_ccu)
		return 0;
	if (!rk_mpp_rkvdec2_ccu_regs_ready(job->rkvdec_ccu))
		return -EOPNOTSUPP;

	core_work = rk_mpp_rkvdec2_ccu_core_mask(job->session->srv,
						 job->rkvdec_ccu);
	if (!core_work)
		core_work = job->hw ? job->hw->core_mask : 0;
	if (!core_work)
		return -ENODEV;

	return rk_mpp_rkvdec2_fill_ccu_descriptor(job, core_work);
}

static void rk_mpp_rkvdec2_stage_link_table(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	dma_addr_t next_iova = 0;
	int ret;

	ret = rk_mpp_rkvdec2_reserve_link_table(job);
	if (ret == -EOPNOTSUPP)
		return;
	if (ret) {
		dev_dbg(hw->dev, "failed to reserve rkvdec link table: %d\n", ret);
		return;
	}

	if (job->rkvdec_link_index + 1 < hw->rkvdec_link_capacity)
		next_iova = job->rkvdec_link_iova + hw->rkvdec_link_node_size;
	ret = rk_mpp_rkvdec2_fill_link_table(&job->reg_image,
					     &rk_mpp_rkvdec2_vdpu383_link_info,
					     job->rkvdec_link_vaddr,
					     job->rkvdec_link_iova,
					     next_iova);
	if (ret) {
		dev_dbg(hw->dev, "failed to stage rkvdec link table: %d\n", ret);
		rk_mpp_rkvdec2_release_link_table(job);
		return;
	}

	ret = rk_mpp_rkvdec2_prepare_ccu_descriptor(job);
	if (ret) {
		dev_dbg(hw->dev, "failed to prepare rkvdec ccu descriptor: %d\n",
			ret);
		rk_mpp_rkvdec2_release_link_table(job);
		return;
	}

	rk_mpp_rkvdec2_link_table_list_add(job);
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
static bool rk_mpp_hw_prepare_active_retry(struct rk_mpp_hw *hw,
					   struct rk_mpp_job *match);
static void rk_mpp_hw_refresh_iommu(struct rk_mpp_hw *hw,
				    struct rk_mpp_job *job);
static bool rk_mpp_job_rkvdec_rcb_enabled(struct rk_mpp_job *job);
static int rk_mpp_rkvdec2_program_soft_ccu(struct rk_mpp_job *job);
static void rk_mpp_rkvenc2_dchs_patch(struct rk_mpp_job *job);
static void rk_mpp_rkvenc2_dchs_release(struct rk_mpp_job *job);
static int rk_mpp_switch_session(struct rk_mpp_session **session,
				 struct fd *held_fd,
				 const struct rk_mpp_msg_v1 *msg);
static int rk_mpp_process_request(struct rk_mpp_session *session,
				  struct mpp_request *req,
				  struct rk_mpp_batch_state *batch);
static int rk_mpp_job_store_reg_offsets(struct rk_mpp_job *job,
					const struct rk_mpp_job_req *job_req);
static int rk_mpp_job_apply_reg_offsets(struct rk_mpp_job *job);
static int rk_mpp_job_translate_reg_image(struct rk_mpp_job *job);
static int rk_mpp_rkvdec2_validate(struct rk_mpp_job *job);
static void rk_mpp_kunit_device_release(struct device *dev);

static void __user *rk_mpp_kunit_user_payload(struct kunit *test,
					      const void *src, size_t size)
{
	unsigned long useraddr;
	void __user *dst;

	useraddr = kunit_vm_mmap(test, NULL, 0, PAGE_ALIGN(size ? size : 1),
				 PROT_READ | PROT_WRITE,
				 MAP_ANONYMOUS | MAP_PRIVATE, 0);
	if (!useraddr || useraddr >= TASK_SIZE) {
		KUNIT_FAIL(test, "failed to allocate userspace payload");
		return NULL;
	}

	dst = (void __user *)useraddr;
	if (size && copy_to_user(dst, src, size)) {
		KUNIT_FAIL(test, "failed to copy userspace payload");
		return NULL;
	}

	return dst;
}

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

static void rk_mpp_support_cmds_kunit(struct kunit *test)
{
#define RK_MPP_EXPECT_SUPPORT_CMD(_idx, _name, _cmd) do {		\
		KUNIT_EXPECT_EQ(test, strcmp(rk_mpp_support_cmds[_idx].name, \
					     _name), 0);		\
		KUNIT_EXPECT_EQ(test, rk_mpp_support_cmds[_idx].cmd,	\
				(__u32)(_cmd));				\
	} while (0)

	KUNIT_EXPECT_EQ(test, ARRAY_SIZE(rk_mpp_support_cmds), (size_t)27);
	RK_MPP_EXPECT_SUPPORT_CMD(0, "QUERY_HW_SUPPORT:",
				  MPP_CMD_QUERY_HW_SUPPORT);
	RK_MPP_EXPECT_SUPPORT_CMD(2, "QUERY_CMD_SUPPORT:",
				  MPP_CMD_QUERY_CMD_SUPPORT);
	KUNIT_EXPECT_PTR_EQ(test, rk_mpp_support_cmds[4].name, NULL);
	RK_MPP_EXPECT_SUPPORT_CMD(6, "INIT_DRIVER_DATA:",
				  MPP_CMD_INIT_DRIVER_DATA);
	RK_MPP_EXPECT_SUPPORT_CMD(13, "SET_RCB_INFO:", MPP_CMD_SET_RCB_INFO);
	RK_MPP_EXPECT_SUPPORT_CMD(14, "SET_SESSION_FD:",
				  MPP_CMD_SET_SESSION_FD);
	RK_MPP_EXPECT_SUPPORT_CMD(18, "POLL_HW_IRQ:", MPP_CMD_POLL_HW_IRQ);
	RK_MPP_EXPECT_SUPPORT_CMD(25, "SET_ERR_REF_HACK:",
				  MPP_CMD_SET_ERR_REF_HACK);
	RK_MPP_EXPECT_SUPPORT_CMD(26, "CONTROL_BUTT:", MPP_CMD_CONTROL_BUTT);

#undef RK_MPP_EXPECT_SUPPORT_CMD
}

static void rk_mpp_abi_layout_kunit(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, sizeof(struct rk_mpp_msg_v1),
			(size_t)RK_MPP_MSG_V1_ABI_SIZE);
	KUNIT_EXPECT_EQ(test, offsetof(struct rk_mpp_msg_v1, data_ptr),
			(size_t)RK_MPP_MSG_V1_DATA_PTR_ABI_OFFSET);
	KUNIT_EXPECT_EQ(test, sizeof(struct mpp_bat_msg),
			(size_t)RK_MPP_BAT_MSG_ABI_SIZE);
	KUNIT_EXPECT_EQ(test, offsetof(struct mpp_bat_msg, ret),
			(size_t)RK_MPP_BAT_MSG_RET_ABI_OFFSET);
	KUNIT_EXPECT_EQ(test, _IOC_SIZE(MPP_IOC_CFG_V1),
			sizeof(unsigned int));
	KUNIT_EXPECT_EQ(test, _IOC_TYPE(MPP_IOC_CFG_V1),
			(unsigned int)MPP_IOC_MAGIC);
	KUNIT_EXPECT_EQ(test, _IOC_NR(MPP_IOC_CFG_V1), 1U);
}

static void rk_mpp_msg_v1_to_request_kunit(struct kunit *test)
{
	struct rk_mpp_msg_v1 msg = {
		.cmd = MPP_CMD_SET_REG_WRITE,
		.flags = MPP_FLAGS_MULTI_MSG | MPP_FLAGS_LAST_MSG,
		.size = 128,
		.offset = 64,
		.data_ptr = 0x12345000,
	};
	struct mpp_request req = {};

	rk_mpp_msg_v1_to_request(&msg, &req);

	KUNIT_EXPECT_EQ(test, req.cmd, msg.cmd);
	KUNIT_EXPECT_EQ(test, req.flags, msg.flags);
	KUNIT_EXPECT_EQ(test, req.size, msg.size);
	KUNIT_EXPECT_EQ(test, req.offset, msg.offset);
	KUNIT_EXPECT_EQ(test, (uintptr_t)req.data, (uintptr_t)msg.data_ptr);
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

static void rk_mpp_set_err_ref_hack_kunit(struct kunit *test)
{
	u8 heap_payload[129] = {};
	u32 stack_payload = 1;
	struct rk_mpp_session session = {};
	struct mpp_request req = {
		.cmd = MPP_CMD_SET_ERR_REF_HACK,
		.size = sizeof(stack_payload),
	};

	req.data = rk_mpp_kunit_user_payload(test, &stack_payload,
					     sizeof(stack_payload));
	KUNIT_ASSERT_NOT_NULL(test, req.data);

	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL),
			-EINVAL);

	session.initialized = true;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL), 0);

	req.size = 0;
	req.data = NULL;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL), 0);

	req.size = sizeof(heap_payload);
	req.data = rk_mpp_kunit_user_payload(test, heap_payload,
					     sizeof(heap_payload));
	KUNIT_ASSERT_NOT_NULL(test, req.data);
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL), 0);

	req.size = PAGE_SIZE + 1;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL),
			-ENOMEM);

	req.size = sizeof(stack_payload);
	req.data = (void __user *)ULONG_MAX;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL),
			-EINVAL);
}

static void rk_mpp_store_codec_info_kunit(struct kunit *test)
{
	struct rk_mpp_codec_info_elem elems[] = {
		{
			.type = RK_MPP_DEC_INFO_WIDTH,
			.flag = 1,
			.data = 1920,
		}, {
			.type = 0,
			.flag = 1,
			.data = 111,
		}, {
			.type = RK_MPP_DEC_INFO_HEIGHT,
			.flag = 2,
			.data = 1080,
		}, {
			.type = RK_MPP_DEC_INFO_BITDEPTH,
			.flag = RK_MPP_CODEC_INFO_FLAG_BUTT,
			.data = 12,
		}, {
			.type = RK_MPP_DEC_INFO_BUTT,
			.flag = 1,
			.data = 4096,
		}, {
			.type = RK_MPP_DEC_INFO_BITDEPTH,
			.flag = 1,
			.data = 10,
		},
	};
	u8 payload[sizeof(elems) + 3] = {};
	struct rk_mpp_session session = {
		.client_type = RK_MPP_DEVICE_RKVDEC,
	};
	struct mpp_request req = {
		.size = sizeof(payload),
	};

	mutex_init(&session.lock);
	memcpy(payload, elems, sizeof(elems));
	req.data = rk_mpp_kunit_user_payload(test, payload, sizeof(payload));
	KUNIT_ASSERT_NOT_NULL(test, req.data);

	KUNIT_EXPECT_EQ(test, rk_mpp_store_codec_info(&session, &req), 0);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_WIDTH].flag, 1U);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_WIDTH].val, 1920ULL);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_HEIGHT].flag, 2U);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_HEIGHT].val, 1080ULL);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_BITDEPTH].flag, 1U);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_BITDEPTH].val, 10ULL);
	KUNIT_EXPECT_EQ(test, session.codec_info[0].flag, 0U);
	KUNIT_EXPECT_EQ(test, session.codec_info[0].val, 0ULL);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_BUTT].flag, 0U);
	KUNIT_EXPECT_EQ(test,
			session.codec_info[RK_MPP_DEC_INFO_BUTT].val, 0ULL);
}

static void rk_mpp_init_trans_table_kunit(struct kunit *test)
{
	u16 entries[] = { 7, 1024, U16_MAX };
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
		.trans_count = 9,
	};
	struct mpp_request req = {
		.cmd = MPP_CMD_INIT_TRANS_TABLE,
		.size = sizeof(entries),
	};
	int ret;

	memset(session.trans_table, 0xa5, sizeof(session.trans_table));
	req.data = rk_mpp_kunit_user_payload(test, entries, sizeof(entries));
	KUNIT_ASSERT_NOT_NULL(test, req.data);

	ret = rk_mpp_process_request(&session, &req, NULL);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, session.trans_count, (u32)ARRAY_SIZE(entries));
	KUNIT_EXPECT_EQ(test, session.trans_table[0], entries[0]);
	KUNIT_EXPECT_EQ(test, session.trans_table[1], entries[1]);
	KUNIT_EXPECT_EQ(test, session.trans_table[2], entries[2]);

	req.size = sizeof(session.trans_table) + 1;
	session.trans_count = 3;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL),
			-ENOMEM);
	KUNIT_EXPECT_EQ(test, session.trans_count, 3U);

	req.size = sizeof(u16);
	req.data = (void __user *)TASK_SIZE;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, session.trans_count, 3U);

	req.size = 0;
	req.data = NULL;
	KUNIT_EXPECT_EQ(test, rk_mpp_process_request(&session, &req, NULL), 0);
	KUNIT_EXPECT_EQ(test, session.trans_count, 0U);
}

static void rk_mpp_reg_offsets_kunit(struct kunit *test)
{
	struct rk_mpp_reg_offset offsets[] = {
		{ .index = 1, .offset = 4 },
		{ .index = 1, .offset = 8 },
		{ .index = 3, .offset = 0x20 },
	};
	struct rk_mpp_job *job;
	struct rk_mpp_job_req job_req = {
		.req = {
			.cmd = MPP_CMD_SET_REG_ADDR_OFFSET,
			.size = sizeof(offsets),
		},
		.payload = offsets,
	};

	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	KUNIT_EXPECT_EQ(test, rk_mpp_job_store_reg_offsets(job, &job_req), 0);
	KUNIT_EXPECT_EQ(test, job->reg_image.offset_count,
			(u32)ARRAY_SIZE(offsets));
	KUNIT_EXPECT_EQ(test, job->reg_image.offsets[0].index, 1U);
	KUNIT_EXPECT_EQ(test, job->reg_image.offsets[0].offset, 4U);
	KUNIT_EXPECT_EQ(test, job->reg_image.offsets[2].index, 3U);
	KUNIT_EXPECT_EQ(test, job->reg_image.offsets[2].offset, 0x20U);

	KUNIT_EXPECT_EQ(test, rk_mpp_job_apply_reg_offsets(job), 0);
	KUNIT_ASSERT_NOT_NULL(test, job->reg_image.regs);
	KUNIT_EXPECT_TRUE(test, job->reg_image.reg_words >= 4);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[1], 12U);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[3], 0x20U);
	kfree(job->reg_image.regs);
	memset(job, 0, sizeof(*job));

	job_req.req.size = 0;
	job_req.payload = NULL;
	KUNIT_EXPECT_EQ(test, rk_mpp_job_store_reg_offsets(job, &job_req), 0);
	KUNIT_EXPECT_EQ(test, job->reg_image.offset_count, 0U);

	job_req.req.size = sizeof(offsets) - 1;
	job_req.payload = offsets;
	KUNIT_EXPECT_EQ(test, rk_mpp_job_store_reg_offsets(job, &job_req),
			-EINVAL);

	job_req.req.size = sizeof(offsets[0]);
	job_req.payload = NULL;
	KUNIT_EXPECT_EQ(test, rk_mpp_job_store_reg_offsets(job, &job_req),
			-EINVAL);

	job->reg_image.offset_count = RK_MPP_MAX_REG_TRANS_NUM;
	job_req.payload = offsets;
	KUNIT_EXPECT_EQ(test, rk_mpp_job_store_reg_offsets(job, &job_req),
			-EINVAL);
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

static void rk_mpp_rkvdec2_ccu_mode_kunit(struct kunit *test)
{
	struct rk_mpp_hw hw = {};
	struct device_node *ccu_node = (struct device_node *)test;

	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_normalize_ccu_mode(0),
			(u32)RK_MPP_RKVDEC_CCU_MODE_SOFT);
	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_normalize_ccu_mode(RK_MPP_RKVDEC_CCU_MODE_SOFT),
			(u32)RK_MPP_RKVDEC_CCU_MODE_SOFT);
	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_normalize_ccu_mode(RK_MPP_RKVDEC_CCU_MODE_HARD),
			(u32)RK_MPP_RKVDEC_CCU_MODE_HARD);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_normalize_ccu_mode(3),
			(u32)RK_MPP_RKVDEC_CCU_MODE_SOFT);

	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_soft_ccu_enabled(&hw));
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_hard_ccu_enabled(&hw));

	hw.ccu_node = ccu_node;
	hw.rkvdec_ccu_mode = RK_MPP_RKVDEC_CCU_MODE_SOFT;
	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_soft_ccu_enabled(&hw));
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_hard_ccu_enabled(&hw));

	hw.rkvdec_ccu_mode = RK_MPP_RKVDEC_CCU_MODE_HARD;
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_soft_ccu_enabled(&hw));
	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_hard_ccu_enabled(&hw));
}

static void rk_mpp_rkvdec2_soft_ccu_program_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw hw = {
		.core_mask = 0x00030000,
	};
	struct rk_mpp_hw ccu = {};
	struct rk_mpp_job job = {
		.hw = &hw,
		.rkvdec_ccu = &ccu,
	};
	u32 *ccu_regs;
	u32 *link;

	link = kunit_kcalloc(test, 0x60 / sizeof(*link), sizeof(*link),
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, link);
	ccu_regs = kunit_kcalloc(test,
				 (RK_MPP_RKVDEC_CCU_CORE_STA_BASE +
				  sizeof(*ccu_regs)) / sizeof(*ccu_regs),
				 sizeof(*ccu_regs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_regs);

	hw.regs[RK_MPP_RKVDEC_LINK_REGION] = (void __iomem *)link;
	hw.reg_size[RK_MPP_RKVDEC_LINK_REGION] = 0x60;
	ccu.regs[0] = (void __iomem *)ccu_regs;
	ccu.reg_size[0] = RK_MPP_RKVDEC_CCU_CORE_STA_BASE + sizeof(*ccu_regs);

	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_program_soft_ccu(&job), 0);
	KUNIT_EXPECT_EQ(test,
			link[info->irq_base / sizeof(*link)] &
			(RK_MPP_RKVDEC_LINK_CORE_WORK_MODE |
			 RK_MPP_RKVDEC_LINK_CCU_WORK_MODE),
			(u32)(RK_MPP_RKVDEC_LINK_CORE_WORK_MODE |
			      RK_MPP_RKVDEC_LINK_CCU_WORK_MODE));
	KUNIT_EXPECT_EQ(test,
			ccu_regs[RK_MPP_RKVDEC_CCU_WORK_BASE / sizeof(*ccu_regs)],
			(u32)RK_MPP_RKVDEC_CCU_WORK_EN);
	KUNIT_EXPECT_EQ(test,
			ccu_regs[RK_MPP_RKVDEC_CCU_WORK_MODE_BASE / sizeof(*ccu_regs)],
			(u32)RK_MPP_RKVDEC_CCU_WORK_MODE);
	KUNIT_EXPECT_EQ(test,
			ccu_regs[RK_MPP_RKVDEC_CCU_CORE_WORK_BASE / sizeof(*ccu_regs)],
			hw.core_mask);
	KUNIT_EXPECT_EQ(test,
			ccu_regs[RK_MPP_RKVDEC_CCU_CORE_STA_BASE / sizeof(*ccu_regs)],
			hw.core_mask);

	hw.core_mask = 0;
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_program_soft_ccu(&job), -EINVAL);
}

static void rk_mpp_rkvdec2_link_info_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;

	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_link_node_size(info),
			(size_t)1024);
	KUNIT_EXPECT_EQ(test, info->table_words, (u16)256);
	KUNIT_EXPECT_EQ(test, info->next_word, (u16)0);
	KUNIT_EXPECT_EQ(test, info->readback_word, (u16)1);
	KUNIT_EXPECT_EQ(test, info->irq_status_word, (u16)16);
	KUNIT_EXPECT_EQ(test, info->cycle_word, (u16)27);
	KUNIT_EXPECT_EQ(test, info->write_part_count, (u8)3);
	KUNIT_EXPECT_EQ(test, info->read_part_count, (u8)2);

	KUNIT_EXPECT_EQ(test, info->write_parts[0].table_word, (u16)80);
	KUNIT_EXPECT_EQ(test, info->write_parts[0].reg_word, (u16)8);
	KUNIT_EXPECT_EQ(test, info->write_parts[0].word_count, (u16)24);
	KUNIT_EXPECT_EQ(test, info->write_parts[2].table_word, (u16)148);
	KUNIT_EXPECT_EQ(test, info->write_parts[2].reg_word, (u16)128);
	KUNIT_EXPECT_EQ(test, info->write_parts[2].word_count, (u16)108);
	KUNIT_EXPECT_EQ(test, info->read_parts[1].table_word, (u16)20);
	KUNIT_EXPECT_EQ(test, info->read_parts[1].reg_word, (u16)320);
	KUNIT_EXPECT_EQ(test, info->read_parts[1].word_count, (u16)40);

	KUNIT_EXPECT_EQ(test, info->irq_base, 0x48U);
	KUNIT_EXPECT_EQ(test, info->irq_mask, 0x30000U);
	KUNIT_EXPECT_EQ(test, info->status_base, 0x4cU);
	KUNIT_EXPECT_EQ(test, info->status_mask, 0x3ff0000U);
	KUNIT_EXPECT_EQ(test, info->ip_time_base, 0x54U);
	KUNIT_EXPECT_EQ(test, info->en_base, 0x40U);
	KUNIT_EXPECT_EQ(test, info->ip_en_base, 0x58U);
	KUNIT_EXPECT_EQ(test, info->ip_en_val, 0x01000000U);
	KUNIT_EXPECT_TRUE(test, info->sw_iommu_zap);
}

static void rk_mpp_rkvdec2_vp9_translate_validate_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
		.client_type = RK_MPP_DEVICE_RKVDEC,
	};
	struct rk_mpp_hw hw = {
		.irq = 42,
	};
	struct rk_mpp_job *job;
	struct rk_mpp_import *import;
	struct device *dev;
	u32 raw_vp9_160 = 7 | (4 << 10);
	u32 raw_vp9_162 = 7 | (8 << 10);
	u32 raw_h264_only_173 = 7 | (12 << 10);

	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	job->reg_image.regs = kunit_kcalloc(test, 200, sizeof(u32), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job->reg_image.regs);
	import = kzalloc(sizeof(*import), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, import);

	device_initialize(dev);
	dev->release = rk_mpp_kunit_device_release;
	mutex_init(&session.lock);
	INIT_LIST_HEAD(&session.imports);
	job->session = &session;
	job->hw = &hw;
	job->req_cnt = 1;
	hw.dev = dev;
	hw.regs[0] = (void __iomem *)0x1;
	hw.reg_size[0] = RK_MPP_RKVDEC_INT_STA_BASE + sizeof(u32);

	import->fd = 7;
	import->dev = dev;
	import->iova = 0x80000000;
	refcount_set(&import->refs, 1);
	INIT_LIST_HEAD(&import->link);
	list_add_tail(&import->link, &session.imports);

	job->reg_image.reg_words = 200;
	job->reg_image.reg_bytes = 200 * sizeof(u32);
	job->reg_image.regs[RK_MPP_RKVDEC_REG_FMT] = RK_MPP_RKVDEC_FMT_VP9D;
	job->reg_image.regs[160] = raw_vp9_160;
	job->reg_image.regs[162] = raw_vp9_162;
	job->reg_image.regs[173] = raw_h264_only_173;
	job->reg_image.read_req_count = 1;
	job->reg_image.read_reqs[0].cmd = MPP_CMD_SET_REG_READ;
	job->reg_image.read_reqs[0].offset = RK_MPP_RKVDEC_INT_STA_BASE;
	job->reg_image.read_reqs[0].size = sizeof(u32);
	job->reqs[0].req.cmd = MPP_CMD_SET_REG_WRITE;
	job->reqs[0].req.offset = RK_MPP_RKVDEC_START_BASE;
	job->reqs[0].req.size = sizeof(u32);

	KUNIT_EXPECT_EQ(test, rk_mpp_job_translate_reg_image(job), 0);
	KUNIT_EXPECT_TRUE(test, job->reg_image.translated);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[160], 0x80000004U);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[162], 0x80000008U);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[173], raw_h264_only_173);
	KUNIT_EXPECT_EQ(test, job->import_count, 1U);
	KUNIT_EXPECT_PTR_EQ(test, job->imports[0], import);
	KUNIT_EXPECT_EQ(test, refcount_read(&import->refs), 2);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_validate(job), 0);

	job->reg_image.translated = false;
	job->reg_image.regs[RK_MPP_RKVDEC_REG_FMT] =
		ARRAY_SIZE(rk_mpp_rkvdec_tables);
	KUNIT_EXPECT_EQ(test, rk_mpp_job_translate_reg_image(job), -EINVAL);

	list_del_init(&import->link);
	if (job->import_count)
		rk_mpp_import_put(job->imports[0]);
	rk_mpp_import_put(import);
}

static void rk_mpp_rkvdec2_link_irq_decode_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	u32 status = 0xdeadbeef;

	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_link_irq_decode(info, 0,
								0x3ff,
								&status));
	KUNIT_EXPECT_EQ(test, status, 0xdeadbeefU);

	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_link_irq_decode(info, 0x3,
							       0x155,
							       &status));
	KUNIT_EXPECT_EQ(test, status, 0x155U);

	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_link_irq_decode(info, 0x3, 0,
								&status));
	KUNIT_EXPECT_EQ(test, status, 0U);

	KUNIT_EXPECT_TRUE(test,
			  rk_mpp_rkvdec2_link_irq_decode(info,
							 RK_MPP_RKVDEC_LINK_IRQ_RAW,
							 0, &status));
	KUNIT_EXPECT_EQ(test, status, 0U);
}

static void rk_mpp_rkvdec2_fill_link_table_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	u32 *regs;
	u32 *table;
	struct rk_mpp_reg_image image = {
		.reg_words = 360,
	};
	struct rk_mpp_job *job;
	dma_addr_t iova = 0x12345000;
	dma_addr_t next = 0x12345400;
	u32 i;

	regs = kunit_kcalloc(test, image.reg_words, sizeof(*regs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, regs);
	table = kunit_kcalloc(test, info->table_words, sizeof(*table),
			      GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table);
	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);
	image.regs = regs;
	job->reg_image = image;
	job->rkvdec_link_vaddr = table;

	for (i = 0; i < image.reg_words; i++)
		regs[i] = 0xa5000000 | i;
	memset(table, 0xff, info->table_words * sizeof(*table));

	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_fill_link_table(&image, info, table,
						       iova, next),
			0);

	KUNIT_EXPECT_EQ(test, table[info->next_word], lower_32_bits(next));
	KUNIT_EXPECT_EQ(test, table[info->readback_word],
			lower_32_bits(iova + 16 * sizeof(u32)));
	KUNIT_EXPECT_EQ(test, table[info->debug_word], table[info->readback_word]);
	KUNIT_EXPECT_EQ(test, table[info->seg0_word],
			lower_32_bits(iova + 80 * sizeof(u32)));
	KUNIT_EXPECT_EQ(test, table[info->seg1_word],
			lower_32_bits(iova + 104 * sizeof(u32)));
	KUNIT_EXPECT_EQ(test, table[info->seg2_word],
			lower_32_bits(iova + 148 * sizeof(u32)));

	KUNIT_EXPECT_EQ(test, table[80], regs[8]);
	KUNIT_EXPECT_EQ(test, table[103], regs[31]);
	KUNIT_EXPECT_EQ(test, table[104], regs[64]);
	KUNIT_EXPECT_EQ(test, table[147], regs[107]);
	KUNIT_EXPECT_EQ(test, table[148], regs[128]);
	KUNIT_EXPECT_EQ(test, table[255], regs[235]);

	KUNIT_EXPECT_EQ(test, table[16], 0U);
	KUNIT_EXPECT_EQ(test, table[20], 0U);
	KUNIT_EXPECT_EQ(test, table[59], 0U);

	table[16] = 0x11111111;
	for (i = 0; i < 40; i++)
		table[20 + i] = 0xbb000000 | i;

	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_read_link_table(&image, info, table,
						       0x1234),
			0);
	KUNIT_EXPECT_EQ(test, regs[RK_MPP_RKVDEC_LINK_STATUS_WORD], 0x1234U);
	KUNIT_EXPECT_EQ(test, regs[320], 0xbb000000U);
	KUNIT_EXPECT_EQ(test, regs[359], 0xbb000027U);

	regs[RK_MPP_RKVDEC_LINK_STATUS_WORD] = 0;
	table[info->irq_status_word] = 0x2222;
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_read_ccu_link_table(job, info,
								 0x1234),
			0);
	KUNIT_EXPECT_EQ(test, regs[RK_MPP_RKVDEC_LINK_STATUS_WORD],
			0x2222U);
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_ccu_job_error(job, info));
	job->rkvdec_ccu_started = true;
	regs[RK_MPP_RKVDEC_LINK_STATUS_WORD] = info->err_mask;
	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_ccu_job_error(job, info));

	image.reg_words = 320;
	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_read_link_table(&image, info, table,
						       0x1234),
			-EINVAL);

	image.reg_words = 128;
	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_fill_link_table(&image, info, table,
						       iova, next),
			-EINVAL);
}

static void rk_mpp_rkvdec2_link_table_ownership_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	unsigned long used[BITS_TO_LONGS(2)] = {};
	struct rk_mpp_reg_image image = {
		.reg_words = 360,
	};
	struct rk_mpp_hw *hw;
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;
	struct rk_mpp_job *job2;
	u32 *regs;
	void *tables;
	u32 i;

	hw = kunit_kzalloc(test, sizeof(*hw), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw);
	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	job2 = kunit_kzalloc(test, sizeof(*job2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job2);

	hw->rkvdec_link_iova = 0x12345000;
	hw->rkvdec_link_capacity = 2;
	hw->rkvdec_link_used = used;
	hw->rkvdec_link_node_size = rk_mpp_rkvdec2_link_node_size(info);
	tables = kunit_kzalloc(test, 2 * hw->rkvdec_link_node_size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, tables);
	regs = kunit_kcalloc(test, image.reg_words, sizeof(*regs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, regs);
	hw->rkvdec_link_vaddr = tables;
	image.regs = regs;
	job0->hw = hw;
	job0->reg_image = image;
	job1->hw = hw;
	job1->reg_image = image;
	job2->hw = hw;
	job2->reg_image = image;
	spin_lock_init(&hw->lock);
	INIT_LIST_HEAD(&hw->rkvdec_link_jobs);
	INIT_LIST_HEAD(&job0->rkvdec_link_node);
	INIT_LIST_HEAD(&job1->rkvdec_link_node);
	INIT_LIST_HEAD(&job2->rkvdec_link_node);

	for (i = 0; i < image.reg_words; i++)
		regs[i] = 0xa5000000 | i;

	rk_mpp_rkvdec2_stage_link_table(job0);
	KUNIT_EXPECT_TRUE(test, job0->rkvdec_link_active);
	KUNIT_EXPECT_EQ(test, job0->rkvdec_link_index, 0U);
	KUNIT_EXPECT_TRUE(test, test_bit(0, used));
	KUNIT_EXPECT_EQ(test, ((u32 *)job0->rkvdec_link_vaddr)[80], regs[8]);
	KUNIT_EXPECT_EQ(test, ((u32 *)job0->rkvdec_link_vaddr)[info->next_word],
			(u32)(hw->rkvdec_link_iova + hw->rkvdec_link_node_size));

	rk_mpp_rkvdec2_stage_link_table(job1);
	KUNIT_EXPECT_TRUE(test, job1->rkvdec_link_active);
	KUNIT_EXPECT_EQ(test, job1->rkvdec_link_index, 1U);
	KUNIT_EXPECT_TRUE(test, test_bit(1, used));
	KUNIT_EXPECT_EQ(test, ((u32 *)job0->rkvdec_link_vaddr)[info->next_word],
			(u32)job1->rkvdec_link_iova);
	KUNIT_EXPECT_EQ(test, ((u32 *)job1->rkvdec_link_vaddr)[info->next_word],
			0U);

	rk_mpp_rkvdec2_release_link_table(job0);
	KUNIT_EXPECT_FALSE(test, job0->rkvdec_link_active);
	KUNIT_EXPECT_FALSE(test, test_bit(0, used));
	KUNIT_EXPECT_TRUE(test, test_bit(1, used));
	KUNIT_EXPECT_EQ(test, ((u32 *)job1->rkvdec_link_vaddr)[info->next_word],
			(u32)hw->rkvdec_link_iova);

	rk_mpp_rkvdec2_stage_link_table(job2);
	KUNIT_EXPECT_TRUE(test, job2->rkvdec_link_active);
	KUNIT_EXPECT_EQ(test, job2->rkvdec_link_index, 0U);
	KUNIT_EXPECT_TRUE(test, test_bit(0, used));
	KUNIT_EXPECT_EQ(test, ((u32 *)job1->rkvdec_link_vaddr)[info->next_word],
			(u32)job2->rkvdec_link_iova);
	KUNIT_EXPECT_EQ(test, ((u32 *)job2->rkvdec_link_vaddr)[info->next_word],
			0U);

	rk_mpp_rkvdec2_release_link_table(job1);
	rk_mpp_rkvdec2_release_link_table(job2);
	KUNIT_EXPECT_FALSE(test, test_bit(0, used));
	KUNIT_EXPECT_FALSE(test, test_bit(1, used));
}

static void rk_mpp_rkvdec2_link_table_ccu_ref_kunit(struct kunit *test)
{
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *job;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	refcount_set(&ccu->refs, 1);
	init_completion(&ccu->released);
	job->rkvdec_ccu = ccu;

	rk_mpp_rkvdec2_release_link_table(job);

	KUNIT_EXPECT_PTR_EQ(test, job->rkvdec_ccu, NULL);
	KUNIT_EXPECT_TRUE(test, completion_done(&ccu->released));
}

static void rk_mpp_rkvdec2_ccu_running_list_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;
	struct rk_mpp_job *done;
	u32 *table0;
	u32 *table1;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	table0 = kunit_kcalloc(test, info->table_words, sizeof(*table0),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table0);
	table1 = kunit_kcalloc(test, info->table_words, sizeof(*table1),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table1);

	spin_lock_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&job0->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job1->rkvdec_ccu_node);
	refcount_set(&job0->refs, 1);
	refcount_set(&job1->refs, 1);
	job0->rkvdec_ccu = ccu;
	job0->rkvdec_link_vaddr = table0;
	job0->rkvdec_link_iova = 0x1000;
	job1->rkvdec_ccu = ccu;
	job1->rkvdec_link_vaddr = table1;
	job1->rkvdec_link_iova = 0x2000;

	rk_mpp_rkvdec2_ccu_job_add(job0);
	KUNIT_EXPECT_EQ(test, table0[info->next_word], 0U);
	rk_mpp_rkvdec2_ccu_job_add(job1);
	KUNIT_EXPECT_TRUE(test, job0->rkvdec_ccu_listed);
	KUNIT_EXPECT_TRUE(test, job1->rkvdec_ccu_listed);
	KUNIT_EXPECT_EQ(test, table0[info->next_word], 0x2000U);
	KUNIT_EXPECT_EQ(test, table1[info->next_word], 0U);
	KUNIT_EXPECT_PTR_EQ(test, rk_mpp_rkvdec2_ccu_first_done_job(ccu),
			    NULL);

	table1[info->irq_status_word] = 0x1234;
	done = rk_mpp_rkvdec2_ccu_first_done_job(ccu);
	KUNIT_EXPECT_PTR_EQ(test, done, job1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 2);
	refcount_dec(&job1->refs);
	KUNIT_EXPECT_PTR_EQ(test, rk_mpp_rkvdec2_ccu_done_active_job(job0),
			    NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 1);
	done = rk_mpp_rkvdec2_ccu_done_active_job(job1);
	KUNIT_EXPECT_PTR_EQ(test, done, job1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 2);
	refcount_dec(&job1->refs);

	table0[info->irq_status_word] = 0x5678;
	done = rk_mpp_rkvdec2_ccu_first_done_job(ccu);
	KUNIT_EXPECT_PTR_EQ(test, done, job0);
	KUNIT_EXPECT_EQ(test, refcount_read(&job0->refs), 2);
	refcount_dec(&job0->refs);
	done = rk_mpp_rkvdec2_ccu_done_active_job(job1);
	KUNIT_EXPECT_PTR_EQ(test, done, job1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 2);
	refcount_dec(&job1->refs);

	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_ccu_job_del(job1, ccu));
	KUNIT_EXPECT_FALSE(test, job1->rkvdec_ccu_listed);
	KUNIT_EXPECT_EQ(test, table0[info->next_word], 0U);
	done = rk_mpp_rkvdec2_ccu_first_done_job(ccu);
	KUNIT_EXPECT_PTR_EQ(test, done, job0);
	refcount_dec(&job0->refs);

	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_ccu_job_del(job0, ccu));
	KUNIT_EXPECT_FALSE(test, job0->rkvdec_ccu_listed);
	KUNIT_EXPECT_TRUE(test, list_empty(&ccu->rkvdec_ccu_jobs));
}

static void rk_mpp_rkvdec2_ccu_job_done_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job job = {};
	u32 *table;

	table = kunit_kcalloc(test, info->table_words, sizeof(*table),
			      GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table);

	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_ccu_job_done(NULL, info));
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_ccu_job_done(&job, info));

	job.rkvdec_link_vaddr = table;
	KUNIT_EXPECT_FALSE(test, rk_mpp_rkvdec2_ccu_job_done(&job, info));
	table[info->irq_status_word] = 0x40;
	KUNIT_EXPECT_TRUE(test, rk_mpp_rkvdec2_ccu_job_done(&job, info));
}

static void rk_mpp_rkvdec2_ccu_power_transfer_kunit(struct kunit *test)
{
	struct rk_mpp_hw *ccu;
	struct rk_mpp_hw *core0;
	struct rk_mpp_hw *core1;
	struct rk_mpp_hw *core2;
	struct rk_mpp_job *from;
	struct rk_mpp_job *to;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	core0 = kunit_kzalloc(test, sizeof(*core0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core0);
	core1 = kunit_kzalloc(test, sizeof(*core1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core1);
	core2 = kunit_kzalloc(test, sizeof(*core2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core2);
	from = kunit_kzalloc(test, sizeof(*from), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, from);
	to = kunit_kzalloc(test, sizeof(*to), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, to);

	spin_lock_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&to->rkvdec_ccu_node);
	list_add_tail(&to->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	from->rkvdec_ccu_powered_cores[0] = core0;
	from->rkvdec_ccu_powered_cores[1] = core1;
	from->rkvdec_ccu_powered_core_count = 2;

	rk_mpp_rkvdec2_transfer_powered_ccu_cores(from, ccu);

	KUNIT_EXPECT_EQ(test, from->rkvdec_ccu_powered_core_count, 0U);
	KUNIT_EXPECT_PTR_EQ(test, from->rkvdec_ccu_powered_cores[0], NULL);
	KUNIT_EXPECT_PTR_EQ(test, from->rkvdec_ccu_powered_cores[1], NULL);
	KUNIT_EXPECT_EQ(test, to->rkvdec_ccu_powered_core_count, 2U);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[0], core0);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[1], core1);

	from->rkvdec_ccu_powered_cores[0] = core2;
	from->rkvdec_ccu_powered_core_count = 1;
	rk_mpp_rkvdec2_transfer_powered_ccu_cores(from, ccu);

	KUNIT_EXPECT_EQ(test, from->rkvdec_ccu_powered_core_count, 1U);
	KUNIT_EXPECT_PTR_EQ(test, from->rkvdec_ccu_powered_cores[0], core2);
	KUNIT_EXPECT_EQ(test, to->rkvdec_ccu_powered_core_count, 2U);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[0], core0);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[1], core1);
}

static void rk_mpp_rkvdec2_release_power_transfer_kunit(struct kunit *test)
{
	struct rk_mpp_hw *ccu;
	struct rk_mpp_hw *core0;
	struct rk_mpp_hw *core1;
	struct rk_mpp_job *from;
	struct rk_mpp_job *to;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	core0 = kunit_kzalloc(test, sizeof(*core0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core0);
	core1 = kunit_kzalloc(test, sizeof(*core1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core1);
	from = kunit_kzalloc(test, sizeof(*from), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, from);
	to = kunit_kzalloc(test, sizeof(*to), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, to);

	spin_lock_init(&ccu->lock);
	mutex_init(&ccu->run_lock);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	refcount_set(&ccu->refs, 1);
	init_completion(&ccu->released);
	INIT_LIST_HEAD(&from->rkvdec_ccu_node);
	INIT_LIST_HEAD(&to->rkvdec_ccu_node);
	list_add_tail(&from->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	list_add_tail(&to->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	from->rkvdec_ccu = ccu;
	from->rkvdec_ccu_started = true;
	from->rkvdec_ccu_listed = true;
	from->rkvdec_ccu_powered_cores[0] = core0;
	from->rkvdec_ccu_powered_cores[1] = core1;
	from->rkvdec_ccu_powered_core_count = 2;

	rk_mpp_rkvdec2_release_link_table(from);

	KUNIT_EXPECT_PTR_EQ(test, from->rkvdec_ccu, NULL);
	KUNIT_EXPECT_FALSE(test, from->rkvdec_ccu_started);
	KUNIT_EXPECT_EQ(test, from->rkvdec_ccu_powered_core_count, 0U);
	KUNIT_EXPECT_FALSE(test, from->rkvdec_ccu_listed);
	KUNIT_EXPECT_EQ(test, to->rkvdec_ccu_powered_core_count, 2U);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[0], core0);
	KUNIT_EXPECT_PTR_EQ(test, to->rkvdec_ccu_powered_cores[1], core1);
	KUNIT_EXPECT_TRUE(test, completion_done(&ccu->released));
}

static void rk_mpp_rkvdec2_ccu_relink_unfinished_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;
	struct rk_mpp_job *job2;
	u32 *table0;
	u32 *table1;
	u32 *table2;
	unsigned long flags;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	job2 = kunit_kzalloc(test, sizeof(*job2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job2);
	table0 = kunit_kcalloc(test, info->table_words, sizeof(*table0),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table0);
	table1 = kunit_kcalloc(test, info->table_words, sizeof(*table1),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table1);
	table2 = kunit_kcalloc(test, info->table_words, sizeof(*table2),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table2);

	spin_lock_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&job0->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job1->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job2->rkvdec_ccu_node);
	job0->rkvdec_link_vaddr = table0;
	job0->rkvdec_link_iova = 0x1000;
	job1->rkvdec_link_vaddr = table1;
	job1->rkvdec_link_iova = 0x2000;
	job2->rkvdec_link_vaddr = table2;
	job2->rkvdec_link_iova = 0x3000;

	list_add_tail(&job0->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	list_add_tail(&job1->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	list_add_tail(&job2->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	job0->rkvdec_ccu_listed = true;
	job1->rkvdec_ccu_listed = true;
	job2->rkvdec_ccu_listed = true;
	table0[info->next_word] = 0x2000;
	table1[info->next_word] = 0x3000;
	table2[info->next_word] = 0x4000;
	table1[info->irq_status_word] = 0x80;

	spin_lock_irqsave(&ccu->lock, flags);
	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_ccu_relink_unfinished_locked(ccu),
			2U);
	spin_unlock_irqrestore(&ccu->lock, flags);

	KUNIT_EXPECT_EQ(test, table0[info->next_word], 0x3000U);
	KUNIT_EXPECT_EQ(test, table1[info->next_word], 0x3000U);
	KUNIT_EXPECT_EQ(test, table2[info->next_word], 0U);
}

static void rk_mpp_rkvdec2_ccu_collect_unfinished_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job **jobs = NULL;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;
	struct rk_mpp_job *job2;
	u32 *table0;
	u32 *table1;
	u32 *table2;
	u32 count = 0;

	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);
	job2 = kunit_kzalloc(test, sizeof(*job2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job2);
	table0 = kunit_kcalloc(test, info->table_words, sizeof(*table0),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table0);
	table1 = kunit_kcalloc(test, info->table_words, sizeof(*table1),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table1);
	table2 = kunit_kcalloc(test, info->table_words, sizeof(*table2),
			       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table2);

	spin_lock_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&job0->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job1->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job2->rkvdec_ccu_node);
	refcount_set(&job0->refs, 1);
	refcount_set(&job1->refs, 1);
	refcount_set(&job2->refs, 1);
	job0->rkvdec_link_vaddr = table0;
	job1->rkvdec_link_vaddr = table1;
	job2->rkvdec_link_vaddr = table2;
	table1[info->irq_status_word] = 0x40;

	list_add_tail(&job0->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	list_add_tail(&job1->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);
	list_add_tail(&job2->rkvdec_ccu_node, &ccu->rkvdec_ccu_jobs);

	KUNIT_EXPECT_EQ(test,
			rk_mpp_rkvdec2_collect_unfinished_ccu_jobs(ccu, &jobs,
								   &count),
			0);
	KUNIT_ASSERT_EQ(test, count, 2U);
	KUNIT_ASSERT_NOT_NULL(test, jobs);
	KUNIT_EXPECT_PTR_EQ(test, jobs[0], job0);
	KUNIT_EXPECT_PTR_EQ(test, jobs[1], job2);
	KUNIT_EXPECT_EQ(test, refcount_read(&job0->refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&job1->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&job2->refs), 2);

	rk_mpp_job_put(job0);
	rk_mpp_job_put(job2);
	kfree(jobs);
}

static void rk_mpp_rkvdec2_ccu_descriptor_kunit(struct kunit *test)
{
	struct rk_mpp_hw ccu = {};
	struct rk_mpp_job *job;

	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	job->rkvdec_ccu = &ccu;
	job->rkvdec_link_iova = 0x12345000;
	job->rkvdec_link_active = true;

	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_fill_ccu_descriptor(job,
								 0x30000), 0);
	KUNIT_EXPECT_TRUE(test, job->rkvdec_ccu_desc_valid);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_core_work, 0x30000U);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_cfg_addr, 0x12345000U);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_link_mode,
			(u32)RK_MPP_RKVDEC_LINK_ADD_CFG_NUM);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_ctrl,
			(u32)RK_MPP_RKVDEC_CCU_AUTOGATE);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_work,
			(u32)RK_MPP_RKVDEC_CCU_WORK_EN);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_cfg_done,
			(u32)RK_MPP_RKVDEC_CCU_CFG_DONE);
	KUNIT_EXPECT_EQ(test, job->rkvdec_link_irq_mode,
			(u32)RK_MPP_RKVDEC_LINK_CCU_WORK_MODE);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_link_mode(job, false),
			(u32)RK_MPP_RKVDEC_LINK_ADD_CFG_NUM);
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_ccu_link_mode(job, true),
			(u32)(RK_MPP_RKVDEC_CCU_ADD_MODE |
			      RK_MPP_RKVDEC_LINK_ADD_CFG_NUM));

	job->rkvdec_ccu_desc_valid = false;
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_fill_ccu_descriptor(job, 0),
			-EINVAL);
	KUNIT_EXPECT_FALSE(test, job->rkvdec_ccu_desc_valid);
}

static void rk_mpp_rkvdec2_ccu_descriptor_core_mask_kunit(struct kunit *test)
{
	struct rk_mpp_session *session;
	struct device_node *ccu_node;
	struct rk_mpp_service *srv;
	struct rk_mpp_hw *core0;
	struct rk_mpp_hw *core1;
	struct device *ccu_dev;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_job *job;

	ccu_node = kunit_kzalloc(test, sizeof(*ccu_node), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_node);
	ccu_dev = kunit_kzalloc(test, sizeof(*ccu_dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_dev);
	srv = kunit_kzalloc(test, sizeof(*srv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, srv);
	session = kunit_kzalloc(test, sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	core0 = kunit_kzalloc(test, sizeof(*core0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core0);
	core1 = kunit_kzalloc(test, sizeof(*core1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core1);
	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	ccu_dev->of_node = ccu_node;
	mutex_init(&srv->hw_lock);
	INIT_LIST_HEAD(&srv->hw_list);
	session->srv = srv;
	ccu->dev = ccu_dev;
	ccu->regs[0] = (void __iomem *)0x1;
	ccu->reg_size[0] = RK_MPP_RKVDEC_CCU_CORE_STA_BASE + sizeof(u32);
	core0->ccu_node = ccu_node;
	core0->core_mask = 0x00010001;
	core0->online = true;
	core1->ccu_node = ccu_node;
	core1->core_mask = 0x00020002;
	core1->online = true;
	INIT_LIST_HEAD(&ccu->link);
	INIT_LIST_HEAD(&core0->link);
	INIT_LIST_HEAD(&core1->link);
	list_add_tail(&core0->link, &srv->hw_list);
	list_add_tail(&core1->link, &srv->hw_list);

	job->session = session;
	job->hw = core0;
	job->rkvdec_ccu = ccu;
	job->rkvdec_link_iova = 0x12345000;
	job->rkvdec_link_active = true;

	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_prepare_ccu_descriptor(job), 0);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_core_work, 0x00030003U);

	core1->online = false;
	job->rkvdec_ccu_desc_valid = false;
	KUNIT_EXPECT_EQ(test, rk_mpp_rkvdec2_prepare_ccu_descriptor(job), 0);
	KUNIT_EXPECT_EQ(test, job->rkvdec_ccu_core_work, 0x00010001U);
}

static void rk_mpp_rkvdec2_fixed_rcb_link_kunit(struct kunit *test)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw hw = {
		.rcb_iova = 0x80000000,
		.rcb_size = 0x400,
		.rcb_count = 2,
	};
	u32 *regs;
	u32 *link;

	regs = kunit_kcalloc(test, 160, sizeof(*regs), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, regs);
	link = kunit_kcalloc(test, 0x60 / sizeof(*link), sizeof(*link),
			     GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, link);

	hw.regs[0] = (void __iomem *)regs;
	hw.reg_size[0] = 160 * sizeof(*regs);
	hw.regs[RK_MPP_RKVDEC_LINK_REGION] = (void __iomem *)link;
	hw.reg_size[RK_MPP_RKVDEC_LINK_REGION] = 0x60;
	hw.rcb_descs[0].index = 10;
	hw.rcb_descs[0].size = 0x100;
	hw.rcb_descs[1].index = 12;
	hw.rcb_descs[1].size = 0x80;

	rk_mpp_rkvdec2_prepare_core_for_ccu(&hw);
	KUNIT_EXPECT_EQ(test, regs[10], 0x80000000U);
	KUNIT_EXPECT_EQ(test, regs[12], 0x80000100U);
	KUNIT_EXPECT_EQ(test,
			link[info->irq_base / sizeof(*link)] &
			RK_MPP_RKVDEC_LINK_FIX_RCB,
			(u32)RK_MPP_RKVDEC_LINK_FIX_RCB);
	KUNIT_EXPECT_EQ(test,
			link[info->irq_base / sizeof(*link)] &
			RK_MPP_RKVDEC_LINK_CCU_WORK_MODE,
			(u32)RK_MPP_RKVDEC_LINK_CCU_WORK_MODE);

	regs[10] = 0;
	rk_mpp_rkvdec2_prepare_core_for_ccu(&hw);
	KUNIT_EXPECT_EQ(test, regs[10], 0U);

	link[info->irq_base / sizeof(*link)] = 0;
	hw.rcb_size = 0x100;
	rk_mpp_rkvdec2_prepare_core_for_ccu(&hw);
	KUNIT_EXPECT_EQ(test,
			link[info->irq_base / sizeof(*link)] &
			RK_MPP_RKVDEC_LINK_FIX_RCB, 0U);
}

static void rk_mpp_hw_take_active_if_kunit(struct kunit *test)
{
	struct rk_mpp_hw hw = {};
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;
	u32 irq_status = 0;

	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);

	spin_lock_init(&hw.lock);
	hw.active_job = job0;
	hw.irq_status = 0x1234;

	KUNIT_EXPECT_FALSE(test, rk_mpp_hw_take_active_if(&hw, job1,
							  &irq_status));
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, job0);
	KUNIT_EXPECT_EQ(test, irq_status, 0U);
	KUNIT_EXPECT_EQ(test, hw.irq_status, 0x1234U);

	KUNIT_EXPECT_TRUE(test, rk_mpp_hw_take_active_if(&hw, job0,
							 &irq_status));
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, NULL);
	KUNIT_EXPECT_EQ(test, irq_status, 0x1234U);
	KUNIT_EXPECT_EQ(test, hw.irq_status, 0U);
}

static void rk_mpp_hw_prepare_active_retry_kunit(struct kunit *test)
{
	static const struct iommu_domain_ops iommu_ops;
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
	};
	struct iommu_domain domain = {
		.ops = &iommu_ops,
	};
	struct rk_mpp_hw hw = {};
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job1;

	job0 = kunit_kzalloc(test, sizeof(*job0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job0);
	job1 = kunit_kzalloc(test, sizeof(*job1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job1);

	spin_lock_init(&hw.lock);
	hw.iommu_domain = &domain;
	job0->session = &session;
	job1->session = &session;
	hw.active_job = job0;
	hw.irq_status = 0x1234;
	atomic_set(&hw.iommu_fault_pending, 1);

	KUNIT_EXPECT_FALSE(test, rk_mpp_hw_prepare_active_retry(&hw, job1));
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, job0);
	KUNIT_EXPECT_EQ(test, hw.irq_status, 0x1234U);
	KUNIT_EXPECT_EQ(test, atomic_read(&hw.iommu_fault_pending), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.iommu_refresh_count), 0);

	KUNIT_EXPECT_TRUE(test, rk_mpp_hw_prepare_active_retry(&hw, job0));
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, job0);
	KUNIT_EXPECT_EQ(test, hw.irq_status, 0U);
	KUNIT_EXPECT_EQ(test, atomic_read(&hw.iommu_fault_pending), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.iommu_refresh_count), 0);

	rk_mpp_hw_refresh_iommu(&hw, job0);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.iommu_refresh_count), 1);

	hw.iommu_domain = NULL;
	rk_mpp_hw_refresh_iommu(&hw, job0);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.iommu_refresh_count), 1);
}

static void rk_mpp_kunit_device_release(struct device *dev)
{
}

static void rk_mpp_hw_abort_ccu_dependents_kunit(struct kunit *test)
{
	struct device_node *ccu_node;
	struct rk_mpp_session *session;
	struct device *ccu_dev;
	struct device *core0_dev;
	struct device *core1_dev;
	struct rk_mpp_hw *ccu;
	struct rk_mpp_hw *core0;
	struct rk_mpp_hw *core1;
	struct rk_mpp_job *queued;
	struct rk_mpp_job *active;

	ccu_node = kunit_kzalloc(test, sizeof(*ccu_node), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_node);
	session = kunit_kzalloc(test, sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	ccu_dev = kunit_kzalloc(test, sizeof(*ccu_dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_dev);
	core0_dev = kunit_kzalloc(test, sizeof(*core0_dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core0_dev);
	core1_dev = kunit_kzalloc(test, sizeof(*core1_dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core1_dev);
	queued = kunit_kzalloc(test, sizeof(*queued), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, queued);
	active = kunit_kzalloc(test, sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);
	ccu = kunit_kzalloc(test, sizeof(*ccu), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu);
	core0 = kunit_kzalloc(test, sizeof(*core0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core0);
	core1 = kunit_kzalloc(test, sizeof(*core1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, core1);

	device_initialize(ccu_dev);
	ccu_dev->of_node = ccu_node;
	ccu_dev->release = rk_mpp_kunit_device_release;
	device_initialize(core0_dev);
	core0_dev->release = rk_mpp_kunit_device_release;
	device_initialize(core1_dev);
	core1_dev->release = rk_mpp_kunit_device_release;
	pm_runtime_set_active(core1_dev);
	pm_runtime_enable(core1_dev);
	pm_runtime_get_noresume(core1_dev);

	mutex_init(&rk_mpp_srv.hw_lock);
	mutex_init(&rk_mpp_srv.sched_lock);
	spin_lock_init(&rk_mpp_srv.rkvenc_dchs_lock);
	INIT_LIST_HEAD(&rk_mpp_srv.hw_list);
	INIT_LIST_HEAD(&rk_mpp_srv.queued_jobs);
	INIT_WORK(&rk_mpp_srv.sched_work, rk_mpp_scheduler_work);
	atomic_set(&rk_mpp_srv.queued_job_count, 1);

	session->srv = &rk_mpp_srv;
	session->active_job_count = 2;
	mutex_init(&session->lock);
	INIT_LIST_HEAD(&session->imports);
	INIT_LIST_HEAD(&session->active_jobs);
	init_waitqueue_head(&session->wait);
	refcount_set(&session->refs, 1);

	ccu->dev = ccu_dev;
	refcount_set(&ccu->refs, 1);
	init_completion(&ccu->released);
	mutex_init(&ccu->run_lock);
	spin_lock_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->link);
	INIT_LIST_HEAD(&ccu->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&ccu->rkvdec_link_jobs);
	INIT_DELAYED_WORK(&ccu->timeout_work, rk_mpp_hw_timeout_work);

	core0->dev = core0_dev;
	core0->ccu_node = ccu_node;
	core0->online = true;
	refcount_set(&core0->refs, 2);
	init_completion(&core0->released);
	mutex_init(&core0->run_lock);
	spin_lock_init(&core0->lock);
	INIT_LIST_HEAD(&core0->link);
	INIT_LIST_HEAD(&core0->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&core0->rkvdec_link_jobs);
	INIT_DELAYED_WORK(&core0->timeout_work, rk_mpp_hw_timeout_work);

	core1->dev = core1_dev;
	core1->ccu_node = ccu_node;
	core1->online = true;
	refcount_set(&core1->refs, 2);
	init_completion(&core1->released);
	mutex_init(&core1->run_lock);
	spin_lock_init(&core1->lock);
	INIT_LIST_HEAD(&core1->link);
	INIT_LIST_HEAD(&core1->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&core1->rkvdec_link_jobs);
	INIT_DELAYED_WORK(&core1->timeout_work, rk_mpp_hw_timeout_work);

	list_add_tail(&core0->link, &rk_mpp_srv.hw_list);
	list_add_tail(&core1->link, &rk_mpp_srv.hw_list);

	queued->session = session;
	queued->hw = core0;
	queued->state = RK_MPP_JOB_ACTIVE;
	queued->result = -EINPROGRESS;
	refcount_set(&queued->refs, 2);
	INIT_LIST_HEAD(&queued->link);
	INIT_LIST_HEAD(&queued->session_link);
	INIT_LIST_HEAD(&queued->sched_link);
	INIT_LIST_HEAD(&queued->rkvdec_ccu_node);
	INIT_LIST_HEAD(&queued->rkvdec_link_node);
	list_add_tail(&queued->session_link, &session->active_jobs);
	list_add_tail(&queued->sched_link, &rk_mpp_srv.queued_jobs);
	atomic_set(&core0->queued_job_count, 1);

	active->session = session;
	active->hw = core1;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session->active_jobs);
	core1->active_job = active;

	rk_mpp_hw_abort_ccu_dependents(ccu);
	flush_work(&rk_mpp_srv.sched_work);

	KUNIT_EXPECT_TRUE(test, list_empty(&rk_mpp_srv.queued_jobs));
	KUNIT_EXPECT_EQ(test, atomic_read(&rk_mpp_srv.queued_job_count), 0);
	KUNIT_EXPECT_TRUE(test, list_empty(&queued->sched_link));
	KUNIT_EXPECT_FALSE(test, list_empty(&queued->session_link));
	KUNIT_EXPECT_EQ(test, queued->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_DONE);
	KUNIT_EXPECT_EQ(test, queued->result, -ENODEV);
	KUNIT_EXPECT_PTR_EQ(test, queued->hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&queued->refs), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&core0->queued_job_count), 0);
	KUNIT_EXPECT_EQ(test, refcount_read(&core0->refs), 1);

	KUNIT_EXPECT_PTR_EQ(test, core1->active_job, NULL);
	KUNIT_EXPECT_TRUE(test, list_empty(&active->sched_link));
	KUNIT_EXPECT_FALSE(test, list_empty(&active->session_link));
	KUNIT_EXPECT_EQ(test, active->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_DONE);
	KUNIT_EXPECT_EQ(test, active->result, -ENODEV);
	KUNIT_EXPECT_PTR_EQ(test, active->hw, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&active->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&core1->refs), 1);
	KUNIT_EXPECT_EQ(test, session->active_job_count, 2U);

	list_del_init(&queued->session_link);
	list_del_init(&active->session_link);
	list_del_init(&core0->link);
	list_del_init(&core1->link);
	INIT_LIST_HEAD(&rk_mpp_srv.hw_list);
	INIT_LIST_HEAD(&rk_mpp_srv.queued_jobs);
	atomic_set(&rk_mpp_srv.queued_job_count, 0);

	pm_runtime_disable(core1_dev);
	put_device(core1_dev);
	put_device(core0_dev);
	put_device(ccu_dev);
}

static void rk_mpp_core_counter_kunit(struct kunit *test)
{
	atomic_t rkvenc[RK_MPP_CORE_COUNTER_COUNT];
	atomic_t rkvdec[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t rkvenc_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t rkvdec_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t rkvenc_max_ns[RK_MPP_CORE_COUNTER_COUNT];
	atomic64_t rkvdec_max_ns[RK_MPP_CORE_COUNTER_COUNT];
	struct rk_mpp_hw hw = {
		.match = &rk_mpp_rkvenc2_core,
		.core_id = 1,
	};

	for (u32 i = 0; i < RK_MPP_CORE_COUNTER_COUNT; i++) {
		atomic_set(&rkvenc[i], 0);
		atomic_set(&rkvdec[i], 0);
		atomic64_set(&rkvenc_ns[i], 0);
		atomic64_set(&rkvdec_ns[i], 0);
		atomic64_set(&rkvenc_max_ns[i], 0);
		atomic64_set(&rkvdec_max_ns[i], 0);
	}

	KUNIT_EXPECT_EQ(test, rk_mpp_core_counter_index(&hw), 1);
	rk_mpp_count_core(rkvenc, rkvdec, &hw);
	rk_mpp_count_core_ns(rkvenc_ns, rkvdec_ns, &hw, 100, false);
	rk_mpp_count_core_ns(rkvenc_max_ns, rkvdec_max_ns, &hw, 7, true);
	rk_mpp_count_core_ns(rkvenc_max_ns, rkvdec_max_ns, &hw, 3, true);
	KUNIT_EXPECT_EQ(test, atomic_read(&rkvenc[1]), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&rkvdec[1]), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvenc_ns[1]), 100LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvdec_ns[1]), 0LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvenc_max_ns[1]), 7LL);

	hw.match = &rk_mpp_rkvdec2_core;
	hw.core_id = 2;
	KUNIT_EXPECT_EQ(test, rk_mpp_core_counter_index(&hw), 2);
	rk_mpp_count_core(rkvenc, rkvdec, &hw);
	rk_mpp_count_core_ns(rkvenc_ns, rkvdec_ns, &hw, 50, false);
	rk_mpp_count_core_ns(rkvenc_max_ns, rkvdec_max_ns, &hw, 9, true);
	KUNIT_EXPECT_EQ(test, atomic_read(&rkvenc[2]), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&rkvdec[2]), 1);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvenc_ns[2]), 0LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvdec_ns[2]), 50LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvdec_max_ns[2]), 9LL);

	hw.core_id = RK_MPP_CORE_COUNTER_COUNT;
	KUNIT_EXPECT_EQ(test, rk_mpp_core_counter_index(&hw), -EINVAL);
	rk_mpp_count_core(rkvenc, rkvdec, &hw);
	rk_mpp_count_core_ns(rkvenc_ns, rkvdec_ns, &hw, 1000, false);

	hw.core_id = -1;
	KUNIT_EXPECT_EQ(test, rk_mpp_core_counter_index(&hw), -EINVAL);
	rk_mpp_count_core(rkvenc, rkvdec, &hw);
	rk_mpp_count_core_ns(rkvenc_ns, rkvdec_ns, &hw, 1000, false);

	hw.match = &rk_mpp_rkvenc2_ccu;
	hw.core_id = 0;
	KUNIT_EXPECT_EQ(test, rk_mpp_core_counter_index(&hw), -EINVAL);
	rk_mpp_count_core(rkvenc, rkvdec, &hw);
	rk_mpp_count_core_ns(rkvenc_ns, rkvdec_ns, &hw, 1000, false);

	KUNIT_EXPECT_EQ(test, atomic_read(&rkvenc[1]), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&rkvdec[2]), 1);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvenc_ns[1]), 100LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvdec_ns[2]), 50LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvenc_max_ns[1]), 7LL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&rkvdec_max_ns[2]), 9LL);
}

static void rk_mpp_hw_select_rotation_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
		.client_type = RK_MPP_DEVICE_RKVENC,
	};
	struct rk_mpp_hw *hw0;
	struct rk_mpp_hw *hw1;
	struct rk_mpp_hw *hw2;
	struct rk_mpp_hw *selected;

	mutex_init(&srv.hw_lock);
	INIT_LIST_HEAD(&srv.hw_list);
	hw0 = kunit_kzalloc(test, sizeof(*hw0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw0);
	hw1 = kunit_kzalloc(test, sizeof(*hw1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw1);
	hw2 = kunit_kzalloc(test, sizeof(*hw2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw2);

	hw0->match = &rk_mpp_rkvenc2_core;
	hw0->core_id = 0;
	hw0->online = true;
	hw1->match = &rk_mpp_rkvenc2_core;
	hw1->core_id = 1;
	hw1->online = true;
	hw2->match = &rk_mpp_rkvenc2_core;
	hw2->core_id = 2;
	hw2->online = true;
	spin_lock_init(&hw0->lock);
	spin_lock_init(&hw1->lock);
	spin_lock_init(&hw2->lock);
	refcount_set(&hw0->refs, 1);
	refcount_set(&hw1->refs, 1);
	refcount_set(&hw2->refs, 1);
	INIT_LIST_HEAD(&hw0->link);
	INIT_LIST_HEAD(&hw1->link);
	INIT_LIST_HEAD(&hw2->link);
	list_add_tail(&hw0->link, &srv.hw_list);
	list_add_tail(&hw1->link, &srv.hw_list);
	list_add_tail(&hw2->link, &srv.hw_list);

	selected = rk_mpp_hw_get_for_session(&session, true);
	KUNIT_EXPECT_PTR_EQ(test, selected, hw0);
	KUNIT_EXPECT_EQ(test, srv.core_select_seq, 1U);
	rk_mpp_hw_put(selected);

	selected = rk_mpp_hw_get_for_session(&session, true);
	KUNIT_EXPECT_PTR_EQ(test, selected, hw1);
	KUNIT_EXPECT_EQ(test, srv.core_select_seq, 2U);
	rk_mpp_hw_put(selected);

	selected = rk_mpp_hw_get_for_session(&session, true);
	KUNIT_EXPECT_PTR_EQ(test, selected, hw2);
	KUNIT_EXPECT_EQ(test, srv.core_select_seq, 3U);
	rk_mpp_hw_put(selected);

	srv.core_select_seq = 2;
	atomic_set(&hw2->queued_job_count, 1);
	selected = rk_mpp_hw_get_for_session(&session, true);
	KUNIT_EXPECT_PTR_EQ(test, selected, hw0);
	KUNIT_EXPECT_EQ(test, srv.core_select_seq, 1U);
	rk_mpp_hw_put(selected);
	atomic_set(&hw2->queued_job_count, 0);

	srv.core_select_seq = 2;
	selected = rk_mpp_hw_get_for_session(&session, false);
	KUNIT_EXPECT_PTR_EQ(test, selected, hw0);
	KUNIT_EXPECT_EQ(test, srv.core_select_seq, 2U);
	rk_mpp_hw_put(selected);
}

static void rk_mpp_iommu_fault_match_kunit(struct kunit *test)
{
	struct rk_mpp_iommu_fault_match_fixture {
		struct iommu_domain domain0;
		struct iommu_domain domain1;
		struct iommu_domain domain2;
		struct device_node node0;
		struct device_node node1;
		struct device_node node2;
		struct device iommu_dev;
		struct rk_mpp_hw hw0;
		struct rk_mpp_hw hw1;
		struct rk_mpp_hw hw2;
	} *fixture;
	struct device *iommu_dev;
	struct rk_mpp_hw *hw0;
	struct rk_mpp_hw *hw1;
	struct rk_mpp_hw *hw2;
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

	INIT_LIST_HEAD(&hw0->fault_link);
	INIT_LIST_HEAD(&hw1->fault_link);
	INIT_LIST_HEAD(&hw2->fault_link);
	list_add_tail(&hw0->fault_link, &fault_hws);
	list_add_tail(&hw1->fault_link, &fault_hws);
	list_add_tail(&hw2->fault_link, &fault_hws);

	KUNIT_EXPECT_PTR_EQ(test,
			    rk_mpp_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       iommu_dev),
			    hw1);

	iommu_dev->of_node = &fixture->node2;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_mpp_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       iommu_dev),
			    hw0);

	iommu_dev->of_node = &fixture->node1;
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_mpp_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain1,
						       iommu_dev),
			    hw2);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_mpp_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain0,
						       NULL),
			    hw0);
	KUNIT_EXPECT_PTR_EQ(test,
			    rk_mpp_iommu_find_fault_hw(&fault_hws,
						       &fixture->domain2,
						       iommu_dev),
			    NULL);
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

static void rk_mpp_rkvenc_slice_fifo_kunit(struct kunit *test)
{
	struct rk_mpp_job *full_job;
	struct rk_mpp_job *last_job;
	u32 value = 0;
	int i;

	full_job = kunit_kzalloc(test, sizeof(*full_job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, full_job);
	last_job = kunit_kzalloc(test, sizeof(*last_job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, last_job);

	spin_lock_init(&last_job->rkvenc_slice_lock);
	INIT_KFIFO(last_job->rkvenc_slice_fifo);

	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_ready(last_job));
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_done(last_job));
	KUNIT_EXPECT_EQ(test,
			rk_mpp_job_pop_rkvenc_slice(last_job, &value),
			-EAGAIN);

	rk_mpp_job_push_rkvenc_slice(last_job,
				     RK_MPP_RKVENC_SLICE_LAST | 0x1234);

	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_ready(last_job));
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_done(last_job));
	KUNIT_EXPECT_EQ(test,
			rk_mpp_job_pop_rkvenc_slice(last_job, &value), 0);
	KUNIT_EXPECT_EQ(test, value, RK_MPP_RKVENC_SLICE_LAST | 0x1234);
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_ready(last_job));
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_done(last_job));

	spin_lock_init(&full_job->rkvenc_slice_lock);
	INIT_KFIFO(full_job->rkvenc_slice_fifo);

	for (i = 0; i < RK_MPP_RKVENC_MAX_SLICE_FIFO; i++)
		rk_mpp_job_push_rkvenc_slice(full_job, i);

	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_ready(full_job));
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvenc_slice_done(full_job));

	rk_mpp_job_push_rkvenc_slice(full_job, RK_MPP_RKVENC_SLICE_LAST);

	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_ready(full_job));
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvenc_slice_done(full_job));
	KUNIT_EXPECT_TRUE(test, full_job->rkvenc_slice_overflow);
	KUNIT_EXPECT_EQ(test,
			rk_mpp_job_pop_rkvenc_slice(full_job, &value),
			-EOVERFLOW);
}

static void rk_mpp_rkvenc2_dchs_remap_kunit(struct kunit *test)
{
	struct rk_mpp_service *srv;
	struct rk_mpp_session *session0;
	struct rk_mpp_session *session1;
	struct device_node *ccu_node;
	struct rk_mpp_hw *hw0;
	struct rk_mpp_hw *hw1;
	struct rk_mpp_hw *hw2;
	struct rk_mpp_job *producer;
	struct rk_mpp_job *consumer;
	struct rk_mpp_job *unrelated;
	u32 reg_words = RK_MPP_RKVENC_DCHS_WORD + 1;
	u32 producer_low;
	u32 consumer_low;
	u32 unrelated_low;
	u32 producer_patched;
	u32 consumer_patched;
	u32 unrelated_patched;

	srv = kunit_kzalloc(test, sizeof(*srv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, srv);
	session0 = kunit_kzalloc(test, sizeof(*session0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session0);
	session1 = kunit_kzalloc(test, sizeof(*session1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session1);
	ccu_node = kunit_kzalloc(test, sizeof(*ccu_node), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_node);
	hw0 = kunit_kzalloc(test, sizeof(*hw0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw0);
	hw1 = kunit_kzalloc(test, sizeof(*hw1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw1);
	hw2 = kunit_kzalloc(test, sizeof(*hw2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, hw2);
	producer = kunit_kzalloc(test, sizeof(*producer), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, producer);
	consumer = kunit_kzalloc(test, sizeof(*consumer), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, consumer);
	unrelated = kunit_kzalloc(test, sizeof(*unrelated), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, unrelated);

	spin_lock_init(&srv->rkvenc_dchs_lock);
	session0->srv = srv;
	session0->client_type = RK_MPP_DEVICE_RKVENC;
	session0->id = 11;
	session1->srv = srv;
	session1->client_type = RK_MPP_DEVICE_RKVENC;
	session1->id = 12;

	hw0->ccu_node = ccu_node;
	hw0->core_id = 0;
	hw1->ccu_node = ccu_node;
	hw1->core_id = 1;
	hw2->ccu_node = ccu_node;
	hw2->core_id = 2;

	producer->session = session0;
	producer->hw = hw0;
	producer->id = 100;
	producer->reg_image.reg_words = reg_words;
	producer->reg_image.regs =
		kunit_kcalloc(test, reg_words, sizeof(u32), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, producer->reg_image.regs);

	consumer->session = session0;
	consumer->hw = hw1;
	consumer->id = 101;
	consumer->reg_image.reg_words = reg_words;
	consumer->reg_image.regs =
		kunit_kcalloc(test, reg_words, sizeof(u32), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, consumer->reg_image.regs);

	unrelated->session = session1;
	unrelated->hw = hw2;
	unrelated->id = 102;
	unrelated->reg_image.reg_words = reg_words;
	unrelated->reg_image.regs =
		kunit_kcalloc(test, reg_words, sizeof(u32), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, unrelated->reg_image.regs);

	producer_low = 2 << RK_MPP_RKVENC_DCHS_TXID_SHIFT;
	producer->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD] = producer_low;
	rk_mpp_rkvenc2_dchs_patch(producer);
	producer_patched = producer->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD];
	KUNIT_EXPECT_TRUE(test, producer->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[0].job, producer);
	KUNIT_EXPECT_EQ(test, srv->rkvenc_dchs[0].txid_orig, 2U);
	KUNIT_EXPECT_EQ(test, producer_patched & RK_MPP_RKVENC_DCHS_TXE,
			(u32)RK_MPP_RKVENC_DCHS_TXE);
	KUNIT_EXPECT_EQ(test, producer_patched & RK_MPP_RKVENC_DCHS_RXE, 0U);
	KUNIT_EXPECT_EQ(test, producer_patched & RK_MPP_RKVENC_DCHS_TXID_MASK,
			0U);
	KUNIT_EXPECT_EQ(test, producer_patched & RK_MPP_RKVENC_DCHS_RXID_MASK,
			0U);

	unrelated_low = (2 << RK_MPP_RKVENC_DCHS_RXID_SHIFT) |
			RK_MPP_RKVENC_DCHS_RXE;
	unrelated->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD] = unrelated_low;
	rk_mpp_rkvenc2_dchs_patch(unrelated);
	unrelated_patched = unrelated->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD];
	KUNIT_EXPECT_TRUE(test, unrelated->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[2].job, unrelated);
	KUNIT_EXPECT_EQ(test, unrelated_patched & RK_MPP_RKVENC_DCHS_RXE,
			0U);
	KUNIT_EXPECT_EQ(test, unrelated_patched & RK_MPP_RKVENC_DCHS_TXID_MASK,
			1U << RK_MPP_RKVENC_DCHS_TXID_SHIFT);
	KUNIT_EXPECT_EQ(test, unrelated_patched & RK_MPP_RKVENC_DCHS_RXID_MASK,
			0U);
	rk_mpp_rkvenc2_dchs_release(unrelated);
	KUNIT_EXPECT_FALSE(test, unrelated->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[2].job, NULL);

	consumer_low = (3 << RK_MPP_RKVENC_DCHS_TXID_SHIFT) |
		       (2 << RK_MPP_RKVENC_DCHS_RXID_SHIFT) |
		       RK_MPP_RKVENC_DCHS_RXE;
	consumer->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD] = consumer_low;
	rk_mpp_rkvenc2_dchs_patch(consumer);
	consumer_patched = consumer->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD];
	KUNIT_EXPECT_TRUE(test, consumer->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[1].job, consumer);
	KUNIT_EXPECT_EQ(test, consumer_patched & RK_MPP_RKVENC_DCHS_RXE,
			(u32)RK_MPP_RKVENC_DCHS_RXE);
	KUNIT_EXPECT_EQ(test, consumer_patched & RK_MPP_RKVENC_DCHS_TXID_MASK,
			2U << RK_MPP_RKVENC_DCHS_TXID_SHIFT);
	KUNIT_EXPECT_EQ(test, consumer_patched & RK_MPP_RKVENC_DCHS_RXID_MASK,
			0U << RK_MPP_RKVENC_DCHS_RXID_SHIFT);

	rk_mpp_rkvenc2_dchs_release(consumer);
	KUNIT_EXPECT_FALSE(test, consumer->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[1].job, NULL);

	rk_mpp_rkvenc2_dchs_release(producer);
	KUNIT_EXPECT_FALSE(test, producer->rkvenc_dchs_active);
	KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[0].job, NULL);
}

static void rk_mpp_rkvenc2_dchs_independent_cores_kunit(struct kunit *test)
{
	struct rk_mpp_service *srv;
	struct rk_mpp_session *session;
	struct device_node *ccu_node;
	struct rk_mpp_hw *hws[RK_MPP_RKVENC_MAX_DCHS_CORES];
	struct rk_mpp_job *jobs[RK_MPP_RKVENC_MAX_DCHS_CORES];
	u32 reg_words = RK_MPP_RKVENC_DCHS_WORD + 1;
	u32 i;

	srv = kunit_kzalloc(test, sizeof(*srv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, srv);
	session = kunit_kzalloc(test, sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	ccu_node = kunit_kzalloc(test, sizeof(*ccu_node), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ccu_node);

	spin_lock_init(&srv->rkvenc_dchs_lock);
	session->srv = srv;
	session->client_type = RK_MPP_DEVICE_RKVENC;
	session->id = 20;

	for (i = 0; i < ARRAY_SIZE(jobs); i++) {
		u32 patched;

		hws[i] = kunit_kzalloc(test, sizeof(*hws[i]), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, hws[i]);
		jobs[i] = kunit_kzalloc(test, sizeof(*jobs[i]), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, jobs[i]);

		hws[i]->ccu_node = ccu_node;
		hws[i]->core_id = i;
		jobs[i]->session = session;
		jobs[i]->hw = hws[i];
		jobs[i]->id = 200 + i;
		jobs[i]->reg_image.reg_words = reg_words;
		jobs[i]->reg_image.regs =
			kunit_kcalloc(test, reg_words, sizeof(u32),
				      GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, jobs[i]->reg_image.regs);

		jobs[i]->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD] =
			i << RK_MPP_RKVENC_DCHS_TXID_SHIFT;
		rk_mpp_rkvenc2_dchs_patch(jobs[i]);
		patched = jobs[i]->reg_image.regs[RK_MPP_RKVENC_DCHS_WORD];

		KUNIT_EXPECT_TRUE(test, jobs[i]->rkvenc_dchs_active);
		KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[i].job, jobs[i]);
		KUNIT_EXPECT_EQ(test,
				patched & RK_MPP_RKVENC_DCHS_TXID_MASK,
				i << RK_MPP_RKVENC_DCHS_TXID_SHIFT);
		KUNIT_EXPECT_EQ(test,
				patched & RK_MPP_RKVENC_DCHS_RXE, 0U);
	}

	for (i = 0; i < ARRAY_SIZE(jobs); i++) {
		rk_mpp_rkvenc2_dchs_release(jobs[i]);
		KUNIT_EXPECT_FALSE(test, jobs[i]->rkvenc_dchs_active);
		KUNIT_EXPECT_PTR_EQ(test, srv->rkvenc_dchs[i].job, NULL);
	}
}

static void rk_mpp_rcb_invalid_index_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {
		.client_type = RK_MPP_DEVICE_RKVENC,
	};
	struct rk_mpp_hw hw = {
		.rcb_iova = 0x80000000,
		.rcb_size = 0x300,
	};
	u32 max_words = RK_MPP_MAX_REG_IMAGE_BYTES / sizeof(u32);
	struct rk_mpp_job *job;

	mutex_init(&session.lock);

	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	job->session = &session;
	job->hw = &hw;
	job->reg_image.rcb_count = 3;
	job->reg_image.rcb_descs[0].index = 2;
	job->reg_image.rcb_descs[0].size = 0x100;
	job->reg_image.rcb_descs[1].index = max_words;
	job->reg_image.rcb_descs[1].size = 0x100;
	job->reg_image.rcb_descs[2].index = 4;
	job->reg_image.rcb_descs[2].size = 0x100;

	KUNIT_EXPECT_EQ(test, rk_mpp_job_apply_rcb_info(job), 0);
	KUNIT_ASSERT_NOT_NULL(test, job->reg_image.regs);
	KUNIT_ASSERT_GT(test, job->reg_image.reg_words, 4U);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[2], 0x80000000U);
	KUNIT_EXPECT_EQ(test, job->reg_image.regs[4], 0x80000100U);

	kfree(job->reg_image.regs);
}

static void rk_mpp_rkvdec_rcb_width_gate_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {
		.client_type = RK_MPP_DEVICE_RKVDEC,
	};
	struct rk_mpp_hw hw = {
		.rcb_min_width = 1920,
	};
	struct rk_mpp_job job = {
		.session = &session,
		.hw = &hw,
	};

	mutex_init(&session.lock);

	session.codec_info[RK_MPP_DEC_INFO_WIDTH].val = 1919;
	KUNIT_EXPECT_FALSE(test, rk_mpp_job_rkvdec_rcb_enabled(&job));

	session.codec_info[RK_MPP_DEC_INFO_WIDTH].val = 1920;
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvdec_rcb_enabled(&job));

	session.codec_info[RK_MPP_DEC_INFO_WIDTH].val = 0;
	hw.rcb_min_width = 0;
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvdec_rcb_enabled(&job));

	hw.rcb_min_width = 1920;
	session.client_type = RK_MPP_DEVICE_RKVENC;
	KUNIT_EXPECT_TRUE(test, rk_mpp_job_rkvdec_rcb_enabled(&job));
}

static void rk_mpp_switch_session_status_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {};
	struct rk_mpp_session *active = &session;
	struct fd held_fd = {};
	struct mpp_bat_msg bat = {
		.fd = U32_MAX,
		.ret = 1234,
	};
	struct mpp_bat_msg out = {};
	struct rk_mpp_msg_v1 msg = {};
	void __user *user;
	unsigned long uncopied;

	user = rk_mpp_kunit_user_payload(test, &bat, sizeof(bat));
	KUNIT_ASSERT_NOT_NULL(test, user);
	msg.data_ptr = (uintptr_t)user;
	msg.size = sizeof(bat);

	KUNIT_EXPECT_EQ(test, rk_mpp_switch_session(&active, &held_fd, &msg),
			0);
	KUNIT_EXPECT_PTR_EQ(test, active, &session);
	KUNIT_EXPECT_PTR_EQ(test, fd_file(held_fd), NULL);
	uncopied = copy_from_user(&out, user, sizeof(out));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, out.ret, -EBADF);
	KUNIT_EXPECT_EQ(test, out.fd, U32_MAX);
	KUNIT_EXPECT_EQ(test, out.flag, 0ULL);

	bat.flag = MPP_BAT_MSG_DONE;
	bat.fd = U32_MAX;
	bat.ret = 77;
	out = (struct mpp_bat_msg){};
	user = rk_mpp_kunit_user_payload(test, &bat, sizeof(bat));
	KUNIT_ASSERT_NOT_NULL(test, user);
	msg.data_ptr = (uintptr_t)user;

	KUNIT_EXPECT_EQ(test, rk_mpp_switch_session(&active, &held_fd, &msg),
			0);
	KUNIT_EXPECT_PTR_EQ(test, active, &session);
	KUNIT_EXPECT_PTR_EQ(test, fd_file(held_fd), NULL);
	uncopied = copy_from_user(&out, user, sizeof(out));
	KUNIT_EXPECT_EQ(test, uncopied, 0UL);
	KUNIT_EXPECT_EQ(test, out.ret, 77);
	KUNIT_EXPECT_EQ(test, out.fd, U32_MAX);
	KUNIT_EXPECT_EQ(test, out.flag, (__u64)MPP_BAT_MSG_DONE);
}

static void rk_mpp_batch_session_switch_split_kunit(struct kunit *test)
{
	struct rk_mpp_session session0 = {};
	struct rk_mpp_session session1 = {};
	struct rk_mpp_batch_state batch = {};
	struct rk_mpp_job *job0;
	struct rk_mpp_job *job0_again;
	struct rk_mpp_job *job0_second;
	struct rk_mpp_job *job1;
	struct rk_mpp_job *iter;

	INIT_LIST_HEAD(&batch.jobs);
	refcount_set(&session0.refs, 1);
	refcount_set(&session1.refs, 1);

	job0 = rk_mpp_batch_get_job(&batch, &session0);
	KUNIT_ASSERT_FALSE(test, IS_ERR(job0));
	job0_again = rk_mpp_batch_get_job(&batch, &session0);
	KUNIT_EXPECT_PTR_EQ(test, job0_again, job0);
	KUNIT_EXPECT_TRUE(test, list_is_singular(&batch.jobs));
	KUNIT_EXPECT_EQ(test, refcount_read(&session0.refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&session1.refs), 1);

	batch.cur_job = NULL;
	job1 = rk_mpp_batch_get_job(&batch, &session1);
	KUNIT_ASSERT_FALSE(test, IS_ERR(job1));
	KUNIT_EXPECT_PTR_NE(test, job1, job0);
	KUNIT_EXPECT_EQ(test, refcount_read(&session0.refs), 2);
	KUNIT_EXPECT_EQ(test, refcount_read(&session1.refs), 2);

	batch.cur_job = NULL;
	job0_second = rk_mpp_batch_get_job(&batch, &session0);
	KUNIT_ASSERT_FALSE(test, IS_ERR(job0_second));
	KUNIT_EXPECT_PTR_NE(test, job0_second, job0);
	KUNIT_EXPECT_PTR_NE(test, job0_second, job1);
	KUNIT_EXPECT_EQ(test, refcount_read(&session0.refs), 3);
	KUNIT_EXPECT_EQ(test, refcount_read(&session1.refs), 2);

	iter = list_first_entry(&batch.jobs, struct rk_mpp_job, link);
	KUNIT_EXPECT_PTR_EQ(test, iter, job0);
	iter = list_next_entry(iter, link);
	KUNIT_EXPECT_PTR_EQ(test, iter, job1);
	iter = list_next_entry(iter, link);
	KUNIT_EXPECT_PTR_EQ(test, iter, job0_second);
	KUNIT_EXPECT_TRUE(test, list_is_last(&iter->link, &batch.jobs));

	rk_mpp_batch_release_jobs(&batch);
	KUNIT_EXPECT_TRUE(test, list_empty(&batch.jobs));
	KUNIT_EXPECT_PTR_EQ(test, batch.cur_job, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&session0.refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&session1.refs), 1);
}

static void rk_mpp_release_fd_all_devices_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {};
	struct device *dev0;
	struct device *dev1;
	struct rk_mpp_import *fd0_dev0;
	struct rk_mpp_import *fd0_dev1;
	struct rk_mpp_import *fd1_dev0;
	struct rk_mpp_import *iter;
	unsigned int remaining = 0;

	mutex_init(&session.lock);
	INIT_LIST_HEAD(&session.imports);

	dev0 = kunit_kzalloc(test, sizeof(*dev0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev0);
	dev1 = kunit_kzalloc(test, sizeof(*dev1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev1);

	fd0_dev0 = kunit_kzalloc(test, sizeof(*fd0_dev0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fd0_dev0);
	fd0_dev0->fd = 7;
	fd0_dev0->dev = dev0;
	refcount_set(&fd0_dev0->refs, 2);
	INIT_LIST_HEAD(&fd0_dev0->link);
	list_add_tail(&fd0_dev0->link, &session.imports);

	fd0_dev1 = kunit_kzalloc(test, sizeof(*fd0_dev1), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fd0_dev1);
	fd0_dev1->fd = 7;
	fd0_dev1->dev = dev1;
	refcount_set(&fd0_dev1->refs, 2);
	INIT_LIST_HEAD(&fd0_dev1->link);
	list_add_tail(&fd0_dev1->link, &session.imports);

	fd1_dev0 = kunit_kzalloc(test, sizeof(*fd1_dev0), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, fd1_dev0);
	fd1_dev0->fd = 8;
	fd1_dev0->dev = dev0;
	refcount_set(&fd1_dev0->refs, 2);
	INIT_LIST_HEAD(&fd1_dev0->link);
	list_add_tail(&fd1_dev0->link, &session.imports);

	KUNIT_EXPECT_EQ(test, rk_mpp_release_fd(&session, 7), 0);
	KUNIT_EXPECT_TRUE(test, list_empty(&fd0_dev0->link));
	KUNIT_EXPECT_TRUE(test, list_empty(&fd0_dev1->link));
	KUNIT_EXPECT_FALSE(test, list_empty(&fd1_dev0->link));
	KUNIT_EXPECT_EQ(test, refcount_read(&fd0_dev0->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&fd0_dev1->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&fd1_dev0->refs), 2);

	list_for_each_entry(iter, &session.imports, link) {
		remaining++;
		KUNIT_EXPECT_EQ(test, iter->fd, 8);
		KUNIT_EXPECT_PTR_EQ(test, iter->dev, dev0);
	}
	KUNIT_EXPECT_EQ(test, remaining, 1U);

	KUNIT_EXPECT_EQ(test, rk_mpp_release_fd(&session, 7), -EINVAL);
	KUNIT_EXPECT_EQ(test, rk_mpp_release_fd(&session, 99), -EINVAL);
	KUNIT_EXPECT_FALSE(test, list_empty(&fd1_dev0->link));

	list_del_init(&fd1_dev0->link);
}

static void rk_mpp_session_poll_nonblock_pending_kunit(struct kunit *test)
{
	struct rk_mpp_session session = {
		.active_job_count = 1,
	};
	struct rk_mpp_job *job;

	mutex_init(&session.lock);
	INIT_LIST_HEAD(&session.active_jobs);
	init_waitqueue_head(&session.wait);

	job = kunit_kzalloc(test, sizeof(*job), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, job);

	job->session = &session;
	job->state = RK_MPP_JOB_ACTIVE;
	job->result = -EINPROGRESS;
	refcount_set(&job->refs, 1);
	INIT_LIST_HEAD(&job->session_link);
	list_add_tail(&job->session_link, &session.active_jobs);

	KUNIT_EXPECT_EQ(test,
			rk_mpp_session_poll_job(&session,
						MPP_FLAGS_POLL_NON_BLOCK),
			-EAGAIN);
	KUNIT_EXPECT_FALSE(test, list_empty(&session.active_jobs));
	KUNIT_EXPECT_EQ(test, session.active_job_count, 1U);
	KUNIT_EXPECT_EQ(test, job->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_ACTIVE);
	KUNIT_EXPECT_EQ(test, job->result, -EINPROGRESS);
	KUNIT_EXPECT_EQ(test, refcount_read(&job->refs), 1);

	list_del_init(&job->session_link);
}

static void rk_mpp_session_abort_jobs_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
		.active_job_count = 2,
	};
	struct rk_mpp_hw hw = {};
	struct rk_mpp_job *queued;
	struct rk_mpp_job *active;

	mutex_init(&srv.sched_lock);
	INIT_LIST_HEAD(&srv.queued_jobs);
	mutex_init(&session.lock);
	INIT_LIST_HEAD(&session.imports);
	INIT_LIST_HEAD(&session.active_jobs);
	init_waitqueue_head(&session.wait);
	spin_lock_init(&hw.lock);
	mutex_init(&hw.run_lock);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_mpp_hw_timeout_work);

	queued = kunit_kzalloc(test, sizeof(*queued), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, queued);
	active = kunit_kzalloc(test, sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);

	queued->session = &session;
	queued->hw = &hw;
	queued->state = RK_MPP_JOB_ACTIVE;
	queued->result = -EINPROGRESS;
	refcount_set(&queued->refs, 3);
	INIT_LIST_HEAD(&queued->link);
	INIT_LIST_HEAD(&queued->session_link);
	INIT_LIST_HEAD(&queued->sched_link);
	INIT_LIST_HEAD(&queued->rkvdec_ccu_node);
	INIT_LIST_HEAD(&queued->rkvdec_link_node);
	list_add_tail(&queued->session_link, &session.active_jobs);
	list_add_tail(&queued->sched_link, &srv.queued_jobs);
	atomic_set(&hw.queued_job_count, 1);
	atomic_set(&srv.queued_job_count, 1);

	active->session = &session;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session.active_jobs);

	rk_mpp_session_abort_jobs(&session);

	KUNIT_EXPECT_TRUE(test, list_empty(&session.active_jobs));
	KUNIT_EXPECT_EQ(test, session.active_job_count, 0U);
	KUNIT_EXPECT_TRUE(test, list_empty(&srv.queued_jobs));
	KUNIT_EXPECT_TRUE(test, list_empty(&queued->session_link));
	KUNIT_EXPECT_TRUE(test, list_empty(&queued->sched_link));
	KUNIT_EXPECT_TRUE(test, list_empty(&active->session_link));
	KUNIT_EXPECT_TRUE(test, list_empty(&active->sched_link));
	KUNIT_EXPECT_EQ(test, queued->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, active->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, queued->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_DONE);
	KUNIT_EXPECT_EQ(test, active->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_DONE);
	KUNIT_EXPECT_EQ(test, refcount_read(&queued->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&active->refs), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&hw.queued_job_count), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.queued_job_count), 0);
	KUNIT_EXPECT_EQ(test,
			rk_mpp_session_poll_job(&session,
						MPP_FLAGS_POLL_NON_BLOCK),
			-EIO);
}

static void rk_mpp_session_abort_hw_active_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session *session;
	struct device *dev;
	struct rk_mpp_hw hw = {};
	struct rk_mpp_job *active;

	session = kunit_kzalloc(test, sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	active = kzalloc(sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);

	device_initialize(dev);
	dev->release = rk_mpp_kunit_device_release;
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);

	mutex_init(&srv.sched_lock);
	INIT_LIST_HEAD(&srv.queued_jobs);

	session->srv = &srv;
	session->active_job_count = 1;
	mutex_init(&session->lock);
	INIT_LIST_HEAD(&session->imports);
	INIT_LIST_HEAD(&session->active_jobs);
	init_waitqueue_head(&session->wait);
	refcount_set(&session->refs, 2);

	hw.dev = dev;
	refcount_set(&hw.refs, 1);
	init_completion(&hw.released);
	spin_lock_init(&hw.lock);
	mutex_init(&hw.run_lock);
	INIT_LIST_HEAD(&hw.rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&hw.rkvdec_link_jobs);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_mpp_hw_timeout_work);

	active->session = session;
	active->hw = &hw;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session->active_jobs);
	hw.active_job = active;

	rk_mpp_session_abort_jobs(session);

	KUNIT_EXPECT_TRUE(test, list_empty(&session->active_jobs));
	KUNIT_EXPECT_EQ(test, session->active_job_count, 0U);
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, NULL);
	KUNIT_EXPECT_TRUE(test, completion_done(&hw.released));
	KUNIT_EXPECT_EQ(test, refcount_read(&session->refs), 1);

	pm_runtime_disable(dev);
	put_device(dev);
}

static void rk_mpp_reset_session_public_cleanup_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session session = {
		.srv = &srv,
		.initialized = true,
		.active_job_count = 2,
	};
	struct rk_mpp_batch_state batch = {};
	struct mpp_request req = {
		.cmd = MPP_CMD_RESET_SESSION,
	};
	struct rk_mpp_import *import;
	struct rk_mpp_hw hw = {};
	struct rk_mpp_job *queued;
	struct rk_mpp_job *active;

	mutex_init(&srv.sched_lock);
	INIT_LIST_HEAD(&srv.queued_jobs);
	mutex_init(&session.lock);
	INIT_LIST_HEAD(&session.imports);
	INIT_LIST_HEAD(&session.active_jobs);
	init_waitqueue_head(&session.wait);
	INIT_LIST_HEAD(&batch.jobs);
	spin_lock_init(&hw.lock);
	mutex_init(&hw.run_lock);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_mpp_hw_timeout_work);

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, import);
	import->fd = 7;
	refcount_set(&import->refs, 2);
	INIT_LIST_HEAD(&import->link);
	list_add_tail(&import->link, &session.imports);

	queued = kunit_kzalloc(test, sizeof(*queued), GFP_KERNEL);
	active = kunit_kzalloc(test, sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, queued);
	KUNIT_ASSERT_NOT_NULL(test, active);

	queued->session = &session;
	queued->hw = &hw;
	queued->state = RK_MPP_JOB_ACTIVE;
	queued->result = -EINPROGRESS;
	refcount_set(&queued->refs, 3);
	INIT_LIST_HEAD(&queued->link);
	INIT_LIST_HEAD(&queued->session_link);
	INIT_LIST_HEAD(&queued->sched_link);
	INIT_LIST_HEAD(&queued->rkvdec_ccu_node);
	INIT_LIST_HEAD(&queued->rkvdec_link_node);
	list_add_tail(&queued->session_link, &session.active_jobs);
	list_add_tail(&queued->sched_link, &srv.queued_jobs);
	atomic_set(&hw.queued_job_count, 1);
	atomic_set(&srv.queued_job_count, 1);

	active->session = &session;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session.active_jobs);

	KUNIT_EXPECT_EQ(test,
			rk_mpp_process_request(&session, &req, &batch),
			0);
	KUNIT_EXPECT_TRUE(test, list_empty(&session.imports));
	KUNIT_EXPECT_EQ(test, refcount_read(&import->refs), 1);
	KUNIT_EXPECT_TRUE(test, list_empty(&session.active_jobs));
	KUNIT_EXPECT_EQ(test, session.active_job_count, 0U);
	KUNIT_EXPECT_TRUE(test, list_empty(&srv.queued_jobs));
	KUNIT_EXPECT_TRUE(test, list_empty(&queued->session_link));
	KUNIT_EXPECT_TRUE(test, list_empty(&queued->sched_link));
	KUNIT_EXPECT_TRUE(test, list_empty(&active->session_link));
	KUNIT_EXPECT_EQ(test, queued->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, active->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, refcount_read(&queued->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&active->refs), 1);
	KUNIT_EXPECT_EQ(test, atomic_read(&hw.queued_job_count), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&srv.queued_job_count), 0);

	rk_mpp_import_put(import);
}

static void rk_mpp_reset_session_hw_active_import_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session *session;
	struct device *dev;
	struct rk_mpp_hw hw = {};
	struct rk_mpp_batch_state batch = {};
	struct mpp_request req = {
		.cmd = MPP_CMD_RESET_SESSION,
	};
	struct rk_mpp_import *import;
	struct rk_mpp_job *active;

	session = kunit_kzalloc(test, sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	dev = kunit_kzalloc(test, sizeof(*dev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dev);
	import = kzalloc(sizeof(*import), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, import);
	active = kzalloc(sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);

	device_initialize(dev);
	dev->release = rk_mpp_kunit_device_release;
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);

	mutex_init(&srv.sched_lock);
	INIT_LIST_HEAD(&srv.queued_jobs);

	session->srv = &srv;
	session->initialized = true;
	session->active_job_count = 1;
	mutex_init(&session->lock);
	INIT_LIST_HEAD(&session->imports);
	INIT_LIST_HEAD(&session->active_jobs);
	init_waitqueue_head(&session->wait);
	refcount_set(&session->refs, 2);

	hw.dev = dev;
	refcount_set(&hw.refs, 1);
	init_completion(&hw.released);
	spin_lock_init(&hw.lock);
	mutex_init(&hw.run_lock);
	INIT_LIST_HEAD(&hw.rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&hw.rkvdec_link_jobs);
	INIT_DELAYED_WORK(&hw.timeout_work, rk_mpp_hw_timeout_work);

	import->fd = 9;
	refcount_set(&import->refs, 3);
	INIT_LIST_HEAD(&import->link);
	list_add_tail(&import->link, &session->imports);

	active->session = session;
	active->hw = &hw;
	active->imports[0] = import;
	active->import_count = 1;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session->active_jobs);
	hw.active_job = active;

	KUNIT_EXPECT_EQ(test,
			rk_mpp_process_request(session, &req, &batch),
			0);
	KUNIT_EXPECT_TRUE(test, list_empty(&session->imports));
	KUNIT_EXPECT_TRUE(test, list_empty(&session->active_jobs));
	KUNIT_EXPECT_EQ(test, session->active_job_count, 0U);
	KUNIT_EXPECT_PTR_EQ(test, hw.active_job, NULL);
	KUNIT_EXPECT_TRUE(test, completion_done(&hw.released));
	KUNIT_EXPECT_EQ(test, refcount_read(&session->refs), 1);
	KUNIT_EXPECT_EQ(test, refcount_read(&import->refs), 1);

	rk_mpp_import_put(import);
	pm_runtime_disable(dev);
	put_device(dev);
}

static void rk_mpp_file_release_public_cleanup_kunit(struct kunit *test)
{
	struct rk_mpp_service srv = {};
	struct rk_mpp_session *session;
	struct rk_mpp_import *import;
	struct rk_mpp_job *active;
	struct file file = {};

	mutex_init(&srv.sched_lock);
	INIT_LIST_HEAD(&srv.queued_jobs);

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, session);
	session->srv = &srv;
	session->active_job_count = 1;
	mutex_init(&session->lock);
	INIT_LIST_HEAD(&session->imports);
	INIT_LIST_HEAD(&session->active_jobs);
	init_waitqueue_head(&session->wait);
	refcount_set(&session->refs, 1);
	file.private_data = session;

	import = kzalloc(sizeof(*import), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, import);
	import->fd = 8;
	refcount_set(&import->refs, 2);
	INIT_LIST_HEAD(&import->link);
	list_add_tail(&import->link, &session->imports);

	active = kunit_kzalloc(test, sizeof(*active), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, active);
	active->session = session;
	active->state = RK_MPP_JOB_ACTIVE;
	active->result = -EINPROGRESS;
	refcount_set(&active->refs, 2);
	INIT_LIST_HEAD(&active->link);
	INIT_LIST_HEAD(&active->session_link);
	INIT_LIST_HEAD(&active->sched_link);
	INIT_LIST_HEAD(&active->rkvdec_ccu_node);
	INIT_LIST_HEAD(&active->rkvdec_link_node);
	list_add_tail(&active->session_link, &session->active_jobs);

	KUNIT_EXPECT_EQ(test, rk_mpp_release(NULL, &file), 0);
	KUNIT_EXPECT_PTR_EQ(test, file.private_data, NULL);
	KUNIT_EXPECT_EQ(test, refcount_read(&import->refs), 1);
	KUNIT_EXPECT_TRUE(test, list_empty(&active->session_link));
	KUNIT_EXPECT_EQ(test, active->result, -ECANCELED);
	KUNIT_EXPECT_EQ(test, active->state,
			(enum rk_mpp_job_state)RK_MPP_JOB_DONE);
	KUNIT_EXPECT_EQ(test, refcount_read(&active->refs), 1);

	rk_mpp_import_put(import);
}

static struct kunit_case rk_mpp_rewrite_test_cases[] = {
	KUNIT_CASE(rk_mpp_check_cmd_v1_kunit),
	KUNIT_CASE(rk_mpp_get_cmd_butt_kunit),
	KUNIT_CASE(rk_mpp_support_cmds_kunit),
	KUNIT_CASE(rk_mpp_abi_layout_kunit),
	KUNIT_CASE(rk_mpp_msg_v1_to_request_kunit),
	KUNIT_CASE(rk_mpp_cmd_copies_payload_kunit),
	KUNIT_CASE(rk_mpp_set_err_ref_hack_kunit),
	KUNIT_CASE(rk_mpp_store_codec_info_kunit),
	KUNIT_CASE(rk_mpp_init_trans_table_kunit),
	KUNIT_CASE(rk_mpp_reg_offsets_kunit),
	KUNIT_CASE(rk_mpp_request_check_reg_span_kunit),
	KUNIT_CASE(rk_mpp_request_check_rkvdec_perf_span_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_timeout_threshold_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_mode_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_soft_ccu_program_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_link_info_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_vp9_translate_validate_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_link_irq_decode_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_fill_link_table_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_link_table_ownership_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_link_table_ccu_ref_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_running_list_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_job_done_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_power_transfer_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_release_power_transfer_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_relink_unfinished_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_collect_unfinished_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_descriptor_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_ccu_descriptor_core_mask_kunit),
	KUNIT_CASE(rk_mpp_rkvdec2_fixed_rcb_link_kunit),
	KUNIT_CASE(rk_mpp_hw_take_active_if_kunit),
	KUNIT_CASE(rk_mpp_hw_prepare_active_retry_kunit),
	KUNIT_CASE(rk_mpp_hw_abort_ccu_dependents_kunit),
	KUNIT_CASE(rk_mpp_core_counter_kunit),
	KUNIT_CASE(rk_mpp_hw_select_rotation_kunit),
	KUNIT_CASE(rk_mpp_iommu_fault_match_kunit),
	KUNIT_CASE(rk_mpp_poll_irq_check_size_kunit),
	KUNIT_CASE(rk_mpp_rkvenc_slice_mode_kunit),
	KUNIT_CASE(rk_mpp_rkvenc_slice_fifo_kunit),
	KUNIT_CASE(rk_mpp_rkvenc2_dchs_remap_kunit),
	KUNIT_CASE(rk_mpp_rkvenc2_dchs_independent_cores_kunit),
	KUNIT_CASE(rk_mpp_rcb_invalid_index_kunit),
	KUNIT_CASE(rk_mpp_rkvdec_rcb_width_gate_kunit),
	KUNIT_CASE(rk_mpp_switch_session_status_kunit),
	KUNIT_CASE(rk_mpp_batch_session_switch_split_kunit),
	KUNIT_CASE(rk_mpp_release_fd_all_devices_kunit),
	KUNIT_CASE(rk_mpp_session_poll_nonblock_pending_kunit),
	KUNIT_CASE(rk_mpp_session_abort_jobs_kunit),
	KUNIT_CASE(rk_mpp_session_abort_hw_active_kunit),
	KUNIT_CASE(rk_mpp_reset_session_public_cleanup_kunit),
	KUNIT_CASE(rk_mpp_reset_session_hw_active_import_kunit),
	KUNIT_CASE(rk_mpp_file_release_public_cleanup_kunit),
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
		u32 max_words = RK_MPP_MAX_REG_IMAGE_BYTES / sizeof(*image->regs);
		u32 next_offset;

		if (desc->index >= max_words)
			continue;
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
		if (busy & RK_MPP_RKVENC_DCHS_RXE)
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

	if (rxid_map < 0)
		rxe_map = false;

	patched = rk_mpp_rkvenc_dchs_set_txid(low, txid_map);
	patched = rk_mpp_rkvenc_dchs_set_rxid(patched,
					       rxid_map >= 0 ? rxid_map : 0);
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
	rk_mpp_count_scheduled_core(job);
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
	rk_mpp_rkvdec2_release_link_table(job);
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
	INIT_LIST_HEAD(&job->rkvdec_ccu_node);
	INIT_LIST_HEAD(&job->rkvdec_link_node);
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

	rk_mpp_job_note_hw_done(job);
	rk_mpp_job_record_hw_stats(job);
	mutex_lock(&session->lock);
	job->result = result;
	job->state = RK_MPP_JOB_DONE;
	mutex_unlock(&session->lock);
	rk_mpp_rkvenc2_dchs_release(job);
	rk_mpp_rkvdec2_release_link_table(job);
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

		rk_mpp_count_dispatched_core(job);
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

static bool
rk_mpp_rkvdec2_link_regs_ready(struct rk_mpp_hw *hw,
			       const struct rk_mpp_rkvdec2_link_info *info)
{
	return rk_mpp_hw_reg_range_valid(hw, RK_MPP_RKVDEC_LINK_REGION,
					 info->en_base, sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(hw, RK_MPP_RKVDEC_LINK_REGION,
					 info->irq_base, sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(hw, RK_MPP_RKVDEC_LINK_REGION,
					 info->status_base, sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(hw, RK_MPP_RKVDEC_LINK_REGION,
					 info->ip_time_base, sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(hw, RK_MPP_RKVDEC_LINK_REGION,
					 info->ip_en_base, sizeof(u32));
}

static bool rk_mpp_rkvdec2_ccu_regs_ready(struct rk_mpp_hw *ccu)
{
	return ccu &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CTRL_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CFG_ADDR_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_LINK_MODE_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CFG_DONE_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_WORK_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CORE_WORK_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CORE_STA_BASE,
					 sizeof(u32));
}

static bool rk_mpp_rkvdec2_soft_ccu_regs_ready(struct rk_mpp_hw *ccu)
{
	return ccu &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_WORK_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_WORK_MODE_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CORE_WORK_BASE,
					 sizeof(u32)) &&
	       rk_mpp_hw_reg_range_valid(ccu, 0, RK_MPP_RKVDEC_CCU_CORE_STA_BASE,
					 sizeof(u32));
}

static int rk_mpp_rkvdec2_program_soft_ccu(struct rk_mpp_job *job)
{
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *hw = job->hw;
	struct rk_mpp_hw *ccu = job->rkvdec_ccu;
	void __iomem *ccu_regs;
	u32 irq_val;

	if (!hw || !ccu || !rk_mpp_rkvdec2_soft_ccu_regs_ready(ccu))
		return -EOPNOTSUPP;
	if (!hw->core_mask)
		return -EINVAL;

	if (rk_mpp_rkvdec2_link_regs_ready(hw, link_info)) {
		void __iomem *link = hw->regs[RK_MPP_RKVDEC_LINK_REGION];

		irq_val = readl_relaxed(link + link_info->irq_base);
		irq_val |= RK_MPP_RKVDEC_LINK_CORE_WORK_MODE |
			   RK_MPP_RKVDEC_LINK_CCU_WORK_MODE;
		writel_relaxed(irq_val, link + link_info->irq_base);
	}

	ccu_regs = ccu->regs[0];
	writel_relaxed(RK_MPP_RKVDEC_CCU_WORK_EN,
		       ccu_regs + RK_MPP_RKVDEC_CCU_WORK_BASE);
	writel_relaxed(RK_MPP_RKVDEC_CCU_WORK_MODE,
		       ccu_regs + RK_MPP_RKVDEC_CCU_WORK_MODE_BASE);
	writel_relaxed(hw->core_mask,
		       ccu_regs + RK_MPP_RKVDEC_CCU_CORE_WORK_BASE);
	writel_relaxed(hw->core_mask,
		       ccu_regs + RK_MPP_RKVDEC_CCU_CORE_STA_BASE);

	return 0;
}

static int rk_mpp_rkvdec2_prepare_soft_ccu(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	struct rk_mpp_hw *ccu;
	int ret;

	if (!rk_mpp_rkvdec2_soft_ccu_enabled(hw))
		return 0;

	if (!job->rkvdec_ccu) {
		job->rkvdec_ccu =
			rk_mpp_hw_get_ccu_for_core(job->session->srv, hw);
		if (!job->rkvdec_ccu)
			return -ENODEV;
	}
	ccu = job->rkvdec_ccu;
	if (!rk_mpp_rkvdec2_soft_ccu_regs_ready(ccu))
		return -EOPNOTSUPP;

	if (!job->rkvdec_ccu_powered) {
		ret = rk_mpp_hw_power_on(ccu);
		if (ret)
			return ret;
		job->rkvdec_ccu_powered = true;
	}
	if (!READ_ONCE(ccu->online))
		return -ENODEV;

	mutex_lock(&ccu->run_lock);
	ret = rk_mpp_rkvdec2_program_soft_ccu(job);
	mutex_unlock(&ccu->run_lock);

	return ret;
}

static u32 rk_mpp_rkvdec2_ccu_core_mask(struct rk_mpp_service *srv,
					struct rk_mpp_hw *ccu)
{
	struct rk_mpp_hw *hw;
	u32 mask = 0;

	if (!srv || !ccu || !ccu->dev)
		return 0;

	mutex_lock(&srv->hw_lock);
	list_for_each_entry(hw, &srv->hw_list, link) {
		if (hw->online && hw->ccu_node == ccu->dev->of_node)
			mask |= hw->core_mask;
	}
	mutex_unlock(&srv->hw_lock);

	return mask;
}

static int rk_mpp_hw_read_clk_rates(struct rk_mpp_hw *hw)
{
	struct device *dev = hw->dev;
	int count;
	int ret;

	if (!hw->num_clks)
		return 0;

	count = device_property_count_u32(dev, "rockchip,normal-rates");
	if (count == -EINVAL || count == -ENODATA)
		return 0;
	if (count < 0)
		return count;
	if (!count)
		return 0;

	if (count != hw->num_clks) {
		dev_warn(dev, "ignoring %d normal clock rates for %d clocks\n",
			 count, hw->num_clks);
		return 0;
	}

	hw->normal_rates = devm_kcalloc(dev, count, sizeof(*hw->normal_rates),
					GFP_KERNEL);
	if (!hw->normal_rates)
		return -ENOMEM;

	ret = device_property_read_u32_array(dev, "rockchip,normal-rates",
					     hw->normal_rates, count);
	if (ret)
		return ret;

	return 0;
}

static void rk_mpp_hw_apply_clk_rates(struct rk_mpp_hw *hw)
{
	int i;

	for (i = 0; hw->normal_rates && i < hw->num_clks; i++) {
		u32 rate = hw->normal_rates[i];
		int ret;

		if (!rate)
			continue;

		ret = clk_set_rate(hw->clks[i].clk, rate);
		if (ret)
			dev_warn_ratelimited(hw->dev,
					     "failed to set %s to %u Hz: %d\n",
					     hw->clks[i].id ?: "clock", rate, ret);
	}
}

static int rk_mpp_hw_power_on(struct rk_mpp_hw *hw)
{
	int ret;

	ret = pm_runtime_resume_and_get(hw->dev);
	if (ret < 0)
		return ret;

	rk_mpp_hw_apply_clk_rates(hw);

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
		atomic_set(&hw->iommu_fault_pending, 0);
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

static bool rk_mpp_hw_take_active_if(struct rk_mpp_hw *hw,
				     struct rk_mpp_job *match,
				     u32 *irq_status)
{
	unsigned long flags;
	bool taken = false;

	spin_lock_irqsave(&hw->lock, flags);
	if (hw->active_job == match) {
		if (irq_status)
			*irq_status = hw->irq_status;
		hw->active_job = NULL;
		hw->irq_status = 0;
		taken = true;
	}
	spin_unlock_irqrestore(&hw->lock, flags);

	return taken;
}

static bool rk_mpp_hw_prepare_active_retry(struct rk_mpp_hw *hw,
					   struct rk_mpp_job *match)
{
	unsigned long flags;
	bool active = false;

	spin_lock_irqsave(&hw->lock, flags);
	if (hw->active_job == match) {
		hw->irq_status = 0;
		atomic_set(&hw->iommu_fault_pending, 0);
		active = true;
	}
	spin_unlock_irqrestore(&hw->lock, flags);

	return active;
}

static void rk_mpp_hw_refresh_iommu(struct rk_mpp_hw *hw,
				    struct rk_mpp_job *job)
{
	struct rk_mpp_service *srv = job && job->session ?
				     job->session->srv : NULL;

	if (!hw->iommu_domain)
		return;

	iommu_flush_iotlb_all(hw->iommu_domain);
	if (srv)
		atomic_inc(&srv->iommu_refresh_count);
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

static int rk_mpp_rkvdec2_start_ccu_job(struct rk_mpp_job *job)
{
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *hw = job->hw;
	struct rk_mpp_hw *ccu = job->rkvdec_ccu;
	void __iomem *link;
	void __iomem *ccu_regs;
	u32 irq_val;
	u32 ccu_en;
	u32 i;
	bool add_mode;
	bool ccu_powered_now = false;
	bool cores_powered_now = false;
	int ret;

	if (!job->rkvdec_ccu_desc_valid || !job->rkvdec_link_active)
		return -EOPNOTSUPP;
	if (!rk_mpp_rkvdec2_link_regs_ready(hw, link_info) ||
	    !rk_mpp_rkvdec2_ccu_regs_ready(ccu))
		return -EOPNOTSUPP;
	if (!job->rkvdec_ccu_core_work)
		return -EINVAL;

	if (!job->rkvdec_ccu_powered) {
		ret = rk_mpp_hw_power_on(ccu);
		if (ret)
			return ret;
		job->rkvdec_ccu_powered = true;
		ccu_powered_now = true;
	}

	if (!READ_ONCE(ccu->online) || !READ_ONCE(hw->online) ||
	    READ_ONCE(job->canceled)) {
		ret = READ_ONCE(job->canceled) ? -ECANCELED : -ENODEV;
		goto err_power_off;
	}

	link = hw->regs[RK_MPP_RKVDEC_LINK_REGION];
	ccu_regs = ccu->regs[0];
	mutex_lock(&ccu->run_lock);
	writel_relaxed(link_info->irq_mask, link + link_info->irq_base);
	writel_relaxed(link_info->status_mask, link + link_info->status_base);
	irq_val = readl_relaxed(link + link_info->irq_base);
	irq_val |= job->rkvdec_link_irq_mode;
	writel_relaxed(irq_val, link + link_info->irq_base);

	ccu_en = readl_relaxed(ccu_regs + RK_MPP_RKVDEC_CCU_WORK_BASE);
	add_mode = ccu_en && rk_mpp_rkvdec2_ccu_has_jobs(ccu);
	if (ccu_en && !add_mode) {
		ret = -EBUSY;
		goto err_unlock_ccu;
	}

	if (!add_mode) {
		if (!job->rkvdec_ccu_powered_core_count) {
			ret = rk_mpp_rkvdec2_power_on_ccu_cores(job);
			if (ret)
				goto err_unlock_ccu;
			cores_powered_now = true;
		}

		rk_mpp_rkvdec2_prepare_core_for_ccu(hw);
		for (i = 0; i < job->rkvdec_ccu_powered_core_count; i++)
			rk_mpp_rkvdec2_prepare_core_for_ccu(
				job->rkvdec_ccu_powered_cores[i]);

		writel_relaxed(job->rkvdec_ccu_core_work,
			       ccu_regs + RK_MPP_RKVDEC_CCU_CORE_WORK_BASE);
		writel_relaxed(job->rkvdec_ccu_ctrl,
			       ccu_regs + RK_MPP_RKVDEC_CCU_CTRL_BASE);
		writel_relaxed(job->rkvdec_ccu_cfg_addr,
			       ccu_regs + RK_MPP_RKVDEC_CCU_CFG_ADDR_BASE);
		writel_relaxed(job->rkvdec_ccu_work,
			       ccu_regs + RK_MPP_RKVDEC_CCU_WORK_BASE);
	}
	writel_relaxed(rk_mpp_rkvdec2_ccu_link_mode(job, add_mode),
		       ccu_regs + RK_MPP_RKVDEC_CCU_LINK_MODE_BASE);

	rk_mpp_rkvdec2_ccu_job_add(job);
	rk_mpp_hw_schedule_timeout(hw);
	/* Ensure CCU descriptor writes land before CFG_DONE starts the job. */
	wmb();
	writel(job->rkvdec_ccu_cfg_done,
	       ccu_regs + RK_MPP_RKVDEC_CCU_CFG_DONE_BASE);
	job->rkvdec_ccu_started = true;
	rk_mpp_count_started_core(job);
	mutex_unlock(&ccu->run_lock);

	return 0;

err_unlock_ccu:
	mutex_unlock(&ccu->run_lock);
err_power_off:
	if (cores_powered_now)
		rk_mpp_rkvdec2_power_off_ccu_cores(job);
	if (ccu_powered_now) {
		rk_mpp_hw_power_off(ccu);
		job->rkvdec_ccu_powered = false;
	}
	return ret;
}

static int rk_mpp_rkvdec2_restart_ccu_job(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	int ret;

	if (!hw || !job->rkvdec_ccu_started || !job->rkvdec_ccu)
		return -EINVAL;
	if (!mutex_trylock(&hw->run_lock))
		return -EBUSY;
	if (!rk_mpp_hw_prepare_active_retry(hw, job)) {
		ret = -ENOENT;
		goto out_unlock;
	}

	cancel_delayed_work(&hw->timeout_work);
	ret = rk_mpp_rkvdec2_start_ccu_job(job);

out_unlock:
	mutex_unlock(&hw->run_lock);
	return ret;
}

static int rk_mpp_rkvdec2_prepare_ccu_retry_job(struct rk_mpp_job *job)
{
	struct rk_mpp_hw *hw = job->hw;
	int ret = 0;

	if (!hw || !job->rkvdec_ccu_started || !job->rkvdec_ccu)
		return -EINVAL;
	if (!mutex_trylock(&hw->run_lock))
		return -EBUSY;
	if (!rk_mpp_hw_prepare_active_retry(hw, job)) {
		ret = -ENOENT;
		goto out_unlock;
	}

	cancel_delayed_work(&hw->timeout_work);
	rk_mpp_hw_reset_active(hw);
	rk_mpp_hw_refresh_iommu(hw, job);

out_unlock:
	mutex_unlock(&hw->run_lock);
	return ret;
}

static int rk_mpp_rkvdec2_restart_ccu_unfinished_jobs(struct rk_mpp_hw *ccu)
{
	struct rk_mpp_job **jobs;
	u32 count;
	u32 i;
	int ret;

	ret = rk_mpp_rkvdec2_collect_unfinished_ccu_jobs(ccu, &jobs, &count);
	if (ret || !count)
		return ret;

	for (i = 0; i < count; i++) {
		ret = rk_mpp_rkvdec2_prepare_ccu_retry_job(jobs[i]);
		if (ret)
			goto out_put_jobs;
	}

	for (i = 0; i < count; i++) {
		ret = rk_mpp_rkvdec2_restart_ccu_job(jobs[i]);
		if (ret)
			break;
	}

out_put_jobs:
	for (i = 0; i < count; i++)
		rk_mpp_job_put(jobs[i]);
	kfree(jobs);

	return ret ?: (int)count;
}

static void rk_mpp_rkvdec2_drain_ccu_done_jobs(struct rk_mpp_hw *ccu)
{
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;

	while ((job = rk_mpp_rkvdec2_ccu_first_done_job(ccu))) {
		struct rk_mpp_hw *hw = job->hw;
		bool ccu_error;
		u32 irq_status = 0;
		int ret;

		if (!hw || !job->rkvdec_ccu_started ||
		    !mutex_trylock(&hw->run_lock)) {
			rk_mpp_job_put(job);
			break;
		}

		if (!rk_mpp_hw_take_active_if(hw, job, &irq_status)) {
			mutex_unlock(&hw->run_lock);
			rk_mpp_job_put(job);
			break;
		}

		cancel_delayed_work(&hw->timeout_work);
		ret = rk_mpp_rkvdec2_read_ccu_link_table(job, link_info,
							 irq_status);
		ccu_error = !ret &&
			rk_mpp_rkvdec2_ccu_job_error(job, link_info);
		if (ccu_error)
			rk_mpp_rkvdec2_force_stop_ccu(ccu);

		if (ccu_error)
			rk_mpp_hw_reset_active(hw);
		rk_mpp_hw_power_off(hw);
		rk_mpp_job_complete(job, ret);
		mutex_unlock(&hw->run_lock);
		rk_mpp_job_put(job);
		rk_mpp_job_put(job);
		if (ccu_error) {
			rk_mpp_hw_abort_ccu_active_dependents(ccu, hw, -EIO);
			break;
		}
	}
}

static void rk_mpp_hw_timeout_work(struct work_struct *work)
{
	struct rk_mpp_hw *hw =
		container_of(to_delayed_work(work), struct rk_mpp_hw,
			     timeout_work);
	struct rk_mpp_job *job;
	struct rk_mpp_hw *ccu = NULL;
	bool hard_ccu_recovery;
	bool ccu_done = false;
	bool ccu_error = false;
	bool iommu_fault;
	int recovery_result;
	int result;

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, NULL);
	if (!job) {
		atomic_set(&hw->iommu_fault_pending, 0);
		mutex_unlock(&hw->run_lock);
		return;
	}
	iommu_fault = atomic_xchg(&hw->iommu_fault_pending, 0);
	hard_ccu_recovery = job->rkvdec_ccu_started && job->rkvdec_ccu;
	if (hard_ccu_recovery) {
		ccu = job->rkvdec_ccu;
		rk_mpp_hw_get(ccu);
		rk_mpp_rkvdec2_force_stop_ccu(ccu);
		ccu_done =
			rk_mpp_rkvdec2_ccu_job_done(job,
						    &rk_mpp_rkvdec2_vdpu383_link_info);
		if (ccu_done) {
			result =
				rk_mpp_rkvdec2_read_ccu_link_table(job,
					&rk_mpp_rkvdec2_vdpu383_link_info, 0);
			ccu_error = !result &&
				rk_mpp_rkvdec2_ccu_job_error(job,
					&rk_mpp_rkvdec2_vdpu383_link_info);
		}
	}

	if (iommu_fault) {
		result = -EIO;
		recovery_result = result;
		dev_err(hw->dev, "session client %u job %u failed on IOMMU fault\n",
			job->session->client_type, job->id);
	} else if (ccu_done) {
		recovery_result = ccu_error ? -EIO : -ETIMEDOUT;
		if (result)
			dev_err(hw->dev, "session client %u job %u hard-CCU readback failed: %d\n",
				job->session->client_type, job->id, result);
	} else {
		result = -ETIMEDOUT;
		recovery_result = result;
		atomic_inc(&job->session->srv->timeout_count);
		dev_err(hw->dev, "session client %u job %u timed out\n",
			job->session->client_type, job->id);
	}

	rk_mpp_hw_reset_active(hw);
	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, result);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);
	if (hard_ccu_recovery) {
		int restart_ret = -EIO;

		rk_mpp_rkvdec2_drain_ccu_done_jobs(ccu);
		if (!iommu_fault) {
			rk_mpp_rkvdec2_ccu_prepare_resend_chain(ccu);
			restart_ret =
				rk_mpp_rkvdec2_restart_ccu_unfinished_jobs(ccu);
		}
		if (restart_ret < 0) {
			rk_mpp_rkvdec2_force_stop_ccu(ccu);
			rk_mpp_hw_abort_ccu_active_dependents(ccu, hw,
							      recovery_result);
		}
	}
	rk_mpp_hw_put(ccu);
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

static void rk_mpp_hw_abort_active_nowait(struct rk_mpp_hw *hw, int result)
{
	struct rk_mpp_job *job;

	if (!mutex_trylock(&hw->run_lock))
		return;

	cancel_delayed_work(&hw->timeout_work);
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

static void rk_mpp_rkvdec2_force_stop_ccu(struct rk_mpp_hw *ccu)
{
	if (!ccu)
		return;

	mutex_lock(&ccu->run_lock);
	if (rk_mpp_rkvdec2_ccu_regs_ready(ccu))
		writel_relaxed(0,
			       ccu->regs[0] +
			       RK_MPP_RKVDEC_CCU_WORK_BASE);
	rk_mpp_hw_reset_active(ccu);
	mutex_unlock(&ccu->run_lock);
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

static void
rk_mpp_hw_abort_ccu_active_dependents(struct rk_mpp_hw *ccu,
				      struct rk_mpp_hw *skip, int result)
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
		if (deps[i] != skip)
			rk_mpp_hw_abort_active_nowait(deps[i], result);
		rk_mpp_hw_put(deps[i]);
	}

	kfree(deps);
}

static struct rk_mpp_hw *
rk_mpp_iommu_find_fault_hw(struct list_head *fault_hws,
			   struct iommu_domain *domain,
			   struct device *iommu_dev)
{
	struct rk_mpp_hw *fallback = NULL;
	struct rk_mpp_hw *match = NULL;
	struct rk_mpp_hw *hw;

	list_for_each_entry(hw, fault_hws, fault_link) {
		if (hw->iommu_domain != domain)
			continue;

		if (!fallback)
			fallback = hw;
		if (iommu_dev && hw->iommu_node == iommu_dev->of_node) {
			match = hw;
			break;
		}
	}

	return match ?: fallback;
}

static int rk_mpp_iommu_fault_handler(struct iommu_domain *domain,
				      struct device *iommu_dev,
				      unsigned long iova, int status,
				      void *arg)
{
	struct rk_mpp_service *srv = arg;
	struct rk_mpp_hw *match = NULL;
	unsigned long flags;

	atomic_inc(&srv->iommu_fault_count);

	spin_lock_irqsave(&srv->fault_lock, flags);
	match = rk_mpp_iommu_find_fault_hw(&srv->fault_hws, domain,
					   iommu_dev);
	if (match) {
		atomic_set(&match->iommu_fault_pending, 1);
		mod_delayed_work(system_wq, &match->timeout_work, 0);
		dev_err_ratelimited(match->dev,
				    "IOMMU fault iova %#lx status %#x\n",
				    iova, status);
	}
	spin_unlock_irqrestore(&srv->fault_lock, flags);

	if (!match)
		pr_err_ratelimited("unmatched IOMMU fault iova %#lx status %#x\n",
				   iova, status);

	return 0;
}

static void rk_mpp_iommu_register_fault_handler(struct rk_mpp_hw *hw)
{
	struct rk_mpp_service *srv = &rk_mpp_srv;
	unsigned long flags;

	hw->iommu_domain = iommu_get_domain_for_dev(hw->dev);
	if (!hw->iommu_domain)
		return;

	spin_lock_irqsave(&srv->fault_lock, flags);
	list_add_tail(&hw->fault_link, &srv->fault_hws);
	spin_unlock_irqrestore(&srv->fault_lock, flags);

	if (rockchip_iommu_set_fault_handler(hw->dev,
					     rk_mpp_iommu_fault_handler, srv))
		iommu_set_fault_handler(hw->iommu_domain,
					rk_mpp_iommu_fault_handler, srv);
}

static void rk_mpp_iommu_unregister_fault_handler(struct rk_mpp_hw *hw)
{
	struct rk_mpp_service *srv = &rk_mpp_srv;
	struct rk_mpp_hw *other;
	unsigned long flags;
	bool clear = true;

	if (!hw->iommu_domain)
		return;

	spin_lock_irqsave(&srv->fault_lock, flags);
	if (!list_empty(&hw->fault_link))
		list_del_init(&hw->fault_link);
	list_for_each_entry(other, &srv->fault_hws, fault_link) {
		if (other->iommu_domain == hw->iommu_domain) {
			clear = false;
			break;
		}
	}
	spin_unlock_irqrestore(&srv->fault_lock, flags);

	if (clear && rockchip_iommu_set_fault_handler(hw->dev, NULL, NULL))
		iommu_set_fault_handler(hw->iommu_domain, NULL, NULL);
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
	rk_mpp_count_started_core(job);
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
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_hw *hw = job->hw;
	u32 start_value = 0;
	bool link_start;
	bool ccu_start = false;
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

	link_start = rk_mpp_rkvdec2_hard_ccu_enabled(hw) &&
		     hw->rkvdec_link_vaddr &&
		     rk_mpp_rkvdec2_link_regs_ready(hw, link_info);

	if (rk_mpp_hw_reg_range_valid(hw, 0, RK_MPP_RKVDEC_MAX_READS_BASE,
				      sizeof(u32)))
		writel_relaxed(RK_MPP_RKVDEC_MAX_READS,
			       hw->regs[0] + RK_MPP_RKVDEC_MAX_READS_BASE);
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
	if (rk_mpp_rkvdec2_hard_ccu_enabled(hw))
		rk_mpp_rkvdec2_stage_link_table(job);

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

	if (link_start && job->rkvdec_ccu_desc_valid) {
		ret = rk_mpp_rkvdec2_start_ccu_job(job);
		if (ret && ret != -EOPNOTSUPP && ret != -EBUSY)
			goto err_power_off;
		ccu_start = !ret;
	}
	if (ccu_start) {
		mutex_unlock(&hw->run_lock);
		return 0;
	}

	ret = rk_mpp_rkvdec2_prepare_soft_ccu(job);
	if (ret)
		goto err_power_off;

	rk_mpp_hw_schedule_timeout(hw);
	if (link_start) {
		writel_relaxed(link_info->irq_mask,
			       hw->regs[RK_MPP_RKVDEC_LINK_REGION] +
			       link_info->irq_base);
		writel_relaxed(link_info->status_mask,
			       hw->regs[RK_MPP_RKVDEC_LINK_REGION] +
			       link_info->status_base);
		writel_relaxed(RK_MPP_RKVDEC_LINK_IP_TIMEOUT,
			       hw->regs[RK_MPP_RKVDEC_LINK_REGION] +
			       link_info->ip_time_base);
		writel_relaxed(link_info->ip_en_val,
			       hw->regs[RK_MPP_RKVDEC_LINK_REGION] +
			       link_info->ip_en_base);
	}
	wmb();
	if (link_start)
		writel(RK_MPP_RKVDEC_START_EN,
		       hw->regs[RK_MPP_RKVDEC_LINK_REGION] +
		       link_info->en_base);
	else
		writel(start_value | RK_MPP_RKVDEC_START_EN,
		       hw->regs[0] + RK_MPP_RKVDEC_START_BASE);
	rk_mpp_count_started_core(job);
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
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	unsigned long flags;
	u32 status;

	if (rk_mpp_rkvdec2_link_regs_ready(hw, link_info)) {
		void __iomem *link = hw->regs[RK_MPP_RKVDEC_LINK_REGION];
		u32 irq_val = readl_relaxed(link + link_info->irq_base);
		u32 link_status = readl_relaxed(link + link_info->status_base);

		if (rk_mpp_rkvdec2_link_irq_decode(link_info, irq_val,
						   link_status, &status)) {
			writel(link_info->irq_mask, link + link_info->irq_base);
			writel(link_info->status_mask,
			       link + link_info->status_base);
			spin_lock_irqsave(&hw->lock, flags);
			hw->irq_status |= status;
			spin_unlock_irqrestore(&hw->lock, flags);

			return IRQ_WAKE_THREAD;
		}
	}

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
	const struct rk_mpp_rkvdec2_link_info *link_info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct rk_mpp_job *job;
	struct rk_mpp_hw *ccu = NULL;
	bool ccu_error = false;
	u32 irq_status = 0;
	int ret;

	mutex_lock(&hw->run_lock);
	job = rk_mpp_hw_take_active_job(hw, &irq_status);
	if (!job) {
		mutex_unlock(&hw->run_lock);
		return IRQ_HANDLED;
	}
	cancel_delayed_work(&hw->timeout_work);
	if (job->rkvdec_ccu_started && job->rkvdec_ccu) {
		ccu = job->rkvdec_ccu;
		rk_mpp_hw_get(ccu);
	}

	if (job->rkvdec_ccu_started) {
		struct rk_mpp_job *done;

		done = rk_mpp_rkvdec2_ccu_done_active_job(job);
		ret = rk_mpp_rkvdec2_read_ccu_link_table(done ?: job, link_info,
							 irq_status);
		rk_mpp_job_put(done);
		ccu_error = !ret &&
			rk_mpp_rkvdec2_ccu_job_error(job, link_info);
	} else {
		ret = rk_mpp_job_read_regs(job);
		if (!ret)
			ret = rk_mpp_job_store_reg_word(job,
							RK_MPP_RKVDEC_INT_STA_BASE,
							irq_status);
	}
	if (!ret && !job->rkvdec_ccu_started &&
	    job->reg_image.reg_words > RK_MPP_RKVDEC_RLC_WORD) {
		u32 dec_get = readl_relaxed(hw->regs[0] + RK_MPP_RKVDEC_RLC_BASE);
		s32 dec_length = (s32)(dec_get - job->rkvdec_stream_addr);

		job->reg_image.regs[RK_MPP_RKVDEC_RLC_WORD] = dec_length << 10;
	}

	if (ccu_error) {
		rk_mpp_rkvdec2_force_stop_ccu(ccu);
		rk_mpp_hw_reset_active(hw);
	}
	rk_mpp_hw_power_off(hw);
	rk_mpp_job_complete(job, ret);
	mutex_unlock(&hw->run_lock);
	rk_mpp_job_put(job);
	if (ccu_error) {
		int restart_ret;

		rk_mpp_rkvdec2_drain_ccu_done_jobs(ccu);
		rk_mpp_rkvdec2_ccu_prepare_resend_chain(ccu);
		restart_ret = rk_mpp_rkvdec2_restart_ccu_unfinished_jobs(ccu);
		if (restart_ret < 0) {
			rk_mpp_rkvdec2_force_stop_ccu(ccu);
			rk_mpp_hw_abort_ccu_active_dependents(ccu, hw, -EIO);
		}
	} else {
		rk_mpp_rkvdec2_drain_ccu_done_jobs(ccu);
	}
	rk_mpp_hw_put(ccu);

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

		rk_mpp_msg_v1_to_request(&msg, &req);

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

static void
rk_mpp_debugfs_create_core_counts(const char *prefix,
				  atomic_t rkvenc_counters[RK_MPP_CORE_COUNTER_COUNT],
				  atomic_t rkvdec_counters[RK_MPP_CORE_COUNTER_COUNT])
{
	char name[48];

	for (u32 i = 0; i < RK_MPP_CORE_COUNTER_COUNT; i++) {
		snprintf(name, sizeof(name), "%s_rkvenc_core%u_count",
			 prefix, i);
		debugfs_create_atomic_t(name, 0444, rk_mpp_srv.debugfs_root,
					&rkvenc_counters[i]);
		snprintf(name, sizeof(name), "%s_rkvdec_core%u_count",
			 prefix, i);
		debugfs_create_atomic_t(name, 0444, rk_mpp_srv.debugfs_root,
					&rkvdec_counters[i]);
	}
}

static void
rk_mpp_debugfs_create_core_times(const char *prefix,
				 atomic64_t rkvenc_counters[RK_MPP_CORE_COUNTER_COUNT],
				 atomic64_t rkvdec_counters[RK_MPP_CORE_COUNTER_COUNT])
{
	char name[48];

	for (u32 i = 0; i < RK_MPP_CORE_COUNTER_COUNT; i++) {
		snprintf(name, sizeof(name), "%s_rkvenc_core%u",
			 prefix, i);
		rk_mpp_debugfs_create_atomic64(name, &rkvenc_counters[i]);
		snprintf(name, sizeof(name), "%s_rkvdec_core%u",
			 prefix, i);
		rk_mpp_debugfs_create_atomic64(name, &rkvdec_counters[i]);
	}
}

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

static int rk_mpp_hw_read_rcb_info(struct rk_mpp_hw *hw)
{
	u32 vals[RK_MPP_MAX_RCB_ELEMS * 2];
	struct device *dev = hw->dev;
	int count;
	int ret;
	u32 i;

	count = device_property_count_u32(dev, "rockchip,rcb-info");
	if (count <= 0)
		return 0;
	if (count % 2 || count > (int)ARRAY_SIZE(vals)) {
		dev_warn(dev, "ignoring invalid rockchip,rcb-info count %d\n",
			 count);
		return 0;
	}

	ret = device_property_read_u32_array(dev, "rockchip,rcb-info",
					     vals, count);
	if (ret)
		return ret;

	hw->rcb_count = count / 2;
	for (i = 0; i < hw->rcb_count; i++) {
		hw->rcb_descs[i].index = vals[i * 2];
		hw->rcb_descs[i].size = vals[i * 2 + 1];
	}

	return 0;
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
	ret = rk_mpp_hw_read_rcb_info(hw);
	if (ret)
		return ret;

	dev_info(dev, "rcb scratch dma %pad size %zu min_width %u descs %u\n",
		 &hw->rcb_iova, hw->rcb_size, hw->rcb_min_width,
		 hw->rcb_count);

	return 0;
}

static void rk_mpp_hw_read_rkvdec_ccu_mode(struct rk_mpp_hw *hw)
{
	struct device *dev = hw->dev;
	u32 mode = RK_MPP_RKVDEC_CCU_MODE_SOFT;
	u32 requested;

	if (hw->match->type != RK_MPP_DEVICE_RKVDEC || !hw->ccu_node)
		return;

	requested = mode;
	if (of_property_read_u32(hw->ccu_node, "rockchip,ccu-mode", &requested))
		requested = mode;

	mode = rk_mpp_rkvdec2_normalize_ccu_mode(requested);
	if (mode != requested)
		dev_warn(dev, "invalid rkvdec ccu-mode %u; using soft mode\n",
			 requested);
	hw->rkvdec_ccu_mode = mode;
}

static int rk_mpp_hw_alloc_rkvdec_link(struct rk_mpp_hw *hw)
{
	const struct rk_mpp_rkvdec2_link_info *info =
		&rk_mpp_rkvdec2_vdpu383_link_info;
	struct device *dev = hw->dev;
	u32 capacity;
	size_t node_size;
	size_t size;

	if (hw->match->type != RK_MPP_DEVICE_RKVDEC ||
	    !rk_mpp_rkvdec2_hard_ccu_enabled(hw))
		return 0;

	if (!rk_mpp_rkvdec2_link_regs_ready(hw, info)) {
		dev_warn(dev, "rkvdec link MMIO unavailable; hard-CCU tables disabled\n");
		return 0;
	}

	capacity = hw->task_capacity ?: 1;
	node_size = rk_mpp_rkvdec2_link_node_size(info);
	if (check_mul_overflow((size_t)capacity, node_size, &size))
		return -EOVERFLOW;

	hw->rkvdec_link_vaddr = dmam_alloc_coherent(dev, size,
						    &hw->rkvdec_link_iova,
						    GFP_KERNEL);
	if (!hw->rkvdec_link_vaddr)
		return -ENOMEM;
	hw->rkvdec_link_used = devm_bitmap_zalloc(dev, capacity, GFP_KERNEL);
	if (!hw->rkvdec_link_used)
		return -ENOMEM;

	hw->rkvdec_link_size = size;
	hw->rkvdec_link_node_size = node_size;
	hw->rkvdec_link_capacity = capacity;

	dev_info(dev, "rkvdec link table dma %pad nodes %u node_size %zu\n",
		 &hw->rkvdec_link_iova, hw->rkvdec_link_capacity,
		 node_size);

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
	INIT_LIST_HEAD(&hw->fault_link);
	INIT_LIST_HEAD(&hw->rkvdec_ccu_jobs);
	INIT_LIST_HEAD(&hw->rkvdec_link_jobs);
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
	rk_mpp_hw_read_rkvdec_ccu_mode(hw);

	of_property_read_u32(dev->of_node, "rockchip,taskqueue-node",
			     &hw->taskqueue_node);
	of_property_read_u32(dev->of_node, "rockchip,task-capacity",
			     &hw->task_capacity);
	of_property_read_u32(dev->of_node, "rockchip,core-mask",
			     &hw->core_mask);
	hw->iommu_node = of_parse_phandle(dev->of_node, "iommus", 0);
	if (hw->iommu_node) {
		ret = devm_add_action_or_reset(dev, rk_mpp_of_node_put,
					       hw->iommu_node);
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

	ret = rk_mpp_hw_read_clk_rates(hw);
	if (ret)
		return ret;

	hw->resets = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(hw->resets))
		return PTR_ERR(hw->resets);

	ret = rk_mpp_hw_alloc_rcb(hw);
	if (ret)
		return ret;

	ret = rk_mpp_hw_alloc_rkvdec_link(hw);
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

	rk_mpp_iommu_register_fault_handler(hw);

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

	dev_info(dev, "bound %s core %d hw_id %#x irq %d regs %d clocks %d%s%s\n",
		 match->name, hw->core_id, hw->hw_id, hw->irq,
		 hw->num_regs, hw->num_clks,
		 hw->ccu_node ? " ccu-gated" : "",
		 rk_mpp_rkvdec2_soft_ccu_enabled(hw) ? " soft-ccu" :
		 rk_mpp_rkvdec2_hard_ccu_enabled(hw) ? " hard-ccu" : "");

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

	rk_mpp_iommu_unregister_fault_handler(hw);
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
	spin_lock_init(&rk_mpp_srv.fault_lock);
	spin_lock_init(&rk_mpp_srv.rkvenc_dchs_lock);
	INIT_LIST_HEAD(&rk_mpp_srv.hw_list);
	INIT_LIST_HEAD(&rk_mpp_srv.fault_hws);
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
	debugfs_create_atomic_t("scheduled_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.scheduled_job_count);
	rk_mpp_debugfs_create_core_counts("scheduled",
					  rk_mpp_srv.scheduled_rkvenc_core_count,
					  rk_mpp_srv.scheduled_rkvdec_core_count);
	debugfs_create_atomic_t("dispatched_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.dispatched_job_count);
	rk_mpp_debugfs_create_core_counts("dispatched",
					  rk_mpp_srv.dispatched_rkvenc_core_count,
					  rk_mpp_srv.dispatched_rkvdec_core_count);
	debugfs_create_atomic_t("started_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.started_job_count);
	rk_mpp_debugfs_create_core_counts("started",
					  rk_mpp_srv.started_rkvenc_core_count,
					  rk_mpp_srv.started_rkvdec_core_count);
	rk_mpp_debugfs_create_atomic64("hw_total_ns", &rk_mpp_srv.hw_total_ns);
	rk_mpp_debugfs_create_atomic64("hw_max_ns", &rk_mpp_srv.hw_max_ns);
	rk_mpp_debugfs_create_core_times("hw_total_ns",
					 rk_mpp_srv.hw_total_rkvenc_core_ns,
					 rk_mpp_srv.hw_total_rkvdec_core_ns);
	rk_mpp_debugfs_create_core_times("hw_max_ns",
					 rk_mpp_srv.hw_max_rkvenc_core_ns,
					 rk_mpp_srv.hw_max_rkvdec_core_ns);
	debugfs_create_atomic_t("queued_job_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.queued_job_count);
	debugfs_create_atomic_t("timeout_count", 0444, rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.timeout_count);
	debugfs_create_atomic_t("iommu_fault_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.iommu_fault_count);
	debugfs_create_atomic_t("iommu_refresh_count", 0444,
				rk_mpp_srv.debugfs_root,
				&rk_mpp_srv.iommu_refresh_count);

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
