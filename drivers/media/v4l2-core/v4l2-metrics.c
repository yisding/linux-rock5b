// SPDX-License-Identifier: GPL-2.0-only
/*
 * v4l2-metrics.c
 *
 * V4L2 metrics management.
 *
 * Maintain a per-file handle list of metrics about the hardware and handle
 * exposing it in the fdinfo.
 *
 * Copyright (C) 2026 Collabora.
 *
 * Contact: Detlev Casanova <detlev.casanova@collabora.com>
 */

#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/clk.h>
#include <media/v4l2-metrics.h>

static const char * const driver_type_name[] = {
	[V4L2_DRIVER_TYPE_UNKNOWN] = "unknown",
	[V4L2_DRIVER_TYPE_STATELESS_ENCODER] = "stateless-encoder",
	[V4L2_DRIVER_TYPE_STATELESS_DECODER] = "stateless-decoder",
};

void v4l2_metrics_init(struct v4l2_metrics *metrics)
{
	metrics->hw_usage_time = 0;
	metrics->hw_usage_cycles = 0;
	metrics->has_hw_usage_cycles = false;
	metrics->driver_type = V4L2_DRIVER_TYPE_UNKNOWN;
}

void v4l2_metrics_exit(struct v4l2_metrics *metrics)
{
}

void v4l2_metrics_update_hw_time(struct v4l2_metrics *metrics, u64 time_ns)
{
	metrics->hw_usage_time += time_ns;
}
EXPORT_SYMBOL_GPL(v4l2_metrics_update_hw_time);

void v4l2_metrics_update_hw_cycles(struct v4l2_metrics *metrics, u64 cycles)
{
	metrics->hw_usage_cycles += cycles;
	metrics->has_hw_usage_cycles = true;
}
EXPORT_SYMBOL_GPL(v4l2_metrics_update_hw_cycles);

void v4l2_metrics_set_driver_type(struct v4l2_metrics *metrics, enum v4l2_driver_type type)
{
	if (type >= V4L2_DRIVER_TYPE_COUNT)
		return;

	metrics->driver_type = type;
}
EXPORT_SYMBOL_GPL(v4l2_metrics_set_driver_type);

void v4l2_metrics_show(struct v4l2_metrics *metrics, struct seq_file *m, unsigned int core_id)
{
	seq_printf(m, "v4l2-driver-type:\t%s\n", driver_type_name[metrics->driver_type]);
	seq_printf(m, "v4l2-core-usage-time-%u:\t%llu ns\n", core_id, metrics->hw_usage_time);

	if (metrics->has_hw_usage_cycles)
		seq_printf(m, "v4l2-core-usage-cycles-%u:\t%llu\n",
			   core_id, metrics->hw_usage_cycles);
}
EXPORT_SYMBOL_GPL(v4l2_metrics_show);

void v4l2_metrics_show_clock(struct seq_file *m, struct clk *clk, unsigned int core_id)
{
	seq_printf(m, "v4l2-maxfreq-%u:\t%lu Hz\n",
		   core_id, clk_get_rate(clk));
	seq_printf(m, "v4l2-curfreq-%u:\t%lu Hz\n",
		   core_id, clk_get_rate(clk));
}
EXPORT_SYMBOL_GPL(v4l2_metrics_show_clock);
