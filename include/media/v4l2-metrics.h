/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * v4l2-metrics.h
 *
 * V4L2 metrics management.
 *
 * Maintain a per-file handle list of statistics about the hardware and handle
 * exposing it in the fdinfo.
 *
 * Copyright (C) 2026 Collabora.
 *
 * Contact: Detlev Casanova <detlev.casanova@collabora.com>
 */
#ifndef V4L2_METRICS_H
#define V4L2_METRICS_H

#include <linux/types.h>

struct clk;
struct seq_file;

enum v4l2_driver_type {
	V4L2_DRIVER_TYPE_UNKNOWN = 0,
	V4L2_DRIVER_TYPE_STATELESS_ENCODER,
	V4L2_DRIVER_TYPE_STATELESS_DECODER,

	V4L2_DRIVER_TYPE_COUNT,
};

struct v4l2_metrics {
	u64 hw_usage_time;
	u64 hw_usage_cycles;
	bool has_hw_usage_cycles;
	enum v4l2_driver_type driver_type;
};

void v4l2_metrics_init(struct v4l2_metrics *metrics);
void v4l2_metrics_exit(struct v4l2_metrics *metrics);

void v4l2_metrics_update_hw_time(struct v4l2_metrics *metrics, u64 time_ns);
void v4l2_metrics_update_hw_cycles(struct v4l2_metrics *metrics, u64 cycles);
void v4l2_metrics_set_driver_type(struct v4l2_metrics *metrics, enum v4l2_driver_type type);

void v4l2_metrics_show(struct v4l2_metrics *metrics, struct seq_file *m, unsigned int core_id);
void v4l2_metrics_show_clock(struct seq_file *m, struct clk *clk, unsigned int core_id);

#endif /* V4L2_METRICS_H */
