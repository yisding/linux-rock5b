// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip Camera Interface (CIF) Driver
 *
 * Copyright (C) 2025 Michael Riesch <michael.riesch@wolfvision.net>
 * Copyright (C) 2025 Collabora, Ltd.
 */

#include <linux/pm_runtime.h>

#include <media/mc-shared-graph.h>
#include <media/v4l2-common.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "rkcif-common.h"
#include "rkcif-interface.h"

static inline struct rkcif_interface *to_rkcif_interface(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rkcif_interface, sd);
}

static u16 rkcif_interface_get_active_source_pad(struct rkcif_interface *interface)
{
	struct media_entity *entity = &interface->sd.entity;
	struct media_link *link;

	list_for_each_entry(link, &entity->links, list) {
		if (link->source->entity != entity ||
		    (link->source->index != RKCIF_IF_PAD_SRC_DMA &&
		     link->source->index != RKCIF_IF_PAD_SRC_TOISP))
			continue;

		if (link->flags & MEDIA_LNK_FL_ENABLED) {
			dev_dbg(interface->rkcif->dev, "%s: active link is %d\n",
				__func__, link->source->index);
			return link->source->index;
		}
	}

	/* Default to DMA if neither link is active */
	return RKCIF_IF_PAD_SRC_DMA;
}

static u32 rkcif_interface_mipi_dt(u32 fourcc)
{
	switch (fourcc) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return RKCIF_CSI2_DT_RAW8;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return RKCIF_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return RKCIF_CSI2_DT_RAW12;
	default:
		return RKCIF_CSI2_DT_RAW10;
	}
}

static u32 rkcif_interface_mipi_parse_type(u32 fourcc)
{
	switch (fourcc) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return RKCIF_MIPI_PARSE_TYPE_RAW8_RGB888;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return RKCIF_MIPI_PARSE_TYPE_RAW10;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return RKCIF_MIPI_PARSE_TYPE_RAW12;
	default:
		return RKCIF_MIPI_PARSE_TYPE_RAW10;
	}
}

static int rkcif_interface_subdev_link_setup(struct media_entity *entity,
					     const struct media_pad *local_pad,
					     const struct media_pad *remote_pad,
					     u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	struct media_link *link;
	u16 other_source_pad_index; 

	dev_dbg(interface->rkcif->dev, "link setup %s -> %s\n",
		local_pad->entity->name, remote_pad->entity->name);

	/* We only care about links being created on a source pad */
	if (!(flags & MEDIA_LNK_FL_ENABLED) ||
	    !(local_pad->flags & MEDIA_PAD_FL_SOURCE) ||
	    (local_pad->index != RKCIF_IF_PAD_SRC_DMA &&
	     local_pad->index != RKCIF_IF_PAD_SRC_TOISP))
		return 0;

	other_source_pad_index =
		local_pad->index == RKCIF_IF_PAD_SRC_DMA ?
				    RKCIF_IF_PAD_SRC_TOISP :
				    RKCIF_IF_PAD_SRC_DMA;

	list_for_each_entry(link, &entity->links, list) {
		if (link->source->entity != local_pad->entity ||
		    link->source->index != other_source_pad_index)
			continue;

		/*
		 * If we are trying to enable DMA source pad but the TOISP
		 * source pad (and vice versa) has an enabled link then return
		 * error
		 */
		return link->flags & MEDIA_LNK_FL_ENABLED ? -EBUSY : 0;
	}

	return 0;
}

static const struct media_entity_operations rkcif_interface_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.link_setup = rkcif_interface_subdev_link_setup,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
};

static int rkcif_interface_set_fmt(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_format *format)
{
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	const struct rkcif_input_fmt *input;
	struct v4l2_mbus_framefmt *sink, *src;
	struct v4l2_rect *crop;
	u32 other_pad, other_stream;
	int ret;

	/* the format on the source pad always matches the sink pad */
	if (format->pad == RKCIF_IF_PAD_SRC_DMA ||
	    format->pad == RKCIF_IF_PAD_SRC_TOISP)
		return v4l2_subdev_get_fmt(sd, state, format);

	input = rkcif_interface_find_input_fmt(interface, true,
					       format->format.code);
	format->format.code = input->mbus_code;

	sink = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!sink)
		return -EINVAL;

	*sink = format->format;

	/* propagate the format to the source pad */
	src = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!src)
		return -EINVAL;

	*src = *sink;

	ret = v4l2_subdev_routing_find_opposite_end(&state->routing,
						    format->pad, format->stream,
						    &other_pad, &other_stream);
	if (ret)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, other_pad, other_stream);
	if (!crop)
		return -EINVAL;

	/* reset crop */
	crop->left = 0;
	crop->top = 0;
	crop->width = sink->width;
	crop->height = sink->height;

	return 0;
}

static int rkcif_interface_get_sel(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *sink;
	struct v4l2_rect *crop;
	int ret = 0;

	if (sel->pad != RKCIF_IF_PAD_SRC_DMA &&
	    sel->pad != RKCIF_IF_PAD_SRC_TOISP)
		return -EINVAL;

	sink = v4l2_subdev_state_get_opposite_stream_format(state, sel->pad,
							    sel->stream);
	if (!sink)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, sel->pad, sel->stream);
	if (!crop)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = sink->width;
		sel->r.height = sink->height;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = *crop;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int rkcif_interface_set_sel(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_selection *sel)
{
	struct v4l2_mbus_framefmt *sink, *src;
	struct v4l2_rect *crop;

	if ((sel->pad != RKCIF_IF_PAD_SRC_DMA &&
	     sel->pad != RKCIF_IF_PAD_SRC_TOISP) || sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sink = v4l2_subdev_state_get_opposite_stream_format(state, sel->pad,
							    sel->stream);
	if (!sink)
		return -EINVAL;

	src = v4l2_subdev_state_get_format(state, sel->pad, sel->stream);
	if (!src)
		return -EINVAL;

	crop = v4l2_subdev_state_get_crop(state, sel->pad, sel->stream);
	if (!crop)
		return -EINVAL;

	*crop = sel->r;

	src->height = sel->r.height;
	src->width = sel->r.width;

	return 0;
}

static int rkcif_interface_set_routing(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       enum v4l2_subdev_format_whence which,
				       struct v4l2_subdev_krouting *routing)
{
	int ret;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	for (unsigned int i = 0; i < routing->num_routes; i++) {
		const struct v4l2_subdev_route *route = &routing->routes[i];

		if (route->source_stream >= RKCIF_ID_MAX)
			return -EINVAL;
	}

	ret = v4l2_subdev_set_routing(sd, state, routing);

	return ret;
}

static int rkcif_interface_apply_crop(struct rkcif_stream *stream,
				      struct v4l2_subdev_state *state)
{
	struct rkcif_interface *interface = stream->interface;
	struct v4l2_rect *crop;

	crop = v4l2_subdev_state_get_crop(
		state,
		rkcif_interface_get_active_source_pad(interface),
		stream->id);
	if (!crop)
		return -EINVAL;

	if (interface->set_crop)
		interface->set_crop(stream, crop->left, crop->top);

	return 0;
}

static int rkcif_interface_enable_streams(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  u32 pad, u64 streams_mask)
{
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	struct rkcif_stream *stream;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev *remote_sd;
	struct media_pad *remote_pad;
	u32 active_pad = rkcif_interface_get_active_source_pad(interface);
	u64 mask;

	interface->inline_mode = (active_pad == RKCIF_IF_PAD_SRC_TOISP);

	remote_pad =
		media_pad_remote_pad_first(&sd->entity.pads[RKCIF_IF_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	/* DVP has one crop setting for all IDs */
	if (interface->type == RKCIF_IF_DVP) {
		stream = &interface->streams[RKCIF_ID0];
		rkcif_interface_apply_crop(stream, state);
	} else {
		for_each_active_route(&state->routing, route) {
			stream = &interface->streams[route->sink_stream];
			rkcif_interface_apply_crop(stream, state);
		}
	}

	mask = v4l2_subdev_state_xlate_streams(state, RKCIF_IF_PAD_SINK,
					       active_pad, &streams_mask);

	return v4l2_subdev_enable_streams(remote_sd, remote_pad->index, mask);
}

static int rkcif_interface_disable_streams(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   u32 pad, u64 streams_mask)
{
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	struct v4l2_subdev *remote_sd;
	struct media_pad *remote_pad;
	u64 mask;

	remote_pad =
		media_pad_remote_pad_first(&sd->entity.pads[RKCIF_IF_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	mask = v4l2_subdev_state_xlate_streams(
		state, RKCIF_IF_PAD_SINK,
		rkcif_interface_get_active_source_pad(interface), &streams_mask);

	return v4l2_subdev_disable_streams(remote_sd, remote_pad->index, mask);
}

static const struct v4l2_subdev_pad_ops rkcif_interface_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = rkcif_interface_set_fmt,
	.get_selection = rkcif_interface_get_sel,
	.set_selection = rkcif_interface_set_sel,
	.set_routing = rkcif_interface_set_routing,
	.enable_streams = rkcif_interface_enable_streams,
	.disable_streams = rkcif_interface_disable_streams,
};

static int rkcif_interface_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	struct rkcif_device *rkcif = interface->rkcif;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev *remote_sd;
	struct media_pad *remote_pad;
	struct v4l2_rect *crop;
	struct v4l2_mbus_framefmt *mbus;
	int ret;
	u32 active_pad = rkcif_interface_get_active_source_pad(interface);
	u32 val, crop_val, offset_val;

	interface->inline_mode = (active_pad == RKCIF_IF_PAD_SRC_TOISP);

	remote_pad =
		media_pad_remote_pad_first(&sd->entity.pads[RKCIF_IF_PAD_SINK]);
	remote_sd = media_entity_to_v4l2_subdev(remote_pad->entity);

	if (!enable) {
		rkcif_write(rkcif, RKCIF_MIPI2_ID0_CTRL0, 0);
		rkcif_write(rkcif, RKCIF_MIPI2_CTRL, 0);

		ret = v4l2_subdev_disable_streams(remote_sd, remote_pad->index, 1);

		pm_runtime_put(rkcif->dev);
		return ret;
	}

	ret = pm_runtime_resume_and_get(rkcif->dev);
	if (ret < 0) {
		dev_err(rkcif->dev, "failed to get runtime pm, %d\n", ret);
		return ret;
	}

	state = v4l2_subdev_lock_and_get_active_state(sd);
	crop = v4l2_subdev_state_get_crop(state, active_pad);
	mbus = v4l2_subdev_state_get_format(state, active_pad);
	v4l2_subdev_unlock_state(state);

	/*
	 * Enable interrupts:
	 * - toisp0 ch{0,1,2} frame start
	 * - toisp1 ch{0,1,2} frame start
	 * - toisp0 ch{0,1,2} frame end
	 * - toisp1 ch{0,1,2} frame end
	 * - toisp0 fifo overflow
	 * - toisp1 fifo overflow
	 * - axi bus errors
	 */
	rkcif_write(rkcif, RKCIF_GLB_INTEN, 0xFFFC003);

	/*
	 * - Enable toisp ch0
	 * - Select MIPI2 ID0
	 */
	val = 0x01;
	val |= 8 << 3;
	rkcif_write(rkcif, RKCIF_TOISP0_CH_CTRL, val);

	crop_val = RKCIF_XY_COORD(3840, 2160);
	offset_val = RKCIF_XY_COORD(0, 0);
	if (!!crop) {
		crop_val = RKCIF_XY_COORD(crop->width, crop->height);
		offset_val = RKCIF_XY_COORD(crop->left, crop->top);
	}

	rkcif_write(rkcif, RKCIF_TOISP0_CROP_SIZE, crop_val);
	rkcif_write(rkcif, RKCIF_TOISP0_CROP_START, offset_val);

	val = rkcif_interface_mipi_parse_type(mbus ? mbus->code : 0);
	val |= RKCIF_MIPI_DATA_SEL_DT(
			rkcif_interface_mipi_dt(mbus ? mbus->code : 0));
	val |= RKCIF_MIPI_CAP_EN;

	rkcif_write(rkcif, RKCIF_MIPI2_ID0_CTRL0, val);
	rkcif_write(rkcif, RKCIF_MIPI2_ID0_CTRL1, crop_val);
	rkcif_write(rkcif, RKCIF_MIPI2_CTRL, RKCIF_MIPI_CAP_EN);

	return v4l2_subdev_enable_streams(remote_sd, remote_pad->index, 1);
}

static const struct v4l2_subdev_video_ops rkcif_interface_video_ops = {
	.s_stream = rkcif_interface_s_stream,
};

static const struct v4l2_subdev_ops rkcif_interface_ops = {
	.video = &rkcif_interface_video_ops,
	.pad = &rkcif_interface_pad_ops,
};

static int rkcif_interface_init_state(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state)
{
	struct rkcif_interface *interface = to_rkcif_interface(sd);
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = RKCIF_IF_PAD_SINK,
			.sink_stream = 0,
			.source_pad = RKCIF_IF_PAD_SRC_DMA,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.sink_pad = RKCIF_IF_PAD_SINK,
			.sink_stream = 0,
			.source_pad = RKCIF_IF_PAD_SRC_TOISP,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting routing = {
		.len_routes = ARRAY_SIZE(routes),
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};
	const struct v4l2_mbus_framefmt dvp_default_format = {
		.width = 3840,
		.height = 2160,
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_REC709,
		.ycbcr_enc = V4L2_YCBCR_ENC_709,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	const struct v4l2_mbus_framefmt mipi_default_format = {
		.width = 3840,
		.height = 2160,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	const struct v4l2_mbus_framefmt *default_format;
	int ret;

	default_format = (interface->type == RKCIF_IF_DVP) ?
				 &dvp_default_format :
				 &mipi_default_format;

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, &routing,
					       default_format);

	return ret;
}

static const struct v4l2_subdev_internal_ops rkcif_interface_internal_ops = {
	.init_state = rkcif_interface_init_state,
};

static int rkcif_interface_add(struct rkcif_interface *interface)
{
	struct rkcif_device *rkcif = interface->rkcif;
	struct rkcif_remote *remote;
	struct v4l2_async_notifier *ntf = &rkcif->notifier;
	struct v4l2_fwnode_endpoint *vep = &interface->vep;
	struct device *dev = rkcif->dev;
	struct fwnode_handle *ep;
	u32 dvp_clk_delay = 0;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), interface->index,
					     0, 0);
	if (!ep)
		return -ENODEV;

	vep->bus_type = V4L2_MBUS_UNKNOWN;
	ret = v4l2_fwnode_endpoint_parse(ep, vep);
	if (ret)
		goto complete;

	if (interface->type == RKCIF_IF_DVP) {
		if (vep->bus_type != V4L2_MBUS_BT656 &&
		    vep->bus_type != V4L2_MBUS_PARALLEL) {
			ret = dev_err_probe(dev, -EINVAL,
					    "unsupported bus type\n");
			goto complete;
		}

		fwnode_property_read_u32(ep, "rockchip,dvp-clk-delay",
					 &dvp_clk_delay);
		interface->dvp.dvp_clk_delay = dvp_clk_delay;
	}

	remote = v4l2_async_nf_add_fwnode_remote(ntf, ep, struct rkcif_remote);
	if (IS_ERR(remote)) {
		ret = PTR_ERR(remote);
		goto complete;
	}

	remote->interface = interface;
	interface->remote = remote;
	interface->status = RKCIF_IF_ACTIVE;
	ret = 0;

complete:
	fwnode_handle_put(ep);

	return ret;
}

int rkcif_interface_register(struct rkcif_device *rkcif,
			     struct rkcif_interface *interface)
{
	struct media_pad *pads = interface->pads;
	struct v4l2_subdev *sd = &interface->sd;
	int ret;

	interface->rkcif = rkcif;

	v4l2_subdev_init(sd, &rkcif_interface_ops);
	sd->dev = rkcif->dev;
	sd->entity.ops = &rkcif_interface_media_ops;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	sd->internal_ops = &rkcif_interface_internal_ops;
	sd->owner = THIS_MODULE;

	interface->inline_mode = false;

	if (interface->type == RKCIF_IF_DVP)
		snprintf(sd->name, sizeof(sd->name), "rkcif-dvp0");
	else if (interface->type == RKCIF_IF_MIPI)
		snprintf(sd->name, sizeof(sd->name), "rkcif-mipi%d",
			 interface->index - RKCIF_MIPI_BASE);

	pads[RKCIF_IF_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					MEDIA_PAD_FL_MUST_CONNECT;
	pads[RKCIF_IF_PAD_SRC_DMA].flags = MEDIA_PAD_FL_SOURCE;
	pads[RKCIF_IF_PAD_SRC_TOISP].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, RKCIF_IF_PAD_MAX, pads);
	if (ret)
		goto err;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_device_register_subdev(&rkcif->v4l2_dev, sd);
	if (ret) {
		dev_err(sd->dev, "failed to register subdev\n");
		goto err_subdev_cleanup;
	}

	ret = rkcif_interface_add(interface);
	if (ret)
		goto err_subdev_unregister;

	ret = media_device_shared_join_link_source(interface->rkcif->media_dev,
						   interface->rkcif->dev,
						   &interface->sd.entity,
						   RKCIF_IF_PAD_SRC_TOISP,
						   0);
	if (ret)
		goto err_subdev_unregister;


	return 0;

err_subdev_unregister:
	v4l2_device_unregister_subdev(sd);
err_subdev_cleanup:
	v4l2_subdev_cleanup(sd);
err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
err:
	return ret;
}

void rkcif_interface_unregister(struct rkcif_interface *interface)
{
	struct v4l2_subdev *sd = &interface->sd;

	if (interface->status != RKCIF_IF_ACTIVE)
		return;

	v4l2_device_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
}

const struct rkcif_input_fmt *
rkcif_interface_find_input_fmt(struct rkcif_interface *interface, bool ret_def,
			       u32 mbus_code)
{
	const struct rkcif_input_fmt *fmt;

	WARN_ON(interface->in_fmts_num == 0);

	for (unsigned int i = 0; i < interface->in_fmts_num; i++) {
		fmt = &interface->in_fmts[i];
		if (fmt->mbus_code == mbus_code)
			return fmt;
	}
	if (ret_def)
		return &interface->in_fmts[0];
	else
		return NULL;
}
