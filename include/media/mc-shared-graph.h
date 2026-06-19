// SPDX-License-Identifier: GPL-2.0
/*
 * mc-shared-graph.h - Media Controller Shared Graph API
 *
 * Copyright (c) 2026 Paul Elder <paul.elder@ideasonboard.com>
 */

/*
 * This file adds the Media Controller Shared Graph API. This allows drivers
 * to create shared media graphs or join existing media graphs from other
 * drivers, so that they can all be in the same media graph. This allows us to
 * have more complex media graphs chaining more complex hardware together,
 * instead of simple async subdevs.
 */

#include <linux/types.h>

#ifndef _MEDIA_SHARED_GRAPH_H
#define _MEDIA_SHARED_GRAPH_H

struct device;
struct media_device;
struct media_entity;

#if defined(CONFIG_MEDIA_CONTROLLER)
/**
 * media_device_shared_join() - Join or create a new shared media device
 *
 * @dev:		struct &device pointer
 *
 * This is the entrance function for a device to join or create a new shared
 * media device. It searches for an existing shared media device based on the
 * neighbours in the device's device tree ports node. If found, then this
 * functions returns the existing shared media device and joins it. If one is
 * not found then one is created and initialized and returned.
 */
struct media_device *media_device_shared_join(struct device *dev);

/**
 * media_device_shared_leave() - Leave the shared media device.
 *
 * @mdev:		struct &media_device pointer
 * @dev:		struct &device pointer
 *
 * This function makes the device leave the shared media device. When all
 * members have left the media device it will be freed.
 */
void media_device_shared_leave(struct media_device *mdev, struct device *dev);

/**
 * media_device_shared_join_link_source() - Register a link source in the shared media device
 *
 * @mdev: The struct &media_device pointer that is part of a shared media device
 * @dev: struct &device pointer
 * @source: The link source
 * @source_pad: The pad
 * @flags: The flags
 *
 * This function registers with the shared media device the source part of a
 * link. When the shared media device receives the matching sink part of a link
 * via media_device_shared_join_link_sink() then the link will be fully created.
 */
int media_device_shared_join_link_source(struct media_device *mdev,
					 struct device *dev,
					 struct media_entity *source,
					 u16 source_pad, u32 flags);

/**
 * media_device_shared_join_link_sink() - Register a link sink in the shared media device
 *
 * Same as media_device_shared_join_link_source() but for sink instead of
 * source.
 */
int media_device_shared_join_link_sink(struct media_device *mdev,
				       struct device *dev,
				       struct media_entity *sink,
				       u16 sink_pad, u32 flags);
#else
static inline struct media_device *media_device_shared_join(struct device *dev)
{ return NULL; }
static inline void media_device_shared_leave(struct media_device *mdev,
					     struct device *dev) { }
static inline int media_device_shared_join_link_source(struct media_device *mdev,
						       struct device *dev,
						       struct media_entity *source,
						       u16 source_pad, u32 flags) { }
static inline int media_device_shared_join_link_sink(struct media_device *mdev,
						     struct device *dev,
						     struct media_entity *sink,
						     u16 sink_pad, u32 flags) { }
#endif /* CONFIG_MEDIA_CONTROLLER */
#endif /* _MEDIA_DEV_SHARED_GRAPH_H */
