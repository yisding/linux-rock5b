// SPDX-License-Identifier: GPL-2.0
/*
 * mc-shared-graph.c - Media Controller Shared Graph API
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

#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/kref.h>
#include <linux/property.h>

#include <media/media-device.h>

#include <media/mc-shared-graph.h>

static LIST_HEAD(media_device_shared_list);
static DEFINE_MUTEX(media_device_shared_lock);

struct media_device_shared_member {
	struct device *dev;
	struct fwnode_handle *fwnode;
	struct list_head list;
};

struct media_device_shared_link {
	struct media_entity *source;
	u16 source_pad;
	struct media_entity *sink;
	u16 sink_pad;
	u32 flags;
	struct list_head list;
};

// TODO figure out locking for when multiple drivers touch the media graph;
// maybe macros for shared versions?
struct media_device_shared {
	struct media_device mdev;
	struct list_head members;
	struct list_head links;

	struct list_head list;
	struct kref refcount;

	struct device *removed_device;
};

static inline struct media_device_shared *
to_media_device_shared(struct media_device *mdev)
{
	return container_of(mdev, struct media_device_shared, mdev);
}

static void media_device_shared_release(struct kref *kref)
{
	struct media_device_shared *mds =
		container_of(kref, struct media_device_shared, refcount);

	dev_dbg(mds->removed_device, "%s: releasing Media Device\n", __func__);

	mutex_lock(&media_device_shared_lock);

	media_device_unregister(&mds->mdev);
	media_device_cleanup(&mds->mdev);

	list_del(&mds->list);
	mutex_unlock(&media_device_shared_lock);

	kfree(mds);
}

/* Callers should hold media_device_shared_lock when calling this function */
static bool __media_device_shared_find_match(struct media_device_shared *mds,
					     struct fwnode_handle *fwnode)
{
	struct media_device_shared_member *member;
	struct fwnode_handle *ep;
	struct fwnode_handle *remote_ep;
	bool match = false;

	// TODO: parse the device tree endpoints graph instead of finding just the
	// first-level neighbours
	fwnode_graph_for_each_endpoint(fwnode, ep) {
		list_for_each_entry(member, &mds->members, list) {
			remote_ep = fwnode_graph_get_remote_port_parent(ep);
			match = (member->fwnode == remote_ep);
			fwnode_handle_put(remote_ep);

			if (!match)
				continue;

			goto match_complete;
		}
	}

match_complete:
	fwnode_handle_put(ep);
	return match;
}

/* Callers should hold media_device_shared_lock when calling this function */
static struct media_device *__media_device_shared_get(struct device *dev)
{
	struct media_device_shared *mds;
	struct media_device_shared_member *member;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	bool ret;

	dev_dbg(dev, "%s: searching for media device for %pfwf", __func__, fwnode);

	list_for_each_entry(mds, &media_device_shared_list, list) {
		ret = __media_device_shared_find_match(mds, fwnode);
		if (ret)
			break;
	}

	if (!ret)
		return NULL;

	member = kzalloc_obj(*member);
	if (!member)
		return NULL;

	member->dev = dev;
	member->fwnode = fwnode;
	list_add_tail(&member->list, &mds->members);
	kref_get(&mds->refcount);

	dev_dbg(dev, "%s: %pfwf joined media device of %pfwf",
		__func__, fwnode,
		list_first_entry(&mds->members, struct media_device_shared_member, list)->fwnode);

	return &mds->mdev;
}

/* Callers should hold media_device_shared_lock when calling this function */
static struct media_device *__media_device_shared_create(struct device *dev)
{
	struct media_device_shared *mds;
	struct media_device_shared_member *member;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	int ret;

	mds = kzalloc_obj(*mds);
	if (!mds)
		return NULL;

	member = kzalloc_obj(*member);
	if (!member)
		goto err_free_mds;

	media_device_init(&mds->mdev);

	ret = media_device_register(&mds->mdev);
	if (ret)
		goto err_free_member;

	INIT_LIST_HEAD(&mds->members);
	member->dev = dev;
	member->fwnode = fwnode;
	list_add_tail(&member->list, &mds->members);

	INIT_LIST_HEAD(&mds->links);

	kref_init(&mds->refcount);
	list_add_tail(&mds->list, &media_device_shared_list);

	// TODO figure out how to reconcile this with multiple members
	mds->mdev.dev = dev;

	devv_dbg(dev, "%s: Allocated media device with %pfwf at %p\n",
		 __func__, fwnode, &mds->mdev);
	return &mds->mdev;

err_free_member:
	kfree(member);
err_free_mds:
	kfree(mds);
	return NULL;
}

// TODO figure out how to resolve the identifiers (model, driver name, etc);
// atm it's racy and whoever gets it last wins
struct media_device *media_device_shared_join(struct device *dev)
{
	struct media_device *mdev;

	mutex_lock(&media_device_shared_lock);

	mdev = __media_device_shared_get(dev);
	if (!!mdev) {
		dev_dbg(dev, "%s: found media device for %pfwf", __func__, dev_fwnode(dev));
		mutex_unlock(&media_device_shared_lock);
		return mdev;
	}

	mdev = __media_device_shared_create(dev);
	if (!mdev) {
		dev_warn(dev, "%s: failed to create media device for %pfwf", __func__, dev_fwnode(dev));
		mutex_unlock(&media_device_shared_lock);
		return ERR_PTR(-ENOMEM);
	}

	dev_dbg(dev, "%s: created media device for %pfwf", __func__, dev_fwnode(dev));
	mutex_unlock(&media_device_shared_lock);
	return mdev;
}
EXPORT_SYMBOL_GPL(media_device_shared_join);

void media_device_shared_leave(struct media_device *mdev, struct device *dev)
{
	struct media_device_shared *mds = to_media_device_shared(mdev);
	struct media_device_shared_member *member;
	struct media_device_shared_member *member_tmp;
	bool removed = false;

	mutex_lock(&media_device_shared_lock);

	list_for_each_entry_safe(member, member_tmp, &mds->members, list) {
		if (member->dev == dev) {
			list_del(&member->list);
			kfree(member);
			removed = true;
		}
	}

	if (!removed)
		dev_err(dev, "%s: %pfwf trying to leave from graph in which not a member",
			__func__, dev_fwnode(dev));

	mds->removed_device = dev;
	mutex_unlock(&media_device_shared_lock);
	kref_put(&mds->refcount, media_device_shared_release);
}
EXPORT_SYMBOL_GPL(media_device_shared_leave);

int media_device_shared_join_link_source(struct media_device *mdev,
					 struct device *dev,
					 struct media_entity *source,
					 u16 source_pad, u32 flags)
{
	struct media_device_shared *mds = to_media_device_shared(mdev);
	struct media_device_shared_link *link;
	struct media_device_shared_link *link_tmp;
	int ret = 0;

	mutex_lock(&media_device_shared_lock);

	/*
	 * TODO Figure out flags. Should we use greatest common denominator? Or
	 * prioritize sink? Or whoever wins the race? For now we just take the flags
	 * from the sink.
	 *
	 * TODO Figure out how to actually do the matching. For now we just match
	 * whoever comes in first. This works with the simple example we're running
	 * with now (rkcif + one rkisp2) but with setups with multiple copies of
	 * hardware this will cause problems, like with rkcif + two rkisp2 and
	 * imx8-isi + two rkisp1.
	 */
	list_for_each_entry_safe(link, link_tmp, &mds->links, list) {
		if (link->sink) {
			ret = media_create_pad_link(source, source_pad,
						    link->sink, link->sink_pad,
						    link->flags);
			list_del(&link->list);
			kfree(link);
			goto exit_join_link_source;
		}
	}

	link = kzalloc_obj(*link);
	if (!link) {
		ret = -ENOMEM;
		goto exit_join_link_source;
	}

	link->source = source;
	link->source_pad = source_pad;
	link->flags = flags;
	list_add_tail(&link->list, &mds->links);

exit_join_link_source:
	mutex_unlock(&media_device_shared_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(media_device_shared_join_link_source);

// TODO deduplicate from above
int media_device_shared_join_link_sink(struct media_device *mdev,
				       struct device *dev,
				       struct media_entity *sink,
				       u16 sink_pad, u32 flags)
{
	struct media_device_shared *mds = to_media_device_shared(mdev);
	struct media_device_shared_link *link;
	struct media_device_shared_link *link_tmp;
	int ret = 0;

	mutex_lock(&media_device_shared_lock);

	list_for_each_entry_safe(link, link_tmp, &mds->links, list) {
		if (link->source) {
			ret = media_create_pad_link(link->source, link->source_pad,
						    sink, sink_pad,
						    flags);
			list_del(&link->list);
			kfree(link);
			goto exit_join_link_sink;
		}
	}

	link = kzalloc_obj(*link);
	if (!link) {
		ret = -ENOMEM;
		goto exit_join_link_sink;
	}

	link->sink = sink;
	link->sink_pad = sink_pad;
	link->flags = flags;
	list_add_tail(&link->list, &mds->links);

exit_join_link_sink:
	mutex_unlock(&media_device_shared_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(media_device_shared_join_link_sink);
