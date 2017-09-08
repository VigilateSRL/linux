/*
 * Crazy dma memcpy syscall implementation
 *
 * Copyright Vigilate S.r.l. 2017
 * Author Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG 1
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/freezer.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/syscalls.h>

struct dma_memcpy_data {
	struct dma_chan		*chan;
	dma_cookie_t		cookie;
	bool			done;
	wait_queue_head_t	*done_queue;
};

static void dma_memcpy_callback(void *arg)
{
	struct dma_memcpy_data *data = arg;

	pr_debug("%s %d\n", __func__, __LINE__);
	data->done = true;
	wake_up_all(data->done_queue);
}

SYSCALL_DEFINE3(dma_memcpy,
		__u32, dst,
		__u32, src,
		__u32, sz)
{
	dma_cap_mask_t mask;
	struct dma_device *dev;
	struct dma_async_tx_descriptor *tx;
	struct dma_memcpy_data data;
	dma_cookie_t cookie;
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(dma_memcpy_wait);
	enum dma_status	dma_status;
	int stat;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	pr_debug("%s, dst = %x, src = %x, sz = %u\n", __func__, dst, src, sz);

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;
	data.done = 0;
	data.done_queue = &dma_memcpy_wait;
	data.chan = dma_request_channel(mask, NULL, NULL);
	if (!data.chan) {
		pr_err("cannot allocate channel\n");
		return -ENOMEM;
	}
	dev = data.chan->device;
	tx = dev->device_prep_dma_memcpy(data.chan, dst, src, sz,
					 DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		pr_err("error in prep_dma_memcpy\n");
		return -EINVAL;
	}
	tx->callback = dma_memcpy_callback;
	tx->callback_param = &data;
	cookie = tx->tx_submit(tx);
	stat = dma_submit_error(cookie);
	if (stat < 0) {
		pr_err("error in dma_submit_error\n");
		return stat;
	}
	pr_debug("%s %d\n", __func__, __LINE__);
	dma_async_issue_pending(data.chan);
	wait_event_freezable_timeout(dma_memcpy_wait, data.done, 1000);
	dma_status = dma_async_is_tx_complete(data.chan, cookie, NULL, NULL);
	if (!data.done) {
		pr_err("!data->done\n");
		return -ENODEV;
	} else if (dma_status != DMA_COMPLETE) {
		pr_err("status != DMA_COMPLETE (%d)\n", dma_status);
		return -ENODEV;
	}
	return sz;
}
