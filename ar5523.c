/*
 * Copyright (c) 2012 Pontus Fuchs <pontus.fuchs@gmail.com>
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Driver for Atheros AR5005UG/AR5005UX chipsets.
 *
 * Copyright (c) 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2007
 *	Christoph Hellwig <hch@lst.de>
 *
 * Copyright (c) 2006 Sam Leffler, Errno Consulting
 * Copyright (c) 2008-2009 Weongyo Jeong <weongyo@freebsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This driver is based on the uath driver written by Damien Bergamini for
 * OpenBSD, who did black-box analysis of the Windows binary driver to find
 * out how the hardware works.  It contains a lot magic numbers because of
 * that and only has minimal functionality.
 *
 * TODO: implement hw WEP support.
 */
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <net/mac80211.h>

#include "ar5523.h"

/*
 * Various supported device vendors/products.
 * UB51: AR5005UG 802.11b/g, UB52: AR5005UX 802.11a/b/g
 */
enum {
	AR5523_FLAG_PRE_FIRMWARE	= (1 << 0),
	AR5523_FLAG_ABG			= (1 << 1),
};

#define AR5523_FIRMWARE_FILE	"uath-ar5523.bin"

enum {
	AR5523_CMD_TX_PIPE	= 0x01,
	AR5523_DATA_TX_PIPE	= 0x02,
	AR5523_CMD_RX_PIPE	= 0x81,
	AR5523_DATA_RX_PIPE	= 0x82,
};
#define ar5523_cmd_tx_pipe(dev) \
	usb_sndbulkpipe((dev), AR5523_CMD_TX_PIPE)
#define ar5523_data_tx_pipe(dev) \
	usb_sndbulkpipe((dev), AR5523_DATA_TX_PIPE)
#define ar5523_cmd_rx_pipe(dev) \
	usb_rcvbulkpipe((dev), AR5523_CMD_RX_PIPE)
#define ar5523_data_rx_pipe(dev) \
	usb_rcvbulkpipe((dev), AR5523_DATA_RX_PIPE)

enum {
	/* XXX: msec or HZ? */
	AR5523_DATA_TIMEOUT		= 10000,
	AR5523_CMD_TIMEOUT		= 1000,
};

enum {
	AR5523_TX_CMD_COUNT	= 2,

	AR5523_TX_DATA_COUNT	= 16,
	AR5523_TX_DATA_RESTART_COUNT = 8,
	AR5523_RX_DATA_COUNT	= 16,
	AR5523_RX_DATA_REFILL_COUNT = 8,
};

#define AR5523_CMD_ID	1

enum AR5523_flags {
	AR5523_TX_QUEUE_STOPPED
};

struct ar5523_tx_cmd {
	struct ar5523		*ar;
	struct urb		*urb_tx;
	void			*buf_tx;
	void			*odata;
	int			olen;
	int			flags;
	int			res;
	struct completion	done;
};

struct ar5523_rx_cmd {
	struct ar5523		*ar;
	struct urb		*urb;
	void			*buf;
};

struct ar5523_tx_data {
	struct ar5523		*ar;
};

struct ar5523_rx_data {
	struct	list_head	list;
	struct ar5523		*ar;
	struct urb		*urb;
	struct sk_buff		*skb;
};

struct ar5523 {
	struct usb_device	*dev;
	struct ieee80211_hw	*hw;

	unsigned long		flags;
	struct mutex		mutex;
	struct ar5523_tx_cmd	tx_cmd;
	atomic_t		tx_data_queued;

	void			*rx_cmd_buf;
	struct urb		*rx_cmd_urb;

	struct ar5523_rx_data	rx_data[AR5523_RX_DATA_COUNT];
	spinlock_t		rx_data_list_lock;
	struct	list_head	rx_data_free;
	struct	list_head	rx_data_used;
	atomic_t		rx_data_free_cnt;

	u8			serial[16];

	struct timer_list	stat_timer;
	struct completion	ready;
	struct work_struct	stat_work;
	struct work_struct	rx_refill_work;

	int			rxbufsz;

	struct ieee80211_channel channels[14];
	struct ieee80211_rate	rates[12];
	struct ieee80211_supported_band band;
	struct ieee80211_vif	*vif;

};

struct ar5523_wme_settings {
	u8	aifsn;
	u8	logcwmin;
	u8	logcwmax;
	u16	txop;
#define AR5523_TXOP_TO_US(txop)	((txop) << 5)
	u8	acm;
};

/* flags for sending firmware commands */
enum {
	AR5523_CMD_FLAG_ASYNC	= (1 << 0),
	AR5523_CMD_FLAG_READ	= (1 << 1),
	AR5523_CMD_FLAG_MAGIC	= (1 << 2),
};

#define ar5523_dbg(ar, format, arg...) \
	dev_dbg(&(ar)->dev->dev, format, ## arg)
#define ar5523_err(ar, format, arg...) \
	dev_err(&(ar)->dev->dev, format, ## arg)
#define ar5523_info(ar, format, arg...)	\
	dev_info(&(ar)->dev->dev, format, ## arg)

static int ar5523_submit_rx_cmd(struct ar5523 *ar);
/*
 * TX/RX command handling.
 */
static void ar5523_read_reply(struct ar5523 *ar, struct ar5523_cmd_hdr *hdr,
			       struct ar5523_tx_cmd *cmd)
{
	int dlen, olen;
	u32 *rp;

	dlen = hdr->len - sizeof(*hdr);

	if (dlen < 0) {
		WARN_ON(1);
		goto out;
	}

	ar5523_dbg(ar, "Code = %d len = %d\n", hdr->code & 0xff, dlen);

	rp = (u32 *)(hdr+1);
	if (dlen >= sizeof(u32)) {
		olen = be32_to_cpu(rp[0]);
		dlen -= sizeof(u32);
		if (olen == 0) {
			/* convention is 0 =>'s one word */
			olen = sizeof(u32);
		}
	} else
		olen = 0;

	if (cmd->odata) {
		if (cmd->olen < olen) {
			ar5523_err(ar, "olen to small %d < %d\n",
				   cmd->olen, olen);
			cmd->olen = 0;
			cmd->res = -EOVERFLOW;
		} else {
			cmd->olen = olen;
			memcpy(cmd->odata, &rp[1], olen);
			cmd->res = 0;
		}
	}

out:
	complete(&cmd->done);
}

static void ar5523_cmd_rx_cb(struct urb *urb)
{
	struct ar5523 *ar = urb->context;
	struct ar5523_tx_cmd *cmd = &ar->tx_cmd;
	struct ar5523_cmd_hdr *hdr = ar->rx_cmd_buf;
	int dlen;

	/* sync/async unlink faults aren't errors */
	if (urb->status && (urb->status != -ENOENT &&
	    urb->status != -ECONNRESET && urb->status != -ESHUTDOWN)) {
		ar5523_dbg(ar,
			   "nonzero write bulk status received: %d\n",
			   urb->status);
		cmd->res = urb->status;
		complete(&cmd->done);
		return;
	}

	if (urb->status) {
		ar5523_err(ar, "RX USB error %d.\n", urb->status);
		cmd->res = urb->status;
		complete(&cmd->done);
		return;
	}

	if (urb->actual_length < sizeof(struct ar5523_cmd_hdr)) {
		ar5523_err(ar, "RX USB to short.\n");
		cmd->res = -1;
		complete(&cmd->done);
		return;
	}

	ar5523_dbg(ar, "%s code %02x priv %d\n", __func__,
		   be32_to_cpu(hdr->code) & 0xff, hdr->priv);

	hdr->code = be32_to_cpu(hdr->code);
	hdr->len = be32_to_cpu(hdr->len);

	switch (hdr->code & 0xff) {
	default:
		/* reply to a read command */
		if (hdr->priv != AR5523_CMD_ID) {
			ar5523_err(ar, "Unexpected command id\n");
			goto skip;
		}
		ar5523_read_reply(ar, hdr, cmd);
		break;

	case WDCMSG_DEVICE_AVAIL:
		ar5523_dbg(ar, "WDCMSG_DEVICE_AVAIL\n");
		cmd->res = 0;
		cmd->olen = 0;
		complete(&cmd->done);
		break;

	case WDCMSG_TARGET_START:
		/* This command returns a bogus id so it needs special
		   handling */
		dlen = hdr->len - sizeof(*hdr);
		if (dlen != (int)sizeof(u32)) {
			ar5523_err(ar, "Invalid reply to WDCMSG_TARGET_START");
			return;
		}
		memcpy(cmd->odata, hdr+1, sizeof(u32));
		cmd->olen = sizeof(u32);
		cmd->res = 0;
		complete(&cmd->done);
		break;
	}

skip:
	ar5523_submit_rx_cmd(ar);
}

static int ar5523_alloc_rx_cmd(struct ar5523 *ar)
{
	ar->rx_cmd_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ar->rx_cmd_urb)
		return -ENOMEM;

	ar->rx_cmd_buf = usb_alloc_coherent(ar->dev, AR5523_MAX_RXCMDSZ,
					    GFP_KERNEL,
					    &ar->rx_cmd_urb->transfer_dma);
	if (!ar->rx_cmd_buf) {
		usb_free_urb(ar->rx_cmd_urb);
		return -ENOMEM;
	}

	return 0;
}

static void ar5523_cancel_rx_cmd(struct ar5523 *ar)
{
	usb_kill_urb(ar->rx_cmd_urb);
}

static void ar5523_free_rx_cmd(struct ar5523 *ar)
{
	usb_free_coherent(ar->dev, AR5523_MAX_RXCMDSZ,
			  ar->rx_cmd_buf, ar->rx_cmd_urb->transfer_dma);
	usb_free_urb(ar->rx_cmd_urb);
}

static int ar5523_submit_rx_cmd(struct ar5523 *ar)
{
	int error;

	usb_fill_bulk_urb(ar->rx_cmd_urb, ar->dev,
			  ar5523_cmd_rx_pipe(ar->dev), ar->rx_cmd_buf,
			  AR5523_MAX_RXCMDSZ, ar5523_cmd_rx_cb, ar);
	ar->rx_cmd_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	error = usb_submit_urb(ar->rx_cmd_urb, GFP_ATOMIC);
	if (error) {
		ar5523_err(ar, "error %d when submitting rx urb\n",
			       error);
		return error;
	}
	return 0;
}

/*
 * Command submitted cb
 */
static void ar5523_cmd_tx_cb(struct urb *urb)
{
	struct ar5523_tx_cmd *cmd = urb->context;
	struct ar5523 *ar = cmd->ar;

	if (urb->status) {
		ar5523_err(ar, "Failed to TX command. Status = %d\n",
			   urb->status);
		cmd->res = urb->status;
		complete(&cmd->done);
		return;
	}

	if (!(cmd->flags & AR5523_CMD_FLAG_READ)) {
		cmd->res = 0;
		complete(&cmd->done);
	}
}

static int ar5523_cmd(struct ar5523 *ar, u32 code, const void *idata,
		int ilen, void *odata, int olen, int flags)
{
	struct ar5523_cmd_hdr *hdr;
	struct ar5523_tx_cmd *cmd = &ar->tx_cmd;
	int xferlen, error;

	/* always bulk-out a multiple of 4 bytes */
	xferlen = (sizeof(struct ar5523_cmd_hdr) + ilen + 3) & ~3;

	hdr = (struct ar5523_cmd_hdr *)cmd->buf_tx;
	memset(hdr, 0, sizeof(struct ar5523_cmd_hdr));
	hdr->len  = cpu_to_be32(xferlen);
	hdr->code = cpu_to_be32(code);
	hdr->priv = AR5523_CMD_ID;

	if (flags & AR5523_CMD_FLAG_MAGIC)
		hdr->magic = cpu_to_be32(1 << 24);
	memcpy(hdr + 1, idata, ilen);

	cmd->odata = odata;
	cmd->olen = olen;
	cmd->flags = flags;

	ar5523_dbg(ar, "do cmd %02x\n", code);

	usb_fill_bulk_urb(cmd->urb_tx, ar->dev, ar5523_cmd_tx_pipe(ar->dev),
			  cmd->buf_tx, xferlen, ar5523_cmd_tx_cb, cmd);
	cmd->urb_tx->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	error = usb_submit_urb(cmd->urb_tx, GFP_KERNEL);
	if (error) {
		ar5523_err(ar, "could not send command 0x%x, error=%d\n",
			       code, error);
		return error;
	}

	if (!wait_for_completion_timeout(&cmd->done, 2 * HZ)) {
		cmd->odata = NULL;
		ar5523_err(ar, "timeout waiting for command %02x reply\n",
			   code);
		cmd->res = -ETIMEDOUT;
	}

	return cmd->res;
}

static int ar5523_cmd_write(struct ar5523 *ar, u32 code, const void *data,
		int len, int flags)
{
	flags &= ~AR5523_CMD_FLAG_READ;
	return ar5523_cmd(ar, code, data, len, NULL, 0, flags);
}

static int ar5523_cmd_read(struct ar5523 *ar, u32 code, const void *idata,
		int ilen, void *odata, int olen, int flags)
{
	flags |= AR5523_CMD_FLAG_READ;
	return ar5523_cmd(ar, code, idata, ilen, odata, olen, flags);
}

static int ar5523_config(struct ar5523 *ar, u32 reg, u32 val)
{
	struct ar5523_write_mac write;
	int error;

	write.reg = cpu_to_be32(reg);
	write.len = cpu_to_be32(0);	/* 0 = single write */
	*(u32 *)write.data = cpu_to_be32(val);

	error = ar5523_cmd_write(ar, WDCMSG_TARGET_SET_CONFIG, &write,
				 3 * sizeof(u32), 0);
	if (error != 0)
		ar5523_err(ar, "could not write register 0x%02x\n", reg);
	return error;
}

static int ar5523_config_multi(struct ar5523 *ar, u32 reg, const void *data,
				int len)
{
	struct ar5523_write_mac write;
	int error;

	write.reg = cpu_to_be32(reg);
	write.len = cpu_to_be32(len);
	memcpy(write.data, data, len);

	/* properly handle the case where len is zero (reset) */
	error = ar5523_cmd_write(ar, WDCMSG_TARGET_SET_CONFIG, &write,
	    (len == 0) ? sizeof(u32) : 2 * sizeof(u32) + len, 0);
	if (error != 0) {
		ar5523_err(ar, "could not write %d bytes to register 0x%02x\n",
			   len, reg);
	}
	return error;
}

static int ar5523_get_status(struct ar5523 *ar, u32 which, void *odata,
			      int olen)
{
	int error;

	which = cpu_to_be32(which);
	error = ar5523_cmd_read(ar, WDCMSG_TARGET_GET_STATUS,
	    &which, sizeof(which), odata, olen, AR5523_CMD_FLAG_MAGIC);
	if (error != 0)
		ar5523_err(ar, "could not read EEPROM offset 0x%02x\n",
			   be32_to_cpu(which));
	return error;
}

static int ar5523_get_capability(struct ar5523 *ar, u32 cap, u32 *val)
{
	int error;

	cap = cpu_to_be32(cap);
	error = ar5523_cmd_read(ar, WDCMSG_TARGET_GET_CAPABILITY,
	    &cap, sizeof(cap), val, sizeof(u32), AR5523_CMD_FLAG_MAGIC);
	if (error != 0) {
		ar5523_err(ar, "could not read capability %u\n",
		    be32_to_cpu(cap));
		return error;
	}
	*val = be32_to_cpu(*val);
	return error;
}

static int ar5523_get_devcap(struct ar5523 *ar)
{
#define	GETCAP(x) do {				\
	error = ar5523_get_capability(ar, x, &cap);		\
	if (error != 0)					\
		return error;				\
	ar5523_info(ar, "Cap: "			\
	    "%s=0x%08x\n", #x, cap);	\
} while (0)
	int error;
	u32 cap;

	/* collect device capabilities */
	GETCAP(CAP_TARGET_VERSION);
	GETCAP(CAP_TARGET_REVISION);
	GETCAP(CAP_MAC_VERSION);
	GETCAP(CAP_MAC_REVISION);
	GETCAP(CAP_PHY_REVISION);
	GETCAP(CAP_ANALOG_5GHz_REVISION);
	GETCAP(CAP_ANALOG_2GHz_REVISION);

	GETCAP(CAP_REG_DOMAIN);
	GETCAP(CAP_REG_CAP_BITS);
	GETCAP(CAP_WIRELESS_MODES);
	GETCAP(CAP_CHAN_SPREAD_SUPPORT);
	GETCAP(CAP_COMPRESS_SUPPORT);
	GETCAP(CAP_BURST_SUPPORT);
	GETCAP(CAP_FAST_FRAMES_SUPPORT);
	GETCAP(CAP_CHAP_TUNING_SUPPORT);
	GETCAP(CAP_TURBOG_SUPPORT);
	GETCAP(CAP_TURBO_PRIME_SUPPORT);
	GETCAP(CAP_DEVICE_TYPE);
	GETCAP(CAP_WME_SUPPORT);
	GETCAP(CAP_TOTAL_QUEUES);
	GETCAP(CAP_CONNECTION_ID_MAX);

	GETCAP(CAP_LOW_5GHZ_CHAN);
	GETCAP(CAP_HIGH_5GHZ_CHAN);
	GETCAP(CAP_LOW_2GHZ_CHAN);
	GETCAP(CAP_HIGH_2GHZ_CHAN);
	GETCAP(CAP_TWICE_ANTENNAGAIN_5G);
	GETCAP(CAP_TWICE_ANTENNAGAIN_2G);

	GETCAP(CAP_CIPHER_AES_CCM);
	GETCAP(CAP_CIPHER_TKIP);
	GETCAP(CAP_MIC_TKIP);
	return 0;
}

/*
 * Helpers.
 */

static void ar5523_stat_work(struct work_struct *work)
{
	struct ar5523 *ar = container_of(work, struct ar5523, stat_work);
	/*
	 * Send request for statistics asynchronously. The timer will be
	 * restarted when we'll get the stats notification.
	 */
	ar5523_cmd_write(ar, WDCMSG_TARGET_GET_STATS, NULL, 0, 0);
}

static void ar5523_stat(unsigned long arg)
{
	struct ar5523 *ar = (struct ar5523 *)arg;
	ieee80211_queue_work(ar->hw, &ar->stat_work);
}

static int ar5523_set_ledsteady(struct ar5523 *ar, int lednum, int ledmode)
{
	struct uath_cmd_ledsteady led;

	led.lednum = cpu_to_be32(lednum);
	led.ledmode = cpu_to_be32(ledmode);

	ar5523_dbg(ar, "set %s led %s (steady)\n",
		   (lednum == UATH_LED_LINK) ? "link" : "activity",
		   ledmode ? "on" : "off");
	return ar5523_cmd_write(ar, WDCMSG_SET_LED_STEADY, &led, sizeof(led),
		0);
}

static int ar5523_set_rxfilter(struct ar5523 *ar, u32 bits, u32 op)
{
	struct ar5523_cmd_rx_filter rxfilter;

	rxfilter.bits = cpu_to_be32(bits);
	rxfilter.op = cpu_to_be32(op);

	ar5523_dbg(ar, "setting Rx filter=0x%x flags=0x%x\n", bits, op);
	return ar5523_cmd_write(ar, WDCMSG_RX_FILTER, &rxfilter,
	    sizeof(rxfilter), 0);
}

static int ar5523_reset_tx_queues(struct ar5523 *ar)
{
	int ac, error;

	for (ac = 0; ac < 4; ac++) {
		const __be32 qid = cpu_to_be32(AR5523_AC_TO_QID(ac));

		ar5523_dbg(ar, "resetting Tx queue %d\n", AR5523_AC_TO_QID(ac));

		error = ar5523_cmd_write(ar, AR5523_CMD_RESET_QUEUE,
					 &qid, sizeof(qid), 0);
		if (error)
			break;
	}

	return error;
}

static int ar5523_set_chan(struct ar5523 *ar)
{
	struct ieee80211_conf *conf = &ar->hw->conf;

	struct ar5523_cmd_reset reset;

	memset(&reset, 0, sizeof(reset));
	reset.flags |= cpu_to_be32(UATH_CHAN_2GHZ);
	reset.flags |= cpu_to_be32(UATH_CHAN_OFDM);
	reset.freq = cpu_to_be32(conf->channel->center_freq);
	reset.maxrdpower = cpu_to_be32(50);	/* XXX */
	reset.channelchange = cpu_to_be32(1);
	reset.keeprccontent = cpu_to_be32(0);

	ar5523_dbg(ar, "set chan flags 0x%x freq %d\n",
		   be32_to_cpu(reset.flags),
		   conf->channel->center_freq);
	return ar5523_cmd_write(ar, WDCMSG_RESET, &reset, sizeof(reset), 0);
}

static int ar5523_wme_init(struct ar5523 *ar)
{
	struct ar5523_qinfo qinfo;
	int ac, error;
	static const struct ar5523_wme_settings uath_wme_11g[] = {
		{ 7, 4, 10,  0, 0 },	/* Background */
		{ 3, 4, 10,  0, 0 },	/* Best-Effort */
		{ 3, 3,  4, 26, 0 },	/* Video */
		{ 2, 2,  3, 47, 0 },	/* Voice */
	};

	memset(&qinfo, 0, sizeof(qinfo));
	qinfo.size   = cpu_to_be32(32);
	qinfo.magic1 = cpu_to_be32(1);      /* XXX ack policy? */
	qinfo.magic2 = cpu_to_be32(1);

	for (ac = 0; ac < 4; ac++) {
		qinfo.qid      = cpu_to_be32(AR5523_AC_TO_QID(ac));
		qinfo.ac       = cpu_to_be32(ac);
		qinfo.aifsn    = cpu_to_be32(uath_wme_11g[ac].aifsn);
		qinfo.logcwmin = cpu_to_be32(uath_wme_11g[ac].logcwmin);
		qinfo.logcwmax = cpu_to_be32(uath_wme_11g[ac].logcwmax);
		qinfo.txop     = cpu_to_be32(AR5523_TXOP_TO_US(
						uath_wme_11g[ac].txop));
		qinfo.acm      = cpu_to_be32(uath_wme_11g[ac].acm);

		ar5523_dbg(ar, "setting up Tx queue %d\n",
			       AR5523_AC_TO_QID(ac));

		error = ar5523_cmd_write(ar, AR5523_CMD_SET_QUEUE,
					 &qinfo, sizeof(qinfo), 0);
		if (error)
			break;
	}

	return error;
}

static int ar5523_switch_channel(struct ar5523 *ar)
{
	int error;

	/* set radio frequency */
	error = ar5523_set_chan(ar);
	if (error) {
		ar5523_err(ar, "could not set channel, error %d\n", error);
		goto failed;
	}
	/* reset Tx rings */
	error = ar5523_reset_tx_queues(ar);
	if (error) {
		ar5523_err(ar, "could not reset Tx queues, error %d\n", error);
		goto failed;
	}
	/* set Tx rings WME properties */
	error = ar5523_wme_init(ar);
	if (error) {
		ar5523_err(ar, "could not init Tx queues, error %d\n", error);
		goto failed;
	}
#if 0
	error = ar5523_set_ledstate(ar, 0);
	if (error) {
		ar5523_err(ar, "could not set led state, error %d\n", error);
		goto failed;
	}


	error = ar5523_flush(sar);
	if (error) {
		ar5523_err(ar, "could not flush pipes, error %d\n", error);
		goto failed;
	}
#endif
failed:
	return error;
}

static void ar5523_data_rx_cb(struct urb *urb)
{
	struct ar5523_rx_data *data = urb->context;
	struct ar5523 *ar = data->ar;
	struct ar5523_rx_desc *desc;
	struct ar5523_chunk *chunk;
	struct ieee80211_hw *hw = ar->hw;
	struct ieee80211_rx_status *rx_status;
	u32 rxlen;
	int usblen = urb->actual_length;
	int hdrlen, pad;
	unsigned long flags;

	/* sync/async unlink faults aren't errors */
	if (urb->status && (urb->status != -ENOENT &&
	    urb->status != -ECONNRESET && urb->status != -ESHUTDOWN)) {
		ar5523_dbg(ar, "%s: nonzero write bulk status received: %d\n",
			   __func__, urb->status);

		goto skip;
	}

	if (urb->status) {
		/* do not try to resubmit urb */
		return;
	}

	if (usblen < AR5523_MIN_RXBUFSZ) {
		ar5523_err(ar, "wrong xfer size (usblen=%d)\n", usblen);
		goto skip;
	}

	chunk = (struct ar5523_chunk *) data->skb->data;

	if (chunk->flags != UATH_CFLAGS_FINAL)
		ar5523_dbg(ar, "chunk seq: %d flags: %02x len: %d\n",
			   chunk->seqnum, chunk->flags,
			   be16_to_cpu(chunk->length));

	if ((chunk->flags & UATH_CFLAGS_FINAL) == 0) {
		ar5523_err(ar, "No final in RX frame\n");
		goto skip;
	}

	/* Rx descriptor is located at the end, 32-bit aligned */
	desc = (struct ar5523_rx_desc *)
		(data->skb->data + usblen - sizeof(struct ar5523_rx_desc));

	rxlen = be32_to_cpu(desc->len);
	if (rxlen > ar->rxbufsz) {
		ar5523_dbg(ar, "bad descriptor (len=%d)\n",
			   be32_to_cpu(desc->len));
		goto skip;
	}

	if (!rxlen)
		goto skip;

	if (be32_to_cpu(desc->status) != 0) {
		ar5523_dbg(ar, "Bad RX status (0x%x). Skip\n",
			   be32_to_cpu(desc->status));
		goto skip;
	}

	skb_reserve(data->skb, sizeof(__be32));

	skb_put(data->skb, rxlen - sizeof(struct ar5523_rx_desc));

	hdrlen = ieee80211_get_hdrlen_from_skb(data->skb);
	if (hdrlen & 3) {
		ar5523_dbg(ar, "eek, alignment workaround activated\n");
		pad = hdrlen % 4;
		memmove(data->skb->data + pad, data->skb->data, hdrlen);
		skb_pull(data->skb, pad);
	}

	rx_status = IEEE80211_SKB_RXCB(data->skb);
	memset(rx_status, 0, sizeof(*rx_status));
	rx_status->freq = be32_to_cpu(desc->channel);
	rx_status->band = hw->conf.channel->band;
	rx_status->signal = -95 + be32_to_cpu(desc->rssi);

	ieee80211_rx_irqsafe(hw, data->skb);


skip:
	data->skb = NULL;
	spin_lock_irqsave(&ar->rx_data_list_lock, flags);
	list_move(&data->list, &ar->rx_data_free);
	spin_unlock_irqrestore(&ar->rx_data_list_lock, flags);
	if (atomic_inc_return(&ar->rx_data_free_cnt) >=
	    AR5523_RX_DATA_REFILL_COUNT) {
		ieee80211_queue_work(ar->hw, &ar->rx_refill_work);
	}

}

static void ar5523_rx_refill_work(struct work_struct *work)
{
	struct ar5523 *ar = container_of(work, struct ar5523, rx_refill_work);
	struct ar5523_rx_data *data;
	unsigned long flags;
	int error;

	while (!list_empty(&ar->rx_data_free)) {
		data = (struct ar5523_rx_data *) ar->rx_data_free.next;

		data->skb = dev_alloc_skb(ar->rxbufsz);
		if (!data->skb) {
			ar5523_err(ar, "could not allocate rx skbuff\n");
			return;
		}

		usb_fill_bulk_urb(data->urb, ar->dev,
				  ar5523_data_rx_pipe(ar->dev), data->skb->data,
				  ar->rxbufsz, ar5523_data_rx_cb, data);


		error = usb_submit_urb(data->urb, GFP_KERNEL);
		if (error) {
			ar5523_err(ar, "error %d when submitting rx data urb\n",
				       error);
			return;
		}

		spin_lock_irqsave(&ar->rx_data_list_lock, flags);
		list_move(&data->list, &ar->rx_data_used);
		spin_unlock_irqrestore(&ar->rx_data_list_lock, flags);
		atomic_dec(&ar->rx_data_free_cnt);
	}
}

static void ar5523_cancel_rx_bufs(struct ar5523 *ar)
{
	struct ar5523_rx_data *data;

	while (!list_empty(&ar->rx_data_used)) {
		data = (struct ar5523_rx_data *) ar->rx_data_used.next;
		usb_kill_urb(data->urb);
		kfree_skb(data->skb);
		data->skb = NULL;
		list_move(&data->list, &ar->rx_data_free);
		atomic_inc(&ar->rx_data_free_cnt);
	}
}

static void ar5523_free_rx_bufs(struct ar5523 *ar)
{
	struct ar5523_rx_data *data;

	ar5523_cancel_rx_bufs(ar);

	while (!list_empty(&ar->rx_data_free)) {
		data = (struct ar5523_rx_data *) ar->rx_data_free.next;
		list_del(&data->list);
		usb_free_urb(data->urb);
	}
}

static int ar5523_alloc_rx_bufs(struct ar5523 *ar)
{
	int i;

	for (i = 0; i < AR5523_RX_DATA_COUNT; i++) {
		struct ar5523_rx_data *data = &ar->rx_data[i];

		data->ar = ar;
		data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!data->urb) {
			ar5523_err(ar, "could not allocate rx data urb\n");
			goto err;
		}
		list_add_tail(&data->list, &ar->rx_data_free);
		atomic_inc(&ar->rx_data_free_cnt);
	}

	return 0;

err:
	ar5523_free_rx_bufs(ar);
	return -ENOMEM;
}

/*
 * Interface routines to the mac80211 stack.
 */
static int ar5523_start(struct ieee80211_hw *hw)
{
	struct ar5523 *ar = hw->priv;
	int error;
	__be32 val;

	ar5523_dbg(ar, "start called\n");

	mutex_lock(&ar->mutex);
	val = cpu_to_be32(0);
	ar5523_cmd_write(ar, WDCMSG_BIND, &val, sizeof(val), 0);

	/* set MAC address */
	ar5523_config_multi(ar, CFG_MAC_ADDR, &ar->hw->wiphy->perm_addr,
			    ETH_ALEN);

	/* XXX honor net80211 state */
	ar5523_config(ar, CFG_RATE_CONTROL_ENABLE, 0x00000001);
	ar5523_config(ar, CFG_DIVERSITY_CTL, 0x00000001);
	ar5523_config(ar, CFG_ABOLT, 0x0000003f);
	ar5523_config(ar, CFG_WME_ENABLED, 0x00000001);

	ar5523_config(ar, CFG_SERVICE_TYPE, 1);
	ar5523_config(ar, CFG_TP_SCALE, 0x00000000);
	ar5523_config(ar, CFG_TPC_HALF_DBM5, 0x0000003c);
	ar5523_config(ar, CFG_TPC_HALF_DBM2, 0x0000003c);
	ar5523_config(ar, CFG_OVERRD_TX_POWER, 0x00000000);
	ar5523_config(ar, CFG_GMODE_PROTECTION, 0x00000000);
	ar5523_config(ar, CFG_GMODE_PROTECT_RATE_INDEX, 0x00000003);
	ar5523_config(ar, CFG_PROTECTION_TYPE, 0x00000000);
	ar5523_config(ar, CFG_MODE_CTS, 0x00000002);

	error = ar5523_cmd_read(ar, WDCMSG_TARGET_START, NULL, 0,
	    &val, sizeof(val), AR5523_CMD_FLAG_MAGIC);
	if (error) {
		ar5523_dbg(ar, "could not start target, error %d\n", error);
		goto err;
	}
	ar5523_dbg(ar, "%s returns handle: 0x%x\n",
	    "WDCMSG_TARGET_START", be32_to_cpu(val));

	/* set default channel */
	error = ar5523_switch_channel(ar);
	if (error) {
		ar5523_err(ar, "could not switch channel, error %d\n", error);
		goto err;
	}

	val = cpu_to_be32(TARGET_DEVICE_AWAKE);
	ar5523_cmd_write(ar, WDCMSG_SET_PWR_MODE, &val, sizeof(val), 0);
	/* XXX? check */
	ar5523_cmd_write(ar, WDCMSG_RESET_KEY_CACHE, NULL, 0, 0);

	ieee80211_queue_work(ar->hw, &ar->rx_refill_work);

	/* enable Rx */
	ar5523_set_rxfilter(ar, 0, UATH_FILTER_OP_INIT);
	ar5523_set_rxfilter(ar,
	    UATH_FILTER_RX_UCAST | UATH_FILTER_RX_MCAST |
	    UATH_FILTER_RX_BCAST,
	    UATH_FILTER_OP_SET);

	ar5523_set_ledsteady(ar, UATH_LED_ACTIVITY, UATH_LED_ON);
	ar5523_dbg(ar, "start OK\n");

err:
	mutex_unlock(&ar->mutex);
	return error;
}

static void ar5523_stop(struct ieee80211_hw *hw)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "stop called\n");

	mutex_lock(&ar->mutex);

	ar5523_set_ledsteady(ar, UATH_LED_LINK, UATH_LED_OFF);
	ar5523_set_ledsteady(ar, UATH_LED_ACTIVITY, UATH_LED_OFF);

	ar5523_cmd_write(ar, WDCMSG_TARGET_STOP, NULL, 0, 0);

	cancel_work_sync(&ar->rx_refill_work);
	ar5523_cancel_rx_bufs(ar);
	mutex_unlock(&ar->mutex);
}

static int ar5523_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	struct ar5523 *ar = hw->priv;
	int ret;

	ar5523_dbg(ar, "set_rts_threshold called\n");
	mutex_lock(&ar->mutex);

	ret = ar5523_config(ar, CFG_USER_RTS_THRESHOLD, value);

	mutex_unlock(&ar->mutex);
	return ret;
}

static void ar5523_data_tx_cb(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
	struct ar5523_tx_data *data = (struct ar5523_tx_data *)
				       txi->driver_data;
	struct ar5523 *ar = data->ar;

	ar5523_dbg(ar, "data tx urb completed\n");

	/* sync/async unlink faults aren't errors */
	if (urb->status && (urb->status != -ENOENT &&
	    urb->status != -ECONNRESET && urb->status != -ESHUTDOWN)) {
		ar5523_dbg(ar, "%s: nonzero write bulk status received: %d\n",
			   __func__, urb->status);
		goto out;
	}

	skb_pull(skb, sizeof(struct ar5523_tx_desc) + sizeof(__be32));

	txi->flags |= IEEE80211_TX_STAT_ACK;
	ieee80211_tx_status_irqsafe(ar->hw, skb);
out:
	atomic_dec(&ar->tx_data_queued);

	if (atomic_read(&ar->tx_data_queued) < AR5523_TX_DATA_RESTART_COUNT) {
		if (test_and_clear_bit(AR5523_TX_QUEUE_STOPPED, &ar->flags)) {
			ar5523_dbg(ar, "restart tx queue\n");
			ieee80211_wake_queues(ar->hw);
		}
	}

	usb_free_urb(urb);
}

static void ar5523_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	struct ieee80211_hdr *wh = (struct ieee80211_hdr *)skb->data;
	struct ieee80211_tx_info *txi = IEEE80211_SKB_CB(skb);
	struct ar5523 *ar = hw->priv;
	struct ar5523_tx_data *data = (struct ar5523_tx_data *)
				       txi->driver_data;
	struct ar5523_tx_desc *desc;
	struct urb *urb;
	int paylen = skb->len;
	int error = 0;
	__be32 *hdr;
	u32 txqid;

	ar5523_dbg(ar, "tx called\n");

	if (atomic_read(&ar->tx_data_queued) >= (AR5523_TX_DATA_COUNT-1)) {
		ar5523_dbg(ar, "tx queue full\n");
		if (!test_and_set_bit(AR5523_TX_QUEUE_STOPPED, &ar->flags)) {
			ar5523_dbg(ar, "stop queues\n");
			ieee80211_stop_queues(hw);
		}
	}

	if (atomic_read(&ar->tx_data_queued) >= AR5523_TX_DATA_COUNT) {
		WARN_ON(1);
		return;
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto out_free_skb;

	data->ar = ar;

	desc = (struct ar5523_tx_desc *)skb_push(skb, sizeof(*desc));
	hdr = (__be32 *)skb_push(skb, sizeof(__be32));

	/* fill Tx descriptor */
	*hdr = AR5523_MAKECTL(1, skb->len - sizeof(__be32));

	desc->msglen  = cpu_to_be32(skb->len);
	desc->msgid   = 0;
	desc->buflen = cpu_to_be32(paylen);
	desc->type   = cpu_to_be32(WDCMSG_SEND);
	desc->flags  = 0;

	/*
	 * XXX(hch): is there a better way to check this than poking into
	 *	     the frame?
	 */
	if (is_multicast_ether_addr(wh->addr1)) {
		desc->connid  = cpu_to_be32(AR5523_ID_BROADCAST);
		txqid = 3;
	} else {
		desc->connid  = cpu_to_be32(AR5523_ID_BSS);
		txqid = 1;
	}

	if (txi->flags & IEEE80211_TX_CTL_USE_MINRATE)
		txqid |= UATH_TXQID_MINRATE;

	desc->txqid = cpu_to_be32(txqid);

	usb_fill_bulk_urb(urb, ar->dev, ar5523_data_tx_pipe(ar->dev),
			  skb->data, skb->len, ar5523_data_tx_cb, skb);

	error = usb_submit_urb(urb, GFP_ATOMIC);
	if (error) {
		ar5523_err(ar, "error %d when submitting tx urb\n", error);
		goto out_free_urb;
	}


	atomic_inc(&ar->tx_data_queued);

	return;

out_free_urb:
	usb_free_urb(urb);
out_free_skb:
	kfree_skb(skb);
	return;
}

static int ar5523_add_interface(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "add interface called\n");

	if (ar->vif) {
		ar5523_dbg(ar, "invalid add_interface\n");
		return -EOPNOTSUPP;
	}

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
		ar->vif = vif;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void ar5523_remove_interface(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "remove interface called\n");

	ar->vif = NULL;
}

static int ar5523_hwconfig(struct ieee80211_hw *hw, u32 changed)
{
	struct ar5523 *ar = hw->priv;

	mutex_lock(&ar->mutex);
	ar5523_dbg(ar, "config called\n");
	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		ar5523_dbg(ar, "Do channel switch\n");
		ar5523_switch_channel(ar);
	}
	mutex_unlock(&ar->mutex);

	return 0;
}

static int ar5523_get_wlan_mode(struct ar5523 *ar,
				 struct ieee80211_bss_conf *bss_conf)
{
	struct ieee80211_supported_band *band;
	int bit;
	struct ieee80211_sta *sta;
	u32 sta_rate_set;

	band = ar->hw->wiphy->bands[ar->hw->conf.channel->band];
	sta = ieee80211_find_sta(ar->vif, bss_conf->bssid);
	if (!sta) {
		ar5523_info(ar, "STA not found!\n");
		return WLAN_MODE_11b;
	}
	sta_rate_set = sta->supp_rates[ar->hw->conf.channel->band];

	for (bit = 0; bit < band->n_bitrates; bit++) {
		if (sta_rate_set & 1) {
			int rate = band->bitrates[bit].bitrate;
			switch (rate) {
			case 60:
			case 90:
			case 120:
			case 180:
			case 240:
			case 360:
			case 480:
			case 540:
				return WLAN_MODE_11g;
			}
		}
		sta_rate_set >>= 1;
	}
	return WLAN_MODE_11b;
}

static void ar5523_create_rateset(struct ar5523 *ar,
				  struct ieee80211_bss_conf *bss_conf,
				  struct ar5523_cmd_rateset *rs,
				  bool basic)
{
	struct ieee80211_supported_band *band;
	struct ieee80211_sta *sta;
	int bit, i = 0;
	u32 sta_rate_set, basic_rate_set;

	sta = ieee80211_find_sta(ar->vif, bss_conf->bssid);
	basic_rate_set = bss_conf->basic_rates;
	if (!sta) {
		ar5523_info(ar, "STA not found. Cannot set rates\n");
		sta_rate_set = bss_conf->basic_rates;
	}
	sta_rate_set = sta->supp_rates[ar->hw->conf.channel->band];

	ar5523_dbg(ar, "sta rate_set = %08x\n", sta_rate_set);

	band = ar->hw->wiphy->bands[ar->hw->conf.channel->band];
	for (bit = 0; bit < band->n_bitrates; bit++) {
		BUG_ON(i >= AR5523_MAX_NRATES);
		ar5523_dbg(ar, "Considering rate %d : %d\n",
			   band->bitrates[bit].hw_value, sta_rate_set & 1);
		if (sta_rate_set & 1) {
			rs->set[i] = band->bitrates[bit].hw_value;
			if (basic_rate_set & 1 && basic)
				rs->set[i] |= 0x80;
			i++;
		}
		sta_rate_set >>= 1;
		basic_rate_set >>= 1;
	}

	rs->length = i;
}

static int ar5523_set_basic_rates(struct ar5523 *ar,
			    struct ieee80211_bss_conf *bss)
{
	struct ar5523_cmd_rates rates;

	memset(&rates, 0, sizeof(rates));
	rates.connid = cpu_to_be32(2);		/* XXX */
	rates.size   = cpu_to_be32(sizeof(struct ar5523_cmd_rateset));
	ar5523_create_rateset(ar, bss, &rates.rateset, true);

	return ar5523_cmd_write(ar, WDCMSG_SET_BASIC_RATE,
				&rates, sizeof(rates), 0);
}

static int ar5523_create_connection(struct ar5523 *ar,
				     struct ieee80211_vif *vif,
				     struct ieee80211_bss_conf *bss)
{
	struct ar5523_cmd_create_connection create;
	int wlan_mode;

	memset(&create, 0, sizeof(create));
	create.connid = cpu_to_be32(2);
	create.bssid = cpu_to_be32(0);
	/* XXX packed or not?  */
	create.size = cpu_to_be32(sizeof(struct ar5523_cmd_rateset));

	ar5523_create_rateset(ar, bss, &create.connattr.rateset, false);

	wlan_mode = ar5523_get_wlan_mode(ar, bss);
	create.connattr.wlanmode = cpu_to_be32(wlan_mode);

	return ar5523_cmd_write(ar, WDCMSG_CREATE_CONNECTION, &create,
				sizeof(create), 0);
}

static int ar5523_write_associd(struct ar5523 *ar,
				 struct ieee80211_bss_conf *bss)
{
	struct ar5523_cmd_set_associd associd;

	memset(&associd, 0, sizeof(associd));
	associd.defaultrateix = cpu_to_be32(0);	/* XXX */
	associd.associd = cpu_to_be32(bss->aid);
	associd.timoffset = cpu_to_be32(0x3b);	/* XXX */
	memcpy(associd.bssid, bss->bssid, ETH_ALEN);
	return ar5523_cmd_write(ar, WDCMSG_WRITE_ASSOCID, &associd,
				sizeof(associd), 0);
}

static void ar5523_bss_info_changed(struct ieee80211_hw *hw,
		struct ieee80211_vif *vif,
		struct ieee80211_bss_conf *bss,
		u32 changed)
{
	struct ar5523 *ar = hw->priv;
	int error;

	ar5523_dbg(ar, "bss_info_changed called\n");

	mutex_lock(&ar->mutex);

	if (!(changed & BSS_CHANGED_ASSOC))
		goto out_unlock;

	if (bss->assoc) {
		error = ar5523_create_connection(ar, vif, bss);
		if (error) {
			ar5523_err(ar, "could not create connection\n");
			return;
		}

		error = ar5523_set_basic_rates(ar, bss);
		if (error) {
			ar5523_err(ar, "could not set negotiated rate set\n");
			return;
		}

		error = ar5523_write_associd(ar, bss);
		if (error) {
			ar5523_err(ar, "could not set association\n");
			return;
		}

		/* turn link LED on */
		ar5523_set_ledsteady(ar, UATH_LED_LINK, UATH_LED_ON);

		/* start statistics timer */
//		mod_timer(&ar->stat_timer, jiffies + HZ);

	} else {
		ar5523_set_ledsteady(ar, UATH_LED_LINK, UATH_LED_OFF);
	}

out_unlock:
	mutex_unlock(&ar->mutex);

}

#define AR5523_SUPPORTED_FILTERS (FIF_PROMISC_IN_BSS | \
				  FIF_ALLMULTI | \
				  FIF_FCSFAIL | \
				  FIF_BCN_PRBRESP_PROMISC | \
				  FIF_OTHER_BSS)

static void ar5523_configure_filter(struct ieee80211_hw *hw,
		unsigned int changed_flags, unsigned int *total_flags,
		u64 multicast)
{
	struct ar5523 *ar = hw->priv;
	u32 filter = 0;

	ar5523_dbg(ar, "configure_filter called\n");
	mutex_lock(&ar->mutex);

	*total_flags &= AR5523_SUPPORTED_FILTERS;

	if (*total_flags & FIF_BCN_PRBRESP_PROMISC)
		filter |= UATH_FILTER_RX_BEACON;

	if (*total_flags & FIF_PROMISC_IN_BSS ||
	    *total_flags & FIF_OTHER_BSS)
		filter |= UATH_FILTER_RX_PROM;

	filter |= UATH_FILTER_RX_UCAST | UATH_FILTER_RX_MCAST |
		  UATH_FILTER_RX_BCAST;

	ar5523_set_rxfilter(ar, 0, UATH_FILTER_OP_INIT);
	ar5523_set_rxfilter(ar, filter, UATH_FILTER_OP_SET);

	mutex_unlock(&ar->mutex);
}

static const struct ieee80211_ops ar5523_ops = {
	.start			= ar5523_start,
	.stop			= ar5523_stop,
	.tx			= ar5523_tx,
	.set_rts_threshold	= ar5523_set_rts_threshold,
	.add_interface		= ar5523_add_interface,
	.remove_interface	= ar5523_remove_interface,
	.config			= ar5523_hwconfig,
	.bss_info_changed	= ar5523_bss_info_changed,
	.configure_filter	= ar5523_configure_filter,
};

static void ar5523_free_tx_cmd(struct ar5523 *ar)
{
	struct ar5523_tx_cmd *cmd = &ar->tx_cmd;

	usb_free_coherent(ar->dev, AR5523_MAX_RXCMDSZ,
			  cmd->buf_tx, cmd->urb_tx->transfer_dma);
	usb_free_urb(cmd->urb_tx);
}

static int ar5523_alloc_tx_cmd(struct ar5523 *ar)
{
	struct ar5523_tx_cmd *cmd = &ar->tx_cmd;

	cmd->ar = ar;
	init_completion(&cmd->done);

	cmd->urb_tx = usb_alloc_urb(0, GFP_KERNEL);
	if (!cmd->urb_tx) {
		ar5523_err(ar, "could not allocate urb\n");
		return -ENOMEM;
	}
	cmd->buf_tx = usb_alloc_coherent(ar->dev, AR5523_MAX_TXCMDSZ,
					 GFP_KERNEL,
					 &cmd->urb_tx->transfer_dma);
	if (!cmd->buf_tx) {
		usb_free_urb(cmd->urb_tx);
		return -ENOMEM;
	}

	return 0;
}

static int ar5523_host_available(struct ar5523 *ar)
{
	struct uath_cmd_host_available setup;

	/* inform target the host is available */
	setup.sw_ver_major = cpu_to_be32(ATH_SW_VER_MAJOR);
	setup.sw_ver_minor = cpu_to_be32(ATH_SW_VER_MINOR);
	setup.sw_ver_patch = cpu_to_be32(ATH_SW_VER_PATCH);
	setup.sw_ver_build = cpu_to_be32(ATH_SW_VER_BUILD);
	return ar5523_cmd_read(ar, WDCMSG_HOST_AVAILABLE,
			       &setup, sizeof(setup), NULL, 0, 0);
}

static int ar5523_get_devstatus(struct ar5523 *ar)
{
	u8 macaddr[ETH_ALEN];
	int error;

	/* retrieve MAC address */
	error = ar5523_get_status(ar, ST_MAC_ADDR, macaddr, ETH_ALEN);
	if (error != 0) {
		ar5523_err(ar, "could not read MAC address\n");
		return error;
	}

	SET_IEEE80211_PERM_ADDR(ar->hw, macaddr);

	error = ar5523_get_status(ar, ST_SERIAL_NUMBER,
	    &ar->serial[0], sizeof(ar->serial));
	if (error) {
		ar5523_err(ar, "could not read device serial number\n");
		return error;
	}

	return 0;
}

#define AR5523_SANE_RXBUFSZ 2000

static int ar5523_get_max_rxsz(struct ar5523 *ar)
{
	int error;
	__be32 rxsize;

	/* Get max rx size */
	error = ar5523_get_status(ar, ST_WDC_TRANSPORT_CHUNK_SIZE, &rxsize,
				  sizeof(rxsize));
	if (error != 0) {
		ar5523_err(ar, "could not read max RX size\n");
		return error;
	}

	ar->rxbufsz = be32_to_cpu(rxsize);

	if (!ar->rxbufsz || ar->rxbufsz > AR5523_SANE_RXBUFSZ) {
		ar5523_err(ar, "Bad rxbufsz from device. Using %d instead\n",
			   AR5523_SANE_RXBUFSZ);
		ar->rxbufsz = AR5523_SANE_RXBUFSZ;
	}

	ar5523_dbg(ar, "Mac RX buf size: %d\n", ar->rxbufsz);
	return 0;
}


/*
 * This is copied from rtl818x, but we should probably move this
 * to common code as in OpenBSD.
 */
static const struct ieee80211_rate ar5523_rates[] = {
	{ .bitrate = 10, .hw_value = 2, },
	{ .bitrate = 20, .hw_value = 4 },
	{ .bitrate = 55, .hw_value = 11, },
	{ .bitrate = 110, .hw_value = 22, },
	{ .bitrate = 60, .hw_value = 12, },
	{ .bitrate = 90, .hw_value = 18, },
	{ .bitrate = 120, .hw_value = 24, },
	{ .bitrate = 180, .hw_value = 36, },
	{ .bitrate = 240, .hw_value = 48, },
	{ .bitrate = 360, .hw_value = 72, },
	{ .bitrate = 480, .hw_value = 96, },
	{ .bitrate = 540, .hw_value = 108, },
};

static const struct ieee80211_channel ar5523_channels[] = {
	{ .center_freq = 2412 },
	{ .center_freq = 2417 },
	{ .center_freq = 2422 },
	{ .center_freq = 2427 },
	{ .center_freq = 2432 },
	{ .center_freq = 2437 },
	{ .center_freq = 2442 },
	{ .center_freq = 2447 },
	{ .center_freq = 2452 },
	{ .center_freq = 2457 },
	{ .center_freq = 2462 },
	{ .center_freq = 2467 },
	{ .center_freq = 2472 },
	{ .center_freq = 2484 },
};

static int ar5523_init_modes(struct ar5523 *ar)
{
	BUILD_BUG_ON(sizeof(ar->channels) != sizeof(ar5523_channels));
	BUILD_BUG_ON(sizeof(ar->rates) != sizeof(ar5523_rates));

	memcpy(ar->channels, ar5523_channels, sizeof(ar5523_channels));
	memcpy(ar->rates, ar5523_rates, sizeof(ar5523_rates));

	ar->band.band = IEEE80211_BAND_2GHZ;
	ar->band.channels = ar->channels;
	ar->band.n_channels = ARRAY_SIZE(ar5523_channels);
	ar->band.bitrates = ar->rates;
	ar->band.n_bitrates = ARRAY_SIZE(ar5523_rates);
	ar->hw->wiphy->bands[IEEE80211_BAND_2GHZ] = &ar->band;

	return 0;
}

/*
 * Load the MIPS R4000 microcode into the device.  Once the image is loaded,
 * the device will detach itself from the bus and reattach later with a new
 * product Id (a la ezusb).
 */
static int ar5523_load_firmware(struct usb_device *dev)
{
	struct ar5523_fwblock *txblock, *rxblock;
	const struct firmware *fw;
	void *fwbuf;
	int len, offset;
	int foolen; /* XXX(hch): handle short transfers */
	int error = -ENXIO;

	if (request_firmware(&fw, AR5523_FIRMWARE_FILE, &dev->dev)) {
		dev_err(&dev->dev, "no firmware found: %s\n",
			AR5523_FIRMWARE_FILE);
		return -ENOENT;
	}

	txblock = kmalloc(sizeof(*txblock), GFP_KERNEL);
	if (!txblock)
		goto out;

	rxblock = kmalloc(sizeof(*rxblock), GFP_KERNEL);
	if (!rxblock)
		goto out_free_txblock;

	fwbuf = kmalloc(AR5523_MAX_FWBLOCK_SIZE, GFP_KERNEL);
	if (!fwbuf)
		goto out_free_rxblock;

	memset(txblock, 0, sizeof(struct ar5523_fwblock));
	txblock->flags = cpu_to_be32(AR5523_WRITE_BLOCK);
	txblock->total = cpu_to_be32(fw->size);

	offset = 0;
	len = fw->size;
	while (len > 0) {
		int mlen = min(len, AR5523_MAX_FWBLOCK_SIZE);

		txblock->remain = cpu_to_be32(len - mlen);
		txblock->len = cpu_to_be32(mlen);

		/* send firmware block meta-data */
		error = usb_bulk_msg(dev, ar5523_cmd_tx_pipe(dev),
				     txblock, sizeof(*txblock), &foolen,
				     AR5523_CMD_TIMEOUT);
		if (error) {
			dev_err(&dev->dev,
				 "could not send firmware block info\n");
			goto out_free_fwbuf;
		}

		/* send firmware block data */
		memcpy(fwbuf, fw->data + offset, mlen);
		error = usb_bulk_msg(dev, ar5523_data_tx_pipe(dev),
				     fwbuf, mlen, &foolen,
				     AR5523_DATA_TIMEOUT);
		if (error) {
			dev_err(&dev->dev,
				 "could not send firmware block data\n");
			goto out_free_fwbuf;
		}

		/* wait for ack from firmware */
		error = usb_bulk_msg(dev, ar5523_cmd_rx_pipe(dev),
				     rxblock, sizeof(*rxblock), &foolen,
				     AR5523_CMD_TIMEOUT);
		if (error) {
			dev_err(&dev->dev,
				 "could not read firmware answer\n");
			goto out_free_fwbuf;
		}

		len -= mlen;
		offset += mlen;
	}

	/*
	 * Set the error to -ENXIO to make sure we continue probing for
	 * a driver.
	 */
	error = -ENXIO;

 out_free_fwbuf:
	kfree(fwbuf);
 out_free_rxblock:
	kfree(rxblock);
 out_free_txblock:
	kfree(txblock);
 out:
	release_firmware(fw);
	return error;
}


static int ar5523_probe(struct usb_interface *intf,
		const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(intf);
	struct ieee80211_hw *hw;
	struct ar5523 *ar;
	int error = -ENOMEM;

	/*
	 * Load firmware if the device requires it.  This will return
	 * -ENXIO on success and we'll get called back afer the usb
	 * id changes to indicate that the firmware is present.
	 */
	if (id->driver_info & AR5523_FLAG_PRE_FIRMWARE)
		return ar5523_load_firmware(dev);


	hw = ieee80211_alloc_hw(sizeof(*ar), &ar5523_ops);
	if (!hw)
		goto out;
	SET_IEEE80211_DEV(hw, &intf->dev);

	ar = hw->priv;
	ar->hw = hw;
	ar->dev = dev;
	mutex_init(&ar->mutex);
	init_completion(&ar->ready);
	atomic_set(&ar->tx_data_queued, 0);
	atomic_set(&ar->rx_data_free_cnt, 0);
	INIT_WORK(&ar->stat_work, ar5523_stat_work);
	INIT_WORK(&ar->rx_refill_work, ar5523_rx_refill_work);

	INIT_LIST_HEAD(&ar->rx_data_free);
	INIT_LIST_HEAD(&ar->rx_data_used);
	spin_lock_init(&ar->rx_data_list_lock);

	error = ar5523_alloc_rx_bufs(ar);
	if (error) {
		ar5523_err(ar, "Could not allocate rx buffers\n");
		goto out_free_ar;
	}

	error = ar5523_alloc_rx_cmd(ar);
	if (error) {
		ar5523_err(ar, "Could not allocate rx command buffers\n");
		goto out_free_rx_bufs;
	}

	error = ar5523_alloc_tx_cmd(ar);
	if (error) {
		ar5523_err(ar, "Could not allocate tx command buffers\n");
		goto out_free_rx_cmd;
	}

	error = ar5523_submit_rx_cmd(ar);
	if (error) {
		ar5523_err(ar, "Failed to submit rx cmd\n");
		goto out_free_tx_cmd;
	}

	/*
	 * We're now ready to send/receive firmware commands.
	 */
	error = ar5523_host_available(ar);
	if (error) {
		ar5523_err(ar, "could not initialize adapter\n");
		goto out_cancel_rx_cmd;
	}

	error = ar5523_get_max_rxsz(ar);
	if (error) {
		ar5523_err(ar, "could not get caps from adapter\n");
		goto out_cancel_rx_cmd;
	}

	error = ar5523_get_devcap(ar);
	if (error) {
		ar5523_err(ar, "could not get caps from adapter\n");
		goto out_cancel_rx_cmd;
	}

	error = ar5523_get_devstatus(ar);
	if (error != 0) {
		ar5523_err(ar, "could not get device status\n");
		goto out_cancel_rx_cmd;
	}

	ar5523_info(ar, "MAC/BBP AR5523, RF AR%c112\n",
			(id->driver_info & AR5523_FLAG_ABG) ? '5' : '2');

	setup_timer(&ar->stat_timer, ar5523_stat, (unsigned long)ar);

	ar->vif = NULL;
	hw->flags = IEEE80211_HW_RX_INCLUDES_FCS |
		    IEEE80211_HW_SIGNAL_DBM |
		    IEEE80211_HW_HAS_RATE_CONTROL;
	hw->extra_tx_headroom = sizeof(struct ar5523_tx_desc) + sizeof(__be32);
	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	hw->queues = 1;

	error = ar5523_init_modes(ar);
	if (error)
		goto out_cancel_rx_cmd;

	usb_set_intfdata(intf, hw);

	error = ieee80211_register_hw(hw);
	if (error) {
		ar5523_err(ar, "could not register device\n");
		goto out_cancel_rx_cmd;
	}

	return 0;

out_cancel_rx_cmd:
	ar5523_cancel_rx_cmd(ar);
out_free_tx_cmd:
	ar5523_free_tx_cmd(ar);
out_free_rx_cmd:
	ar5523_free_rx_cmd(ar);
out_free_rx_bufs:
	ar5523_free_rx_bufs(ar);
out_free_ar:
	ieee80211_free_hw(hw);
out:
	return error;
}

static void ar5523_disconnect(struct usb_interface *intf)
{
	struct ieee80211_hw *hw = usb_get_intfdata(intf);
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "detaching\n");

	ieee80211_unregister_hw(hw);

	del_timer_sync(&ar->stat_timer);

	ar5523_cancel_rx_cmd(ar);
	ar5523_free_tx_cmd(ar);
	ar5523_free_rx_cmd(ar);
	ar5523_free_rx_bufs(ar);

	ieee80211_free_hw(hw);
	usb_set_intfdata(intf, NULL);
}

#define AR5523_DEVICE_UG(vendor, device) \
	{ USB_DEVICE((vendor), (device)) }, \
	{ USB_DEVICE((vendor), (device) + 1), \
		.driver_info = AR5523_FLAG_PRE_FIRMWARE }
#define AR5523_DEVICE_UX(vendor, device) \
	{ USB_DEVICE((vendor), (device)), \
		.driver_info = AR5523_FLAG_ABG }, \
	{ USB_DEVICE((vendor), (device) + 1), \
		.driver_info = AR5523_FLAG_ABG|AR5523_FLAG_PRE_FIRMWARE }

static struct usb_device_id ar5523_id_table[] = {
	AR5523_DEVICE_UG(0x168c, 0x0001),	/* Atheros / AR5523 */
	AR5523_DEVICE_UG(0x0cf3, 0x0001),	/* Atheros2 / AR5523_1 */
	AR5523_DEVICE_UG(0x0cf3, 0x0003),	/* Atheros2 / AR5523_2 */
	AR5523_DEVICE_UX(0x0cf3, 0x0005),	/* Atheros2 / AR5523_3 */
	AR5523_DEVICE_UG(0x0d8e, 0x7801),	/* Conceptronic / AR5523_1 */
	AR5523_DEVICE_UX(0x0d8e, 0x7811),	/* Conceptronic / AR5523_2 */
	AR5523_DEVICE_UX(0x2001, 0x3a00),	/* Dlink / DWLAG132 */
	AR5523_DEVICE_UG(0x2001, 0x3a02),	/* Dlink / DWLG132 */
	AR5523_DEVICE_UX(0x2001, 0x3a04),	/* Dlink / DWLAG122 */
	AR5523_DEVICE_UG(0x1690, 0x0712),	/* Gigaset / AR5523 */
	AR5523_DEVICE_UG(0x1690, 0x0710),	/* Gigaset / SMCWUSBTG */
	AR5523_DEVICE_UG(0x129b, 0x160c),	/* Gigaset / USB stick 108
						   (CyberTAN Technology) */
	AR5523_DEVICE_UG(0x16ab, 0x7801),	/* Globalsun / AR5523_1 */
	AR5523_DEVICE_UX(0x16ab, 0x7811),	/* Globalsun / AR5523_2 */
	AR5523_DEVICE_UG(0x0d8e, 0x7802),	/* Globalsun / AR5523_3 */
	AR5523_DEVICE_UX(0x0846, 0x4300),	/* Netgear / WG111U */
	AR5523_DEVICE_UG(0x0846, 0x4250),	/* Netgear / WG111T */
	AR5523_DEVICE_UG(0x0846, 0x5f00),	/* Netgear / WPN111 */
	AR5523_DEVICE_UG(0x157e, 0x3006),	/* Umedia / AR5523_1 */
	AR5523_DEVICE_UX(0x157e, 0x3205),	/* Umedia / AR5523_2 */
	AR5523_DEVICE_UG(0x157e, 0x3006),	/* Umedia / TEW444UBEU */
	AR5523_DEVICE_UG(0x1435, 0x0826),	/* Wistronneweb / AR5523_1 */
	AR5523_DEVICE_UX(0x1435, 0x0828),	/* Wistronneweb / AR5523_2 */
	AR5523_DEVICE_UG(0x0cde, 0x0012),	/* Zcom / AR5523 */
	AR5523_DEVICE_UG(0x1385, 0x4250),	/* Netgear3 / WG111T (2) */
	AR5523_DEVICE_UG(0x1385, 0x5f00),	/* Netgear / WPN111 */
	{ }
};
MODULE_DEVICE_TABLE(usb, ar5523_id_table);

static struct usb_driver ar5523_driver = {
	.name		= "ar5523",
	.id_table	= ar5523_id_table,
	.probe		= ar5523_probe,
	.disconnect	= ar5523_disconnect,
};

static int __init ar5523_init(void)
{
	return usb_register(&ar5523_driver);
}

static void __exit ar5523_exit(void)
{
	usb_deregister(&ar5523_driver);
}

MODULE_LICENSE("Dual BSD/GPL");
MODULE_FIRMWARE(AR5523_FIRMWARE_FILE);

module_init(ar5523_init);
module_exit(ar5523_exit);
