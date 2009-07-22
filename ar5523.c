/*
 * Driver for Atheros AR5005UG/AR5005UX chipsets.
 *
 * Copyright (c) 2006
 *	Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2007
 *	Christoph Hellwig <hch@lst.de>
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
	// XXX: msec or HZ?
	AR5523_DATA_TIMEOUT		= 10000,
	AR5523_CMD_TIMEOUT		= 1000,
};

enum {
	AR5523_TX_CMD_COUNT	= 30,
	AR5523_RX_CMD_COUNT	= 30,

	AR5523_TX_DATA_COUNT	= 16,
	AR5523_RX_DATA_COUNT	= 128,
};

struct ar5523_tx_cmd {
        struct ar5523		*ar;
	struct urb		*urb;
	void			*buf;
	void			*odata;
	int			flags;
	struct completion	done;
};

struct ar5523_rx_cmd {
        struct ar5523		*ar;
	struct urb		*urb;
	void			*buf;
};

struct ar5523_tx_data {
	struct ar5523		*ar;
	struct sk_buff		*skb;
	struct ieee80211_tx_control *control;
};

struct ar5523_rx_data {
	struct ar5523		*ar;
	struct urb		*urb;
	struct sk_buff		*skb;
};

struct ar5523 {
	struct usb_device	*dev;
	struct ieee80211_hw	*hw;

	struct ar5523_tx_cmd	tx_cmd[AR5523_TX_CMD_COUNT];
	struct ar5523_rx_cmd	rx_cmd[AR5523_RX_CMD_COUNT];
	int			cmd_idx;
	atomic_t		tx_cmd_queued;
	wait_queue_head_t	tx_cmd_wait;

	struct ar5523_rx_data	rx_data[AR5523_RX_DATA_COUNT];
	atomic_t		tx_data_queued;

	struct timer_list	stat_timer;
	struct completion	ready;

	int			rxbufsz;

	struct ieee80211_channel channels[14];
	struct ieee80211_rate	 rates[12];
	struct ieee80211_hw_mode modes[2];
	int			mode;

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
	AR5523_CMD_FLAG_ATOMIC	= (1 << 3),
};

#define ar5523_dbg(ar, format, arg...)            \
	dev_dbg(&(ar)->dev->dev, format, ## arg)
#define ar5523_err(ar, format, arg...)            \
	dev_err(&(ar)->dev->dev, format, ## arg)
#define ar5523_info(ar, format, arg...)            \
	dev_info(&(ar)->dev->dev, format, ## arg)


/*
 * Low-level functions to send read or write commands to the firmware.
 */
static void ar5523_cmd_tx_cb(struct urb *urb)
{
	struct ar5523_tx_cmd *cmd = urb->context;
	struct ar5523_cmd_hdr *hdr = cmd->buf;
	struct ar5523 *ar = cmd->ar;

	ar5523_dbg(ar, "tx urb %d completed\n", hdr->priv);

	/*
	 * No ones is waiting for async write commands and read commands
	 * get completed by ar5523_cmd_rx_cb.
	 */
	if ((cmd->flags & (AR5523_CMD_FLAG_READ|AR5523_CMD_FLAG_ASYNC)) == 0)
		complete(&cmd->done);
	wake_up(&ar->tx_cmd_wait);
}

/*
 * Yes, this is potentially racy and buggy, will fix later..
 */
static int ar5523_cmd(struct ar5523 *ar, u32 code, const void *idata,
		int ilen, void *odata, int flags)
{
	struct ar5523_cmd_hdr *hdr;
	struct ar5523_tx_cmd *cmd;
	int xferlen, error;


	if (atomic_read(&ar->tx_cmd_queued) >= AR5523_TX_CMD_COUNT) {
		if (flags & AR5523_CMD_FLAG_ATOMIC)
			return -EAGAIN;
		wait_event(ar->tx_cmd_wait,
			   (atomic_read(&ar->tx_cmd_queued) <
			    AR5523_TX_CMD_COUNT));
	}

	/* grab a xfer */
	cmd = &ar->tx_cmd[ar->cmd_idx];

	/* always bulk-out a multiple of 4 bytes */
	xferlen = (sizeof(struct ar5523_cmd_hdr) + ilen + 3) & ~3;

	hdr = (struct ar5523_cmd_hdr *)cmd->buf;
	memset(hdr, 0, sizeof(struct ar5523_cmd_hdr));
	hdr->len   = cpu_to_be32(xferlen);
	hdr->code  = cpu_to_be32(code);
	hdr->priv  = ar->cmd_idx;	/* don't care about endianness */
	if (flags & AR5523_CMD_FLAG_MAGIC)
		hdr->magic = cpu_to_be32(1 << 24);
	memcpy(hdr + 1, idata, ilen);

	cmd->odata = odata;
	cmd->flags = flags;

	usb_fill_bulk_urb(cmd->urb, ar->dev, ar5523_cmd_tx_pipe(ar->dev),
			  cmd->buf, xferlen, ar5523_cmd_tx_cb, cmd);
	cmd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	while (cmd->urb->hcpriv) {
		ar5523_info(ar, "urgg, hcpriv set, cmd_idx = %d\n", ar->cmd_idx);
		msleep_interruptible(200);
	}

	error = usb_submit_urb(cmd->urb, (flags & AR5523_CMD_FLAG_ATOMIC) ?
					 GFP_ATOMIC : GFP_KERNEL);
	if (error) {
		ar5523_err(ar, "could not send command 0x%x, error=%d\n",
			       code, error);
		return error;
	}

	ar->cmd_idx = (ar->cmd_idx + 1) % AR5523_TX_CMD_COUNT;

	/* wait at most two seconds for command reply */
	if ((flags & AR5523_CMD_FLAG_READ) ||
	    !(flags & (AR5523_CMD_FLAG_ASYNC|AR5523_CMD_FLAG_ATOMIC))) {
		if (!wait_for_completion_timeout(&cmd->done, 2 * HZ)) {
			cmd->odata = NULL;
			ar5523_err(ar, "timeout waiting for command reply\n");
			return -EIO;
		}
		cmd->odata = NULL;
	}

	return 0;
}

static int ar5523_cmd_write(struct ar5523 *ar, u32 code, const void *data,
		int len, int flags)
{
	flags &= ~AR5523_CMD_FLAG_READ; 
	return ar5523_cmd(ar, code, data, len, NULL, flags);
}

static int ar5523_cmd_read(struct ar5523 *ar, u32 code, const void *idata,
		int ilen, void *odata, int flags)
{
	flags |= AR5523_CMD_FLAG_READ;
	return ar5523_cmd(ar, code, idata, ilen, odata, flags);
}

static int ar5523_write_reg(struct ar5523 *ar, u32 reg, u32 val)
{
	struct ar5523_write_mac write;
	int error;

	write.reg = cpu_to_be32(reg);
	write.len = 0;			/* 0 = single write */
	*(__be32 *)write.data = cpu_to_be32(val);

	error = ar5523_cmd_write(ar, AR5523_CMD_WRITE_MAC, &write,
				 3 * sizeof(__be32), 0);
	if (error)
		ar5523_err(ar, "could not write register 0x%02x\n", reg);
	return error;
}

static int ar5523_write_multi(struct ar5523 *ar, u32 reg,
		const void *data, int len)
{
	struct ar5523_write_mac write;
	int xferlen;
	int error;

	write.reg = cpu_to_be32(reg);
	write.len = cpu_to_be32(len);
	memcpy(write.data, data, len);

	/* properly handle the case where len is zero (reset) */
	xferlen = len ? 2 * sizeof(__be32) + len : sizeof(__be32);
	error = ar5523_cmd_write(ar, AR5523_CMD_WRITE_MAC, &write, xferlen, 0);
	if (error) {
		ar5523_err(ar, "could not write %d bytes to register 0x%02x\n",
			       len, reg);
	}

	return error;
}

static int ar5523_read_reg(struct ar5523 *ar, u32 reg, u32 *val)
{
	struct ar5523_read_mac read;
	__be32 bereg = cpu_to_be32(reg);
	int error;

	error = ar5523_cmd_read(ar, AR5523_CMD_READ_MAC, &bereg,
				sizeof(reg), &read, 0);
	if (error) {
		ar5523_err(ar, "could not read register 0x%02x\n", reg);
		return error;
	}
	
	*val = be32_to_cpu(*(__be32 *)read.data);
	return error;
}

static int ar5523_read_eeprom(struct ar5523 *ar, u32 reg, void *odata) 
{
        struct ar5523_read_mac read;
	__be32 bereg = cpu_to_be32(reg);
	int error;

	error = ar5523_cmd_read(ar, AR5523_CMD_READ_EEPROM, &bereg,
				sizeof(bereg), &read, 0);
	if (error) {
		ar5523_err(ar, "could not read EEPROM offset 0x%02x\n", reg);
		return error;
	}

	memcpy(odata, read.data,
		read.len ? be32_to_cpu(read.len) : sizeof(__be32));
	return error;
}

/*
 * Helpers.
 */

static void ar5523_stat(unsigned long arg)
{
	struct ar5523 *ar = (struct ar5523 *)arg;

	/*
	 * Send request for statistics asynchronously. The timer will be
	 * restarted when we'll get the stats notification.
	 */
	ar5523_cmd_write(ar, AR5523_CMD_STATS, NULL, 0,
			AR5523_CMD_FLAG_ATOMIC);
}

static int ar5523_set_led(struct ar5523 *ar, int which, int on)
{
	struct ar5523_cmd_led led;

	led.which = cpu_to_be32(which);
	led.state = cpu_to_be32(on ? AR5523_LED_ON : AR5523_LED_OFF);

	ar5523_dbg(ar, "switching %s led %s\n",
		       (which == AR5523_LED_LINK) ? "link" : "activity",
		       on ? "on" : "off");

	return ar5523_cmd_write(ar, AR5523_CMD_SET_LED, &led, sizeof(led), 0);
}

static int ar5523_set_xled(struct ar5523 *ar, int which)
{
	struct ar5523_cmd_xled xled;

	memset(&xled, 0, sizeof(xled));
	xled.which = cpu_to_be32(which);
	xled.rate = cpu_to_be32(1);
	xled.mode = cpu_to_be32(2);

	return ar5523_cmd_write(ar, AR5523_CMD_SET_XLED,
				&xled, sizeof(xled), 0);
}

static int ar5523_set_rxfilter(struct ar5523 *ar, u32 filter, u32 flags)
{
	struct ar5523_cmd_filter rxfilter;

	rxfilter.filter = cpu_to_be32(filter);
	rxfilter.flags  = cpu_to_be32(flags);

	ar5523_dbg(ar, "setting Rx filter=0x%x flags=0x%x\n", filter, flags);

	return ar5523_cmd_write(ar, AR5523_CMD_SET_FILTER,
				&rxfilter, sizeof(rxfilter), 0);
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

static int ar5523_set_chan(struct ar5523 *ar, struct ieee80211_conf *conf)
{
	struct ar5523_set_chan chan;

	memset(&chan, 0, sizeof(chan));
	chan.flags  = cpu_to_be32(0x1400);
	chan.freq   = cpu_to_be32(conf->freq);
	chan.magic1 = cpu_to_be32(20);
	chan.magic2 = cpu_to_be32(50);
	chan.magic3 = cpu_to_be32(1);

	ar5523_dbg(ar, "switching to channel %d\n", conf->channel);

	return ar5523_cmd_write(ar, AR5523_CMD_SET_CHAN,
				&chan, sizeof(chan), 0);
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

static int ar5523_tx_null(struct ar5523 *ar)
{
	struct {
		__be32 hdr;
		struct ar5523_tx_desc desc;
	} *p;
	int foolen;
	int error;

	ar5523_dbg(ar, "ar5523_tx_null called\n");

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->hdr = AR5523_MAKECTL(1, sizeof(struct ar5523_tx_desc));
	memset(&p->desc, 0, sizeof(struct ar5523_tx_desc));
	p->desc.len  = cpu_to_be32(sizeof(struct ar5523_tx_desc));
	p->desc.type = cpu_to_be32(AR5523_TX_NULL);

	error = usb_bulk_msg(ar->dev, ar5523_data_tx_pipe(ar->dev),
			     p, sizeof(*p), &foolen, AR5523_DATA_TIMEOUT);
	if (error) {
		ar5523_err(ar, "error %d when submitting tx null urb\n", error);
		goto out;
	}

	error = ar5523_cmd_write(ar, AR5523_CMD_0F, NULL, 0, AR5523_CMD_FLAG_ASYNC);

 out:
	kfree(p);
	return error;
}

static int ar5523_set_rates(struct ar5523 *ar, struct ieee80211_if_conf *ifconf)
{
        struct ar5523_cmd_rates rates;

	memset(&rates, 0, sizeof(rates));
	rates.magic1 = cpu_to_be32(0x02);
#if 0
	rates.size   = cpu_to_be32(1 + sizeof(rates.rates));
	rates.nrates = rs->rs_nrates;
	memcpy(rates.rates, rs->rs_rates, rs->rs_nrates);
#endif
	
//	ar5523_dbg(ar, "setting supported rates nrates=%d\n", rs->rs_nrates);
	
	return ar5523_cmd_write(ar, AR5523_CMD_SET_RATES,
				&rates, sizeof(rates), 0);
}

static void ar5523_data_rx_cb(struct urb *urb)
{
	struct ar5523_rx_data *data = urb->context;
	struct ar5523 *ar = data->ar;
	struct ar5523_rx_desc *desc;
	struct ieee80211_hw *hw = ar->hw;
	struct ieee80211_rx_status rx_status = { 0 };
	int len = urb->actual_length;
	int hdrlen, pad;
	u32 hdr;
	int error;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		ar5523_dbg(ar, "non-zero write bulk status received: %d\n",
			       urb->status);
		goto skip;
	}

	if (len < AR5523_MIN_RXBUFSZ) {
		ar5523_err(ar, "wrong xfer size (len=%d)\n", len);
//		ifp->if_ierrors++;
		goto skip;
	}

	hdr = be32_to_cpu(*(__be32 *)data->skb->data);

	/* Rx descriptor is located at the end, 32-bit aligned */
	desc = (struct ar5523_rx_desc *)
		(data->skb->data + len - sizeof(struct ar5523_rx_desc));

	if (be32_to_cpu(desc->len) > ar->rxbufsz) {
		ar5523_err(ar, "bad descriptor (len=%d)\n",
			       be32_to_cpu(desc->len));
//		ifp->if_ierrors++;
		goto skip;
	}

	skb_reserve(data->skb, sizeof(__be32));

	skb_put(data->skb, be32_to_cpu(desc->len) -
			sizeof(struct ar5523_rx_desc));

	hdrlen = ieee80211_get_hdrlen_from_skb(data->skb);
	if (hdrlen & 3) {
		ar5523_dbg(ar, "eek, alignment workaround activated\n");
		pad = hdrlen % 4;
		memmove(data->skb->data + pad, data->skb->data, hdrlen);
		skb_pull(data->skb, pad);
	}

	/*
	 * XXX: not a whole lot of information provided here..
	 *
	 * need to poke into the descriptor if there might be more useful
	 * information in there.
	 */
	rx_status.freq =  be32_to_cpu(desc->freq);
	rx_status.channel = hw->conf.channel;
	rx_status.phymode = hw->conf.phymode;
	rx_status.ssi = be32_to_cpu(desc->rssi);
	
	ieee80211_rx_irqsafe(hw, data->skb, &rx_status);
	
	data->skb = __dev_alloc_skb(ar->rxbufsz, GFP_ATOMIC);
	if (!data->skb) {
		ar5523_err(ar, "could not allocate rx skbuff\n");
		return;
	}

 skip:
	/* re-submit the urb */
	usb_fill_bulk_urb(data->urb, ar->dev, ar5523_data_rx_pipe(ar->dev),
			  data->skb->data, ar->rxbufsz, ar5523_data_rx_cb,
			  data);

	error = usb_submit_urb(urb, GFP_ATOMIC);
	if (error) {
		// XXX: handle
		ar5523_err(ar, "error %d when resubmitting rx data urb\n",
			       error);
	}
}

static void ar5523_free_rx_bufs(struct ar5523 *ar)
{
	int i;

	for (i = 0; i < AR5523_RX_DATA_COUNT; i++) {
		struct ar5523_rx_data *data = &ar->rx_data[i];

		usb_kill_urb(data->urb);
		usb_free_urb(data->urb);
		kfree_skb(data->skb);
	}
}

static int ar5523_alloc_rx_bufs(struct ar5523 *ar)
{
	int error = -ENOMEM;
	int i;
	
	for (i = 0; i < AR5523_RX_DATA_COUNT; i++) {
		struct ar5523_rx_data *data = &ar->rx_data[i];

		data->ar = ar;
		data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!data->urb) {
			ar5523_err(ar, "could not allocate rx data urb\n");
			goto out;
		}
	
		data->skb = dev_alloc_skb(ar->rxbufsz);
		if (!data->skb) {
			ar5523_err(ar, "could not allocate rx skbuff\n");
			usb_free_urb(data->urb);
			goto out;
		}

		usb_fill_bulk_urb(data->urb, ar->dev,
				  ar5523_data_rx_pipe(ar->dev), data->skb->data,
				  ar->rxbufsz, ar5523_data_rx_cb, data);

		error = usb_submit_urb(data->urb, GFP_KERNEL);
		if (error) {
			ar5523_err(ar, "error %d when submitting rx data urb\n",
				       error);
			kfree_skb(data->skb);
			usb_free_urb(data->urb);
			return error;
		}
	}

	return 0;

 out:
 	while (--i >= 0) {
		struct ar5523_rx_data *data = &ar->rx_data[i];

		usb_kill_urb(data->urb);
		usb_free_urb(data->urb);
		kfree_skb(data->skb);
	}

	return error;
}

/*
 * Interface routines to the mac80211 stack.
 */
static int ar5523_start(struct ieee80211_hw *hw)
{
	struct ar5523 *ar = hw->priv;
	struct ar5523_cmd_31 cmd31;
	int error;
	__be32 val;

	ar5523_dbg(ar, "start called\n");

	/* reset data and command rings */
	ar->cmd_idx = 0;

	val = 0;
	ar5523_cmd_write(ar, AR5523_CMD_02, &val, sizeof(val), 0);

	/* set MAC address */
	ar5523_write_multi(ar, 0x13, &ar->hw->wiphy->perm_addr,
			   AR5523_ADDR_LEN);

	ar5523_write_reg(ar, 0x02, 0x00000001);
	ar5523_write_reg(ar, 0x0e, 0x0000003f);
	ar5523_write_reg(ar, 0x10, 0x00000001);
	ar5523_write_reg(ar, 0x06, 0x0000001e);

	error = ar5523_alloc_rx_bufs(ar);
	if (error)
		goto out;

	error = ar5523_cmd_read(ar, AR5523_CMD_07, NULL, 0, &val,
				AR5523_CMD_FLAG_MAGIC);
	if (error) {
		ar5523_err(ar, "could not send read command 07h\n");
		goto out_free_rx_bufs;
	}
	
	ar5523_dbg(ar, "command 07h return code: %x\n", be32_to_cpu(val));

#if 0
	/* set default channel */
	ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	error = ar5523_set_chan(ar, ic->ic_bss->ni_chan);
	if (error) {
		ar5523_err(ar, "could not set channel\n");
		goto fail;
	}
#endif

	error = ar5523_wme_init(ar);
	if (error) {
		ar5523_err(ar, "could not setup WME parameters\n");
		return error;
	}

	/* init MAC registers */
	ar5523_write_reg(ar, 0x19, 0x00000000);
	ar5523_write_reg(ar, 0x1a, 0x0000003c);
	ar5523_write_reg(ar, 0x1b, 0x0000003c);
	ar5523_write_reg(ar, 0x1c, 0x00000000);
	ar5523_write_reg(ar, 0x1e, 0x00000000);
	ar5523_write_reg(ar, 0x1f, 0x00000003);
	ar5523_write_reg(ar, 0x0c, 0x00000000);
	ar5523_write_reg(ar, 0x0f, 0x00000002);
	ar5523_write_reg(ar, 0x0a, 0x00000007);     /* XXX retry? */

	val = cpu_to_be32(4);
	ar5523_cmd_write(ar, AR5523_CMD_27, &val, sizeof(val), 0);
	ar5523_cmd_write(ar, AR5523_CMD_27, &val, sizeof(val), 0);
	ar5523_cmd_write(ar, AR5523_CMD_1B, NULL, 0, 0);

	/* enable Rx */
	ar5523_set_rxfilter(ar, 0x0000, 4);
	ar5523_set_rxfilter(ar, 0x0817, 1);

	cmd31.magic1 = cpu_to_be32(0xffffffff);
	cmd31.magic2 = cpu_to_be32(0xffffffff);
	ar5523_cmd_write(ar, AR5523_CMD_31, &cmd31, sizeof(cmd31), 0);

	return 0;

 out_free_rx_bufs:
	ar5523_free_rx_bufs(ar);
 out:
 	return error;
}

static void ar5523_stop(struct ieee80211_hw *hw)
{
	struct ar5523 *ar = hw->priv;
	__be32 val;

	ar5523_dbg(ar, "stop called\n");

	ar5523_set_led(ar, AR5523_LED_LINK, 0);
	ar5523_set_led(ar, AR5523_LED_ACTIVITY, 0);

	val = 0;
	ar5523_cmd_write(ar, AR5523_CMD_SET_STATE, &val, sizeof(val), 0);
	ar5523_cmd_write(ar, AR5523_CMD_RESET, NULL, 0, 0);

	val = 0;
	ar5523_cmd_write(ar, AR5523_CMD_15, &val, sizeof(val), 0);

	ar5523_free_rx_bufs(ar);
}

static int ar5523_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "set_rts_threshold called\n");

	return ar5523_write_reg(ar, 0x09, value);
}

static void ar5523_data_tx_cb(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct ar5523_tx_data *data = (struct ar5523_tx_data *)skb->cb;
	struct ar5523 *ar = data->ar;
	struct ieee80211_tx_status status = { {0} };

	ar5523_dbg(ar, "data tx urb completed\n");

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		ar5523_dbg(ar, "non-zero write bulk status received: %d\n",
			       urb->status);
		goto out;
	}

	memcpy(&status.control, data->control, sizeof(status.control));
	skb_pull(skb, sizeof(struct ar5523_tx_desc) + sizeof(__be32));

	status.flags |= IEEE80211_TX_STATUS_ACK;
	ieee80211_tx_status_irqsafe(ar->hw, skb, &status);
 out:
	atomic_dec(&ar->tx_data_queued);
	usb_free_urb(urb);
}

static int ar5523_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *control)
{
	struct ieee80211_hdr *wh = (struct ieee80211_hdr *)skb->data;
	struct ar5523 *ar = hw->priv;
	struct ar5523_tx_data *data;
	struct ar5523_tx_desc *desc;
	struct urb *urb;
	int paylen = skb->len;
	int error = 0;
	__be32 *hdr;

	ar5523_dbg(ar, "tx called\n");

	if (atomic_read(&ar->tx_data_queued) >= AR5523_TX_DATA_COUNT) {
		ar5523_dbg(ar, "tx queue full\n");
		return NETDEV_TX_BUSY;
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		goto out_free_skb;
	urb->context = skb;
	
	data = (struct ar5523_tx_data *)skb->cb;
	data->ar = ar;

	data->control = kmemdup(control, sizeof(*control), GFP_ATOMIC);
	if (!data->control)
		goto out_free_urb;

	desc = (struct ar5523_tx_desc *)skb_push(skb, sizeof(*desc));
	hdr = (__be32 *)skb_push(skb, sizeof(__be32));

	/* fill Tx descriptor */
	*hdr = AR5523_MAKECTL(1, skb->len - sizeof(__be32));

	desc->len    = cpu_to_be32(skb->len);
	desc->priv   = 0;
	desc->paylen = cpu_to_be32(paylen);
	desc->type   = cpu_to_be32(AR5523_TX_DATA);
	desc->flags  = 0;

	/*
	 * XXX(hch): is there a better way to check this than poking into
	 *	     the frame?
	 */
	if (is_multicast_ether_addr(wh->addr1)) {
		desc->dest  = cpu_to_be32(AR5523_ID_BROADCAST);
		desc->magic = cpu_to_be32(3);
	} else {
		desc->dest  = cpu_to_be32(AR5523_ID_BSS);
		desc->magic = cpu_to_be32(1);
	}

	usb_fill_bulk_urb(urb, ar->dev, ar5523_data_tx_pipe(ar->dev),
			  skb->data, skb->len, ar5523_data_tx_cb, skb);

	error = usb_submit_urb(urb, GFP_ATOMIC);
	if (error) {
		ar5523_err(ar, "error %d when submitting tx urb\n", error);
		goto out_free_control;
	}


	atomic_inc(&ar->tx_data_queued);

	return NETDEV_TX_OK;

 out_free_control:
	kfree(control);
 out_free_urb:
	usb_free_urb(urb);
 out_free_skb:
	kfree_skb(skb);
	return NETDEV_TX_BUSY;
}

static int ar5523_add_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "add interface called\n");

	/* NOTE: using IEEE80211_IF_TYPE_MNTR to indicate no mode selected */
	if (ar->mode != IEEE80211_IF_TYPE_MNTR) {
		ar5523_dbg(ar, "invalid add_interface\n");
		return -EOPNOTSUPP;
	}

	switch (conf->type) {
	case IEEE80211_IF_TYPE_STA:
	case IEEE80211_IF_TYPE_MNTR:
		ar->mode = conf->type;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void ar5523_remove_interface(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "remove interface called\n");

	ar->mode = IEEE80211_IF_TYPE_MNTR;
}

static int ar5523_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct ar5523 *ar = hw->priv;
	__be32 val;
	int error;

	ar5523_dbg(ar, "config called\n");

	/* set radio frequency */
	error = ar5523_set_chan(ar, conf);
	if (error) {
		ar5523_err(ar, "could not set channel\n");
		return error;
	}

	/* reset Tx rings */
	error = ar5523_reset_tx_queues(ar);
	if (error) {
		ar5523_err(ar, "could not reset Tx queues\n");
		return error;
	}

	/* set Tx rings WME properties */
	error = ar5523_wme_init(ar);
	if (error) {
		ar5523_err(ar, "could not setup WME parameters\n");
		return error;
	}

	val = 0;
	error = ar5523_cmd_write(ar, AR5523_CMD_SET_STATE,
				 &val, sizeof(val), 0);
	if (error) {
		ar5523_err(ar, "could not set state\n");
		return error;
	}

	error = ar5523_tx_null(ar);
	if (error) {
		ar5523_err(ar, "submitting null tx failed\n");
		return error;
	}

	return 0;
}

static int ar5523_config_interface(struct ieee80211_hw *hw, int if_id,
		struct ieee80211_if_conf *ifconf)
{
	struct ar5523 *ar = hw->priv;
	struct ar5523_cmd_bssid bssid;
	struct ar5523_cmd_0b cmd0b;
	struct ar5523_cmd_0c cmd0c;
	__be32 val;
	int error;

	ar5523_dbg(ar, "config_interface called\n");

	ar5523_cmd_write(ar, AR5523_CMD_24, NULL, 0, 0);

	memset(&bssid, 0, sizeof(bssid));
	bssid.len = cpu_to_be32(AR5523_ADDR_LEN);
	memcpy(&bssid.bssid, &ifconf->bssid, AR5523_ADDR_LEN);
	ar5523_cmd_write(ar, AR5523_CMD_SET_BSSID, &bssid, sizeof(bssid), 0);

	memset(&cmd0b, 0, sizeof(cmd0b));
	cmd0b.code = cpu_to_be32(2);
	cmd0b.size = cpu_to_be32(sizeof (cmd0b.data));
	ar5523_cmd_write(ar, AR5523_CMD_0B, &cmd0b, sizeof(cmd0b), 0);

	memset(&cmd0c, 0, sizeof(cmd0c));
	cmd0c.magic1 = cpu_to_be32(2);
	cmd0c.magic2 = cpu_to_be32(7);
	cmd0c.magic3 = cpu_to_be32(1);
	ar5523_cmd_write(ar, AR5523_CMD_0C, &cmd0c, sizeof(cmd0c), 0);

	error = ar5523_set_rates(ar, ifconf);
	if (error) {
		ar5523_err(ar, "could not set negotiated rate set\n");
		return error;
	}


	val = cpu_to_be32(1);
	ar5523_cmd_write(ar, AR5523_CMD_2E, &val, sizeof(val), 0);

	memset(&bssid, 0, sizeof(bssid));
	bssid.flags1 = cpu_to_be32(0xc004);
	bssid.flags2 = cpu_to_be32(0x003b);
	bssid.len    = cpu_to_be32(AR5523_ADDR_LEN);
	memcpy(&bssid.bssid, &ifconf->bssid, AR5523_ADDR_LEN);
	ar5523_cmd_write(ar, AR5523_CMD_SET_BSSID, &bssid, sizeof(bssid), 0);

	/* turn link LED on */
	ar5523_set_led(ar, AR5523_LED_LINK, 1);

	/* make activity LED blink */
	ar5523_set_xled(ar, 1);

	/* set state to associated */
	val = cpu_to_be32(1);
	ar5523_cmd_write(ar, AR5523_CMD_SET_STATE, &val, sizeof(val), 0);

	/* start statistics timer */
	mod_timer(&ar->stat_timer, jiffies + HZ);

	return 0;
}

static void ar5523_configure_filter(struct ieee80211_hw *hw,
		unsigned int changed_flags, unsigned int *total_flags,
		int mc_count, struct dev_addr_list *mc_list)
{
	struct ar5523 *ar = hw->priv;

	ar5523_dbg(ar, "configure_filter called\n");

	/* XXX: implement properly */
	*total_flags = 0;
}

static const struct ieee80211_ops ar5523_ops = {
	.start			= ar5523_start,
	.stop			= ar5523_stop,
	.tx			= ar5523_tx,
	.set_rts_threshold	= ar5523_set_rts_threshold,
	.add_interface		= ar5523_add_interface,
	.remove_interface	= ar5523_remove_interface,
	.config			= ar5523_config,
	.config_interface	= ar5523_config_interface,
	.configure_filter	= ar5523_configure_filter,
};

/*
 * TX/RX command handling.
 */
static void ar5523_read_reply(struct ar5523 *ar, struct ar5523_cmd_hdr *hdr)
{
	struct ar5523_tx_cmd *txcmd = &ar->tx_cmd[hdr->priv];

	if (txcmd->odata) {
		memcpy(txcmd->odata, hdr + 1,
		       be32_to_cpu(hdr->len) - sizeof(struct ar5523_cmd_hdr));
	}

	complete(&txcmd->done);
}

static void ar5523_cmd_rx_cb(struct urb *urb)
{
	struct ar5523_rx_cmd *cmd = urb->context;
	struct ar5523_cmd_hdr *hdr = cmd->buf;
	struct ar5523 *ar = cmd->ar;
	int error;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		ar5523_dbg(ar, "non-zero write bulk status received: %d\n",
			       urb->status);
		goto resubmit;
	}

	switch (be32_to_cpu(hdr->code) & 0xff) {
	default:
		/* reply to a read command */
		ar5523_read_reply(ar, hdr);
		break;
	case AR5523_NOTIF_READY:
		ar5523_dbg(ar, "received device ready notification\n");
		complete(&ar->ready);
		break;
	case AR5523_NOTIF_TX:
		/* this notification is sent when AR5523_TX_NOTIFY is set */
		ar5523_dbg(ar, "received Tx notification\n");
		break;
	case AR5523_NOTIF_STATS:
		ar5523_dbg(ar, "received device statistics\n");
		mod_timer(&ar->stat_timer, jiffies + HZ);
		break;
	}

	/* re-submit the urb */
	usb_fill_bulk_urb(cmd->urb, ar->dev, ar5523_cmd_rx_pipe(ar->dev),
			  cmd->buf, AR5523_MAX_RXCMDSZ, ar5523_cmd_rx_cb, cmd);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

 resubmit:
	error = usb_submit_urb(urb, GFP_ATOMIC);
	if (error) {
		// XXX: handle
		ar5523_err(ar, "error %d when resubmitting rx urb\n", error);
	}
}

static void ar5523_free_tx_cmds(struct ar5523 *ar)
{
	int i;

	for (i = 0; i < AR5523_TX_CMD_COUNT; i++) {
		struct ar5523_tx_cmd *cmd = &ar->tx_cmd[i];

		usb_kill_urb(cmd->urb);
		usb_buffer_free(ar->dev, AR5523_MAX_TXCMDSZ,
				cmd->buf, cmd->urb->transfer_dma);
		usb_free_urb(cmd->urb);
	}
}

static int ar5523_alloc_tx_cmds(struct ar5523 *ar)
{
	int error = -ENOMEM;
        int i;

	for (i = 0; i < AR5523_TX_CMD_COUNT; i++) {
		struct ar5523_tx_cmd *cmd = &ar->tx_cmd[i];

		cmd->ar = ar;
		cmd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!cmd->urb) {
			ar5523_err(ar, "could not allocate tx urb\n");
			goto out;
		}
		cmd->buf = usb_buffer_alloc(ar->dev, AR5523_MAX_TXCMDSZ,
					    GFP_KERNEL,
					    &cmd->urb->transfer_dma);
		if (!cmd->buf) {
			ar5523_err(ar, "could not allocate tx buffer\n");
			usb_free_urb(cmd->urb);
			goto out;
		}
		init_completion(&cmd->done);
	}

	return 0;

 out:
 	while (--i >= 0) {
		struct ar5523_tx_cmd *cmd = &ar->tx_cmd[i];

		usb_buffer_free(ar->dev, AR5523_MAX_TXCMDSZ,
				cmd->buf, cmd->urb->transfer_dma);
		usb_free_urb(cmd->urb);
	}

	return error;
}

static void ar5523_free_rx_cmds(struct ar5523 *ar)
{
	int i;

	for (i = 0; i < AR5523_RX_CMD_COUNT; i++) {
		struct ar5523_rx_cmd *cmd = &ar->rx_cmd[i];

		usb_kill_urb(cmd->urb);
		usb_buffer_free(ar->dev, AR5523_MAX_RXCMDSZ,
				cmd->buf, cmd->urb->transfer_dma);
		usb_free_urb(cmd->urb);
	}
}

static int ar5523_alloc_rx_cmds(struct ar5523 *ar)
{
	int error = -ENOMEM;
        int i;

	for (i = 0; i < AR5523_RX_CMD_COUNT; i++) {
		struct ar5523_rx_cmd *cmd = &ar->rx_cmd[i];

		cmd->ar = ar;
		cmd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!cmd->urb) {
			ar5523_err(ar, "could not allocate rx urb\n");
			goto out;
		}
		cmd->buf = usb_buffer_alloc(ar->dev, AR5523_MAX_TXCMDSZ,
					    GFP_KERNEL,
					    &cmd->urb->transfer_dma);
		if (!cmd->buf) {
			ar5523_err(ar, "could not allocate rx buffer\n");
			usb_free_urb(cmd->urb);
			goto out;
		}

		usb_fill_bulk_urb(cmd->urb, ar->dev,
				  ar5523_cmd_rx_pipe(ar->dev), cmd->buf,
				  AR5523_MAX_RXCMDSZ, ar5523_cmd_rx_cb, cmd);
		cmd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		error = usb_submit_urb(cmd->urb, GFP_KERNEL);
		if (error) {
			ar5523_err(ar, "error %d when submitting rx urb\n",
				       error);
			usb_buffer_free(ar->dev, AR5523_MAX_RXCMDSZ,
					cmd->buf, cmd->urb->transfer_dma);
			usb_free_urb(cmd->urb);
			return error;
		}
	}

	return 0;

 out:
 	while (--i >= 0) {
		struct ar5523_rx_cmd *cmd = &ar->rx_cmd[i];

		usb_kill_urb(cmd->urb);

		usb_buffer_free(ar->dev, AR5523_MAX_RXCMDSZ,
				cmd->buf, cmd->urb->transfer_dma);
		usb_free_urb(cmd->urb);
	}

	return error;
}

/*
 * Device initialization and teardown.
 */
static int ar5523_reset(struct ar5523 *ar)
{
        struct ar5523_cmd_setup setup;
	u32 reg, val;
	int error;

	/* init device with some voodoo incantations.. */
	setup.magic1 = cpu_to_be32(1);
	setup.magic2 = cpu_to_be32(5);
	setup.magic3 = cpu_to_be32(200);
	setup.magic4 = cpu_to_be32(27);

	error = ar5523_cmd_write(ar, AR5523_CMD_SETUP, &setup, sizeof(setup),
				 AR5523_CMD_FLAG_ASYNC);
	if (error)
		return error;

	/* ..and wait until firmware notifies us that it is ready */
	if (!wait_for_completion_timeout(&ar->ready, 5 * HZ)) {
		ar5523_err(ar,
			"failed to reset device - initialization timeout\n");
		return -EIO;
	}
		
	/* read PHY registers */
	for (reg = 0x09; reg <= 0x24; reg++) {
		if (reg == 0x0b || reg == 0x0c)
			continue;
		udelay(100);
		error = ar5523_read_reg(ar, reg, &val);
		if (error)
			return error;
		ar5523_dbg(ar, "reg 0x%02x=0x%08x\n", reg, val);
	}

	return error;
}

static int ar5523_query_eeprom(struct ar5523 *ar)
{
        int error;
	__be32 tmp;

	/* retrieve MAC address */
	error = ar5523_read_eeprom(ar, AR5523_EEPROM_MACADDR,
				   &ar->hw->wiphy->perm_addr);
	if (error) {
		ar5523_err(ar, "could not read MAC address\n");
		return error;
	}

	/* retrieve the maximum frame size that the hardware can receive */
	error = ar5523_read_eeprom(ar, AR5523_EEPROM_RXBUFSZ, &tmp);
	if (error) {
		ar5523_err(ar, "could not read maximum Rx buffer size\n");
		return error;
	}
	
	ar->rxbufsz = be32_to_cpu(tmp) & 0xfff;
	ar5523_dbg(ar, "maximum Rx buffer size %d\n", ar->rxbufsz);
	return 0;
}

/*
 * This is copied from rtl818x, but we should probably move this
 * to common code as in OpenBSD.
 */
static const struct ieee80211_rate ar5523_rates[] = {
        { .rate = 10,
	  .val = 0,
	  .flags = IEEE80211_RATE_CCK },
	{ .rate = 20,
	  .val = 1,
	  .flags = IEEE80211_RATE_CCK },
	{ .rate = 55,
	  .val = 2,
	  .flags = IEEE80211_RATE_CCK },
	{ .rate = 110,
	  .val = 3,
	  .flags = IEEE80211_RATE_CCK },
	{ .rate = 60,
	  .val = 4,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 90,
	  .val = 5,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 120,
	  .val = 6,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 180,
	  .val = 7,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 240,
	  .val = 8,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 360,
	  .val = 9,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 480,
	  .val = 10,
	  .flags = IEEE80211_RATE_OFDM },
	{ .rate = 540,
	  .val = 11,
	  .flags = IEEE80211_RATE_OFDM },
};

static const struct ieee80211_channel ar5523_channels[] = {
        { .chan = 1,
	  .freq = 2412},
	{ .chan = 2,
	  .freq = 2417},
	{ .chan = 3,
	  .freq = 2422},
	{ .chan = 4,
	  .freq = 2427},
	{ .chan = 5,
	  .freq = 2432},
	{ .chan = 6,
	  .freq = 2437},
	{ .chan = 7,
	  .freq = 2442},
	{ .chan = 8,
	  .freq = 2447},
	{ .chan = 9,
	  .freq = 2452},
	{ .chan = 10,
	  .freq = 2457},
	{ .chan = 11,
	  .freq = 2462},
	{ .chan = 12,
	  .freq = 2467},
	{ .chan = 13,
	  .freq = 2472},
	{ .chan = 14,
	  .freq = 2484},
};

static int ar5523_init_modes(struct ar5523 *ar)
{
	int error;

	memcpy(ar->channels, ar5523_channels, sizeof(ar5523_channels));
	memcpy(ar->rates, ar5523_rates, sizeof(ar5523_rates));

	ar->modes[0].mode	= MODE_IEEE80211G;
	ar->modes[0].num_rates	= ARRAY_SIZE(ar5523_rates);
	ar->modes[0].rates	= ar->rates;
	ar->modes[0].num_channels = ARRAY_SIZE(ar5523_channels);
	ar->modes[0].channels	= ar->channels;

	error = ieee80211_register_hwmode(ar->hw, &ar->modes[0]);
	if (error)
		return error;

	ar->modes[1].mode	= MODE_IEEE80211B;
	ar->modes[1].num_rates	= 4;
	ar->modes[1].rates	= ar->rates;
	ar->modes[1].num_channels = ARRAY_SIZE(ar5523_channels);
	ar->modes[1].channels	= ar->channels;

	error = ieee80211_register_hwmode(ar->hw, &ar->modes[1]);
	if (error)
		return error;

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
	int foolen; // XXX(hch): handle short transfers
	int error = -ENXIO;
	
	if (request_firmware(&fw, AR5523_FIRMWARE_FILE, &dev->dev)) {
		dev_err(&dev->dev, "no firmware found\n");
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
	init_completion(&ar->ready);
	init_waitqueue_head(&ar->tx_cmd_wait);
	atomic_set(&ar->tx_data_queued, 0);

	error = ar5523_alloc_tx_cmds(ar);
	if (error)
		goto out_free_ar;

	error = ar5523_alloc_rx_cmds(ar);
	if (error)
		goto out_free_tx_cmds;

	/*
	 * We're now ready to send/receive firmware commands.
	 */
	error = ar5523_reset(ar);
	if (error) {
		ar5523_err(ar, "could not initialize adapter\n");
		goto out_free_rx_cmds;
	}

	error = ar5523_query_eeprom(ar);
	if (error) {
		ar5523_err(ar, "could not read EEPROM\n");
		goto out_free_rx_cmds;
	}

	ar5523_info(ar, "MAC/BBP AR5523, RF AR%c112\n",
			(id->driver_info & AR5523_FLAG_ABG) ? '5': '2');

	setup_timer(&ar->stat_timer, ar5523_stat, (unsigned long)ar);

	ar->mode = IEEE80211_IF_TYPE_MNTR;

	hw->flags |= IEEE80211_HW_RX_INCLUDES_FCS;
	hw->extra_tx_headroom = sizeof(struct ar5523_tx_desc) + sizeof(__be32);
	hw->queues = 1;

	error = ar5523_init_modes(ar);
	if (error)
		goto out_free_rx_cmds;

	usb_set_intfdata(intf, hw);

	error = ieee80211_register_hw(hw);
	if (error) {
		ar5523_err(ar, "could not register device\n");
		goto out_free_rx_cmds;
	}

	return 0;

 out_free_rx_cmds:
	ar5523_free_rx_cmds(ar);
 out_free_tx_cmds:
	ar5523_free_tx_cmds(ar);
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

	ar5523_free_tx_cmds(ar);
	ar5523_free_rx_cmds(ar);

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
	AR5523_DEVICE_UG(0x16ab, 0x7801),	/* Globalsun / AR5523_1 */
	AR5523_DEVICE_UX(0x16ab, 0x7811),	/* Globalsun / AR5523_2 */
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

MODULE_LICENSE("GPL");

module_init(ar5523_init);
module_exit(ar5523_exit);
