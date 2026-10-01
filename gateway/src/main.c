/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Gateway bring-up image for samr21_xpro.
 *
 * Raw-socket 802.15.4 receiver: the radio is brought up promiscuous on
 * channel 15 with our PAN ID (0xCAFE) set in the hardware filter. Frames whose
 * MAC payload is
 * exactly the 18-byte sensor payload decode via sensor_frame_decode() and
 * are printed to the console; anything else is just counted.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/ieee802154.h>
#include <zephyr/net/ieee802154_mgmt.h>
#include <zephyr/sys/atomic.h>
#include "radio_socket.h"

#include <app/lib/sensor_frame.h>

#include <zephyr/app_version.h>

LOG_MODULE_REGISTER(gateway, CONFIG_GATEWAY_LOG_LEVEL);

#define GW_CHANNEL 15
#define GW_PAN_ID  0xCAFE

static const struct device *const radio =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

static atomic_t rx_ok;
static atomic_t rx_other;
static struct radio_socket radio_sock = { .fd = -1 };
static K_SEM_DEFINE(rx_ready, 0, 1);

/*
 * Socket copies the complete frame (including MAC header) out of all packet
 * fragments. Decode/print cannot block the radio driver's RX thread.
 */
static void receive_frames(void *p1, void *p2, void *p3)
{
	struct sensor_reading r;
	uint8_t psdu[RADIO_SOCKET_FRAME_MAX];

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	k_sem_take(&rx_ready, K_FOREVER);
	while (true) {
		int len = radio_socket_recv(&radio_sock, psdu, sizeof(psdu), 0);

		if (len < 0 && len != -EMSGSIZE) {
			LOG_ERR("Raw socket receive failed (%d)", len);
			k_sleep(K_MSEC(100));
			continue;
		}
		if (len >= 0 && sensor_frame_decode(psdu, len, &r) == 0) {
			atomic_inc(&rx_ok);
			printk("rx node=%u seq=%u light=%u temp_c_x100=%d "
			       "accel=%d,%d,%d uptime=%u flags=%u\n",
			       r.node_id, r.seq, r.light, r.temp_c_x100,
			       r.accel[0], r.accel[1], r.accel[2],
			       r.uptime_ms, r.flags);
		} else {
			atomic_inc(&rx_other);
		}
	}
}

K_THREAD_DEFINE(radio_rx_id, 2048, receive_frames, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
	const struct ieee802154_radio_api *api;
	struct net_if *iface;
	uint16_t channel = GW_CHANNEL;
	int ret;

	printk("Gateway %s (samr21_xpro, ch %d, pan 0x%04x, promiscuous)\n",
	       APP_VERSION_STRING, GW_CHANNEL, GW_PAN_ID);

	if (!device_is_ready(radio)) {
		LOG_ERR("Radio not ready");
		return 0;
	}

	api = (const struct ieee802154_radio_api *)radio->api;
	iface = net_if_lookup_by_dev(radio);
	ret = radio_socket_open(&radio_sock, iface);
	if (ret < 0) {
		LOG_ERR("Raw socket open failed (%d)", ret);
		return 0;
	}
	/* Set both L2 state and hardware: L2 refuses to start without a channel. */
	ret = net_mgmt(NET_REQUEST_IEEE802154_SET_CHANNEL, iface, &channel, sizeof(channel));
	if (ret < 0) {
		LOG_ERR("set_channel(%d) failed (%d)", GW_CHANNEL, ret);
		radio_socket_close(&radio_sock);
		return 0;
	}

	/* Our PAN ID, in the hardware filter. Promiscuous mode below accepts
	 * everything regardless for now; the ID is set so the filter is ready
	 * when we stop being promiscuous.
	 */
	{
		struct ieee802154_filter f = { .pan_id = GW_PAN_ID };

		ret = api->filter(radio, true, IEEE802154_FILTER_TYPE_PAN_ID, &f);
		if (ret < 0) {
			LOG_WRN("PAN ID filter refused (%d), continuing", ret);
		}
	}

	/* Promiscuous for now: accept any on-channel frame. */
	{
		struct ieee802154_config cfg = { .promiscuous = true };

		ret = api->configure(radio, IEEE802154_CONFIG_PROMISCUOUS, &cfg);
		if (ret < 0) {
			LOG_WRN("Promiscuous mode refused (%d), using default filter",
				ret);
		}
	}

	ret = net_if_up(iface);
	if (ret < 0) {
		LOG_ERR("Radio interface up failed (%d)", ret);
		radio_socket_close(&radio_sock);
		return 0;
	}
	k_sem_give(&rx_ready);

	LOG_INF("Listening promiscuous on ch %d, PAN ID 0x%04x set", GW_CHANNEL, GW_PAN_ID);

	while (1) {
		k_sleep(K_MSEC(5000));
		LOG_INF("gw ok=%u other=%u", (unsigned int)atomic_get(&rx_ok),
			(unsigned int)atomic_get(&rx_other));
	}

	return 0;
}
