/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT microchip_ksz8851snl

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>
#include "eth.h"

LOG_MODULE_REGISTER(eth_ksz8851snl, CONFIG_ETHERNET_LOG_LEVEL);

#define KS_CCR         0x08
#define KS_CIDER       0xc0
#define KS_MARL        0x10
#define KS_MARH        0x14
#define KS_TXCR        0x70
#define KS_TXSR        0x72
#define KS_RXCR1       0x74
#define KS_RXCR2       0x76
#define KS_TXMIR       0x78
#define KS_MBIR        0x24
#define KS_RXFHSR      0x7c
#define KS_RXFHBCR     0x7e
#define KS_GRR         0x26
#define KS_TXQCR       0x80
#define KS_RXQCR       0x82
#define KS_TXFDPR      0x84
#define KS_RXFDPR      0x86
#define KS_RXDTTR      0x8c
#define KS_RXDBCTR     0x8e
#define KS_IER         0x90
#define KS_ISR         0x92
#define KS_RXFCTR      0x9c
#define KS_PMECR       0xd4
#define KS_P1MBCR      0xe4
#define KS_P1MBSR      0xe6
#define KS_P1CR        0xf6
#define KS_P1SR        0xf8
#define KS_CIDER_ID    0x8870
#define GRR_QMU        BIT(1)
#define MBIR_TXMBF     BIT(12)
#define MBIR_TXMBFA    BIT(11)
#define MBIR_RXMBF     BIT(4)
#define MBIR_RXMBFA    BIT(3)
#define PMECR_PM_MASK  0x0003
#define P1CR_RESTARTAN BIT(13)
#define RXQCR_RXDTTE  BIT(7)
#define RXQCR_RXDBCTE BIT(6)
#define RXQCR_RXFCTE  BIT(5)
#define RXQCR_ADRFE   BIT(4)
/* ADRFE is backup. Linux always releases with RRXEF; a short read does not auto-dequeue. */
#define RXQCR_BASE    (RXQCR_ADRFE | RXQCR_RXFCTE | RXQCR_RXDBCTE | RXQCR_RXDTTE)
#define RXQCR_SDA     BIT(3)
#define RXQCR_RRXEF   BIT(0)
#define RXFDPR_RXFPAI  BIT(14)
#define TXFDPR_TXFPAI  BIT(14)
#define TXQCR_METFE    BIT(0)
#define TXCR_TXFCE     BIT(3)
#define TXCR_TXPE      BIT(2)
#define TXCR_TXCRC     BIT(1)
#define TXCR_TXE       BIT(0)
#define RXCR1_RXPAFMA  BIT(11)
#define RXCR1_RXFCE    BIT(10)
#define RXCR1_RXBE     BIT(7)
#define RXCR1_RXUE     BIT(5)
#define RXCR1_RXE      BIT(0)
#define RXCR2_SRDBL_FRAME (4U << 5)
#define IRQ_LCI        BIT(15)
#define IRQ_TXI        BIT(14)
#define IRQ_RXI        BIT(13)
#define IRQ_RXOI       BIT(11)
#define IRQ_RXPSI      BIT(8)
/* Wake-up IRQs (RXWFDI/RXMPDI/LDI) are not ackable via ISR W1C; enabling
 * them would wedge edge-triggered INTRN. Leave them masked. */
#define KSZ_IRQ_MASK   (IRQ_LCI | IRQ_TXI | IRQ_RXI | IRQ_RXOI | IRQ_RXPSI)
#define RXFSHR_RXFV    BIT(15)
/* Checksum-status bits 13:10 are not errors unless that offload is enabled. */
#define RXFSHR_ERROR   (BIT(4) | BIT(2) | BIT(1) | BIT(0))
#define RXFHBCR_MASK   0x0fff
#define RXFCTR_COUNT(v) (((v) >> 8) & 0xff)
#define P1SR_LINK_GOOD BIT(5)
#define KS_RX_MAX_WIRE 2004U
#define KS_RX_PREFIX   8U
#define KS_FRAME_MAX   (KS_RX_MAX_WIRE + KS_RX_PREFIX)
#define KS_TX_MAX_FRAME 1514U
#define KS_MIN_FRAME   60U
#define KS_TX_MIN      14U
#define KS_TX_WAIT_MS  100
#define KS_WORKER_STACK 1024
/* Exceeds the threshold, so 0 fires on the first frame. 1 would wait for two. */
#define KS_RXFCT_THRESHOLD 0U
#define KS_RXDTTR_US   1000U
#define KS_RXDBCTR_BYTES 4096U

struct ksz_debug {
	uint16_t cider;
	uint16_t ccr;
	uint16_t mbir;
	uint16_t pmecr;
	uint16_t p1mbcr;
	uint16_t p1mbsr;
	uint16_t p1cr;
	uint16_t p1sr;
	uint16_t p1sr_before;
	uint16_t isr;
	uint16_t rxfhsr;
	uint16_t rxfhbcr;
	uint16_t rxfctr;
	uint16_t txqcr;
	uint16_t txsr;
	uint16_t txmir;
	uint8_t mac_wr[6];
	uint8_t mac_rd[6];
	uint32_t tx_timeouts;
	uint32_t rx_drops;
	int last_errno;
};

struct ksz_debug ksz_debug;

struct ksz_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec reset;
	struct net_eth_mac_config mac_cfg;
};

struct ksz_data {
	struct net_if *iface;
	struct gpio_callback gpio_cb;
	struct k_mutex lock;
	struct k_mutex tx_lock;
	struct k_sem event;
	struct k_thread worker;
	K_KERNEL_STACK_MEMBER(stack, KS_WORKER_STACK);
	uint8_t mac[6];
	uint8_t frame[KS_FRAME_MAX];
	uint8_t fid;
	bool started;
	bool carrier;
	bool rx_prefix_logged;
	bool rx_drop_logged;
};

static int reg_read(const struct device *dev, uint8_t reg, uint16_t *value)
{
	const struct ksz_config *cfg = dev->config;
	uint16_t command = (uint16_t)((((reg & 2) ? 0x0c : 0x03) << 2) |
				      ((reg & 0x3f) << 10) | ((reg & 0xc0) >> 6));
	uint8_t tx[4] = { (uint8_t)command, (uint8_t)(command >> 8), 0, 0 };
	uint8_t rx[4];
	struct spi_buf txb = { .buf = tx, .len = sizeof(tx) };
	struct spi_buf rxb = { .buf = rx, .len = sizeof(rx) };
	struct spi_buf_set txset = { .buffers = &txb, .count = 1 };
	struct spi_buf_set rxset = { .buffers = &rxb, .count = 1 };
	int ret = spi_transceive_dt(&cfg->spi, &txset, &rxset);

	if (ret == 0) {
		*value = sys_get_le16(&rx[2]);
	}
	return ret;
}

static int reg_write(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct ksz_config *cfg = dev->config;
	uint16_t command = (uint16_t)(0x40 | (((reg & 2) ? 0x0c : 0x03) << 2) |
				      ((reg & 0x3f) << 10) | ((reg & 0xc0) >> 6));
	uint8_t tx[4] = { (uint8_t)command, (uint8_t)(command >> 8),
			  (uint8_t)value, (uint8_t)(value >> 8) };
	struct spi_buf b = { .buf = tx, .len = sizeof(tx) };
	struct spi_buf_set s = { .buffers = &b, .count = 1 };

	return spi_write_dt(&cfg->spi, &s);
}

static void debug_note(int err)
{
	if (err < 0) {
		ksz_debug.last_errno = err;
	}
}

static void debug_capture_locked(const struct device *dev)
{
	uint16_t v;

	if (reg_read(dev, KS_CIDER, &v) == 0) {
		ksz_debug.cider = v;
	}
	if (reg_read(dev, KS_CCR, &v) == 0) {
		ksz_debug.ccr = v;
	}
	if (reg_read(dev, KS_MBIR, &v) == 0) {
		ksz_debug.mbir = v;
	}
	if (reg_read(dev, KS_PMECR, &v) == 0) {
		ksz_debug.pmecr = v;
	}
	if (reg_read(dev, KS_P1MBCR, &v) == 0) {
		ksz_debug.p1mbcr = v;
	}
	if (reg_read(dev, KS_P1MBSR, &v) == 0) {
		ksz_debug.p1mbsr = v;
	}
	if (reg_read(dev, KS_P1CR, &v) == 0) {
		ksz_debug.p1cr = v;
	}
	if (reg_read(dev, KS_P1SR, &v) == 0) {
		ksz_debug.p1sr = v;
	}
	if (reg_read(dev, KS_ISR, &v) == 0) {
		ksz_debug.isr = v;
	}
	if (reg_read(dev, KS_RXFCTR, &v) == 0) {
		ksz_debug.rxfctr = v;
	}
}

static int force_pm_normal_locked(const struct device *dev)
{
	uint16_t pmecr;
	int ret = reg_read(dev, KS_PMECR, &pmecr);

	if (ret < 0) {
		return ret;
	}
	pmecr &= (uint16_t)~PMECR_PM_MASK;
	ret = reg_write(dev, KS_PMECR, pmecr);
	if (ret == 0) {
		ksz_debug.pmecr = pmecr;
	}
	return ret;
}

static int write_mac_locked(const struct device *dev, const uint8_t mac[6])
{
	int ret = force_pm_normal_locked(dev);

	if (ret == 0) {
		ret = reg_write(dev, KS_MARH, ((uint16_t)mac[0] << 8) | mac[1]);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_MARH - 2, ((uint16_t)mac[2] << 8) | mac[3]);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_MARL, ((uint16_t)mac[4] << 8) | mac[5]);
	}
	if (ret == 0) {
		memcpy(ksz_debug.mac_wr, mac, sizeof(ksz_debug.mac_wr));
	}
	return ret;
}

static int read_mac_locked(const struct device *dev, uint8_t mac[6])
{
	uint16_t hi;
	uint16_t mid;
	uint16_t lo;
	int ret = reg_read(dev, KS_MARH, &hi);

	if (ret == 0) {
		ret = reg_read(dev, KS_MARH - 2, &mid);
	}
	if (ret == 0) {
		ret = reg_read(dev, KS_MARL, &lo);
	}
	if (ret < 0) {
		return ret;
	}
	mac[0] = (uint8_t)(hi >> 8);
	mac[1] = (uint8_t)hi;
	mac[2] = (uint8_t)(mid >> 8);
	mac[3] = (uint8_t)mid;
	mac[4] = (uint8_t)(lo >> 8);
	mac[5] = (uint8_t)lo;
	memcpy(ksz_debug.mac_rd, mac, sizeof(ksz_debug.mac_rd));
	return 0;
}

static int fifo_read(const struct device *dev, uint8_t *dst, size_t len)
{
	const struct ksz_config *cfg = dev->config;
	uint8_t opcode = 0x80;
	struct spi_buf txb[2] = { { .buf = &opcode, .len = 1 }, { .buf = NULL, .len = len } };
	struct spi_buf rxb[2] = { { .buf = NULL, .len = 1 }, { .buf = dst, .len = len } };
	struct spi_buf_set txs = { .buffers = txb, .count = 2 };
	struct spi_buf_set rxs = { .buffers = rxb, .count = 2 };

	return spi_transceive_dt(&cfg->spi, &txs, &rxs);
}

static int fifo_write(const struct device *dev, const uint8_t *frame, size_t len)
{
	const struct ksz_config *cfg = dev->config;
	struct ksz_data *data = dev->data;
	uint8_t hdr[5];
	uint16_t ctrl = (uint16_t)((data->fid++ & 0x3f) | BIT(15));
	uint16_t count = (uint16_t)len;
	uint8_t zero[3] = { 0 };
	size_t pad = ROUND_UP(len, 4) - len;
	struct spi_buf txb[3];
	struct spi_buf_set txs = { .buffers = txb, .count = 3 };

	txs.count = pad == 0 ? 2 : 3;
	hdr[0] = 0xc0;
	hdr[1] = (uint8_t)ctrl;
	hdr[2] = (uint8_t)(ctrl >> 8);
	hdr[3] = (uint8_t)count;
	hdr[4] = (uint8_t)(count >> 8);
	txb[0] = (struct spi_buf){ .buf = hdr, .len = sizeof(hdr) };
	txb[1] = (struct spi_buf){ .buf = (void *)frame, .len = len };
	txb[2] = (struct spi_buf){ .buf = zero, .len = pad };
	return spi_write_dt(&cfg->spi, &txs);
}

static int update_link_locked(const struct device *dev)
{
	struct ksz_data *data = dev->data;
	uint16_t status;
	int ret = reg_read(dev, KS_P1SR, &status);

	if (ret < 0) {
		return ret;
	}
	ksz_debug.p1sr = status;
	bool up = (status & P1SR_LINK_GOOD) != 0;

	if (up != data->carrier) {
		data->carrier = up;
		debug_capture_locked(dev);
		LOG_INF("link %s p1sr=0x%04x p1mbsr=0x%04x p1cr=0x%04x",
			up ? "up" : "down", ksz_debug.p1sr, ksz_debug.p1mbsr,
			ksz_debug.p1cr);
		if (up) {
			net_eth_carrier_on(data->iface);
		} else {
			net_eth_carrier_off(data->iface);
		}
	}
	return 0;
}

static void rx_drop(struct ksz_data *data, uint16_t stat, uint16_t wire_len)
{
	ksz_debug.rx_drops++;
	ksz_debug.rxfhsr = stat;
	ksz_debug.rxfhbcr = wire_len;
	eth_stats_update_errors_rx(data->iface);
	if (!data->rx_drop_logged) {
		data->rx_drop_logged = true;
		LOG_WRN("rx drop stat=0x%04x len=%u", stat, wire_len);
	}
}

static int rx_drain(const struct device *dev)
{
	struct ksz_data *data = dev->data;
	uint16_t ctr;
	int ret = reg_read(dev, KS_RXFCTR, &ctr);

	if (ret < 0) {
		return ret;
	}
	ksz_debug.rxfctr = ctr;
	unsigned int count = RXFCTR_COUNT(ctr);

	if (count == 0U) {
		return 0;
	}
	unsigned int limit = count;

	if (limit > 8U) {
		limit = 8U;
	}
	for (unsigned int n = 0; n < limit; n++) {
		uint16_t stat;
		uint16_t wire_len;

		ret = reg_read(dev, KS_RXFHSR, &stat);
		if (ret < 0) {
			return ret;
		}
		ret = reg_read(dev, KS_RXFHBCR, &wire_len);
		if (ret < 0) {
			return ret;
		}
		wire_len &= RXFHBCR_MASK;
		ksz_debug.rxfhsr = stat;
		ksz_debug.rxfhbcr = wire_len;

		ret = reg_write(dev, KS_RXFDPR, RXFDPR_RXFPAI);
		if (ret < 0) {
			return ret;
		}
		ret = reg_write(dev, KS_RXQCR, RXQCR_BASE | RXQCR_SDA);
		if (ret < 0) {
			return ret;
		}

		struct net_pkt *pkt = NULL;
		bool readable = (stat & RXFSHR_RXFV) != 0 && wire_len >= 4U &&
				wire_len <= KS_RX_MAX_WIRE;
		size_t frame_len = readable ? (size_t)wire_len - 4U : 0U;
		size_t transfer_len = readable ? ROUND_UP(wire_len, 4) + KS_RX_PREFIX : 0U;
		bool deliver = readable && (stat & RXFSHR_ERROR) == 0 &&
			       wire_len >= KS_MIN_FRAME + 4U &&
			       frame_len <= KS_TX_MAX_FRAME &&
			       transfer_len <= sizeof(data->frame);

		if (readable && transfer_len <= sizeof(data->frame)) {
			ret = fifo_read(dev, data->frame, transfer_len);
			if (ret == 0 && !data->rx_prefix_logged) {
				data->rx_prefix_logged = true;
				LOG_HEXDUMP_INF(data->frame, KS_RX_PREFIX + 8U, "rx prefix");
				LOG_INF("rx stat=0x%04x wire=%u", stat, wire_len);
			}
			if (ret == 0 && deliver) {
				pkt = net_pkt_rx_alloc_with_buffer(data->iface, frame_len,
								   NET_AF_UNSPEC, 0, K_MSEC(50));
				if (pkt == NULL) {
					rx_drop(data, stat, wire_len);
				} else if (net_pkt_write(pkt, data->frame + KS_RX_PREFIX,
							 frame_len) < 0) {
					net_pkt_unref(pkt);
					pkt = NULL;
					rx_drop(data, stat, wire_len);
				}
			} else if (ret == 0) {
				rx_drop(data, stat, wire_len);
			}
		} else {
			ret = 0;
			rx_drop(data, stat, wire_len);
		}

		int qret = reg_write(dev, KS_RXQCR, RXQCR_BASE | RXQCR_RRXEF);

		if (ret == 0) {
			ret = qret;
		}
		if (ret < 0) {
			if (pkt != NULL) {
				net_pkt_unref(pkt);
			}
			return ret;
		}
		if (pkt != NULL) {
			k_mutex_unlock(&data->lock);
			ret = net_recv_data(data->iface, pkt);
			if (ret < 0) {
				net_pkt_unref(pkt);
				rx_drop(data, stat, wire_len);
			} else {
				eth_stats_update_bytes_rx(data->iface, frame_len);
				eth_stats_update_pkts_rx(data->iface);
			}
			k_mutex_lock(&data->lock, K_FOREVER);
			if (ret < 0) {
				return ret;
			}
		}
	}
	if (count > limit) {
		k_sem_give(&data->event);
	}
	return 0;
}

static int wait_metfe_clear(const struct device *dev)
{
	struct ksz_data *data = dev->data;
	int64_t deadline = k_uptime_get() + KS_TX_WAIT_MS;

	for (;;) {
		uint16_t txqcr = 0;
		int ret;

		k_mutex_lock(&data->lock, K_FOREVER);
		ret = reg_read(dev, KS_TXQCR, &txqcr);
		if (ret < 0 || (txqcr & TXQCR_METFE) == 0) {
			ksz_debug.txqcr = txqcr;
			k_mutex_unlock(&data->lock);
			return ret;
		}
		if (k_uptime_get() >= deadline) {
			uint16_t v;

			ksz_debug.txqcr = txqcr;
			if (reg_read(dev, KS_TXSR, &v) == 0) {
				ksz_debug.txsr = v;
			}
			if (reg_read(dev, KS_ISR, &v) == 0) {
				ksz_debug.isr = v;
			}
			if (reg_read(dev, KS_TXMIR, &v) == 0) {
				ksz_debug.txmir = v;
			}
			ksz_debug.tx_timeouts++;
			ksz_debug.last_errno = -ETIMEDOUT;
			k_mutex_unlock(&data->lock);
			LOG_ERR("tx timeout txqcr=0x%04x txsr=0x%04x isr=0x%04x txmir=0x%04x",
				ksz_debug.txqcr, ksz_debug.txsr, ksz_debug.isr, ksz_debug.txmir);
			return -ETIMEDOUT;
		}
		k_mutex_unlock(&data->lock);
		k_msleep(1);
	}
}

static void worker(void *a, void *b, void *c)
{
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	const struct device *dev = a;
	struct ksz_data *data = dev->data;

	while (true) {
		(void)k_sem_take(&data->event, K_MSEC(100));
		if (!data->started) {
			continue;
		}
		k_mutex_lock(&data->lock, K_FOREVER);
		uint16_t irq = 0;
		int ret = reg_read(dev, KS_ISR, &irq);
		uint16_t handled = irq & KSZ_IRQ_MASK;

		ksz_debug.isr = irq;
		/* Poll reads ISR, so a missed edge on level-low INTRN still drains. */
		if (ret == 0 && handled != 0) {
			ret = reg_write(dev, KS_ISR, handled);
		}
		if (ret == 0 && (handled & (IRQ_RXI | IRQ_RXOI | IRQ_RXPSI)) != 0) {
			ret = rx_drain(dev);
		}
		if (ret == 0) {
			ret = update_link_locked(dev);
		}
		if (ret < 0) {
			debug_note(ret);
			LOG_ERR("KSZ service failed: %d", ret);
		}
		k_mutex_unlock(&data->lock);
	}
}

static void irq_cb(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	struct ksz_data *data = CONTAINER_OF(cb, struct ksz_data, gpio_cb);

	k_sem_give(&data->event);
}

static int start(const struct device *dev, struct net_if *iface)
{
	struct ksz_data *data = dev->data;
	uint8_t mac_rd[6];
	uint16_t p1cr;
	uint16_t p1sr;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = force_pm_normal_locked(dev);
	if (ret == 0) {
		ret = reg_write(dev, KS_GRR, GRR_QMU);
	}
	if (ret == 0) {
		k_sleep(K_MSEC(1));
		ret = reg_write(dev, KS_GRR, 0);
	}
	if (ret == 0) {
		k_sleep(K_MSEC(1));
		ret = reg_write(dev, KS_TXCR, TXCR_TXFCE | TXCR_TXPE | TXCR_TXCRC | TXCR_TXE);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXCR1, RXCR1_RXPAFMA | RXCR1_RXFCE | RXCR1_RXBE |
				RXCR1_RXUE | RXCR1_RXE);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXCR2, RXCR2_SRDBL_FRAME);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXDTTR, KS_RXDTTR_US);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXDBCTR, KS_RXDBCTR_BYTES);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXFCTR, KS_RXFCT_THRESHOLD);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXQCR, RXQCR_BASE);
	}
	if (ret == 0) {
		ret = write_mac_locked(dev, data->mac);
	}
	if (ret == 0) {
		ret = read_mac_locked(dev, mac_rd);
	}
	if (ret == 0 && memcmp(mac_rd, data->mac, sizeof(mac_rd)) != 0) {
		LOG_ERR("MAC readback mismatch");
		debug_note(-EIO);
	}
	if (ret == 0) {
		ret = reg_read(dev, KS_P1SR, &p1sr);
	}
	if (ret == 0) {
		ksz_debug.p1sr_before = p1sr;
		ret = reg_read(dev, KS_P1CR, &p1cr);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_P1CR, p1cr | P1CR_RESTARTAN);
	}
	if (ret == 0) {
		ret = reg_read(dev, KS_P1SR, &p1sr);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_ISR, 0xffff);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_IER, KSZ_IRQ_MASK);
	}
	if (ret == 0) {
		data->iface = iface;
		data->started = true;
		debug_capture_locked(dev);
		LOG_INF("up cider=0x%04x pmecr=0x%04x p1cr=0x%04x p1sr 0x%04x->0x%04x mac %s",
			ksz_debug.cider, ksz_debug.pmecr, ksz_debug.p1cr,
			ksz_debug.p1sr_before, ksz_debug.p1sr,
			memcmp(ksz_debug.mac_rd, ksz_debug.mac_wr, 6) == 0 ? "ok" : "mismatch");
	}
	k_mutex_unlock(&data->lock);
	if (ret == 0) {
		k_sem_give(&data->event);
	} else {
		debug_note(ret);
	}
	return ret;
}

static int stop(const struct device *dev, struct net_if *iface)
{
	struct ksz_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->started = false;
	int ret = reg_write(dev, KS_IER, 0);

	if (ret == 0) {
		ret = reg_write(dev, KS_TXCR, 0);
	}
	if (ret == 0) {
		ret = reg_write(dev, KS_RXCR1, 0);
	}
	data->carrier = false;
	net_eth_carrier_off(iface);
	k_mutex_unlock(&data->lock);
	return ret;
}

static int send(const struct device *dev, struct net_pkt *pkt)
{
	struct ksz_data *data = dev->data;
	size_t len = net_pkt_get_len(pkt);
	int ret;

	if (len < KS_TX_MIN || len > KS_TX_MAX_FRAME) {
		return -EMSGSIZE;
	}
	k_mutex_lock(&data->tx_lock, K_FOREVER);
	ret = wait_metfe_clear(dev);
	if (ret < 0) {
		k_mutex_unlock(&data->tx_lock);
		return ret;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	if (net_pkt_read(pkt, data->frame, len) < 0) {
		ret = -EIO;
	} else {
		uint16_t free;

		ret = reg_read(dev, KS_TXMIR, &free);
		size_t required = ROUND_UP(len + 4, 4);

		if (ret == 0 && (free & 0x1fff) < required) {
			ret = -ENOBUFS;
		}
		if (ret == 0) {
			ret = reg_write(dev, KS_TXFDPR, TXFDPR_TXFPAI);
		}
		if (ret == 0) {
			ret = reg_write(dev, KS_RXQCR, RXQCR_BASE | RXQCR_SDA);
		}
		bool qmu_started = (ret == 0);

		if (ret == 0) {
			ret = fifo_write(dev, data->frame, len);
		}
		if (qmu_started) {
			int qret = reg_write(dev, KS_RXQCR, RXQCR_BASE);

			if (ret == 0) {
				ret = qret;
			}
		}
		if (ret == 0) {
			ret = reg_write(dev, KS_TXQCR, TXQCR_METFE);
		}
	}
	k_mutex_unlock(&data->lock);
	if (ret == 0) {
		ret = wait_metfe_clear(dev);
	}
	if (ret == 0) {
		eth_stats_update_bytes_tx(data->iface, len);
		eth_stats_update_pkts_tx(data->iface);
	} else {
		debug_note(ret);
	}
	k_mutex_unlock(&data->tx_lock);
	return ret;
}

static int set_config(const struct device *dev, struct net_if *iface,
		      enum ethernet_config_type type, const struct ethernet_config *config)
{
	ARG_UNUSED(iface);
	if (type != ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		return -ENOTSUP;
	}
	struct ksz_data *data = dev->data;

	memcpy(data->mac, config->mac_address.addr, sizeof(data->mac));
	k_mutex_lock(&data->lock, K_FOREVER);
	int ret = write_mac_locked(dev, data->mac);

	k_mutex_unlock(&data->lock);
	return ret;
}

static enum ethernet_hw_caps caps(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static void iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct ksz_data *data = dev->data;

	net_if_set_link_addr(iface, data->mac, sizeof(data->mac), NET_LINK_ETHERNET);
	data->iface = iface;
	ethernet_init(iface);
	net_if_carrier_off(iface);
}

static const struct ethernet_api api = {
	.iface_api.init = iface_init,
	.get_capabilities = caps,
	.set_config = set_config,
	.start = start,
	.stop = stop,
	.send = send,
};

static int init(const struct device *dev)
{
	const struct ksz_config *cfg = dev->config;
	struct ksz_data *data = dev->data;
	uint16_t id = 0;
	int ret;

	k_mutex_init(&data->lock);
	k_mutex_init(&data->tx_lock);
	k_sem_init(&data->event, 0, 1);
	if (!spi_is_ready_dt(&cfg->spi)) {
		return -ENODEV;
	}
	bool has_irq = cfg->irq.port != NULL;
	bool has_reset = cfg->reset.port != NULL;

	if (has_irq) {
		if (!gpio_is_ready_dt(&cfg->irq)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}
	if (has_reset) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return ret;
		}
		k_busy_wait(10000);
		/* PA28 is shared with SW0; actively drive reset released instead of
		 * relying on a pull-up that may not hold the line high. */
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
		k_busy_wait(1000);
	}
	for (unsigned int attempt = 0; attempt < 100; attempt++) {
		ret = reg_read(dev, KS_CIDER, &id);
		if (ret == 0 && (id & 0xfff0) == KS_CIDER_ID) {
			break;
		}
		k_busy_wait(1000);
	}
	ksz_debug.cider = id;
	if (ret < 0) {
		debug_note(ret);
		return ret;
	}
	if ((id & 0xfff0) != KS_CIDER_ID) {
		debug_note(-ENODEV);
		return -ENODEV;
	}
	uint16_t mbir = 0;

	for (unsigned int attempt = 0; attempt < 100; attempt++) {
		ret = reg_read(dev, KS_MBIR, &mbir);
		if (ret < 0) {
			debug_note(ret);
			return ret;
		}
		if ((mbir & (MBIR_TXMBFA | MBIR_RXMBFA)) != 0) {
			ksz_debug.mbir = mbir;
			debug_note(-EIO);
			return -EIO;
		}
		if ((mbir & (MBIR_TXMBF | MBIR_RXMBF)) == (MBIR_TXMBF | MBIR_RXMBF)) {
			break;
		}
		k_busy_wait(1000);
	}
	ksz_debug.mbir = mbir;
	if ((mbir & (MBIR_TXMBF | MBIR_RXMBF)) != (MBIR_TXMBF | MBIR_RXMBF)) {
		debug_note(-ETIMEDOUT);
		return -ETIMEDOUT;
	}
	ret = net_eth_mac_load(&cfg->mac_cfg, data->mac);
	if (ret < 0) {
		return ret;
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	ret = write_mac_locked(dev, data->mac);
	if (ret == 0) {
		uint8_t mac_rd[6];

		ret = read_mac_locked(dev, mac_rd);
		if (ret == 0 && memcmp(mac_rd, data->mac, sizeof(mac_rd)) != 0) {
			LOG_ERR("MAC readback mismatch");
			debug_note(-EIO);
		}
	}
	if (ret == 0) {
		debug_capture_locked(dev);
		LOG_INF("cid=0x%04x ccr=0x%04x mbir=0x%04x pmecr=0x%04x",
			ksz_debug.cider, ksz_debug.ccr, ksz_debug.mbir, ksz_debug.pmecr);
		LOG_INF("mac %02x:%02x:%02x:%02x:%02x:%02x rd %02x:%02x:%02x:%02x:%02x:%02x",
			ksz_debug.mac_wr[0], ksz_debug.mac_wr[1], ksz_debug.mac_wr[2],
			ksz_debug.mac_wr[3], ksz_debug.mac_wr[4], ksz_debug.mac_wr[5],
			ksz_debug.mac_rd[0], ksz_debug.mac_rd[1], ksz_debug.mac_rd[2],
			ksz_debug.mac_rd[3], ksz_debug.mac_rd[4], ksz_debug.mac_rd[5]);
	}
	k_mutex_unlock(&data->lock);
	if (ret < 0) {
		debug_note(ret);
		return ret;
	}
	if (has_irq) {
		gpio_init_callback(&data->gpio_cb, irq_cb, BIT(cfg->irq.pin));
		ret = gpio_add_callback(cfg->irq.port, &data->gpio_cb);
		if (ret < 0) {
			return ret;
		}
	}
	k_thread_create(&data->worker, data->stack, K_KERNEL_STACK_SIZEOF(data->stack), worker,
			(void *)dev, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->worker, "ksz8851");
	if (has_irq) {
		ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}

#define KSZ_INST(inst) \
	static struct ksz_data data_##inst; \
	static const struct ksz_config config_##inst = { \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB), \
		.irq = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(inst), \
	}; \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, init, NULL, &data_##inst, &config_##inst, \
		CONFIG_ETH_INIT_PRIORITY, &api, NET_ETH_MTU)

DT_INST_FOREACH_STATUS_OKAY(KSZ_INST)
