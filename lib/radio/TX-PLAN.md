# TX-PLAN — transmitting sensor frames from the SAM R21 (AT86RF233)

Plan for getting the 27-byte sensor PSDU produced by
`lib/radio/sensor_frame.c` onto the air from the SAM R21's on-board
AT86RF233, as the **sensor side** of the link described in `gateway/AGENTS.md`.

- Companion document: **`RX-PLAN.md`** (gateway receive + decode + table).
- Source of the receive-side context: `gateway/radio-plan.md`.
- Codec contract: `include/app/lib/sensor_frame.h`.
- Codec tests: `tests/lib/radio/` (16 cases, all passing on `unit_testing`).
- Datasheet: `datasheets/Atmel-42223–SAM-R21_Datasheet.pdf` (AT86RF233
  integrated radio).

Both ends of the link are SAM R21 boards with the same AT86RF233, so there is
one radio driver (`rf2xx`) and no board-diversity branch anywhere in this plan.

---

## 0. Goal & scope

Deliver, in order of dependency:

1. A Zephyr glue layer that turns a `struct sensor_reading` into a transmitted
   802.15.4 data frame using the in-tree `rf2xx` driver.
2. A reporting loop in the sensor application (`app/`) that samples, encodes,
   and transmits periodically.
3. Configuration knobs for channel, PAN, TX power and MAC sequence.
4. Off-hardware tests of the glue by mocking the radio API.

Out of scope: 802.15.4 security, 6LoWPAN/IPv6, replying to sensors, and the
receive path (see `RX-PLAN.md`).

---

## 1. What we hand to the radio

`sensor_frame_encode()` already produces the complete **PSDU** — a 9-byte MAC
header plus the 18-byte payload, 27 bytes, **without FCS**. Everything below
just moves those 27 bytes to the radio.

Datasheet ground truth (SAM R21 datasheet, AT86RF233):

- **§35.3.2 Frame Buffer Access Mode** — a write access is
  `command byte + PHR + PSDU`; PHR is the frame length *including* the 2-byte
  FCS. Max frame length is 127, so the SPI burst is `2 + frame_length` ≤ 129.
- **§36.1.2.8 BUSY_TX** — transmission starts from `PLL_ON` by a **SLP_TR rising
  edge** or a **`TX_START`** command; the chip generates the preamble/SFD and
  the **FCS** itself, transmits the frame-buffer contents, raises
  `IRQ_3 (TRX_END)`, and returns to `PLL_ON`.
- **§36.2 Extended Operating Mode** — `TX_ARET_ON` adds hardware **CSMA-CA,
  ACK wait and automatic retransmission**, reporting the result in
  `TRAC_STATUS`.

Consequence: write 27 bytes; the chip adds SHR + FCS. **Never include the FCS
ourselves** — handing the driver 29 bytes would make the chip put 31 on air.

---

## 2. What the `rf2xx` driver already does

The in-tree driver owns the whole datasheet sequence:

- `drivers/ieee802154/ieee802154_rf2xx.c:645` `rf2xx_tx()` —
  configures the CSMA/retry registers per `enum ieee802154_tx_mode`, enters
  `TX_ARET_ON` (`rf2xx_trx_set_tx_state()`, line 104), writes the frame, pulses
  SLP_TR, blocks on `trx_tx_sync`, and maps `TRAC_STATUS` to errno:

  | `TRAC_STATUS` | errno |
  |---|---|
  | channel access failed | `-EBUSY` |
  | no ACK received | `-EAGAIN` |
  | transaction not finished | `-EINTR` |
  | success (ACK or no-ACK requested) | `0` |

- `drivers/ieee802154/ieee802154_rf2xx_iface.c:200`
  `rf2xx_iface_frame_write()` — writes `PHR = len + 2` then the PSDU, exactly
  §35.3.2.
- `drivers/ieee802154/ieee802154_rf2xx_iface.c:42`
  `rf2xx_iface_phy_tx_start()` — the SLP_TR pulse of §36.1.2.8.
- `drivers/ieee802154/ieee802154_rf2xx.c:1082` `rf2xx_radio_api` exposes
  `.tx`, `.set_channel`, `.set_txpower`, `.start`, `.stop`, `.cca`.

TX modes we care about (`include/zephyr/net/ieee802154_radio.h:601`):

| Mode | Hardware behaviour | Use |
|---|---|---|
| `IEEE802154_TX_MODE_DIRECT` | no CSMA, no retries | bench / timing tests |
| `IEEE802154_TX_MODE_CCA` | single CCA, no retries | bench |
| `IEEE802154_TX_MODE_CSMA_CA` | CSMA-CA + retries (+ ACK wait if AR set) | **production** |

Reference implementation for a direct, L2-free call:
`samples/net/wpan_serial/src/main.c:263`

```c
radio_api->tx(ieee802154_dev, IEEE802154_TX_MODE_DIRECT, pkt, buf);
```

with, at lines 472/511/516:

```c
radio_api = (struct ieee802154_radio_api *)ieee802154_dev->api;
radio_api->set_channel(ieee802154_dev, chan);
radio_api->start(ieee802154_dev);
```

---

## 3. Decision: raw mode on the sensor

`RX-PLAN.md` §1 explains why the gateway must use a **custom 802.15.4 L2**:
`CONFIG_IEEE802154_RAW_MODE` is a *global* net-stack switch that compiles only
`net_pkt.c` and deletes IPv4/ARP/TCP/sockets, which the gateway needs for
Ethernet + HTTP.

The sensor does **not** need IP. It only transmits 27 bytes. So the sensor uses
**raw mode** and calls the radio API directly:

| | Sensor (TX) | Gateway (RX) |
|---|---|---|
| L2 | **raw mode** (`CONFIG_IEEE802154_RAW_MODE`) | custom L2 (`RX-PLAN.md` §1–2) |
| Sends via | `ieee802154_radio_api.tx()` directly | n/a |
| rf2xx patch needed | **none** — raw device init already exists | ~4 lines (`RX-PLAN.md` §2) |
| Shared code | `lib/radio/sensor_frame.c` | same |

In raw mode the driver registers a plain `DEVICE_DT_INST_DEFINE`
(`ieee802154_rf2xx.c:1146` `IEEE802154_RF2XX_RAW_DEVICE_INIT`), so `dev->api` is
the `ieee802154_radio_api` and there is no `net_if`/L2 in the path — exactly the
`wpan_serial` shape.

---

## 4. New code

### 4.1 `include/app/lib/sensor_tx.h`

```c
int sensor_tx_init(void);
int sensor_tx_send(struct sensor_frame_cfg *cfg,
                   const struct sensor_reading *reading);
```

### 4.2 `lib/radio/sensor_tx.c` (Zephyr glue)

Compiled into `lib/radio` under `CONFIG_RADIO` and `CONFIG_IEEE802154_RAW_MODE`;
the pure codec stays in `sensor_frame.c`.

```c
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ieee802154_radio.h>
#include <app/lib/sensor_frame.h>

static const struct device *const radio =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));
static const struct ieee802154_radio_api *api;

int sensor_tx_init(void)
{
        if (!device_is_ready(radio)) {
                return -ENODEV;
        }

        api = (const struct ieee802154_radio_api *)radio->api;
        api->set_channel(radio, CONFIG_RADIO_TX_CHANNEL);
        api->set_txpower(radio, CONFIG_RADIO_TX_POWER);

        return api->start(radio);
}

int sensor_tx_send(struct sensor_frame_cfg *cfg,
                   const struct sensor_reading *reading)
{
        uint8_t psdu[SENSOR_FRAME_LEN];
        struct net_pkt *pkt;
        int ret;

        ret = sensor_frame_encode(psdu, sizeof(psdu), cfg, reading);
        if (ret < 0) {
                return ret;
        }

        pkt = net_pkt_alloc_with_buffer(NULL, SENSOR_FRAME_LEN,
                                        NET_AF_UNSPEC, 0, K_NO_WAIT);
        if (pkt == NULL) {
                return -ENOMEM;
        }

        if (net_pkt_write(pkt, psdu, SENSOR_FRAME_LEN) != 0) {
                net_pkt_unref(pkt);
                return -ENOMEM;
        }

        ret = api->tx(radio, IEEE802154_TX_MODE_CSMA_CA, pkt,
                      net_buf_frag_last(pkt->buffer));
        net_pkt_unref(pkt);

        return ret;  /* 0 ok; -EBUSY / -EAGAIN -> backoff and retry */
}
```

### 4.3 `app/src/main.c` — reporting loop

Sample the sensors, fill a `struct sensor_reading`, then transmit:

```c
static struct sensor_frame_cfg cfg = {
        .pan_id = CONFIG_RADIO_TX_PAN_ID,
        .dst_short_addr = 0xFFFF,        /* broadcast in v1 */
        .src_short_addr = CONFIG_RADIO_TX_NODE_ID,
        .ack_request = false,
};

/* once, before the loop */
sensor_tx_init();

for (;;) {
        struct sensor_reading r = { .node_id = CONFIG_RADIO_TX_NODE_ID };

        /* sample temperature/light/accel into r ... */
        r.seq = seq16++;
        r.uptime_ms = k_uptime_get_32();
        cfg.mac_seq++;

        switch (sensor_tx_send(&cfg, &r)) {
        case 0:
                break;
        case -EBUSY:                     /* channel busy after CSMA retries */
        case -EAGAIN:                    /* only with ack_request = true */
                /* optional short backoff; broadcast v1 relies on app seq */
                break;
        default:
                LOG_WRN("tx failed (%d)", ret);
                break;
        }

        k_sleep(K_MSEC(CONFIG_RADIO_TX_PERIOD_MS));
}
```

### 4.4 Kconfig (`lib/radio/Kconfig`)

```kconfig
config RADIO_TX_CHANNEL
        int "802.15.4 channel (11-26)"
        depends on RADIO
        range 11 26
        default 15

config RADIO_TX_PAN_ID
        hex "802.15.4 PAN ID"
        depends on RADIO
        default 0xCAFE

config RADIO_TX_NODE_ID
        int "Sensor node id / MAC short address (0-255)"
        depends on RADIO
        range 0 255
        default 1

config RADIO_TX_POWER
        int "TX power (dBm, driver-mapped)"
        depends on RADIO
        default 0

config RADIO_TX_PERIOD_MS
        int "Sensor reporting period (ms)"
        depends on RADIO
        default 1000
```

### 4.5 `app/prj.conf`

```
CONFIG_NETWORKING=y
CONFIG_IEEE802154=y
CONFIG_IEEE802154_RAW_MODE=y
CONFIG_NET_PKT_TX_COUNT=4
CONFIG_NET_BUF_DATA_SIZE=128
CONFIG_RADIO=y
```

`CONFIG_NET_BUF_DATA_SIZE` must be at least 27 so the whole PSDU is one fragment
(the driver writes a single `frag->data`/`frag->len`).

---

## 5. Off-hardware testing (replaces the impossible link test)

The RF medium can't be unit-tested, but the **glue** can. Link `sensor_tx.c`
against a fake `ieee802154_radio_api` whose `.tx` captures the `net_buf`, and
assert:

1. the captured bytes are exactly the 27-byte golden frame
   (`test_encode_golden_bytes` already pins the codec, so this proves the glue
   passes it through unchanged);
2. the TX mode is `CSMA_CA` in production builds;
3. `-EBUSY` / `-EAGAIN` from the fake propagate to the caller (retry policy);
4. an encode failure (NULL reading, tiny buffer) never reaches the radio;
5. `sensor_tx_send()` allocates and releases the `net_pkt` (no leak).

Run it with the existing `type: unit` harness in `tests/lib/radio` (`unit_testing`
board, no kernel), using ztest mocking. This is the seam that catches "the
codec is right but we asked the radio to send the wrong thing".

`native_sim` cannot exercise `rf2xx` (SPI/GPIO), so hardware bring-up stays a
manual bench milestone (§7).

---

## 6. On-air configuration

| Item | Where | Notes |
|---|---|---|
| Channel | `api->set_channel()` | 11–26; ch 11 = 2405 MHz. Must match `RX-PLAN.md`. |
| TX power | `api->set_txpower()` | board DTS supplies the `tx-pwr-*` table (`rf2xx.c:1115`). Start low. |
| PAN ID | codec `cfg.pan_id` | written into the MAC header; TX needs no radio filter. |
| Src short addr | codec `cfg.src_short_addr` | set to `node_id` so sniffers can filter. |
| Dest | codec `cfg.dst_short_addr` | `0xFFFF` broadcast in v1. |
| AR | codec `cfg.ack_request` | `false` in v1 (broadcast is not ACKed). |
| MAC seq | `cfg.mac_seq` | increment per frame, uint8 wrap. |
| App seq | `reading.seq` | increment per reading, uint16; loss detection. |

Because the codec writes the full MAC header, the radio's frame filter and the
hardware address registers (datasheet §"Frame Filter", regs 0x20–0x2B) are
irrelevant on TX — only channel and power matter.

---

## 7. Bring-up sequence (two SAM R21 boards)

1. Build the sensor image in raw mode; boot; confirm `rf2xx` probe logs the part
   number/version and the chosen channel (same as `RX-PLAN.md` R2).
2. Start with `IEEE802154_TX_MODE_DIRECT` and a 1 s period; confirm on a second
   board (or sniffer) that the 27 bytes match the golden frame.
3. Switch to `IEEE802154_TX_MODE_CSMA_CA`.
4. Raise the period/power; confirm the gateway decodes with plausible RSSI/LQI.
5. Leave running; watch for `-EBUSY` (channel busy) and app-seq gaps.

---

## 8. Milestones

| # | Deliverable | Depends on | Hardware |
|---|---|---|---|
| T0 | `sensor_tx.c` + fake-radio host test (exact PSDU + mode) | codec (done) | no |
| T1 | Sensor image builds in raw mode and transmits on a fixed channel | T0 | no |
| T2 | Gateway receives and decodes live frames | T1 + `RX-PLAN.md` R1/R2 | yes |
| T3 | CSMA-CA policy, `-EBUSY`/`-EAGAIN` backoff, seq wraparound | T2 | yes |
| T4 | Kconfig channel/PAN/power; range and power tuning | T3 | yes |

---

## 9. Pitfalls

1. **Do not append the FCS.** The driver writes `PHR = len + 2`; the chip adds
   the FCS (datasheet §35.3.2 / §36.1.2.8). Our codec already returns 27.
2. **`net_pkt` buffer must fit one PSDU.** `CONFIG_NET_BUF_DATA_SIZE=128`; a
   smaller value could split the 27 bytes across fragments, and `rf2xx_tx()`
   only writes the single `frag` it is given.
3. **TX is blocking.** `rf2xx_tx()` waits on `trx_tx_sync` with `K_FOREVER`;
   transmit from one thread only.
4. **No unaligned access.** The 4-byte `uptime_ms` sits at payload offset 13 on
   a Cortex-M0+ (no unaligned word access). The codec already assembles it
   byte-wise — keep it that way.
5. **`mac_seq` ≠ app `seq`.** MAC sequence number is 8-bit and per frame; the
   application `seq` is 16-bit and per reading. Loss statistics use the app one.
6. **Broadcast has no MAC retransmission.** With `ack_request = false` the
   hardware TX_ARET still does CSMA-CA but does not wait for an ACK; packet loss
   is visible only as gaps in the app `seq`.
7. **Raw mode is global.** It is fine on the sensor because there is no other
   network stack; do **not** enable it on the gateway image.
8. **Channel/PAN mismatch** is the most likely "nothing works" failure at T2 —
   fix both ends from Kconfig before first bring-up.

---

## 10. Open decisions

1. **Reporting period** — drives `CONFIG_GATEWAY_RADIO_ONLINE_TIMEOUT_S` on the
   gateway (`RX-PLAN.md` §11); confirm before T2.
2. **Broadcast vs ACK'd unicast** — v1 is broadcast; revisit if link quality
   needs MAC-level retries.
3. **`flags` semantics** — still TBD in `AGENTS.md`; transmit raw.
4. **Channel/PAN/power defaults** — pick deployment values.
5. **Whether `sensor_tx.c` lives in `lib/radio` or `app/`** — `lib/radio` keeps
   the codec + transport together and testable; `app/` reduces indirection.
