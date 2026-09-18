# RX-PLAN — receiving and decoding sensor frames on the SAM R21 gateway

Plan for making the gateway's on-board AT86RF233 receive the 27-byte sensor
PSDUs and feed the decoded 18-byte reading into the gateway's sensor table, as
required by `gateway/AGENTS.md`.

- Companion document: **`TX-PLAN.md`** (sensor transmit path).
- Source: this plan expands `gateway/radio-plan.md` with the receive-side
  detail and supersedes it.
- Ethernet/IP/HTTP side: `gateway/ethernet-roadmap.md` (out of scope here).
- Codec contract: `include/app/lib/sensor_frame.h`.
- Datasheet: `datasheets/Atmel-42223–SAM-R21_Datasheet.pdf` (AT86RF233
  integrated radio).

Both ends of the link are SAM R21 boards with the same AT86RF233, so there is a
single radio driver (`rf2xx`) and no board-diversity branch in this plan.

---

## 0. Goal & scope

Deliver, in order of dependency:

1. A **custom 802.15.4 L2** that receives the raw PSDU and hands the application
   payload to the gateway (no IPv6/6LoWPAN).
2. A small (~4-line) patch to the in-tree `rf2xx` driver so it can bind that L2.
3. A **local MAC-header parser + payload decoder** (the codec's
   `sensor_frame_decode()`, already implemented).
4. A **sensor table** (`node_id → reading/timestamp/stats`) with a walk API.
5. Radio bring-up (channel, PAN filter, start) and RSSI/LQI capture.
6. Integration with `GET /sensors` / `GET /health` (ethernet-roadmap R3).

Out of scope for v1: 802.15.4 security (AES), 6LoWPAN/IPv6 on the radio,
OpenThread, replying to sensors, transmitting anything other than a diagnostic
beacon/ACK test (see `TX-PLAN.md`).

**Status:** the shared codec (`sensor_frame.c`) is implemented and unit-tested
in `tests/lib/radio` (16 cases, passing on the `unit_testing` board). The sensor
table and the custom-L2 glue are still to do.

---

## 1. The decisive constraint: raw mode cannot coexist with Ethernet

`CONFIG_IEEE802154_RAW_MODE` selects `CONFIG_NET_RAW_MODE`, which is a *global*
net-stack switch:

```cmake
# $ZEPHYR_BASE/subsys/net/CMakeLists.txt
if(CONFIG_NET_RAW_MODE)
  zephyr_library_sources(ip/net_pkt.c)   # ONLY net_pkt.c
else()
  add_subdirectory(ip)                   # IPv4, ARP, TCP, UDP, sockets...
endif()
```

Enabling it compiles **only** `net_pkt.c` — no IPv4, no ARP, no TCP, no sockets.
The gateway must serve HTTP over Ethernet, so raw mode is a dead end for this
board image. (It *is* the right choice for the sensor, which has no IP stack —
see `TX-PLAN.md` §3.)

| Option | Coexists with Ethernet/IP? | Patch needed? | Fits 18-byte contract? |
|--------|----------------------------|---------------|------------------------|
| `CONFIG_IEEE802154_RAW_MODE` | **No** (kills IP stack) | No | Yes |
| `CONFIG_NET_L2_IEEE802154` (6LoWPAN) | Yes | No | **No** (wants IPv6/6LoWPAN) |
| `CONFIG_NET_L2_OPENTHREAD` | Yes | No | No |
| **`CONFIG_NET_L2_CUSTOM_IEEE802154`** | **Yes** | **~4 lines in rf2xx** | **Yes** |

Additional facts verified in this tree:

- `CONFIG_NET_L2_CUSTOM_IEEE802154` **selects `NET_L2_PHY_IEEE802154`**
  (`subsys/net/l2/Kconfig:34`), and `IEEE802154_RAW_MODE` is defined under
  `if !NET_L2_PHY_IEEE802154`, so the two are mutually exclusive by
  construction. Selecting the custom L2 guarantees the IP stack stays.
- Zephyr provides **no implementation** for the custom L2 symbol — the
  application supplies it with `NET_L2_INIT(CUSTOM_IEEE802154_L2, ...)`. See the
  reference test `tests/net/ieee802154/custom_l2/src/main.c:50`.
- Of the in-tree 802.15.4 drivers only `nrf5`, `mcxw`, `silabs_efr32` and
  `stm32wba` wire up `CUSTOM_IEEE802154_L2`. **`rf2xx` does not**
  (`drivers/ieee802154/ieee802154_rf2xx.c:1097`), hence §2.

**Decision: custom 802.15.4 L2 + a small rf2xx patch.**

---

## 2. The rf2xx patch

`ieee802154_rf2xx.c` selects the L2 at compile time and has no branch for the
custom L2. Add the same branch `nrf5` already carries:

```c
#if !defined(CONFIG_IEEE802154_RAW_MODE)
    #if defined(CONFIG_NET_L2_IEEE802154)
	#define L2 IEEE802154_L2
	#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
	#define MTU RF2XX_MAX_PSDU_LENGTH
    #elif defined(CONFIG_NET_L2_OPENTHREAD)
	#define L2 OPENTHREAD_L2
	#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
	#define MTU RF2XX_OT_PSDU_LENGTH
    #elif defined(CONFIG_NET_L2_CUSTOM_IEEE802154)          /* <-- add */
	#define L2 CUSTOM_IEEE802154_L2
	#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(CUSTOM_IEEE802154_L2)
	#define MTU CONFIG_NET_L2_CUSTOM_IEEE802154_MTU
    #endif
#endif /* CONFIG_IEEE802154_RAW_MODE */
```

Reference: `drivers/ieee802154/ieee802154_nrf5.c:1326` (identical shape).

### 2.1 How to carry the patch

`west.yml` pins `zephyr` to `main`, so `west update` will not preserve a local
edit. Options, in order of preference:

1. **Upstream it.** The branch already exists for four other drivers; the change
   is mechanical. Best long-term fix.
2. **Carry a patch file.**
   `patches/0001-ieee802154-rf2xx-support-custom-l2.patch`, applied by a
   workspace setup step. `west` has no built-in patch step, but this repo already
   ships west commands (`scripts/west-commands.yml`); add e.g.
   `west apply-patches` that applies `patches/*.patch` to the Zephyr tree.
3. **Vendor a copy** of the driver under `drivers/ieee802154/rf2xx/` and set
   `CONFIG_IEEE802154_RF2XX=n`. **Rejected** — duplicates ~1200 lines for a
   4-line difference.

Record the chosen mechanism in `README.md` so CI and fresh clones reproduce it.

---

## 3. On-air frame contract and RX parsing

### 3.1 What the receiver gets

The 18 bytes are the MAC payload of a well-formed 802.15.4 data frame:

| Offset | Size | Field | Type |
|--------|------|-------|------|
| 0  | 1 | `node_id` | `uint8_t` |
| 1  | 2 | `seq` | `uint16_t` |
| 3  | 2 | `light` | `uint16_t` |
| 5  | 2 | `temp_c_x100` | `int16_t` (°C × 100) |
| 7  | 2 | `accel_x` | `int16_t` |
| 9  | 2 | `accel_y` | `int16_t` |
| 11 | 2 | `accel_z` | `int16_t` |
| 13 | 4 | `uptime_ms` | `uint32_t` |
| 17 | 1 | `flags` | `uint8_t` |
| = 18 | | | |

v1 MAC profile (produced by `sensor_frame_encode()`): data frame, no security,
short destination and source addresses, PAN ID compression enabled, AR=0 for
broadcast. Header = 9 bytes, full PSDU = 27 bytes, FCS appended by the radio.

### 3.2 MAC header length algorithm (receive side)

Frame control is little-endian. The header may be longer than the v1 encoder's
9 bytes, so size it from the FCF:

```c
fcf      = sys_get_le16(buf);
type     = fcf & 0x7;                       /* 1 = data */
security = (fcf >> 3) & 0x1;                /* must be 0 */
pan_comp = (fcf >> 6) & 0x1;
dst_mode = (fcf >> 10) & 0x3;               /* 0 none, 2 short, 3 ext */
src_mode = (fcf >> 14) & 0x3;

if (type != IEEE802154_FRAME_TYPE_DATA || security) -> drop;

hdr = 2 /* FCF */ + 1 /* seq */;
if (dst_mode) { hdr += (dst_mode == 2) ? 2 : 8; hdr += 2; /* dst PAN */ }
if (src_mode) {
        /* src PAN present unless PAN-compressed with a dst address present */
        if (!(pan_comp && dst_mode)) hdr += 2;
        hdr += (src_mode == 2) ? 2 : 8;
}
/* no aux security header: security bit already rejected */
payload = buf + hdr;
payload_len = len - hdr;                     /* FCS already removed by driver */
```

`subsys/net/l2/ieee802154/ieee802154_frame.c` does this, but that file is built
only for `CONFIG_NET_L2_IEEE802154`, which we are not enabling, and its header is
marked "not to be included by the application". So the gateway owns a ~40-line
parser. In this tree it is `sensor_frame_decode()` in `lib/radio/sensor_frame.c`,
already unit-tested (`test_decode_accepts_uncompressed_pan`,
`test_decode_accepts_extended_dst`, `test_decode_rejects_malformed`, field
isolation for both directions).

### 3.3 FCS handling — do not strip twice

The rf2xx RX path reads the frame buffer and, when
`CONFIG_IEEE802154_L2_PKT_INCL_FCS` is **not** set, removes the 2-byte FCS
before handing the packet up
(`drivers/ieee802154/ieee802154_rf2xx.c:211`). That option defaults to `y` only
for raw/OpenThread; with the custom L2 it is **`n`**, so the driver strips it and
the L2 sees exactly header + 18 payload bytes. **Do not subtract 2 again.**

The driver also attaches link quality to the packet:

```c
net_pkt_set_ieee802154_lqi(pkt, ctx->pkt_lqi);
net_pkt_set_ieee802154_rssi_dbm(pkt, ctx->pkt_ed + ctx->trx_rssi_base);
```

read back with `net_pkt_ieee802154_lqi()` / `net_pkt_ieee802154_rssi_dbm()`
(`include/zephyr/net/ieee802154_pkt.h`).

---

## 4. Datasheet receive path (AT86RF233)

The driver implements all of this; the notes below are the ground truth behind
its behaviour.

- **§35.3.2 Frame Buffer Access Mode (read)** — a read access returns
  `PHY_STATUS + PHR + PSDU + LQI + ED + RX_STATUS`, `n = 5 + frame_length`. That
  is where the driver's `pkt_lqi`/`pkt_ed` come from. Max 127-byte PSDU.
- **§36.1.2.7 RX_ON / BUSY_RX** — from `RX_ON` the chip detects a valid SHR,
  stores the PSDU into the frame buffer, raises `IRQ_3 (TRX_END)` and sets
  `RX_CRC_VALID` (register 0x06) with the **automatic FCS check** result.
- **Frame Filter** (datasheet §"Frame Filter", registers 0x20–0x2B) — an address
  match raises `IRQ_5 (AMI)`. The driver's `rf2xx_filter()` programs short/PAN
  address filtering.
- **§36.2 Extended Operating Mode** — `RX_AACK_ON` performs the automatic FCS
  check, frame filtering and (if requested by the frame) automatic ACK
  transmission. The driver enters this via `rf2xx_trx_set_rx_state()` and
  `rf2xx_start()`.
- **Invalid frames** — the driver drops frames whose `TRAC_STATUS` is
  `INVALID` or whose PHR is below the minimum
  (`ieee802154_rf2xx.c:182,200`) before the L2 ever sees them.

---

## 5. Architecture and files

`AGENTS.md`: "We will share code over `../lib` for the radio communications and
ethernet/IP." Keep pure logic in `lib/`, Zephyr glue in `drivers/`/app.

| Layer | Location | Depends on |
|-------|----------|------------|
| Frame codec, encode + decode (pure) | `lib/radio/sensor_frame.c` | `stdint`/`string` only |
| Sensor table + stats | `lib/radio/sensor_table.c` | `k_mutex`, `k_uptime` |
| Custom L2 glue | `lib/radio/gateway_l2.c` | `net_if`, `net_pkt`, decoder, table |
| Radio bring-up | `gateway/src/main.c` (`radio_start()`) | `ieee802154_radio_api` |
| Public headers | `include/app/lib/sensor_frame.h`, `sensor_table.h` | |

Target tree:

```
include/app/lib/sensor_frame.h        # 18-byte contract, decode API (done)
include/app/lib/sensor_table.h        # table + walk API, stats
lib/radio/
  CMakeLists.txt
  Kconfig                             # CONFIG_RADIO, CONFIG_GATEWAY_RADIO_*
  sensor_frame.c                      # MAC header parse + payload validate (done)
  sensor_table.c                      # direct-index array, thread-safe
  gateway_l2.c                        # NET_L2_INIT + ieee802154_init()
tests/lib/radio/                      # type: unit host tests
```

`lib/Kconfig` and `lib/CMakeLists.txt` already carry a `radio/` entry.

---

## 6. Interfaces

### 6.1 Frame codec (done) — `include/app/lib/sensor_frame.h`

```c
#define SENSOR_PAYLOAD_LEN 18U
#define SENSOR_FRAME_HEADER_LEN 9U
#define SENSOR_FRAME_LEN (SENSOR_FRAME_HEADER_LEN + SENSOR_PAYLOAD_LEN)

struct sensor_reading {
        uint8_t  node_id;
        uint16_t seq;
        uint16_t light;
        int16_t  temp_c_x100;
        int16_t  accel[3];
        uint32_t uptime_ms;
        uint8_t  flags;
};

int sensor_frame_decode(const uint8_t *psdu, size_t len,
                        struct sensor_reading *out);
```

Decode rejects wrong length / non-data frames / security / malformed addressing
and never reads past `len`; it is not the encoder's inverse only — the decoder is
general over the §3.2 header rules.

### 6.2 Sensor table — `include/app/lib/sensor_table.h`

Direct-index array (`node_id` is the index, never stored in the entry).

**Liveness is the only state, and it is computed at read time.** A node is
`online` iff it has been heard (`received != 0`) and
`(uint32_t)(now_ms - ts_ms) <= timeout_ms`. A never-heard node is simply
**absent** — no presence sentinel, no seen bitmap, no tri-state.

```c
struct sensor_entry {          /* 20 B */
        uint32_t ts_ms;        /* last-heard monotonic uptime (k_uptime_get) */
        uint32_t received;     /* frames seen; 0 = never heard */
        uint16_t last_seq;     /* application seq; duplicate/reorder detection */
        uint16_t light;
        int16_t  temp_c_x100;
        int16_t  accel[3];
};

struct sensor_stats {          /* 3 B side array, 768 B total */
        uint8_t flags;
        int8_t  rssi_dbm;
        uint8_t lqi;
};

int  sensor_table_store(const struct sensor_reading *r, uint32_t now_ms,
                        int8_t rssi_dbm, uint8_t lqi);
int  sensor_table_get(uint8_t node_id, struct sensor_entry *out);
typedef void (*sensor_walk_cb)(uint8_t node_id, const struct sensor_entry *e,
                               const struct sensor_stats *s, void *ctx);
void sensor_table_foreach(sensor_walk_cb cb, void *ctx);
void sensor_table_reset(void);           /* POST /reset */
size_t sensor_table_online_count(uint32_t now_ms, uint32_t timeout_ms);
```

- **Never-heard is not a state.** Presence is `received != 0`; `ts_ms` is never a
  "0 = never heard" sentinel. That also removes pre-SNTP ambiguity.
- **Packet counters are not liveness.** When a node falls silent `received`
  stops advancing, so a loss rate freezes at its pre-silence value. Loss rate is
  a link-quality signal, deferred to R4 and requiring an `expected` counter.
- **Compute on read, never cache.** `online` is derived from `now_ms` on every
  `online_count`/`foreach`/JSON call; there is no stored flag (which could never
  flip to offline — nothing arrives to trigger it).
- **Monotonic time only.** `ts_ms` is `k_uptime_get()` (uint32 ms, wraps ~49
  days; unsigned `now_ms - ts_ms` is correct across the wrap). The HTTP layer
  renders wall-clock from one global SNTP offset.
- **Timeout.** `CONFIG_GATEWAY_RADIO_ONLINE_TIMEOUT_S` (default 30 s) until the
  sensor reporting period is confirmed (see `TX-PLAN.md` §10).
- Storage: `table[256]` (5 120 B) + `stats[256]` (768 B) + one mutex.
- Bounds-check `node_id` before indexing; a corrupt byte must never index OOB.
- `foreach` iterates ascending `node_id` over `received != 0` entries
  (deterministic JSON for tests).

### 6.3 Custom L2 — `lib/radio/gateway_l2.c`

```c
NET_L2_INIT(CUSTOM_IEEE802154_L2, gateway_l2_recv, gateway_l2_send,
            gateway_l2_enable, gateway_l2_flags);

/* The rf2xx driver may call this from ieee802154_init(); custom L2s must
 * provide it (only the stock L2 and OpenThread define it in-tree). */
void ieee802154_init(struct net_if *iface);
```

- `recv`: read `net_pkt` frag data/len, call `sensor_frame_decode`, and on
  success `sensor_table_store(...)` with
  `net_pkt_ieee802154_rssi_dbm(pkt)` / `net_pkt_ieee802154_lqi(pkt)`. Return
  `NET_DROP` for non-sensor frames (wrong length/malformed) and `NET_OK` for
  stored ones. Always let the caller unref the pkt.
- `send`: return `-ENOTSUP` in v1 (no replies). A diagnostic ACK path can be
  added later.
- `enable`: `NET_L2_MULTICAST`-free; return 0. `get_flags`: 0.
- `ieee802154_init`: stash the iface (for link stats) and mark the L2 ready. Do
  not start the radio here — do it from the app after channel/PAN are set (use
  `CONFIG_IEEE802154_NET_IF_NO_AUTO_START`, see §9 R1).

Reference shape: `tests/net/ieee802154/custom_l2/src/main.c` and the L2 contract
in `include/zephyr/net/net_l2.h`.

---

## 7. Concurrency rules

1. One `k_mutex` guards `sensor_table_*`. `recv` runs in the rf2xx RX thread;
   `foreach`/`get`/`reset` run in the HTTP thread.
2. Keep the critical section short: decode **outside** the lock (into a local
   `struct sensor_reading`), then lock only to store.
3. Never block in `recv`; never call `net_pkt_unref` twice. The L2 returns a
   verdict and the net core releases the pkt.
4. `sensor_table_foreach` must not hold the lock while calling a callback that
   could re-enter. v1: hold the lock, forbid re-entry.

---

## 8. Resource budget

Board: 256 KB flash / 32 KB SRAM (SAM R21). Radio-side additions:

| Item | Size |
|------|------|
| `sensor_table` `table[256]` × 20 B | 5 120 B |
| `sensor_stats[256]` × 3 B | 768 B |
| Custom L2 + frame decoder (`.text`) | < 1 KB |
| rf2xx RX thread stack (`CONFIG_IEEE802154_RF2XX_RX_STACK_SIZE`) | 800 B (driver) |
| net_pkt buffers (shared with Ethernet) | tune at R3/R4 |
| **Total radio table** | **~5.8 KB** (~18 % SRAM) |

If Ethernet + IP + HTTP later forces cuts, the documented fallback is a fixed
pool + `uint8_t idx[256]` map (drop `stats[]` first; presence is `received != 0`
and online is always recomputed). Do **not** switch to a hash map.

Kconfig knobs to measure with `west build -t ram_report`/`rom_report`:
`CONFIG_NET_PKT_RX_COUNT`, `CONFIG_NET_BUF_RX_COUNT`,
`CONFIG_NET_BUF_DATA_SIZE`, `CONFIG_IEEE802154_RF2XX_RX_STACK_SIZE`,
`CONFIG_MAIN_STACK_SIZE`.

---

## 9. Milestones

Ordered so most logic is testable off-hardware; the board build depends on the
ethernet-roadmap M1/M2 SPI bring-up only for the Ethernet side (radio uses its
own SERCOM4, an independent bus).

| # | Milestone | Depends on |
|---|-----------|------------|
| R0 | Frame decoder + table, unit-tested on `unit_testing` | — |
| R1 | rf2xx custom-L2 patch + `gateway_l2.c`, builds for the board | R0 |
| R2 | Radio bring-up on hardware (RX debug logging) | R1 |
| R3 | Table + stats integration, `/sensors` JSON | R2, ethernet-roadmap M3 |
| R4 | Robustness, loss metrics, resource pass | R3 |

### R0 — Codec + sensor table (no hardware)

- [x] `sensor_frame.c` / `sensor_frame.h`: MAC parser, decoder, encoder.
- [ ] `sensor_table.h` / `sensor_table.c`: store/get/foreach/reset, staleness
      online count, bounds checks.
- [x] `tests/lib/radio`: golden vectors, round-trip, truncation/length attacks,
      field isolation (both directions), randomized property test, bounds.
- [ ] `tests/lib/radio`: table store/get/reset, seq wraparound, staleness
      thresholds, bounds.

**Exit criteria:** tests pass under Twister on `unit_testing`; the decoder is
fuzzable (no read past `len` for any 0…127-byte input).

### R1 — rf2xx custom-L2 patch + L2 glue

- [ ] Apply/carry the patch from §2 (upstream or `west apply-patches`).
- [ ] `lib/radio/gateway_l2.c`: `NET_L2_INIT`, `gateway_l2_recv`,
      `ieee802154_init`, `send`/`enable`/`flags`.
- [ ] `Kconfig`: `CONFIG_GATEWAY_RADIO_CHANNEL`, `CONFIG_GATEWAY_RADIO_PAN_ID`,
      `CONFIG_GATEWAY_RADIO_ONLINE_TIMEOUT_S`.
- [ ] `gateway/prj.conf`: `CONFIG_NETWORKING=y`,
      `CONFIG_NET_L2_CUSTOM_IEEE802154=y`,
      `CONFIG_NET_L2_CUSTOM_IEEE802154_MTU=127`,
      `CONFIG_IEEE802154_NET_IF_NO_AUTO_START=y`. Keep the Ethernet options.
      **Do not** set `CONFIG_IEEE802154_RAW_MODE`.
- [ ] `gateway/src/main.c`: `radio_start()` — resolve
      `DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154))`, set the channel, set the PAN
      filter (and gateway short address / broadcast acceptance), then
      `net_if_up()` on the radio iface.

**Exit criteria:** `west build -b samr21_xpro gateway` links with both Ethernet
and radio L2s; the map contains `CUSTOM_IEEE802154_L2` and no `NET_RAW_MODE`.

### R2 — Hardware bring-up

- [ ] Boot on `samr21_xpro`; log `TRX_STATUS` / part number and channel.
- [ ] Receive frames from the sensor image (`TX-PLAN.md` T1/T2); log each decoded
      reading + RSSI/LQI.
- [ ] Confirm rejection of wrong-length, non-data and security-enabled frames.
- [ ] Optional: promiscuous diagnostic build for sniffing field traffic.

**Exit criteria:** sensor frame → decoded `node_id`/fields in the log on real
hardware; no dropped valid frames over a sustained run; RSSI plausible.

### R3 — Table + HTTP integration

- [ ] Wire `sensor_table_store` into `gateway_l2_recv` with monotonic uptime
      stamps; render wall-clock in the HTTP layer from the SNTP offset.
- [ ] `GET /sensors`: per-node JSON (id, timestamp, light, temp, accel, seq,
      online). `GET /health`: uptime, online count, per-node age.
      `POST /reset`: `sensor_table_reset()`.

### R4 — Hardening

- [ ] Online timeout enforcement + per-node `age_s`.
- [ ] Optional per-node loss rate (needs an `expected` counter).
- [ ] Handle/soak: table-full, id collisions, duplicate `seq`, burst traffic.
- [ ] Resource pass with the full Ethernet+HTTP image.
- [ ] Decide on replying to sensors (contract open) — likely no.

---

## 10. Test strategy

| Level | What | Where | Hardware? |
|-------|------|-------|-----------|
| Unit | MAC parse, encode/decode, length guards, field isolation | `tests/lib/radio` (`unit_testing`) | no |
| Unit | Table store/get/reset, seq wraparound, bounds | `tests/lib/radio` | no |
| Compose | `NET_L2_INIT` links, `ieee802154_init` resolves | board build | no |
| Bring-up | channel/PAN, RX of real frames, RSSI | `samr21_xpro` + peer | yes |
| App | `/sensors`, `/health`, `/reset` JSON | host client | yes |

Golden vectors must include short and extended source/destination addresses and
both PAN-compression settings, since those change the header length. Encode them
as raw byte arrays in the test, never as C structs, so the test cannot inherit
the packing/alignment bugs it is meant to catch. Add an encode → decode
round-trip over negative temperatures and `seq`/`uptime_ms` extremes. All of this
is already in `tests/lib/radio`.

---

## 11. Pitfalls

1. **`char` signedness.** The contract says `char node_id`; store and index with
   `uint8_t` (already done).
2. **Unaligned `uint32_t uptime_ms`.** Payload offset 13 on a Cortex-M0+ with no
   unaligned word access. Never `memcpy` the 18 bytes into a packed struct and
   read `->uptime_ms`; assemble byte-wise (already done).
3. **FCS.** Driver strips it in custom-L2 mode
   (`ieee802154_rf2xx.c:211`). Do not subtract 2 again.
4. **Endianness.** Wire order is little-endian; decode with byte-wise reads.
5. **Wrong-length frames.** Validate `payload_len == 18` *before* touching
   fields. Never read a fixed offset without a length guard.
6. **`seq` wraparound.** `uint16_t`; use unsigned subtraction.
7. **Concurrency.** RX thread stores, HTTP thread walks — one mutex (§7).
8. **SNTP step corrupts liveness.** `ts_ms` is monotonic uptime; wall-clock is
   rendered from one global SNTP offset at HTTP time.

---

## 12. Risks & mitigations

| Risk | Mitigation |
|------|------------|
| Encoder/decoder address-profile mismatch | One shared `sensor_frame_cfg`; decoder general over §3.2 header rules. |
| Upstream patch lost on `west update` | Upstream it, or carry `patches/*.patch` + `west apply-patches`. |
| rf2xx custom-L2 branch drifts as Zephyr `main` moves | Pin `zephyr` to a tag once bring-up works. |
| Custom L2 misses an API the driver calls (`ieee802154_init`) | Link test is part of R1 exit criteria. |
| ~6 KB table + Ethernet/IP exceed 32 KB SRAM | Sizing chosen; fallback pool + `idx[]`; measure at R1/R4. |
| Unaligned access fault on Cortex-M0+ | Byte-wise decode only. |
| `net_pkt` leak in `recv` | Return `NET_DROP`/`NET_OK`; never `unref` in the L2; soak test. |
| MAC `seq` vs app `seq` confusion | Loss stats use the application `seq` only. |
| Sensor reports slower than the online timeout | Set the timeout to a small multiple of the reporting period (`TX-PLAN.md` §10). |

---

## 13. Open decisions

1. **`flags` semantics** — TBD in `AGENTS.md`; store raw.
2. **Field endianness** — little-endian; confirm with the sensor team.
3. **MAC source address** — v1 short `src_short_addr == node_id`.
4. **Reply to sensors?** Contract open. v1: no (`send` → `-ENOTSUP`).
5. **Patch carriage** — upstream vs local patch + `west apply-patches`.
6. **Channel/PAN** — pick values; must match `TX-PLAN.md`.
7. **Sensor reporting period / online timeout** —
   `CONFIG_GATEWAY_RADIO_ONLINE_TIMEOUT_S` defaults to 30 s.
8. **Promiscuous diagnostic build** — nice-to-have for field debugging.
