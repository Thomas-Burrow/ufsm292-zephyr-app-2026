# Gateway radio transport

The gateway uses normal IEEE 802.15.4 L2 and a radio-bound `AF_PACKET` /
`SOCK_RAW` socket, not global `IEEE802154_RAW_MODE`. The socket is bound to the
interface belonging to `zephyr,ieee802154`, never a hard-coded interface index.
The application no longer overrides `net_recv_data()`.

`lib/radio` and its public headers are unchanged. Both directions use complete
MAC frames without FCS:

```c
/* sock is opened once with radio_socket_open(&sock, radio_iface). */
int len = sensor_frame_encode(psdu, sizeof(psdu), &cfg, &reading);
if (len > 0) {
        len = net_send_raw(&sock, psdu, len);
}

len = radio_socket_recv(&sock, psdu, sizeof(psdu), 0);
if (len > 0) {
        int ret = sensor_frame_decode(psdu, len, &reading);
        /* Handle ret and reading. */
}
```

The transport lives in `gateway/src/radio_socket.{c,h}`. Its calls return a
byte count or negative errno; setup returns zero or negative errno. TX uses
`sendto()` with an explicit radio destination interface and does not wait for
buffer availability. A successful send means accepted by the network stack,
not confirmed delivery over the air. It never retries a partial packet.
Frames are limited to 125 bytes excluding the two-byte hardware FCS.
RX rejects truncation rather than decoding a partial frame.

One RX consumer and a separate TX thread may share the same socket. Blocking
RX polls before a nonblocking receive: this avoids holding Zephyr's packet
socket fd lock while idle, which would stall TX. Close only after both users
stop. Initialize/open each transport once; do not reopen a live descriptor.

The bring-up image still only prints received readings: no unsolicited TX has
been added. Decode/print runs on an application thread; the socket queue uses
the finite network packet/buffer pools. Slow console output can exhaust those
pools and lose frames; this is not a reliable forwarding queue. RSSI/LQI are
not exposed by this adapter.

## Hardware-free validation

```sh
west build -b samr21_xpro gateway -d build-gateway-socket
west build -b native_sim/native/64 tests/net/radio_socket \
  -d build-radio-socket-test -- -DZEPHYR_TOOLCHAIN_VARIANT=host
set -o pipefail
build-radio-socket-test/zephyr/zephyr.exe 2>&1 | tee /tmp/ufsm292_socket_local_01.log
```

The simulator uses real Zephyr sockets/L2 and fake radio/Ethernet devices. It
checks codec TX bytes at the driver boundary, RX round-trip, interface isolation
(including Ethernet exclusion), truncation, invalid input, and TX while RX waits
on the same fd. A separate synthetic fragmented RX case bypasses IEEE802154 L2
(which requires contiguous RX frames) to check socket copying independently.

Validated against Zephyr 4.4.99, commit `c605ea46e15f`. The SAM R21 image builds,
but radio startup, promiscuous reception, FCS removal and actual TX/RX still need
hardware validation. Ethernet/IP operation is not validated by these tests.

## Application threads

`gateway_id` configures the radio/socket and logs statistics every five seconds.
It starts automatically with a 1024-byte stack at priority 0. `radio_rx_id`
uses a 2048-byte stack at priority 5 and waits on `rx_ready` until setup succeeds.
`main()` returns immediately, leaving room to start other services such as an
HTTP server. The main stack remains available for Zephyr initialization; the
dedicated gateway thread adds its own stack and thread bookkeeping.

The RX loop shows the application flow: receive a complete frame, decode a
`sensor_reading`, then call `handle_sensor_reading()`. That handler currently
prints the reading and is the place to add application processing. It runs
synchronously in the RX thread: keep work bounded so reception can continue.
The reading belongs to the loop's stack and is reused on the next decode; copy
it into a message queue or mutex-protected shared state for an HTTP server or
other thread rather than retaining its pointer. HTTP service setup and shared
reading storage are not implemented by this example.
