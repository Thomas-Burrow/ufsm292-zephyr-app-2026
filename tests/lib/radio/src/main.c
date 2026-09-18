/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Unit tests for the shared 802.15.4 sensor-frame codec (lib/radio).
 *
 * These run on the host via the `unit_testing` board: no Zephyr kernel is
 * linked in, only the real sensor_frame.c and Ztest. That keeps the TDD loop
 * in the sub-second range compared to qemu/native_sim builds.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include <app/lib/sensor_frame.h>

/*
 * Golden v1 frame. This is the wire contract between the sensor nodes and the
 * gateway: if this test has to change, the on-air format changed and every
 * receiver must be updated in lock-step.
 *
 * cfg:     pan_id 0x1234, dst 0xFFFF, src 0x0001, seq 0x07, no ack
 * reading: node 0x01, seq 0x0203, light 0x0A0B, temp -1234 (0xFB2E),
 *          accel {-1, 2, -32768}, uptime 0x11223344, flags 0xA5
 */
static const uint8_t golden_frame[SENSOR_FRAME_LEN] = {
	/* FCF = 0x9841 (data, 2006, PAN compress, short dst + src), LE */
	0x41, 0x98,
	0x07,             /* sequence number            */
	0x34, 0x12,       /* dst PAN 0x1234             */
	0xFF, 0xFF,       /* dst short addr (broadcast) */
	0x01, 0x00,       /* src short addr             */
	0x01,             /* node_id                    */
	0x03, 0x02,       /* seq 0x0203                 */
	0x0B, 0x0A,       /* light 0x0A0B               */
	0x2E, 0xFB,       /* temp -1234                 */
	0xFF, 0xFF,       /* accel[0] = -1              */
	0x02, 0x00,       /* accel[1] = 2               */
	0x00, 0x80,       /* accel[2] = -32768          */
	0x44, 0x33, 0x22, 0x11, /* uptime_ms 0x11223344 */
	0xA5,             /* flags                      */
};

static const struct sensor_frame_cfg golden_cfg = {
	.pan_id = 0x1234,
	.dst_short_addr = 0xFFFF,
	.src_short_addr = 0x0001,
	.mac_seq = 0x07,
	.ack_request = false,
};

static const struct sensor_reading golden_reading = {
	.node_id = 0x01,
	.seq = 0x0203,
	.light = 0x0A0B,
	.temp_c_x100 = -1234,
	.accel = { -1, 2, -32768 },
	.uptime_ms = 0x11223344,
	.flags = 0xA5,
};

/* Compare field-by-field: memcmp on a struct also compares padding bytes,
 * whose values are unspecified. */
static void assert_reading_eq(const struct sensor_reading *got,
			      const struct sensor_reading *exp)
{
	zassert_equal(got->node_id, exp->node_id, "node_id");
	zassert_equal(got->seq, exp->seq, "seq");
	zassert_equal(got->light, exp->light, "light");
	zassert_equal(got->temp_c_x100, exp->temp_c_x100, "temp_c_x100");
	zassert_equal(got->accel[0], exp->accel[0], "accel[0]");
	zassert_equal(got->accel[1], exp->accel[1], "accel[1]");
	zassert_equal(got->accel[2], exp->accel[2], "accel[2]");
	zassert_equal(got->uptime_ms, exp->uptime_ms, "uptime_ms");
	zassert_equal(got->flags, exp->flags, "flags");
}

/* --- Encoder ------------------------------------------------------------ */

ZTEST(sensor_frame, test_encode_golden_bytes)
{
	uint8_t buf[SENSOR_FRAME_LEN];
	int ret;

	ret = sensor_frame_encode(buf, sizeof(buf), &golden_cfg, &golden_reading);

	zassert_equal(ret, SENSOR_FRAME_LEN, "wrong byte count: %d", ret);
	zassert_mem_equal(buf, golden_frame, SENSOR_FRAME_LEN,
			  "frame bytes diverged from the on-air contract");
}

ZTEST(sensor_frame, test_encode_ack_request_sets_fcf_bit)
{
	struct sensor_frame_cfg cfg = golden_cfg;
	uint8_t buf[SENSOR_FRAME_LEN];
	int ret;

	cfg.ack_request = true;
	ret = sensor_frame_encode(buf, sizeof(buf), &cfg, &golden_reading);
	zassert_equal(ret, SENSOR_FRAME_LEN, "encode failed: %d", ret);

	/* Ack Request is FCF bit 5 -> low byte of the little-endian FCF. */
	zassert_equal(buf[0] & 0x20, 0x20, "Ack Request bit not set");
}

ZTEST(sensor_frame, test_encode_rejects_small_buffer)
{
	uint8_t buf[SENSOR_FRAME_LEN];

	zassert_equal(sensor_frame_encode(buf, SENSOR_FRAME_LEN - 1,
					  &golden_cfg, &golden_reading),
		      -ENOSPC, "undersized buffer not rejected");
}

ZTEST(sensor_frame, test_encode_rejects_null)
{
	uint8_t buf[SENSOR_FRAME_LEN];

	zassert_equal(sensor_frame_encode(NULL, sizeof(buf), &golden_cfg,
					  &golden_reading),
		      -EINVAL, "NULL buffer accepted");
	zassert_equal(sensor_frame_encode(buf, sizeof(buf), NULL,
					  &golden_reading),
		      -EINVAL, "NULL cfg accepted");
	zassert_equal(sensor_frame_encode(buf, sizeof(buf), &golden_cfg, NULL),
		      -EINVAL, "NULL reading accepted");
}

/* --- Decoder ------------------------------------------------------------ */

ZTEST(sensor_frame, test_decode_golden_frame)
{
	struct sensor_reading out;

	zassert_ok(sensor_frame_decode(golden_frame, sizeof(golden_frame), &out),
		   "decode failed");

	assert_reading_eq(&out, &golden_reading);
}

ZTEST(sensor_frame, test_encode_decode_roundtrip)
{
	uint8_t buf[SENSOR_FRAME_LEN];
	struct sensor_reading out;
	int ret;

	ret = sensor_frame_encode(buf, sizeof(buf), &golden_cfg, &golden_reading);
	zassert_equal(ret, SENSOR_FRAME_LEN, "encode failed");

	zassert_ok(sensor_frame_decode(buf, (size_t)ret, &out), "decode failed");
	assert_reading_eq(&out, &golden_reading);
}

ZTEST(sensor_frame, test_decode_accepts_uncompressed_pan)
{
	/* Same payload, but FCF without PAN compression and with a source PAN,
	 * i.e. an 11-byte header instead of 9.
	 */
	uint8_t frame[SENSOR_FRAME_LEN + 2] = { 0 };
	struct sensor_reading out;
	size_t i;

	/* FCF: data, v2006, no PAN compress, short dst + short src = 0x9801. */
	frame[0] = 0x01;
	frame[1] = 0x98;
	frame[2] = 0x07;
	frame[3] = 0x34; /* dst PAN 0x1234 */
	frame[4] = 0x12;
	frame[5] = 0xFF; /* dst short */
	frame[6] = 0xFF;
	frame[7] = 0x34; /* src PAN */
	frame[8] = 0x12;
	frame[9] = 0x01; /* src short */
	frame[10] = 0x00;
	for (i = 0; i < SENSOR_PAYLOAD_LEN; i++) {
		frame[11 + i] = golden_frame[SENSOR_FRAME_HEADER_LEN + i];
	}

	zassert_ok(sensor_frame_decode(frame, sizeof(frame), &out),
		   "uncompressed-PAN frame not accepted");
	zassert_equal(out.uptime_ms, golden_reading.uptime_ms, "payload shifted");
}

ZTEST(sensor_frame, test_decode_accepts_extended_dst)
{
	/* Extended (8-byte) destination address, compressed source PAN. */
	uint8_t frame[SENSOR_FRAME_LEN + 6] = { 0 };
	struct sensor_reading out;
	size_t i;

	/* FCF: data, v2006, PAN compress, ext dst + short src = 0x9C41. */
	frame[0] = 0x41;
	frame[1] = 0x9C;
	frame[2] = 0x07;
	frame[3] = 0x34; /* dst PAN */
	frame[4] = 0x12;
	for (i = 5; i < 13; i++) {
		frame[i] = (uint8_t)i; /* arbitrary 8-byte extended address */
	}
	frame[13] = 0x01; /* src short (PAN compressed away) */
	frame[14] = 0x00;
	for (i = 0; i < SENSOR_PAYLOAD_LEN; i++) {
		frame[15 + i] = golden_frame[SENSOR_FRAME_HEADER_LEN + i];
	}

	zassert_ok(sensor_frame_decode(frame, sizeof(frame), &out),
		   "extended-address frame not accepted");
	zassert_equal(out.node_id, golden_reading.node_id, "payload shifted");
}

ZTEST(sensor_frame, test_decode_rejects_malformed)
{
	struct sensor_reading out;
	uint8_t bad[SENSOR_FRAME_LEN];

	zassert_equal(sensor_frame_decode(NULL, sizeof(golden_frame), &out),
		      -EINVAL, "NULL psdu accepted");
	zassert_equal(sensor_frame_decode(golden_frame, sizeof(golden_frame),
					  NULL),
		      -EINVAL, "NULL out accepted");

	/* Truncated / payload length mismatch. */
	zassert_equal(sensor_frame_decode(golden_frame, sizeof(golden_frame) - 1,
					  &out),
		      -EINVAL, "short frame accepted");
	zassert_equal(sensor_frame_decode(golden_frame, sizeof(golden_frame) + 1,
					  &out),
		      -EINVAL, "over-long frame accepted");

	memcpy(bad, golden_frame, sizeof(bad));

	/* Non-data frame type (type = 0, beacon). */
	bad[0] = 0x00;
	zassert_equal(sensor_frame_decode(bad, sizeof(bad), &out), -EINVAL,
		      "non-data frame accepted");

	memcpy(bad, golden_frame, sizeof(bad));
	/* Security Enabled bit (FCF bit 3) -> aux security header expected. */
	bad[0] |= 0x08;
	zassert_equal(sensor_frame_decode(bad, sizeof(bad), &out), -EINVAL,
		      "security-enabled frame accepted");

	memcpy(bad, golden_frame, sizeof(bad));
	/* Reserved frame version (bits 12-13 == 3) -> FCF byte 1 |= 0x30. */
	bad[1] |= 0x30;
	zassert_equal(sensor_frame_decode(bad, sizeof(bad), &out), -EINVAL,
		      "reserved frame version accepted");

	memcpy(bad, golden_frame, sizeof(bad));
	/* Reserved destination addressing mode (1) -> FCF byte 1 |= 0x04. */
	bad[1] |= 0x04;
	zassert_equal(sensor_frame_decode(bad, sizeof(bad), &out), -EINVAL,
		      "reserved address mode accepted");
}

ZTEST_SUITE(sensor_frame, NULL, NULL, NULL, NULL, NULL);
