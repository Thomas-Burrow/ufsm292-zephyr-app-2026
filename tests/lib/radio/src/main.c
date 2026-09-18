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

/* --- Characterization: is the data moved without mangling? -------------- */

/*
 * Field isolation. Starting from an all-zero reading, setting exactly one
 * field to all-ones must change exactly that field's bytes in the frame and
 * leave every other byte untouched. This pins each field to its offset and
 * width, and catches off-by-one/aliasing bugs that a single golden vector
 * cannot see.
 */
static void assert_payload_field_writes_only(size_t off, size_t len,
					     const struct sensor_reading *r)
{
	const struct sensor_frame_cfg cfg = { 0 };
	const struct sensor_reading zero = { 0 };
	uint8_t base[SENSOR_FRAME_LEN];
	uint8_t got[SENSOR_FRAME_LEN];
	size_t i;

	zassert_equal(sensor_frame_encode(base, sizeof(base), &cfg, &zero),
		      SENSOR_FRAME_LEN, "baseline encode failed");
	zassert_equal(sensor_frame_encode(got, sizeof(got), &cfg, r),
		      SENSOR_FRAME_LEN, "field encode failed");

	for (i = 0; i < SENSOR_FRAME_LEN; i++) {
		bool in_field = (i >= off) && (i < off + len);
		uint8_t want = in_field ? 0xFF : base[i];

		zassert_equal(got[i], want,
			      "byte %zu wrong for field at [%zu,%zu)",
			      i, off, off + len);
	}
}

ZTEST(sensor_frame, test_encode_payload_field_isolation)
{
	struct sensor_reading r;

	r = (struct sensor_reading){ .node_id = 0xFF };
	assert_payload_field_writes_only(9, 1, &r);

	r = (struct sensor_reading){ .seq = 0xFFFF };
	assert_payload_field_writes_only(10, 2, &r);

	r = (struct sensor_reading){ .light = 0xFFFF };
	assert_payload_field_writes_only(12, 2, &r);

	r = (struct sensor_reading){ .temp_c_x100 = -1 };
	assert_payload_field_writes_only(14, 2, &r);

	r = (struct sensor_reading){ .accel = { -1, 0, 0 } };
	assert_payload_field_writes_only(16, 2, &r);

	r = (struct sensor_reading){ .accel = { 0, -1, 0 } };
	assert_payload_field_writes_only(18, 2, &r);

	r = (struct sensor_reading){ .accel = { 0, 0, -1 } };
	assert_payload_field_writes_only(20, 2, &r);

	r = (struct sensor_reading){ .uptime_ms = 0xFFFFFFFFU };
	assert_payload_field_writes_only(22, 4, &r);

	r = (struct sensor_reading){ .flags = 0xFF };
	assert_payload_field_writes_only(26, 1, &r);
}

/* Same idea for the MAC header fields taken from struct sensor_frame_cfg. */
static void assert_header_field_writes_only(size_t off, size_t len,
					    const struct sensor_frame_cfg *cfg)
{
	const struct sensor_frame_cfg zero = { 0 };
	const struct sensor_reading reading = { 0 };
	uint8_t base[SENSOR_FRAME_LEN];
	uint8_t got[SENSOR_FRAME_LEN];
	size_t i;

	zassert_equal(sensor_frame_encode(base, sizeof(base), &zero, &reading),
		      SENSOR_FRAME_LEN, "baseline encode failed");
	zassert_equal(sensor_frame_encode(got, sizeof(got), cfg, &reading),
		      SENSOR_FRAME_LEN, "header encode failed");

	for (i = 0; i < SENSOR_FRAME_LEN; i++) {
		bool in_field = (i >= off) && (i < off + len);
		uint8_t want = in_field ? 0xFF : base[i];

		zassert_equal(got[i], want,
			      "header byte %zu wrong for field at [%zu,%zu)",
			      i, off, off + len);
	}
}

ZTEST(sensor_frame, test_encode_header_field_isolation)
{
	struct sensor_frame_cfg cfg;

	cfg = (struct sensor_frame_cfg){ .mac_seq = 0xFF };
	assert_header_field_writes_only(2, 1, &cfg);

	cfg = (struct sensor_frame_cfg){ .pan_id = 0xFFFF };
	assert_header_field_writes_only(3, 2, &cfg);

	cfg = (struct sensor_frame_cfg){ .dst_short_addr = 0xFFFF };
	assert_header_field_writes_only(5, 2, &cfg);

	cfg = (struct sensor_frame_cfg){ .src_short_addr = 0xFFFF };
	assert_header_field_writes_only(7, 2, &cfg);
}

/*
 * The encoder must fill exactly SENSOR_FRAME_LEN bytes and never write past
 * it, even when it is handed a larger buffer.
 */
ZTEST(sensor_frame, test_encode_writes_exactly_frame_length)
{
	uint8_t buf[SENSOR_FRAME_LEN + 8];
	size_t i;

	memset(buf, 0xAA, sizeof(buf));

	zassert_equal(sensor_frame_encode(buf, sizeof(buf), &golden_cfg,
					  &golden_reading),
		      SENSOR_FRAME_LEN, "wrong byte count");
	zassert_mem_equal(buf, golden_frame, SENSOR_FRAME_LEN,
			  "frame bytes diverged");

	for (i = SENSOR_FRAME_LEN; i < sizeof(buf); i++) {
		zassert_equal(buf[i], 0xAA,
			      "encoder wrote past the frame at byte %zu", i);
	}
}

/* Signed payload fields must survive the full two's-complement range. */
ZTEST(sensor_frame, test_roundtrip_signed_boundaries)
{
	static const int16_t vals[] = {
		INT16_MIN, INT16_MIN + 1, -1234, -1, 0, 1, 1234,
		INT16_MAX - 1, INT16_MAX
	};
	const size_t n = sizeof(vals) / sizeof(vals[0]);
	uint8_t buf[SENSOR_FRAME_LEN];
	struct sensor_reading r = { 0 };
	struct sensor_reading out;
	size_t i;

	for (i = 0; i < n; i++) {
		r.temp_c_x100 = vals[i];
		zassert_equal(sensor_frame_encode(buf, sizeof(buf), &golden_cfg,
						  &r),
			      SENSOR_FRAME_LEN, "encode temp=%d failed", vals[i]);
		zassert_ok(sensor_frame_decode(buf, sizeof(buf), &out),
			   "decode temp=%d failed", vals[i]);
		zassert_equal(out.temp_c_x100, vals[i],
			      "temp mangled: %d -> %d",
			      (int)vals[i], (int)out.temp_c_x100);

		r.temp_c_x100 = 0;
		r.accel[0] = vals[i];
		r.accel[1] = vals[i];
		r.accel[2] = vals[i];
		zassert_equal(sensor_frame_encode(buf, sizeof(buf), &golden_cfg,
						  &r),
			      SENSOR_FRAME_LEN, "encode accel=%d failed", vals[i]);
		zassert_ok(sensor_frame_decode(buf, sizeof(buf), &out),
			   "decode accel=%d failed", vals[i]);
		zassert_equal(out.accel[0], vals[i],
			      "accel[0] mangled: %d", (int)vals[i]);
		zassert_equal(out.accel[1], vals[i],
			      "accel[1] mangled: %d", (int)vals[i]);
		zassert_equal(out.accel[2], vals[i],
			      "accel[2] mangled: %d", (int)vals[i]);

		r.accel[0] = 0;
		r.accel[1] = 0;
		r.accel[2] = 0;
	}
}

/* Deterministic xorshift32 so any failure reproduces exactly. */
static uint32_t rng_state;

static uint32_t rng_next(void)
{
	uint32_t x = rng_state;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng_state = x;
	return x;
}

/*
 * Property test: for a large batch of pseudo-random readings and MAC
 * parameters, encode -> decode must return every field bit-for-bit, and
 * re-encoding the decoded value must reproduce the exact same frame. This is
 * the broadest "no mangling" characterization available on the host.
 */
ZTEST(sensor_frame, test_roundtrip_random_values)
{
	uint8_t frame[SENSOR_FRAME_LEN];
	uint8_t reencoded[SENSOR_FRAME_LEN];
	struct sensor_frame_cfg cfg;
	struct sensor_reading r;
	struct sensor_reading out;
	int iter;

	rng_state = 0x9E3779B9U;

	for (iter = 0; iter < 2000; iter++) {
		cfg.pan_id = (uint16_t)rng_next();
		cfg.dst_short_addr = (uint16_t)rng_next();
		cfg.src_short_addr = (uint16_t)rng_next();
		cfg.mac_seq = (uint8_t)rng_next();
		cfg.ack_request = (rng_next() & 1U) != 0U;

		r.node_id = (uint8_t)rng_next();
		r.seq = (uint16_t)rng_next();
		r.light = (uint16_t)rng_next();
		r.temp_c_x100 = (int16_t)(uint16_t)rng_next();
		r.accel[0] = (int16_t)(uint16_t)rng_next();
		r.accel[1] = (int16_t)(uint16_t)rng_next();
		r.accel[2] = (int16_t)(uint16_t)rng_next();
		r.uptime_ms = rng_next();
		r.flags = (uint8_t)rng_next();

		zassert_equal(sensor_frame_encode(frame, sizeof(frame), &cfg, &r),
			      SENSOR_FRAME_LEN, "encode failed at iter %d", iter);
		zassert_ok(sensor_frame_decode(frame, sizeof(frame), &out),
			   "decode failed at iter %d", iter);
		assert_reading_eq(&out, &r);

		zassert_equal(sensor_frame_encode(reencoded, sizeof(reencoded),
						  &cfg, &out),
			      SENSOR_FRAME_LEN, "re-encode failed at iter %d", iter);
		zassert_mem_equal(reencoded, frame, sizeof(frame),
				  "re-encode diverged at iter %d", iter);
	}
}

/* decode() takes a const pointer; make sure it really does not write. */
ZTEST(sensor_frame, test_decode_does_not_modify_input)
{
	uint8_t frame[SENSOR_FRAME_LEN];
	uint8_t snapshot[SENSOR_FRAME_LEN];
	struct sensor_reading out;

	memcpy(frame, golden_frame, sizeof(frame));
	memcpy(snapshot, frame, sizeof(snapshot));

	zassert_ok(sensor_frame_decode(frame, sizeof(frame), &out),
		   "decode failed");
	zassert_mem_equal(frame, snapshot, sizeof(frame),
			  "decoder modified its input buffer");
}

/*
 * Decoder-side field isolation. Build raw frames (known header + all-zero
 * payload) with exactly one payload field set to all-ones, and check that the
 * decoder fills exactly the matching struct field and leaves the rest zero.
 * This characterizes decode on its own, without trusting the encoder.
 */
static void assert_payload_field_decodes_only(size_t off, size_t len,
					      const struct sensor_reading *expected)
{
	uint8_t frame[SENSOR_FRAME_LEN];
	struct sensor_reading out;
	size_t i;

	memcpy(frame, golden_frame, SENSOR_FRAME_HEADER_LEN);
	memset(&frame[SENSOR_FRAME_HEADER_LEN], 0, SENSOR_PAYLOAD_LEN);
	for (i = 0; i < len; i++) {
		frame[off + i] = 0xFF;
	}

	zassert_ok(sensor_frame_decode(frame, sizeof(frame), &out),
		   "decode failed for field [%zu,%zu)", off, off + len);
	assert_reading_eq(&out, expected);
}

ZTEST(sensor_frame, test_decode_payload_field_isolation)
{
	struct sensor_reading r;

	r = (struct sensor_reading){ .node_id = 0xFF };
	assert_payload_field_decodes_only(9, 1, &r);

	r = (struct sensor_reading){ .seq = 0xFFFF };
	assert_payload_field_decodes_only(10, 2, &r);

	r = (struct sensor_reading){ .light = 0xFFFF };
	assert_payload_field_decodes_only(12, 2, &r);

	r = (struct sensor_reading){ .temp_c_x100 = -1 };
	assert_payload_field_decodes_only(14, 2, &r);

	r = (struct sensor_reading){ .accel = { -1, 0, 0 } };
	assert_payload_field_decodes_only(16, 2, &r);

	r = (struct sensor_reading){ .accel = { 0, -1, 0 } };
	assert_payload_field_decodes_only(18, 2, &r);

	r = (struct sensor_reading){ .accel = { 0, 0, -1 } };
	assert_payload_field_decodes_only(20, 2, &r);

	r = (struct sensor_reading){ .uptime_ms = 0xFFFFFFFFU };
	assert_payload_field_decodes_only(22, 4, &r);

	r = (struct sensor_reading){ .flags = 0xFF };
	assert_payload_field_decodes_only(26, 1, &r);
}

ZTEST_SUITE(sensor_frame, NULL, NULL, NULL, NULL, NULL);
