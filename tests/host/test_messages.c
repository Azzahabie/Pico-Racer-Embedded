/*
 * Host unit test for app_program/common/messages.h (M7 gate: pack/unpack
 * round trip on a PC).  Lives outside app_program/ because every .c there is
 * compiled into the firmware.
 *
 * The expected bytes come from docs/comms/Tribunal/packed_size.py through
 * tests/host/gen_vectors.py (vectors.h), so the C packer is checked against an
 * independent implementation rather than against itself.
 *
 * Run in the build container (there is no host compiler):
 *   docker compose run --rm build bash -lc \
 *     "gcc -std=c99 -Wall -Wextra -Werror -o /tmp/t tests/host/test_messages.c && /tmp/t"
 */

#include <stdio.h>
#include <string.h>

#include "../../app_program/common/messages.h"
#include "vectors.h"

static int failures;
static int passes;

static void ok(int cond, const char *what, const char *name)
{
    if(cond) {
        passes++;
    } else {
        failures++;
        printf("FAIL %s: %s\n", name, what);
    }
}

static void check_bytes(const char *name, const uint8_t *got, uint32_t got_len,
                        const uint8_t *want, uint32_t want_len)
{
    if(got_len == want_len && memcmp(got, want, want_len) == 0) {
        passes++;
        return;
    }
    failures++;
    printf("FAIL %s\n  got  (%u):", name, got_len);
    for(uint32_t i = 0; i < got_len; i++) printf(" %02x", got[i]);
    printf("\n  want (%u):", want_len);
    for(uint32_t i = 0; i < want_len; i++) printf(" %02x", want[i]);
    printf("\n");
}

/* Build a record from a field-width list with the generic bit writer. */
static uint32_t pack_fields(uint8_t *out, uint32_t size, const uint8_t *widths,
                            const uint32_t *values, uint32_t n)
{
    uint32_t pos = 0;

    comm_w_zero(out, size);
    for(uint32_t i = 0; i < n; i++) {
        comm_w_put_bits(out, &pos, values[i], widths[i]);
    }
    return COMM_W_BYTES(pos);
}

/* 1. Sanity: the bit writer reproduces schema v2's S example exactly. */
static void test_s_v2_sanity(void)
{
    static const uint8_t widths[] = {2, 3, 16, 32, 5, 2, 8, 2, 8, 5, 2, 4, 8, 8, 2, 3, 4, 4};
    static const uint32_t values[] = {0, 2, 137, 41230, 0, 2, 20, 1, 3 + 128, 4, 2, 3, 12, 0, 0, 1, 3, 0};
    static const uint8_t want[] = {0x28, 0x11, 0xc0, 0x21, 0x14, 0x00, 0x00, 0x48,
                                   0xd1, 0x20, 0x71, 0x18, 0x00, 0xc8, 0x00};
    uint8_t rec[15];
    uint32_t len = pack_fields(rec, sizeof rec, widths, values, sizeof widths);

    check_bytes("S v2 sanity (MQTT_SCHEMA v2 section 10)", rec, len, want, sizeof want);
}

/* 2. S v3: mot widened to 4 bits, still 15 bytes. */
static void test_s_v3(void)
{
    static const uint8_t widths[] = {2, 3, 16, 32, 5, 4, 8, 2, 8, 5, 2, 4, 8, 8, 2, 3, 4, 4};
    uint8_t rec[15];
    uint32_t len = pack_fields(rec, sizeof rec, widths, s_v3_values, sizeof widths);

    check_bytes("S v3 example", rec, len, s_v3_bytes, sizeof s_v3_bytes);
}

/* 2b. The firmware S packer agrees with the field list, and clamps. */
static void test_pack_s(void)
{
    const uint32_t *v = s_v3_values;   /* t, v, s, up, then the 14 fields */
    T_COMM_W_S st = {
        (uint8_t)v[4], (uint8_t)v[5], (uint8_t)v[6], (uint8_t)v[7], (uint8_t)v[8],
        (uint8_t)v[9], (uint8_t)v[10], (uint8_t)v[11], (uint8_t)v[12], (uint8_t)v[13],
        (uint8_t)v[14], (uint8_t)v[15], (uint8_t)v[16], (uint8_t)v[17],
    };
    uint8_t rec[COMM_W_S_SIZE];
    uint8_t ref[COMM_W_S_SIZE];
    T_COMM_W_S big = st;
    uint32_t len;

    ok(COMM_W_S_BITS == 120U && COMM_W_S_SIZE == 15U, "S is 120 bits, 15 B", "S size");
    len = comm_w_pack_s(rec, (uint16_t)v[2], v[3], &st);
    check_bytes("comm_w_pack_s, S v3 example", rec, len, s_v3_bytes, sizeof s_v3_bytes);

    /* mot 20 does not fit 4 bits: stored 15, every other field unchanged. */
    big.mot = 20;
    st.mot = 15;
    comm_w_pack_s(rec, (uint16_t)v[2], v[3], &big);
    comm_w_pack_s(ref, (uint16_t)v[2], v[3], &st);
    check_bytes("comm_w_pack_s clamps mot 20 -> 15", rec, sizeof rec, ref, sizeof ref);
}

/* 3. H, schema v3. */
static void test_h(void)
{
    static const uint8_t h_min[] = {0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t h_ex[]  = {0x0e, 0x12, 0x00, 0x82, 0x14, 0x00, 0x00};
    static const uint8_t h_max[] = {0xee, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f};
    uint8_t rec[COMM_W_H_SIZE];
    T_COMM_W_HEADER h;
    uint32_t pos = 0;

    check_bytes("H s=0 up=0", rec, comm_w_pack_h(rec, 0, 0), h_min, sizeof h_min);
    check_bytes("H s=144 up=42000 (section 10)", rec, comm_w_pack_h(rec, 144, 42000),
                h_ex, sizeof h_ex);
    check_bytes("H max", rec, comm_w_pack_h(rec, 65535, 0xffffffffUL), h_max, sizeof h_max);
    comm_w_get_header(rec, &pos, &h);
    ok(h.t == COMM_W_REC_H && h.v == 3 && h.s == 65535 && h.up == 0xffffffffUL
       && pos == COMM_W_HDR_BITS, "header round trip", "H max");
}

static uint32_t pack_vector(const T_VECTOR *v, uint8_t *body)
{
    const int32_t *a = v->in;

    switch(v->kind) {
    case COMM_W_KIND_MOTION:
        return comm_w_pack_motion(body, (uint32_t)a[0], a[1], a[2], (uint32_t)a[3],
                                  a[4], (uint32_t)a[5], (uint32_t)a[6]);
    case COMM_W_KIND_LINE:
        return comm_w_pack_line(body, (uint32_t)a[0], a[1]);
    case COMM_W_KIND_BARCODE:
        return comm_w_pack_barcode(body, (uint32_t)a[0], (uint32_t)a[1]);
    case COMM_W_KIND_IMU:
        return comm_w_pack_imu(body, (uint32_t)a[0], (uint32_t)a[1], (uint32_t)a[2]);
    case COMM_W_KIND_OBSTACLE:
        return comm_w_pack_obstacle(body, (uint32_t)a[0], (uint32_t)a[1]);
    default:
        return comm_w_pack_mission(body, (uint32_t)a[0]);
    }
}

/* The unpacked fields after `kind`, in body order, as stored. */
static uint32_t unpacked_fields(const T_COMM_W_BODY *b, uint32_t out[8])
{
    switch(b->kind) {
    case COMM_W_KIND_MOTION:
        out[0] = b->motion_state; out[1] = b->speed_l; out[2] = b->speed_r;
        out[3] = b->dist_mm; out[4] = b->turn_rate; out[5] = b->fault; out[6] = b->ovf;
        return 7;
    case COMM_W_KIND_LINE:
        out[0] = b->line_state; out[1] = b->deviation;
        return 2;
    case COMM_W_KIND_BARCODE:
        out[0] = b->letter; out[1] = b->action;
        return 2;
    case COMM_W_KIND_IMU:
        out[0] = b->event_class; out[1] = b->hump_count; out[2] = b->hump_height_mm;
        return 3;
    case COMM_W_KIND_OBSTACLE:
        out[0] = b->width_cm; out[1] = b->bypass;
        return 2;
    default:
        out[0] = b->mission_state;
        return 1;
    }
}

/* 4. Every shared vector: pack -> bytes, unpack -> fields, copy -> wire E. */
static void test_vectors(void)
{
    for(uint32_t i = 0; i < sizeof vectors / sizeof vectors[0]; i++) {
        const T_VECTOR *v = &vectors[i];
        uint8_t body[COMM_W_INTAKE_SIZE_MAX];
        uint8_t e[COMM_W_E_SIZE_LIMIT];
        T_COMM_W_BODY b;
        uint32_t fields[8];
        uint32_t len, n, elen;
        char name[80];

        len = pack_vector(v, body);
        snprintf(name, sizeof name, "%s: body", v->name);
        check_bytes(name, body, len, v->body, v->body_len);

        memset(&b, 0, sizeof b);
        snprintf(name, sizeof name, "%s: unpack", v->name);
        ok(comm_w_unpack_body(body, len, &b) == COMM_W_OK && b.kind == v->kind,
           "unpack accepted", name);
        n = unpacked_fields(&b, fields);
        ok(memcmp(fields, v->stored, n * sizeof fields[0]) == 0, "fields round trip", name);

        elen = comm_w_copy_body(e, sizeof e, v->s, v->up, body, len);
        snprintf(name, sizeof name, "%s: wire E", v->name);
        check_bytes(name, e, elen, v->e, v->e_len);
        ok(elen == comm_w_e_size(v->kind), "E size matches the kind", name);
    }
}

/* 5. Rejection: reserved kinds and wrong lengths. */
static void test_reject(void)
{
    uint8_t body[COMM_W_INTAKE_SIZE_MAX + 1] = {0};
    uint8_t e[COMM_W_E_SIZE_LIMIT];
    T_COMM_W_BODY b;

    for(uint32_t kind = 6; kind <= 7; kind++) {
        body[0] = (uint8_t)kind;
        ok(comm_w_unpack_body(body, 2, &b) == COMM_W_ERR_KIND, "kind rejected", kind == 6 ? "kind 6" : "kind 7");
        ok(comm_w_copy_body(e, sizeof e, 0, 0, body, 2) == 0, "copy rejected", kind == 6 ? "kind 6" : "kind 7");
    }
    for(uint32_t kind = 0; kind < COMM_W_KIND_COUNT; kind++) {
        uint32_t good = comm_w_intake_size(kind);
        char name[40];

        comm_w_zero(body, sizeof body);
        body[0] = (uint8_t)kind;
        snprintf(name, sizeof name, "kind %u wrong length", kind);
        ok(comm_w_unpack_body(body, good - 1U, &b) == COMM_W_ERR_LENGTH, "short rejected", name);
        ok(comm_w_unpack_body(body, good + 1U, &b) == COMM_W_ERR_LENGTH, "long rejected", name);
        ok(comm_w_unpack_body(body, good, &b) == COMM_W_OK, "exact accepted", name);
        ok(comm_w_copy_body(e, sizeof e, 0, 0, body, good + 1U) == 0, "copy rejects length", name);
        ok(comm_w_copy_body(e, comm_w_e_size(kind) - 1U, 0, 0, body, good) == 0,
           "copy rejects a short output", name);
    }
    ok(comm_w_unpack_body(body, 0, &b) == COMM_W_ERR_LENGTH, "empty rejected", "length 0");
}

/* 6. Sizes and the dist_mm wrap. */
static void test_sizes_and_wrap(void)
{
    static const uint32_t intake[] = {7, 2, 2, 3, 2, 1};
    static const uint32_t wire[] = {13, 9, 8, 9, 9, 8};

    for(uint32_t k = 0; k < COMM_W_KIND_COUNT; k++) {
        ok(comm_w_intake_size(k) == intake[k] && comm_w_e_size(k) == wire[k],
           "intake and E sizes", "per-kind sizes");
    }
    ok(comm_w_intake_size(6) == 0 && comm_w_e_size(7) == 0, "reserved sizes 0", "per-kind sizes");
    ok(comm_w_dist_delta_mm(0, 32767) == 1, "32767 -> 0 is 1 mm", "dist wrap");
    ok(comm_w_dist_delta_mm(4, 32760) == 12, "across the wrap", "dist wrap");
    ok(comm_w_dist_delta_mm(1500, 1000) == 500, "no wrap", "dist wrap");
}

int main(void)
{
    test_s_v2_sanity();
    test_s_v3();
    test_pack_s();
    test_h();
    test_vectors();
    test_reject();
    test_sizes_and_wrap();

    printf("%d checks passed, %d failed\n", passes, failures);
    return failures ? 1 : 0;
}
