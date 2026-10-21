/*
 * Host unit test for app_program/common/messages.h (M7 gate: pack/unpack
 * round trip on a PC).  Lives outside app_program/ because every .c there is
 * compiled into the firmware.
 *
 * The expected bytes come from docs/comms/Tribunal/packed_size.py, the same
 * generator as MQTT_SCHEMA.md section 10, so the C packer is checked against
 * an independent implementation rather than against itself.
 *
 * Run in the build container (there is no host compiler):
 *   docker compose run --rm build bash -lc \
 *     "gcc -std=c99 -Wall -Wextra -Werror -o /tmp/t tests/host/test_messages.c && /tmp/t"
 */

#include <stdio.h>
#include <string.h>

#include "../../app_program/common/messages.h"

static int failures;

static void check_bytes(const char *name, const uint8_t *got,
                        const uint8_t *want, uint32_t len)
{
    if(memcmp(got, want, len) == 0) {
        printf("ok   %s\n", name);
        return;
    }
    failures++;
    printf("FAIL %s\n  got: ", name);
    for(uint32_t i = 0; i < len; i++) printf("%02x ", got[i]);
    printf("\n  want:");
    for(uint32_t i = 0; i < len; i++) printf(" %02x", want[i]);
    printf("\n");
}

static void check_h(const char *name, uint16_t s, uint32_t up,
                    const uint8_t want[MSG_H_LEN])
{
    uint8_t rec[MSG_H_LEN];
    T_MSG_HEADER h;
    uint32_t pos = 0;
    uint32_t len = msg_pack_h(rec, s, up);

    if(len != 7U) {
        failures++;
        printf("FAIL %s: length %u, want 7\n", name, len);
    }
    check_bytes(name, rec, want, MSG_H_LEN);

    msg_get_header(rec, &pos, &h);
    if(h.t != MSG_REC_H || h.v != MSG_SCHEMA_VERSION || h.s != s
       || h.up != up || pos != MSG_H_BITS) {
        failures++;
        printf("FAIL %s round trip: t=%u v=%u s=%u up=%u pos=%u\n",
               name, h.t, h.v, h.s, h.up, pos);
    } else {
        printf("ok   %s round trip\n", name);
    }
    /* The 3 bits after the 53-bit header must stay zero. */
    if(rec[6] & 0xe0U) {
        failures++;
        printf("FAIL %s: padding bits set in byte 6 (%02x)\n", name, rec[6]);
    }
}

int main(void)
{
    /* MQTT_SCHEMA.md section 10's E example has this header (s 138,
       up 41310); only t differs, 1 there and 2 here. */
    static const uint8_t h_example[] = { 0x4a, 0x11, 0xc0, 0x2b, 0x14, 0x00, 0x00 };
    static const uint8_t h_min[]     = { 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static const uint8_t h_max[]     = { 0xea, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f };

    check_h("H s=138 up=41310", 138, 41310, h_example);
    check_h("H s=0 up=0 (minimum)", 0, 0, h_min);
    check_h("H s=65535 up=0xffffffff (maximum)", 65535, 0xffffffffUL, h_max);

    printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}
