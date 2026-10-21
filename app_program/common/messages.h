#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

/* Shared message structs, mailbox/queue IDs, and pin assignments.
 * Every buddy's task depends on this file -- route changes through
 * a pull request so nobody's build breaks silently.
 *
 * MQTT records (docs/comms/MQTT_SCHEMA.md, schema v2).  Packed with explicit
 * shifts and masks, never C bit-fields, LSB-first and little-endian: bit n of
 * a record is bit (n mod 8) of byte (n div 8).  Plain C with no kernel or
 * hardware calls, so the same code runs in the PC unit test
 * (tests/host/test_messages.c).  This file and MQTT_SCHEMA.md change in the
 * same pull request. */

#include <stdint.h>

/* ------------------------------------------------------------------ *
 *  Record header -- every outbound record starts with it (schema 3.6)
 * ------------------------------------------------------------------ */

#define MSG_SCHEMA_VERSION   2U

#define MSG_REC_S            0U   /* periodic state line, ...telemetry */
#define MSG_REC_E            1U   /* edge event, ...telemetry          */
#define MSG_REC_H            2U   /* heartbeat, ...heartbeat           */
#define MSG_REC_D            3U   /* snapshot, ...state                */

#define MSG_HDR_T_BITS       2U
#define MSG_HDR_V_BITS       3U
#define MSG_HDR_S_BITS      16U
#define MSG_HDR_UP_BITS     32U
#define MSG_HDR_BITS        (MSG_HDR_T_BITS + MSG_HDR_V_BITS \
                             + MSG_HDR_S_BITS + MSG_HDR_UP_BITS)   /* 53 */

/* Bytes needed for a record of n bits. */
#define MSG_BYTES(bits)     (((bits) + 7U) / 8U)

/* H is the header alone (schema section 6). */
#define MSG_H_BITS          MSG_HDR_BITS
#define MSG_H_LEN           MSG_BYTES(MSG_H_BITS)                  /* 7 */

typedef struct {
    uint8_t  t;    /* MSG_REC_*                                  */
    uint8_t  v;    /* schema version, MSG_SCHEMA_VERSION         */
    uint16_t s;    /* per-run sequence number, shared by all records */
    uint32_t up;   /* ms since boot, tk_get_otm() low word        */
} T_MSG_HEADER;

/* ------------------------------------------------------------------ *
 *  Bit writer / reader
 * ------------------------------------------------------------------ */

/* Write the low `width` bits of `value` at bit offset `*pos` and advance it.
 * The buffer must be zeroed first: bits are OR-ed in.  width <= 32. */
static inline void msg_put_bits(uint8_t *buf, uint32_t *pos, uint32_t value,
                                uint32_t width)
{
    for(uint32_t i = 0; i < width; i++, (*pos)++) {
        if((value >> i) & 1U) {
            buf[*pos / 8U] |= (uint8_t)(1U << (*pos % 8U));
        }
    }
}

/* Read `width` bits at bit offset `*pos` and advance it.  width <= 32. */
static inline uint32_t msg_get_bits(const uint8_t *buf, uint32_t *pos,
                                    uint32_t width)
{
    uint32_t value = 0;

    for(uint32_t i = 0; i < width; i++, (*pos)++) {
        if((buf[*pos / 8U] >> (*pos % 8U)) & 1U) value |= 1UL << i;
    }
    return value;
}

static inline void msg_put_header(uint8_t *buf, uint32_t *pos,
                                  const T_MSG_HEADER *h)
{
    msg_put_bits(buf, pos, h->t, MSG_HDR_T_BITS);
    msg_put_bits(buf, pos, h->v, MSG_HDR_V_BITS);
    msg_put_bits(buf, pos, h->s, MSG_HDR_S_BITS);
    msg_put_bits(buf, pos, h->up, MSG_HDR_UP_BITS);
}

static inline void msg_get_header(const uint8_t *buf, uint32_t *pos,
                                  T_MSG_HEADER *h)
{
    h->t = (uint8_t)msg_get_bits(buf, pos, MSG_HDR_T_BITS);
    h->v = (uint8_t)msg_get_bits(buf, pos, MSG_HDR_V_BITS);
    h->s = (uint16_t)msg_get_bits(buf, pos, MSG_HDR_S_BITS);
    h->up = msg_get_bits(buf, pos, MSG_HDR_UP_BITS);
}

/* ------------------------------------------------------------------ *
 *  Record H -- heartbeat (7 bytes)
 * ------------------------------------------------------------------ */

/* Pack a heartbeat into out[MSG_H_LEN].  Returns the length. */
static inline uint32_t msg_pack_h(uint8_t out[MSG_H_LEN], uint16_t s,
                                  uint32_t up)
{
    const T_MSG_HEADER h = { MSG_REC_H, MSG_SCHEMA_VERSION, s, up };
    uint32_t pos = 0;

    for(uint32_t i = 0; i < MSG_H_LEN; i++) out[i] = 0;
    msg_put_header(out, &pos, &h);
    return MSG_H_LEN;
}

#endif /* COMMON_MESSAGES_H */
