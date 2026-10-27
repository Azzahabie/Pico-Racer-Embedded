#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

/* Shared message structs, mailbox/queue IDs, and pin assignments.
 * Every buddy's task depends on this file -- route changes through
 * a pull request so nobody's build breaks silently.
 *
 * MQTT records and event bodies (docs/comms/MQTT_SCHEMA.md, schema v4).
 * Packed with explicit shifts and masks, never C bit-fields, LSB-first and
 * little-endian: bit n of a record is bit (n mod 8) of byte (n div 8).  Plain
 * C with no kernel or hardware calls, so the same code runs in the PC unit
 * test (tests/host/test_messages.c).  This file and MQTT_SCHEMA.md change in
 * the same commit.
 *
 * Naming: everything that packs or unpacks a bit layout is prefixed comm_w_
 * (comms, wire), so it is obvious the bytes are the packed wire format and not
 * a C struct.  Producers only ever call their comm_w_pack_* helper: it does
 * the clamping and offset-encoding, so no producer writes a shift or a mask.
 * Each event body is owned by the subsystem that produces it; to change one,
 * ask Buddy 1 (MQTT_SCHEMA.md section 5.2).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ------------------------------------------------------------------ *
 *  motion_flg bits (Buddy 2 / Buddy 5 integration)
 * ------------------------------------------------------------------ */

#define MOTION_EVT_TICK        (1U << 0)   /* motion_task cyclic handler */
#define MOTION_EVT_CMD         (1U << 1)   /* queued motion command       */
#define MOTION_EVT_STOP_SCAN   (1U << 2)   /* persistent obstacle stop   */
#define MOTION_EVT_STOP_IMU    (1U << 3)   /* persistent IMU stop         */
#define MOTION_EVT_STOPS       (MOTION_EVT_STOP_SCAN | MOTION_EVT_STOP_IMU)

/* scan_task -> motion_task avoid command, stored in a 16-bit MBF payload.
 * Bits 0-2 are the operation, bit 3 is the bypass side (0=left, 1=right),
 * and bits 4-15 are the distance in mm. Side is zero for reverse commands. */
#define MOTION_AVOID_OP_REVERSE    1U
#define MOTION_AVOID_OP_BYPASS     2U
#define MOTION_AVOID_SIDE_NONE     0U
#define MOTION_AVOID_SIDE_LEFT     1U
#define MOTION_AVOID_SIDE_RIGHT    2U
#define MOTION_AVOID_DIST_MAX      4095U

static inline bool motion_avoid_cmd_pack(uint8_t op, uint8_t side,
                                         uint16_t dist_mm, uint16_t *p_word)
{
    bool valid;
    uint8_t side_bit = 0U;

    valid = (NULL != p_word) && (0U < dist_mm) &&
            (dist_mm <= MOTION_AVOID_DIST_MAX) &&
            (((MOTION_AVOID_OP_REVERSE == op) &&
              (MOTION_AVOID_SIDE_NONE == side)) ||
             ((MOTION_AVOID_OP_BYPASS == op) &&
              ((MOTION_AVOID_SIDE_LEFT == side) ||
               (MOTION_AVOID_SIDE_RIGHT == side))));
    if (valid) {
        if (MOTION_AVOID_SIDE_RIGHT == side) {
            side_bit = 1U;
        }
        *p_word = (uint16_t)(((uint16_t)op & 0x07U) |
                             ((uint16_t)side_bit << 3) |
                             ((dist_mm & 0x0FFFU) << 4));
    }
    return valid;
}

static inline bool motion_avoid_cmd_unpack(uint16_t word, uint8_t *p_op,
                                           uint8_t *p_side,
                                           uint16_t *p_dist_mm)
{
    uint8_t op = (uint8_t)(word & 0x07U);
    uint8_t side_bit = (uint8_t)((word >> 3) & 0x01U);
    uint8_t side = MOTION_AVOID_SIDE_NONE;
    uint16_t dist_mm = (uint16_t)((word >> 4) & 0x0FFFU);
    uint16_t check_word;

    if (MOTION_AVOID_OP_BYPASS == op) {
        side = (0U == side_bit) ? MOTION_AVOID_SIDE_LEFT
                                : MOTION_AVOID_SIDE_RIGHT;
    }
    if ((NULL == p_op) || (NULL == p_side) || (NULL == p_dist_mm) ||
        !motion_avoid_cmd_pack(op, side, dist_mm, &check_word) ||
        (word != check_word)) {
        return false;
    }
    *p_op = op;
    *p_side = side;
    *p_dist_mm = dist_mm;
    return true;
}

/* ------------------------------------------------------------------ *
 *  Shared fault bitmask (schema section 4.1)
 * ------------------------------------------------------------------ */

/* Bit positions in the shared fault event flag that nav_ctrl_task creates.
 * Owned by Buddy 5; agreed with Buddies 1, 2, 4 and 5 on 2026-10-23.  Each
 * task sets and clears only its own bit.  Record S carries these bits as they
 * are, so moving a bit or adding one changes the wire: send it to Buddy 1 as
 * a pull request (MQTT_SCHEMA.md section 5.2). */
#define FAULT_MOTOR_STALL       (1U << 0)   /* motion_task, core 1 (Buddy 2)    */
#define FAULT_ENCODER           (1U << 1)   /* motion_task, core 1 (Buddy 2)    */
#define FAULT_IMU               (1U << 2)   /* imu_task, core 1 (Buddy 4)       */
#define FAULT_SCAN_SENSOR_LOST  (1U << 3)   /* scan_task, core 1 (Buddy 5)      */
#define FAULT_LINE_SENSOR       (1U << 4)   /* ir_sense_task, core 1 (Buddy 3)  */
#define FAULT_WIFI_LOSS         (1U << 5)   /* comms_task, core 0 (Buddy 1)     */
#define FAULT_COUNT             6U
#define FAULT_ALL               ((1U << FAULT_COUNT) - 1U)

/* The bits that send the mission to Fault.  Wi-Fi loss is report-only: it is
 * there for the laptop, and a mission never waits on Wi-Fi, so nav_ctrl_task
 * tests the flag against this mask, never against FAULT_ALL. */
#define FAULT_STOP_MASK         (FAULT_ALL & ~FAULT_WIFI_LOSS)

/* ------------------------------------------------------------------ *
 *  Record header -- every outbound record starts with it (schema 3.6)
 * ------------------------------------------------------------------ */

#define COMM_W_SCHEMA_VERSION   4U

#define COMM_W_REC_S            0U   /* periodic state line, ...telemetry */
#define COMM_W_REC_E            1U   /* edge event, ...telemetry          */
#define COMM_W_REC_H            2U   /* heartbeat, ...heartbeat           */
#define COMM_W_REC_D            3U   /* snapshot, ...state                */

#define COMM_W_HDR_T_BITS       2U
#define COMM_W_HDR_V_BITS       3U
#define COMM_W_HDR_S_BITS      16U
#define COMM_W_HDR_UP_BITS     32U
#define COMM_W_HDR_BITS        (COMM_W_HDR_T_BITS + COMM_W_HDR_V_BITS \
                                + COMM_W_HDR_S_BITS + COMM_W_HDR_UP_BITS) /* 53 */

/* Bytes needed for n bits. */
#define COMM_W_BYTES(bits)     (((bits) + 7U) / 8U)

/* H is the header alone (schema section 6). */
#define COMM_W_H_SIZE          COMM_W_BYTES(COMM_W_HDR_BITS)            /* 7 */

typedef struct {
    uint8_t  t;    /* COMM_W_REC_*                                  */
    uint8_t  v;    /* schema version, COMM_W_SCHEMA_VERSION         */
    uint16_t s;    /* per-run sequence number, shared by all records */
    uint32_t up;   /* ms since boot, tk_get_otm() low word           */
} T_COMM_W_HEADER;

/* ------------------------------------------------------------------ *
 *  Event bodies (schema section 5)
 *
 *  An event body is `kind` in bits 0-2 followed by that kind's fields at
 *  their minimum widths; unused trailing bits are zero.  The same body
 *  travels on the intake queue (tele_mbf, COMM_W_INTAKE_SIZE_<KIND> bytes)
 *  and, after the 53-bit header, on the wire as record E
 *  (COMM_W_E_SIZE_<KIND> bytes).
 * ------------------------------------------------------------------ */

#define COMM_W_KIND_BITS        3U

#define COMM_W_KIND_MOTION      0U   /* Buddy 2            */
#define COMM_W_KIND_LINE        1U   /* Buddy 3            */
#define COMM_W_KIND_BARCODE     2U   /* Buddy 3            */
#define COMM_W_KIND_IMU         3U   /* Buddy 4            */
#define COMM_W_KIND_OBSTACLE    4U   /* Buddy 5            */
#define COMM_W_KIND_MISSION     5U   /* Vehicle Controller */
#define COMM_W_KIND_COUNT       6U   /* 6 and 7 are reserved and rejected */

/* motion (Buddy 2).  PLACEHOLDER: speed_l, speed_r and ovf are Buddy 1's
   proposal until Buddy 2 confirms the unit and range. */
#define COMM_W_MOTION_STATE_BITS   4U   /* 0-15                               */
#define COMM_W_MOTION_SPEED_BITS   8U   /* PLACEHOLDER: -128..127 cm/s, +128  */
#define COMM_W_MOTION_DIST_BITS   15U   /* mm, wrapping counter 0..32767       */
#define COMM_W_MOTION_TURN_BITS   10U   /* -512..511 deg/s, +512               */
#define COMM_W_MOTION_FAULT_BITS   2U   /* bit 0 stall, bit 1 encoder          */
#define COMM_W_MOTION_OVF_BITS     1U   /* PLACEHOLDER: a field was clamped   */
#define COMM_W_BODY_BITS_MOTION   (COMM_W_KIND_BITS + COMM_W_MOTION_STATE_BITS \
                                   + 2U * COMM_W_MOTION_SPEED_BITS            \
                                   + COMM_W_MOTION_DIST_BITS                   \
                                   + COMM_W_MOTION_TURN_BITS                   \
                                   + COMM_W_MOTION_FAULT_BITS                  \
                                   + COMM_W_MOTION_OVF_BITS)              /* 51 */

/* line (Buddy 3) */
#define COMM_W_LINE_STATE_BITS     2U   /* 0 on-line, 1 lost, 2 crossing */
#define COMM_W_LINE_DEV_BITS       8U   /* -128..127 tenths, +128        */
#define COMM_W_BODY_BITS_LINE     (COMM_W_KIND_BITS + COMM_W_LINE_STATE_BITS \
                                   + COMM_W_LINE_DEV_BITS)                /* 13 */

/* barcode (Buddy 3) */
#define COMM_W_BARCODE_LETTER_BITS 5U   /* 0 = A .. 25 = Z, 31 = unreadable */
#define COMM_W_BARCODE_ACTION_BITS 2U   /* 0-3                              */
#define COMM_W_BARCODE_UNREADABLE 31U
#define COMM_W_BODY_BITS_BARCODE  (COMM_W_KIND_BITS + COMM_W_BARCODE_LETTER_BITS \
                                   + COMM_W_BARCODE_ACTION_BITS)          /* 10 */

/* imu (Buddy 4) */
#define COMM_W_IMU_CLASS_BITS      3U   /* FR-27 event class, 0-4 */
#define COMM_W_IMU_COUNT_BITS      4U   /* humps, 0-15            */
#define COMM_W_IMU_HEIGHT_BITS     8U   /* mm, 0-255, clamped     */
#define COMM_W_BODY_BITS_IMU      (COMM_W_KIND_BITS + COMM_W_IMU_CLASS_BITS \
                                   + COMM_W_IMU_COUNT_BITS                 \
                                   + COMM_W_IMU_HEIGHT_BITS)              /* 18 */

/* obstacle (Buddy 5) */
#define COMM_W_OBSTACLE_WIDTH_BITS 8U   /* cm, 0-255, clamped        */
#define COMM_W_OBSTACLE_BYPASS_BITS 2U  /* 0 none, 1 left, 2 right   */
#define COMM_W_BODY_BITS_OBSTACLE (COMM_W_KIND_BITS + COMM_W_OBSTACLE_WIDTH_BITS \
                                   + COMM_W_OBSTACLE_BYPASS_BITS)         /* 13 */

/* mission (Vehicle Controller) */
#define COMM_W_MISSION_STATE_BITS  3U   /* 0-6 */
#define COMM_W_BODY_BITS_MISSION  (COMM_W_KIND_BITS + COMM_W_MISSION_STATE_BITS) /* 6 */

#define COMM_W_INTAKE_SIZE_MOTION   COMM_W_BYTES(COMM_W_BODY_BITS_MOTION)    /* 7 */
#define COMM_W_INTAKE_SIZE_LINE     COMM_W_BYTES(COMM_W_BODY_BITS_LINE)      /* 2 */
#define COMM_W_INTAKE_SIZE_BARCODE  COMM_W_BYTES(COMM_W_BODY_BITS_BARCODE)   /* 2 */
#define COMM_W_INTAKE_SIZE_IMU      COMM_W_BYTES(COMM_W_BODY_BITS_IMU)       /* 3 */
#define COMM_W_INTAKE_SIZE_OBSTACLE COMM_W_BYTES(COMM_W_BODY_BITS_OBSTACLE)  /* 2 */
#define COMM_W_INTAKE_SIZE_MISSION  COMM_W_BYTES(COMM_W_BODY_BITS_MISSION)   /* 1 */
#define COMM_W_INTAKE_SIZE_MAX      COMM_W_INTAKE_SIZE_MOTION

#define COMM_W_E_SIZE_MOTION   COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_MOTION)   /* 13 */
#define COMM_W_E_SIZE_LINE     COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_LINE)     /* 9 */
#define COMM_W_E_SIZE_BARCODE  COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_BARCODE)  /* 8 */
#define COMM_W_E_SIZE_IMU      COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_IMU)      /* 9 */
#define COMM_W_E_SIZE_OBSTACLE COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_OBSTACLE) /* 9 */
#define COMM_W_E_SIZE_MISSION  COMM_W_BYTES(COMM_W_HDR_BITS + COMM_W_BODY_BITS_MISSION)  /* 8 */

/* A wire E may not exceed the radio's outbound record limit
   (CYW43_MQTT_RECORD_MAX, 21 B), i.e. a body of at most 115 bits. */
#define COMM_W_E_SIZE_LIMIT    21U
#define COMM_W_E_SIZE_MAX      COMM_W_E_SIZE_MOTION

typedef char comm_w_e_fits[(COMM_W_E_SIZE_MAX <= COMM_W_E_SIZE_LIMIT
                            && COMM_W_INTAKE_SIZE_MAX >= COMM_W_INTAKE_SIZE_IMU
                            && COMM_W_INTAKE_SIZE_MAX >= COMM_W_INTAKE_SIZE_LINE)
                           ? 1 : -1];

/* comm_w_unpack_body() results */
#define COMM_W_OK               0
#define COMM_W_ERR_KIND        (-1)   /* kind 6 or 7          */
#define COMM_W_ERR_LENGTH      (-2)   /* wrong size for kind  */

/* One unpacked event body.  Values are as stored on the wire: offset-encoded
   fields keep their offset (subtract it to get the signed value). */
typedef struct {
    uint8_t  kind;
    uint8_t  motion_state;     /* motion                          */
    uint8_t  speed_l;          /* motion, stored +128 (PLACEHOLDER) */
    uint8_t  speed_r;          /* motion, stored +128 (PLACEHOLDER) */
    uint16_t dist_mm;          /* motion, 15-bit wrapping counter  */
    uint16_t turn_rate;        /* motion, stored +512              */
    uint8_t  fault;            /* motion                          */
    uint8_t  ovf;              /* motion (PLACEHOLDER)            */
    uint8_t  line_state;       /* line                            */
    uint8_t  deviation;        /* line, stored +128               */
    uint8_t  letter;           /* barcode                         */
    uint8_t  action;           /* barcode                         */
    uint8_t  event_class;      /* imu                             */
    uint8_t  hump_count;       /* imu                             */
    uint8_t  hump_height_mm;   /* imu                             */
    uint8_t  width_cm;         /* obstacle                        */
    uint8_t  bypass;           /* obstacle                        */
    uint8_t  mission_state;    /* mission                         */
} T_COMM_W_BODY;

/* ------------------------------------------------------------------ *
 *  Bit writer / reader
 * ------------------------------------------------------------------ */

/* Write the low `width` bits of `value` at bit offset `*pos` and advance it.
 * The buffer must be zeroed first: bits are OR-ed in.  width <= 32. */
static inline void comm_w_put_bits(uint8_t *buf, uint32_t *pos, uint32_t value,
                                   uint32_t width)
{
    for(uint32_t i = 0; i < width; i++, (*pos)++) {
        if((value >> i) & 1U) {
            buf[*pos / 8U] |= (uint8_t)(1U << (*pos % 8U));
        }
    }
}

/* Read `width` bits at bit offset `*pos` and advance it.  width <= 32. */
static inline uint32_t comm_w_get_bits(const uint8_t *buf, uint32_t *pos,
                                       uint32_t width)
{
    uint32_t value = 0;

    for(uint32_t i = 0; i < width; i++, (*pos)++) {
        if((buf[*pos / 8U] >> (*pos % 8U)) & 1U) {
            value |= 1UL << i;
        }
    }
    return value;
}

static inline void comm_w_zero(uint8_t *buf, uint32_t size)
{
    for(uint32_t i = 0; i < size; i++) {
        buf[i] = 0;
    }
}

/* Clamp an unsigned value to `width` bits.  Sets *clamped when it had to. */
static inline uint32_t comm_w_clamp(uint32_t value, uint32_t width,
                                    uint8_t *clamped)
{
    const uint32_t max = (1UL << width) - 1U;

    if(value > max) {
        *clamped = 1U;
        return max;
    }
    return value;
}

/* Offset-encode a signed value into `width` bits (stored = value + 2^(width-1)),
 * clamping to -2^(width-1) .. 2^(width-1)-1.  Sets *clamped when it had to. */
static inline uint32_t comm_w_offset(int32_t value, uint32_t width,
                                     uint8_t *clamped)
{
    const int32_t half = (int32_t)(1L << (width - 1U));

    if(value < -half) {
        *clamped = 1U;
        value = -half;
    } else if(value > half - 1) {
        *clamped = 1U;
        value = half - 1;
    }
    return (uint32_t)(value + half);
}

/* ------------------------------------------------------------------ *
 *  Header and record H
 * ------------------------------------------------------------------ */

static inline void comm_w_put_header(uint8_t *buf, uint32_t *pos,
                                     const T_COMM_W_HEADER *h)
{
    comm_w_put_bits(buf, pos, h->t, COMM_W_HDR_T_BITS);
    comm_w_put_bits(buf, pos, h->v, COMM_W_HDR_V_BITS);
    comm_w_put_bits(buf, pos, h->s, COMM_W_HDR_S_BITS);
    comm_w_put_bits(buf, pos, h->up, COMM_W_HDR_UP_BITS);
}

static inline void comm_w_get_header(const uint8_t *buf, uint32_t *pos,
                                     T_COMM_W_HEADER *h)
{
    h->t = (uint8_t)comm_w_get_bits(buf, pos, COMM_W_HDR_T_BITS);
    h->v = (uint8_t)comm_w_get_bits(buf, pos, COMM_W_HDR_V_BITS);
    h->s = (uint16_t)comm_w_get_bits(buf, pos, COMM_W_HDR_S_BITS);
    h->up = comm_w_get_bits(buf, pos, COMM_W_HDR_UP_BITS);
}

/* Pack a heartbeat into out[COMM_W_H_SIZE].  Returns the length. */
static inline uint32_t comm_w_pack_h(uint8_t out[COMM_W_H_SIZE], uint16_t s,
                                     uint32_t up)
{
    const T_COMM_W_HEADER h = { COMM_W_REC_H, COMM_W_SCHEMA_VERSION, s, up };
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_H_SIZE);
    comm_w_put_header(out, &pos, &h);
    return COMM_W_H_SIZE;
}

/* ------------------------------------------------------------------ *
 *  Record S -- the periodic state line (schema section 4), 16 bytes
 * ------------------------------------------------------------------ */

#define COMM_W_S_FAULTS_BITS    FAULT_COUNT /* shared fault bitmask, 4.1  */
#define COMM_W_S_MOT_BITS       4U   /* motion state                        */
#define COMM_W_S_DST_BITS       8U   /* cm since the last motion change     */
#define COMM_W_S_LIN_BITS       2U   /* line state                          */
#define COMM_W_S_DEV_BITS       8U   /* line deviation, stored +128         */
#define COMM_W_S_BAR_BITS       5U   /* letter index, 31 = none yet         */
#define COMM_W_S_ACT_BITS       2U   /* action code                         */
#define COMM_W_S_HUM_BITS       4U   /* humps detected                      */
#define COMM_W_S_HGT_BITS       8U   /* last hump height, mm                */
#define COMM_W_S_OBS_BITS       8U   /* last obstacle width, cm             */
#define COMM_W_S_BYP_BITS       2U   /* bypass taken                        */
#define COMM_W_S_MIS_BITS       3U   /* mission state                       */
#define COMM_W_S_MPV_BITS       4U   /* barcode table version running       */
#define COMM_W_S_DRP_BITS       4U   /* intake drops, saturating            */
#define COMM_W_S_BITS          (COMM_W_HDR_BITS + COMM_W_S_FAULTS_BITS        \
                                + COMM_W_S_MOT_BITS + COMM_W_S_DST_BITS       \
                                + COMM_W_S_LIN_BITS + COMM_W_S_DEV_BITS       \
                                + COMM_W_S_BAR_BITS + COMM_W_S_ACT_BITS       \
                                + COMM_W_S_HUM_BITS + COMM_W_S_HGT_BITS       \
                                + COMM_W_S_OBS_BITS + COMM_W_S_BYP_BITS       \
                                + COMM_W_S_MIS_BITS + COMM_W_S_MPV_BITS       \
                                + COMM_W_S_DRP_BITS)                  /* 121 */
#define COMM_W_S_SIZE          COMM_W_BYTES(COMM_W_S_BITS)            /* 16 */

/* The latest value per source, as stored on the wire (offset-encoded fields
   keep their offset).  comms_task keeps one and folds every event into it. */
typedef struct {
    uint8_t faults, mot, dst, lin, dev, bar, act, hum, hgt, obs, byp, mis, mpv, drp;
} T_COMM_W_S;

/* Pack record S into out[COMM_W_S_SIZE].  Each field is clamped to its width.
   Returns the length. */
static inline uint32_t comm_w_pack_s(uint8_t out[COMM_W_S_SIZE], uint16_t s,
                                     uint32_t up, const T_COMM_W_S *st)
{
    const T_COMM_W_HEADER h = { COMM_W_REC_S, COMM_W_SCHEMA_VERSION, s, up };
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_S_SIZE);
    comm_w_put_header(out, &pos, &h);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->faults, COMM_W_S_FAULTS_BITS, &clamped), COMM_W_S_FAULTS_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->mot, COMM_W_S_MOT_BITS, &clamped), COMM_W_S_MOT_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->dst, COMM_W_S_DST_BITS, &clamped), COMM_W_S_DST_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->lin, COMM_W_S_LIN_BITS, &clamped), COMM_W_S_LIN_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->dev, COMM_W_S_DEV_BITS, &clamped), COMM_W_S_DEV_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->bar, COMM_W_S_BAR_BITS, &clamped), COMM_W_S_BAR_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->act, COMM_W_S_ACT_BITS, &clamped), COMM_W_S_ACT_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->hum, COMM_W_S_HUM_BITS, &clamped), COMM_W_S_HUM_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->hgt, COMM_W_S_HGT_BITS, &clamped), COMM_W_S_HGT_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->obs, COMM_W_S_OBS_BITS, &clamped), COMM_W_S_OBS_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->byp, COMM_W_S_BYP_BITS, &clamped), COMM_W_S_BYP_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->mis, COMM_W_S_MIS_BITS, &clamped), COMM_W_S_MIS_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->mpv, COMM_W_S_MPV_BITS, &clamped), COMM_W_S_MPV_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(st->drp, COMM_W_S_DRP_BITS, &clamped), COMM_W_S_DRP_BITS);
    return COMM_W_S_SIZE;
}

/* ------------------------------------------------------------------ *
 *  Per-kind size tables
 * ------------------------------------------------------------------ */

/* Body bits for a kind, or 0 for a reserved kind. */
static inline uint32_t comm_w_body_bits(uint32_t kind)
{
    static const uint8_t bits[COMM_W_KIND_COUNT] = {
        COMM_W_BODY_BITS_MOTION, COMM_W_BODY_BITS_LINE,
        COMM_W_BODY_BITS_BARCODE, COMM_W_BODY_BITS_IMU,
        COMM_W_BODY_BITS_OBSTACLE, COMM_W_BODY_BITS_MISSION,
    };

    return (kind < COMM_W_KIND_COUNT) ? bits[kind] : 0U;
}

/* Intake (tele_mbf) message size for a kind, or 0 for a reserved kind. */
static inline uint32_t comm_w_intake_size(uint32_t kind)
{
    const uint32_t bits = comm_w_body_bits(kind);

    return bits ? COMM_W_BYTES(bits) : 0U;
}

/* Wire E size for a kind, or 0 for a reserved kind. */
static inline uint32_t comm_w_e_size(uint32_t kind)
{
    const uint32_t bits = comm_w_body_bits(kind);

    return bits ? COMM_W_BYTES(COMM_W_HDR_BITS + bits) : 0U;
}

/* ------------------------------------------------------------------ *
 *  Producer pack helpers -- one per kind.  Each writes the body into `out`
 *  (COMM_W_INTAKE_SIZE_<KIND> bytes) and returns its length, ready for
 *  comms_tele_send().  Out-of-range values are clamped; signed values are
 *  offset-encoded.
 * ------------------------------------------------------------------ */

/* state 0-15; speeds in cm/s (PLACEHOLDER); dist_mm is the running distance,
   stored modulo 32768 (the laptop takes differences); turn rate in deg/s;
   fault: bit 0 stall, bit 1 encoder.  ovf (PLACEHOLDER) is sent as given,
   and is also set when this helper had to clamp a field. */
static inline uint32_t comm_w_pack_motion(uint8_t out[COMM_W_INTAKE_SIZE_MOTION],
                                          uint32_t state, int32_t speed_l_cms,
                                          int32_t speed_r_cms, uint32_t dist_mm,
                                          int32_t turn_rate_dps, uint32_t fault,
                                          uint32_t ovf)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;
    uint32_t st = comm_w_clamp(state, COMM_W_MOTION_STATE_BITS, &clamped);
    uint32_t sl = comm_w_offset(speed_l_cms, COMM_W_MOTION_SPEED_BITS, &clamped);
    uint32_t sr = comm_w_offset(speed_r_cms, COMM_W_MOTION_SPEED_BITS, &clamped);
    uint32_t tr = comm_w_offset(turn_rate_dps, COMM_W_MOTION_TURN_BITS, &clamped);

    comm_w_zero(out, COMM_W_INTAKE_SIZE_MOTION);
    comm_w_put_bits(out, &pos, COMM_W_KIND_MOTION, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, st, COMM_W_MOTION_STATE_BITS);
    comm_w_put_bits(out, &pos, sl, COMM_W_MOTION_SPEED_BITS);
    comm_w_put_bits(out, &pos, sr, COMM_W_MOTION_SPEED_BITS);
    comm_w_put_bits(out, &pos, dist_mm & 0x7FFFU, COMM_W_MOTION_DIST_BITS);
    comm_w_put_bits(out, &pos, tr, COMM_W_MOTION_TURN_BITS);
    comm_w_put_bits(out, &pos, fault & 0x3U, COMM_W_MOTION_FAULT_BITS);
    comm_w_put_bits(out, &pos, (ovf || clamped) ? 1U : 0U, COMM_W_MOTION_OVF_BITS);
    return COMM_W_INTAKE_SIZE_MOTION;
}

/* state 0 on-line, 1 lost, 2 crossing; deviation in tenths of a sensor unit. */
static inline uint32_t comm_w_pack_line(uint8_t out[COMM_W_INTAKE_SIZE_LINE],
                                        uint32_t state, int32_t deviation_tenths)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_INTAKE_SIZE_LINE);
    comm_w_put_bits(out, &pos, COMM_W_KIND_LINE, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(state, COMM_W_LINE_STATE_BITS, &clamped),
                    COMM_W_LINE_STATE_BITS);
    comm_w_put_bits(out, &pos,
                    comm_w_offset(deviation_tenths, COMM_W_LINE_DEV_BITS, &clamped),
                    COMM_W_LINE_DEV_BITS);
    return COMM_W_INTAKE_SIZE_LINE;
}

/* letter 0 = A .. 25 = Z; anything above 25 is sent as 31 (unreadable).
   action 0-3. */
static inline uint32_t comm_w_pack_barcode(uint8_t out[COMM_W_INTAKE_SIZE_BARCODE],
                                           uint32_t letter, uint32_t action)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_INTAKE_SIZE_BARCODE);
    comm_w_put_bits(out, &pos, COMM_W_KIND_BARCODE, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, (letter <= 25U) ? letter : COMM_W_BARCODE_UNREADABLE,
                    COMM_W_BARCODE_LETTER_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(action, COMM_W_BARCODE_ACTION_BITS, &clamped),
                    COMM_W_BARCODE_ACTION_BITS);
    return COMM_W_INTAKE_SIZE_BARCODE;
}

/* event_class 0-4 (FR-27); hump_count 0-15; hump height in mm. */
static inline uint32_t comm_w_pack_imu(uint8_t out[COMM_W_INTAKE_SIZE_IMU],
                                       uint32_t event_class, uint32_t hump_count,
                                       uint32_t hump_height_mm)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_INTAKE_SIZE_IMU);
    comm_w_put_bits(out, &pos, COMM_W_KIND_IMU, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(event_class, COMM_W_IMU_CLASS_BITS, &clamped),
                    COMM_W_IMU_CLASS_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(hump_count, COMM_W_IMU_COUNT_BITS, &clamped),
                    COMM_W_IMU_COUNT_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(hump_height_mm, COMM_W_IMU_HEIGHT_BITS, &clamped),
                    COMM_W_IMU_HEIGHT_BITS);
    return COMM_W_INTAKE_SIZE_IMU;
}

/* width in cm; bypass 0 none, 1 left, 2 right. */
static inline uint32_t comm_w_pack_obstacle(uint8_t out[COMM_W_INTAKE_SIZE_OBSTACLE],
                                            uint32_t width_cm, uint32_t bypass)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_INTAKE_SIZE_OBSTACLE);
    comm_w_put_bits(out, &pos, COMM_W_KIND_OBSTACLE, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(width_cm, COMM_W_OBSTACLE_WIDTH_BITS, &clamped),
                    COMM_W_OBSTACLE_WIDTH_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(bypass, COMM_W_OBSTACLE_BYPASS_BITS, &clamped),
                    COMM_W_OBSTACLE_BYPASS_BITS);
    return COMM_W_INTAKE_SIZE_OBSTACLE;
}

/* mission state 0-6. */
static inline uint32_t comm_w_pack_mission(uint8_t out[COMM_W_INTAKE_SIZE_MISSION],
                                           uint32_t state)
{
    uint8_t clamped = 0;
    uint32_t pos = 0;

    comm_w_zero(out, COMM_W_INTAKE_SIZE_MISSION);
    comm_w_put_bits(out, &pos, COMM_W_KIND_MISSION, COMM_W_KIND_BITS);
    comm_w_put_bits(out, &pos, comm_w_clamp(state, COMM_W_MISSION_STATE_BITS, &clamped),
                    COMM_W_MISSION_STATE_BITS);
    return COMM_W_INTAKE_SIZE_MISSION;
}

/* ------------------------------------------------------------------ *
 *  comms_task side
 * ------------------------------------------------------------------ */

/* Unpack an intake body.  The kind comes from the low 3 bits of byte 0, and
   the length must be exactly that kind's intake size.  Returns COMM_W_OK, or
   COMM_W_ERR_KIND / COMM_W_ERR_LENGTH with *b untouched apart from kind. */
static inline int32_t comm_w_unpack_body(const uint8_t *body, uint32_t length,
                                         T_COMM_W_BODY *b)
{
    uint32_t pos = 0;
    uint32_t kind;

    if(length == 0U) {
        return COMM_W_ERR_LENGTH;
    }
    kind = comm_w_get_bits(body, &pos, COMM_W_KIND_BITS);
    b->kind = (uint8_t)kind;
    if(kind >= COMM_W_KIND_COUNT) {
        return COMM_W_ERR_KIND;
    }
    if(length != comm_w_intake_size(kind)) {
        return COMM_W_ERR_LENGTH;
    }

    switch(kind) {
    case COMM_W_KIND_MOTION:
        b->motion_state = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_STATE_BITS);
        b->speed_l = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_SPEED_BITS);
        b->speed_r = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_SPEED_BITS);
        b->dist_mm = (uint16_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_DIST_BITS);
        b->turn_rate = (uint16_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_TURN_BITS);
        b->fault = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_FAULT_BITS);
        b->ovf = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MOTION_OVF_BITS);
        break;
    case COMM_W_KIND_LINE:
        b->line_state = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_LINE_STATE_BITS);
        b->deviation = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_LINE_DEV_BITS);
        break;
    case COMM_W_KIND_BARCODE:
        b->letter = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_BARCODE_LETTER_BITS);
        b->action = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_BARCODE_ACTION_BITS);
        break;
    case COMM_W_KIND_IMU:
        b->event_class = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_IMU_CLASS_BITS);
        b->hump_count = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_IMU_COUNT_BITS);
        b->hump_height_mm = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_IMU_HEIGHT_BITS);
        break;
    case COMM_W_KIND_OBSTACLE:
        b->width_cm = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_OBSTACLE_WIDTH_BITS);
        b->bypass = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_OBSTACLE_BYPASS_BITS);
        break;
    default:   /* COMM_W_KIND_MISSION */
        b->mission_state = (uint8_t)comm_w_get_bits(body, &pos, COMM_W_MISSION_STATE_BITS);
        break;
    }
    return COMM_W_OK;
}

/* Build a wire E from an intake body: header at bit 0, the body copied from
   bit 53 (in pieces of up to 32 bits, since a record can exceed 64 bits).
   No field is remapped.  Returns the E length, or 0 if the body is invalid
   or `size` is too small. */
static inline uint32_t comm_w_copy_body(uint8_t *out, uint32_t size, uint16_t s,
                                        uint32_t up, const uint8_t *body,
                                        uint32_t length)
{
    const T_COMM_W_HEADER h = { COMM_W_REC_E, COMM_W_SCHEMA_VERSION, s, up };
    uint32_t kind, bits, e_size, chunk;
    uint32_t pos = 0;
    uint32_t src = 0;

    if(length == 0U) {
        return 0U;
    }
    kind = body[0] & 0x7U;
    bits = comm_w_body_bits(kind);
    e_size = comm_w_e_size(kind);
    if(bits == 0U || length != COMM_W_BYTES(bits) || size < e_size) {
        return 0U;
    }

    comm_w_zero(out, e_size);
    comm_w_put_header(out, &pos, &h);
    while(src < bits) {
        chunk = (bits - src > 32U) ? 32U : bits - src;
        comm_w_put_bits(out, &pos, comm_w_get_bits(body, &src, chunk), chunk);
    }
    return e_size;
}

/* Distance travelled between two 15-bit wrapping dist_mm readings, in mm.
   Correct across one wrap (32767 -> 0); a gap of 32768 mm or more aliases. */
static inline uint32_t comm_w_dist_delta_mm(uint32_t now_mm, uint32_t then_mm)
{
    return (now_mm - then_mm) & 0x7FFFU;
}

#endif /* COMMON_MESSAGES_H */
