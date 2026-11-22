/*
 *  Adapted from line_cfg.h and line_types.h by Boon (Buddy 3, another
 *  team in the same module), used with the author's permission
 *  (confirmed by Hanis, 20 Nov 2026).  Changes: file renamed
 *  bc_analog_cfg.h; only the barcode settings are kept (steering, slip,
 *  lift and sensor-channel settings for Boon's car are left out); most
 *  values can be set with -D (#ifndef); the capture states
 *  (line_bc_state_t in line_types.h) moved here as bc_state_t, with
 *  BC_ST_DECODING added for the decode on core 0; BC_QUIET_WINDOWS added
 *  (ours); the fixed white/black
 *  levels (LINE_BC_WHITE_RAW / BLACK_RAW) are replaced by our front end
 *  (bc_analog_front.h, line_config.h).
 *
 *  Pure C header.  Widths are micrometres of travel.
 */

#ifndef BC_ANALOG_CFG_H
#define BC_ANALOG_CFG_H

/* Capture states (Boon's line_bc_state_t).  BC_ST_DECODING (ours): the
   trailing white ended a code; barcode_task on core 0 decodes it while the
   capture waits (bc_analog_capture.h). */
typedef enum
{
    BC_ST_WAIT_QUIET = 0,
    BC_ST_ARMED      = 1,               /* quiet zone seen, waiting for a bar */
    BC_ST_CAPTURING  = 2,
    BC_ST_COOLDOWN   = 3,
    BC_ST_DECODING   = 4                /* ours: core 0 holds the capture     */
} bc_state_t;

/* Bars and gaps by swing (2 Nov; replaces the fixed 350 / 220 levels,
   which missed every 3 mm element on the robot: darkest ~900 on wide
   bars, 3 elements instead of 29).  A bar starts when the reading rises
   BC_DELTA above the lightest point of the gap before it, and ends when
   it falls BC_DELTA below the darkest point of the bar.  Must stay above
   the peak-to-peak sample noise (host model +-120).                    */
#ifndef BC_DELTA
#define BC_DELTA                    250
#endif
#ifndef BC_DELTA_MIN
#define BC_DELTA_MIN                200         /* a bar is at least this dark */
#endif

/* ---- Barcode (FR3, A6) -----------------------------------------------
   Element widths are measured in encoder distance, so they do not depend
   on speed.  Nothing assumes a fixed print size: the instructor's sheets
   come in three scales (narrow bar ~3.4 / 5.1 / 6.9 mm on screen, ~144 /
   213 / 288 mm per code when printed on A4), so the end of a code is a
   white run longer than both the quiet zone and 2.5 x its widest element. */
#ifndef BC_QUIET_UM
#define BC_QUIET_UM                 25000       /* 25 mm white before a code */
#endif
#ifndef BC_END_FACTOR_X10
#define BC_END_FACTOR_X10           25          /* end: white > 2.5 x widest */
#endif
/* 7 Nov: and at least this.  On the robot only the 9 mm bars of "*Z*" were
   seen (3 mm bars too faint at speed): the white between them inside the
   code reached 33 mm and ended the capture after 51 mm, before the
   profile pass could look for the faint bars.  After a real code the mat
   is white, so a longer end costs nothing.                            */
#ifndef BC_END_MIN_UM
#define BC_END_MIN_UM               40000
#endif
#ifndef BC_MAX_LEN_UM
#define BC_MAX_LEN_UM               420000      /* 420 mm: longer = REJECT   */
#endif
#define BC_MAX_ELEMENTS             59          /* 6 characters              */
#ifndef BC_COOLDOWN_UM
#define BC_COOLDOWN_UM              100000      /* 100 mm after a read       */
#endif
#ifndef BC_RATIO_MIN_X10
#define BC_RATIO_MIN_X10            15          /* wide >= 1.5 x narrow      */
#endif
/* 14 Nov: an end bar thinner than this % of the read's median bar is a
   speck before the start or after the stop (a crease, a hair, a tape
   edge): 1.4 mm on the car beside 2.0-3.6 mm narrow bars.  It is dropped
   with the space beside it if the read does not decode as it is.      */
#ifndef BC_SPECK_PCT
#define BC_SPECK_PCT                67
#endif
/* 7 Nov second try: the capture keeps the darkest reading of every
   BC_PROF_BIN_UM of travel; if the swing reading does not decode, bars
   are found again from that profile against this code's own levels
   (light = BC_PROF_WHITE_PCT percentile, dark = BC_PROF_DARK_PCT):
   first at mid-way, then at BC_PROF_LOW_DIV of the way up for faint thin
   bars.  Bars or spaces under BC_PROF_MIN_BINS bins are noise.          */
#define BC_PROF_BIN_UM              330
/* 7 Nov: the darkness record also keeps this much from before the first
   bar the swing reading saw ("*" starts with a narrow bar, and faint
   narrow bars do not start a capture): 25 mm = the quiet zone.       */
#define BC_PRE_BINS                 (BC_QUIET_UM / BC_PROF_BIN_UM)
#define BC_PROF_BINS                ((BC_MAX_LEN_UM / BC_PROF_BIN_UM) + BC_PRE_BINS + 64u)
#define BC_PROF_WHITE_PCT           10u
#define BC_PROF_DARK_PCT            95u
#define BC_PROF_LOW_DIV             4u
#define BC_PROF_MIN_BINS            3u
/* 9 Nov: a light print (bars only ~700 / 1000 on the robot, 3 mm bars
   rising less than BC_DELTA) is read once more by swing over the
   darkness record, with the swing set by THIS code: BC_ADAPT_PCT % of
   its dark - white span (9 Nov: then BC_ADAPT_PCT2 and BC_ADAPT_PCT3 %,
   01:54 run: 3 of 9 thin bars just reached the 20 % swing), but never under BC_ADAPT_NOISE_X x the white
   noise measured before the code, nor under BC_ADAPT_MIN.  Only used
   when that is below BC_DELTA (otherwise it is the first reading).    */
#define BC_ADAPT_PCT                20u
#define BC_ADAPT_PCT2               15u
#define BC_ADAPT_PCT3               10u
#define BC_ADAPT_NOISE_X            4u
#define BC_ADAPT_MIN                60u
/* Ours (20 Nov): the white noise is the quietest of this many of the last
   BC_QUIET_UM windows (bc_analog_capture.c, quiet_commit()). */
#define BC_QUIET_WINDOWS            3u

#endif /* BC_ANALOG_CFG_H */
