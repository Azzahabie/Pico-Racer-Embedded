/*
 *	imu_task.c
 *	Bring-up test for the "GY-511" accelerometer/magnetometer module.
 *
 *	Boards sold under that name carry different chips, so the chip is
 *	identified by its ID register rather than assumed:
 *
 *	    chip         addr       ID register      sensors
 *	    LSM303D      0x1D/0x1E  0x0F = 0x49      accel + mag, one address
 *	    LSM303DLHC   0x19+0x1E  mag 0x0A = "H43" accel + mag
 *	    LSM303AGR    0x19+0x1E  mag 0x4F = 0x40  accel + mag
 *	    ADXL345      0x1D/0x53  0x00 = 0xE5      accel only
 *	    MMA845x      0x1D/0x1C  0x0D = 0x1A/2A/3A accel only
 *
 *	Wiring, I2C0:
 *
 *	    VIN -> 3V3     SDA -> GP16
 *	    GND -> GND     SCL -> GP17
 *
 *	The accel is read every SAMPLE_MS from a cyclic handler, the mag every
 *	MAG_EVERY samples.  At boot the accel is zeroed against 1 s of samples
 *	taken while the car is still (calibrate()).  Every REPORT_SAMPLES:
 *	    [imu] acc mg x=.. y=.. z=.. (mean)  z ..-.. |a|max ..  car .. mm/s
 *	          <state> turn .. dps  ds ..  lost ..  rderr ..
 *	    [imu] mag mG x=.. y=.. z=..
 *
 *	Lying flat and still, the mean should be close to 0, 0, 1000 mg.
 *	Turning the board should swing the mag values.
 *
 *	Accel range is +/-8 g on every chip, so bumps and crashes on the car
 *	do not clip (at +/-2 g they pinned at 1998 mg).
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../../device/include/dev_i2c.h"

#include "imu_task.h"
#include "../common/messages.h"
#include "../usb_console_compat.h"
#include "../motor_task/motion_ds.h"
#include "hump.h"

#define I2C_DEV		"iica"		/* I2C0: GP16 SDA, GP17 SCL          */
#define AUTO_INC	0x80		/* ST sub-address MSB: auto-increment */

#define SAMPLE_MS	10		/* accel period; every chip runs 100 Hz */
#define MAG_EVERY	25		/* mag every 250 ms (LSM303D: 6.25 Hz) */
#define REPORT_SAMPLES	100		/* one status line every 1 s (FR-29)  */
#define IDLE_REPORTS	5		/* still, no hump, no errors: only
					   every 5th line (5 s)              */
#ifndef IMU_VERBOSE
#define IMU_VERBOSE	0		/* 1: also the raw axes, z range,
					   distance_state count and mag      */
#endif

#define CAL_SAMPLES	100		/* 1 s of samples per attempt         */
#define CAL_TRIES	5
#define CAL_SPREAD_MG	80		/* max - min on any axis while still  */
#define CAL_TILT_MG	150		/* x, y within 0 +/- this, z 1000 +/- */

/* Accel zero-g offsets in mg, subtracted from every sample.  calibrate()
   replaces them at boot; these are the fallback if it never sees the car
   still.  Measured on our LSM303D: reading flat and upside down, the
   offset is the midpoint, (up + down) / 2.  z: (1070 + -868) / 2 = +101.
   x flipped sign in the same pair, giving +30.  y is unmeasured. */
LOCAL W acc_ofs[3] = { 30, 0, 101 };

#if IMU_VERBOSE
LOCAL const char * const ds_state_name[] = { "steady", "accel", "decel", "?" };
#endif

/* Tilt in 0.1 deg from its sine in mg, without libm: asin(s) is about
   s + s^3 / 6, within 0.3 deg up to 30 deg (well past any hump). */
LOCAL UW tilt_deg10(UW mg)
{
	UW	s3;

	if(mg > 1000) mg = 1000;
	s3 = mg * mg / 1000 * mg / 1000;
	return (mg + s3 / 6) * 573 / 1000;
}

typedef enum {
	CHIP_NONE, CHIP_LSM303D, CHIP_DLHC, CHIP_AGR, CHIP_ADXL345, CHIP_MMA845X
} CHIP;

LOCAL const char * const chip_name[] = {
	"none", "LSM303D", "LSM303DLHC", "LSM303AGR", "ADXL345", "MMA845x"
};

LOCAL ID	dd;			/* I2C device descriptor            */
LOCAL ID	imu_tid;		/* woken by imu_tick()              */
LOCAL CHIP	chip;
LOCAL UW	addr;			/* LSM303D / ADXL345 / MMA845x      */
LOCAL BOOL	acc_only;		/* DLHC/AGR: mag not identified     */

/* Write sub-address, repeated start, read n bytes. */
LOCAL ER read_regs(UW sadr, UB reg, UB *buf, SZ n)
{
	T_I2C_EXEC	ex;
	SZ		asz;

	ex.sadr		= sadr;
	ex.snd_size	= 1;
	ex.snd_data	= &reg;
	ex.rcv_size	= n;
	ex.rcv_data	= buf;

	return tk_swri_dev(dd, TDN_I2C_EXEC, &ex, sizeof ex, &asz);
}

LOCAL ER write_reg(UW sadr, UB reg, UB val)
{
	UB	b[2];
	SZ	asz;

	b[0] = reg;
	b[1] = val;
	return tk_swri_dev(dd, sadr, b, sizeof b, &asz);
}

/* Read one register; 0xFF ... is not a valid ID, so it doubles as "no". */
LOCAL UB reg8(UW sadr, UB reg)
{
	UB	v;

	return (read_regs(sadr, reg, &v, 1) >= E_OK) ? v : 0xFF;
}

LOCAL H le16(const UB *p) { return (H)(p[0] | (p[1] << 8)); }
LOCAL H be16(const UB *p) { return (H)((p[0] << 8) | p[1]); }

/* Which addresses answer, and what their usual ID registers hold, so an
   unsupported chip can still be identified from the log. */
LOCAL void bus_scan(void)
{
	UW	a;
	UB	b;
	INT	found = 0;

	tm_printf((UB *)"[imu] scanning I2C0 ...\n");
	for(a = 0x08; a < 0x78; a++) {
		if(read_regs(a, 0x00, &b, 1) >= E_OK) {
			tm_printf((UB *)"[imu]   device at 0x%02x: reg00=%02x"
				       " reg0D=%02x reg0F=%02x\n",
				  a, b, reg8(a, 0x0D), reg8(a, 0x0F));
			found++;
		}
	}
	if(found == 0) {
		tm_printf((UB *)"[imu]   nothing answered: check VIN/GND,"
			       " SDA->GP16, SCL->GP17\n");
	}
}

LOCAL CHIP detect(void)
{
	UB	id[3];
	UB	v;

	/* LSM303D: SA0 high -> 0x1D, low -> 0x1E. */
	for(addr = 0x1D; addr <= 0x1E; addr++) {
		if(reg8(addr, 0x0F) == 0x49) return CHIP_LSM303D;
	}

	addr = 0x1D;
	if(reg8(0x1D, 0x00) == 0xE5) return CHIP_ADXL345;
	addr = 0x53;
	if(reg8(0x53, 0x00) == 0xE5) return CHIP_ADXL345;

	for(addr = 0x1C; addr <= 0x1D; addr++) {
		v = reg8(addr, 0x0D);
		if(v == 0x1A || v == 0x2A || v == 0x3A) return CHIP_MMA845X;
	}

	/* LSM303DLHC / AGR: accel 0x19, mag 0x1E. */
	if(read_regs(0x19, 0x20, &v, 1) >= E_OK) {
		acc_only = FALSE;
		if(reg8(0x1E, 0x4F) == 0x40) return CHIP_AGR;
		if(read_regs(0x1E, 0x0A, id, 3) >= E_OK
		   && id[0] == 'H' && id[1] == '4' && id[2] == '3') {
			return CHIP_DLHC;
		}
		acc_only = TRUE;
		return CHIP_DLHC;	/* same accel registers on both */
	}
	return CHIP_NONE;
}

LOCAL void setup(void)
{
	switch(chip) {
	case CHIP_LSM303D:
		write_reg(addr, 0x20, 0x6F);	/* CTRL1: 100 Hz, BDU, XYZ  */
		write_reg(addr, 0x21, 0x18);	/* CTRL2: +/-8 g            */
		write_reg(addr, 0x24, 0x64);	/* CTRL5: mag hi-res 6.25 Hz*/
		write_reg(addr, 0x25, 0x20);	/* CTRL6: mag +/-4 gauss    */
		write_reg(addr, 0x26, 0x00);	/* CTRL7: mag continuous    */
		break;
	case CHIP_DLHC:
	case CHIP_AGR:
		write_reg(0x19, 0x20, 0x57);	/* 100 Hz, XYZ              */
		write_reg(0x19, 0x23, 0xA8);	/* BDU, hi-res, +/-8 g      */
		if(chip == CHIP_DLHC && !acc_only) {
			write_reg(0x1E, 0x00, 0x10);	/* 15 Hz            */
			write_reg(0x1E, 0x01, 0x20);	/* +/-1.3 gauss     */
			write_reg(0x1E, 0x02, 0x00);	/* continuous       */
		} else if(chip == CHIP_AGR) {
			write_reg(0x1E, 0x60, 0x80);	/* temp comp, 10 Hz */
			write_reg(0x1E, 0x62, 0x10);	/* BDU              */
		}
		break;
	case CHIP_ADXL345:
		write_reg(addr, 0x31, 0x0A);	/* full res, +/-8 g         */
		write_reg(addr, 0x2D, 0x08);	/* measure                  */
		break;
	case CHIP_MMA845X:
		write_reg(addr, 0x2A, 0x00);	/* standby to configure     */
		write_reg(addr, 0x0E, 0x02);	/* +/-8 g                   */
		write_reg(addr, 0x2A, 0x19);	/* 100 Hz, active           */
		break;
	default:
		break;
	}
}

/* Fill acceleration in mg and, when want_mag and there is a magnetometer,
   field in mG.  Returns a negative error if the accel read failed. */
LOCAL ER read_sample(W a[3], W m[3], BOOL *has_mag, BOOL want_mag)
{
	UB	r[6];
	ER	er;
	INT	i;

	*has_mag = FALSE;

	switch(chip) {
	case CHIP_LSM303D:
		er = read_regs(addr, 0x28 | AUTO_INC, r, 6);
		if(er < E_OK) return er;
		/* 0.244 mg/LSB at +/-8 g */
		for(i = 0; i < 3; i++) a[i] = (W)le16(&r[2 * i]) * 244 / 1000;
		if(want_mag && read_regs(addr, 0x08 | AUTO_INC, r, 6) >= E_OK) {
			/* 0.160 mG/LSB at +/-4 gauss */
			for(i = 0; i < 3; i++) m[i] = (W)le16(&r[2 * i]) * 4 / 25;
			*has_mag = TRUE;
		}
		return E_OK;

	case CHIP_DLHC:
	case CHIP_AGR:
		er = read_regs(0x19, 0x28 | AUTO_INC, r, 6);
		if(er < E_OK) return er;
		/* 12-bit left-justified, ~4 mg/LSB at +/-8 g */
		for(i = 0; i < 3; i++) a[i] = (W)(le16(&r[2 * i]) >> 4) * 4;
		if(!want_mag) return E_OK;
		if(chip == CHIP_DLHC && !acc_only
		   && read_regs(0x1E, 0x03, r, 6) >= E_OK) {
			/* X Z Y, big-endian; 1100 LSB/G X/Y, 980 Z */
			m[0] = (W)be16(&r[0]) * 1000 / 1100;
			m[2] = (W)be16(&r[2]) * 1000 / 980;
			m[1] = (W)be16(&r[4]) * 1000 / 1100;
			*has_mag = TRUE;
		} else if(chip == CHIP_AGR
			  && read_regs(0x1E, 0x68, r, 6) >= E_OK) {
			for(i = 0; i < 3; i++) m[i] = (W)le16(&r[2 * i]) * 3 / 2;
			*has_mag = TRUE;
		}
		return E_OK;

	case CHIP_ADXL345:
		er = read_regs(addr, 0x32, r, 6);
		if(er < E_OK) return er;
		/* 3.9 mg/LSB in full resolution, at any range */
		for(i = 0; i < 3; i++) a[i] = (W)le16(&r[2 * i]) * 39 / 10;
		return E_OK;

	case CHIP_MMA845X:
		er = read_regs(addr, 0x01, r, 6);
		if(er < E_OK) return er;
		/* left-justified, so 1 g = 4096 at +/-8 g whatever the resolution */
		for(i = 0; i < 3; i++) a[i] = (W)be16(&r[2 * i]) * 1000 / 4096;
		return E_OK;

	default:
		return E_NOEXS;
	}
}

/* Cyclic handler: one wakeup per SAMPLE_MS.  Wakeups queue, so a late
   task catches up and wait_tick() can count what it missed. */
LOCAL void imu_tick(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(imu_tid);
}

LOCAL ER start_ticks(void)
{
	T_CCYC	ccyc;
	ID	cycid;

	imu_tid = tk_get_tid();
	ccyc.exinf  = NULL;
	ccyc.cycatr = TA_HLNG | TA_STA;
	ccyc.cychdr = (FP)imu_tick;
	ccyc.cyctim = SAMPLE_MS;
	ccyc.cycphs = SAMPLE_MS;

	cycid = tk_cre_cyc(&ccyc);
	return (cycid < E_OK) ? cycid : E_OK;
}

/* Sleep until the next tick.  Returns how many ticks went by unserved
   since the last call (0 when the task keeps up). */
LOCAL UW wait_tick(void)
{
	INT	missed;

	tk_slp_tsk(TMO_FEVR);
	missed = tk_can_wup(TSK_SELF);
	return (missed > 0) ? (UW)missed : 0;
}

/* Zero the accel at boot.  Assumes the car sits still and level with the
   board's z axis up, so the true reading is 0, 0, +1000 mg; any tilt at
   boot is absorbed into the offsets.  An attempt is rejected if someone
   bumps the car (axis spread over CAL_SPREAD_MG), the wheels turn
   (distance_state), or the mean is too far from flat (wrong mounting).
   After CAL_TRIES rejects the constants in acc_ofs stay. */
LOCAL void calibrate(void)
{
	W	a[3], m[3], sum[3], lo[3], hi[3], mean[3], spread;
	BOOL	has_mag, moved;
	UW	ds0, ds;
	INT	t, k, n, i;

	for(t = 1; t <= CAL_TRIES; t++) {
		for(i = 0; i < 3; i++) {
			sum[i] = 0; lo[i] = 32767; hi[i] = -32768;
		}
		moved = FALSE;
		n = 0;
		ds0 = motion_ds_read();

		for(k = 0; k < CAL_SAMPLES; k++) {
			(void)wait_tick();
			if(read_sample(a, m, &has_mag, FALSE) < E_OK) continue;

			ds = motion_ds_read();
			if(motion_ds_delta_mm(ds, ds0) != 0
			   || motion_ds_turn_dps(ds) != 0
			   || motion_ds_state(ds) != MOTION_DS_STEADY) {
				moved = TRUE;
			}
			for(i = 0; i < 3; i++) {
				sum[i] += a[i];
				if(a[i] < lo[i]) lo[i] = a[i];
				if(a[i] > hi[i]) hi[i] = a[i];
			}
			n++;
		}

		if(n < CAL_SAMPLES * 9 / 10) {
			tm_printf((UB *)"[imu] cal %d/%d: only %d of %d reads"
				       " succeeded\n", t, CAL_TRIES, n, CAL_SAMPLES);
			continue;
		}
		spread = 0;
		for(i = 0; i < 3; i++) {
			mean[i] = sum[i] / n;
			if(hi[i] - lo[i] > spread) spread = hi[i] - lo[i];
		}
		if(moved) {
			tm_printf((UB *)"[imu] cal %d/%d: car moving\n",
				  t, CAL_TRIES);
		} else if(spread > CAL_SPREAD_MG) {
			tm_printf((UB *)"[imu] cal %d/%d: not still, spread %d mg"
				       " (max %d)\n", t, CAL_TRIES, spread,
				  CAL_SPREAD_MG);
		} else if(mean[0] < -CAL_TILT_MG || mean[0] > CAL_TILT_MG
			  || mean[1] < -CAL_TILT_MG || mean[1] > CAL_TILT_MG
			  || mean[2] < 1000 - CAL_TILT_MG
			  || mean[2] > 1000 + CAL_TILT_MG) {
			tm_printf((UB *)"[imu] cal %d/%d: not level, raw mean"
				       " x=%d y=%d z=%d mg\n", t, CAL_TRIES,
				  mean[0], mean[1], mean[2]);
		} else {
			acc_ofs[0] = mean[0];
			acc_ofs[1] = mean[1];
			acc_ofs[2] = mean[2] - 1000;
			tm_printf((UB *)"[imu] cal ok: offsets x=%d y=%d z=%d mg,"
				       " spread %d mg\n", acc_ofs[0], acc_ofs[1],
				  acc_ofs[2], spread);
			return;
		}
		tk_dly_tsk(500);
	}
	tm_printf((UB *)"[imu] cal failed, using fixed offsets x=%d y=%d"
		       " z=%d mg\n", acc_ofs[0], acc_ofs[1], acc_ofs[2]);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	W	a[3], m[3];
	W	sum[3], zmin, zmax;
	W	hsum[2];
	UW	ds_hump;
	INT	hn;
	BOOL	has_mag = FALSE, got_mag;
	UW	amag, amax;
	UW	ds, ds_prev, ds_win, ds_pub;
	UW	lost, rderr, k;
	INT	n, i, idle_n = 0;
	ER	er;
	UB	line[200], *p;

	(void)stacd; (void)exinf;

	/* Same enumeration wait as the monitor, so nothing printed before the
	   host opens the port is lost. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(1000);		/* after the demo banner */

	dd = tk_opn_dev((UB *)I2C_DEV, TD_UPDATE);
	if(dd < E_OK) {
		tm_printf((UB *)"[imu] tk_opn_dev(%s) failed, er=%d\n",
			  I2C_DEV, dd);
		tk_ext_tsk();
	}

	/* Retry until a known chip shows up, so it can be plugged in late. */
	while((chip = detect()) == CHIP_NONE) {
		tm_printf((UB *)"[imu] no supported IMU found\n");
		bus_scan();
		tk_dly_tsk(3000);
	}
	setup();
	tm_printf((UB *)"[imu] found %s at 0x%02x%s\n", chip_name[chip],
		  (chip == CHIP_DLHC || chip == CHIP_AGR) ? 0x19 : addr,
		  (chip == CHIP_ADXL345 || chip == CHIP_MMA845X || acc_only)
		  ? " (accel only, no magnetometer)" : "");

	er = start_ticks();
	if(er < E_OK) {
		tm_printf((UB *)"[imu] tk_cre_cyc failed, er=%d\n", er);
		tk_ext_tsk();
	}
	calibrate();

	ds_prev = ds_win = ds_hump = motion_ds_read();
	hsum[0] = hsum[1] = 0;
	hn = 0;
	ds_pub = 0;
	lost = 0;
	rderr = 0;
	n = 0;
	sum[0] = sum[1] = sum[2] = 0;
	zmin = 32767; zmax = -32768; amax = 0;

	for(k = 0; ; k++) {
		lost += wait_tick();

		/* distance_state: count publishes so a stalled motion_task
		   shows up as ds well under REPORT_SAMPLES. */
		ds = motion_ds_read();
		ds_pub += motion_ds_updates(ds, ds_prev);
		ds_prev = ds;

		er = read_sample(a, m, &got_mag, (k % MAG_EVERY) == 0);
		if(er < E_OK) {
			rderr++;
		} else {
			if(got_mag) has_mag = TRUE;
			for(i = 0; i < 3; i++) {
				a[i] -= acc_ofs[i];
				sum[i] += a[i];
			}
			amag = imu_isqrt((UW)(a[0] * a[0] + a[1] * a[1]
					  + a[2] * a[2]));
			if(amag > amax) amax = amag;
			if(a[2] < zmin) zmin = a[2];
			if(a[2] > zmax) zmax = a[2];
			n++;
			hsum[0] += a[0];
			hsum[1] += a[1];
			hn++;
		}

		if((k + 1) % HUMP_SAMPLES == 0) {
			if(hn > 0) {
				hump_step(hsum[0] / hn, hsum[1] / hn,
					  motion_ds_delta_mm(ds, ds_hump),
					  motion_ds_state(ds) == MOTION_DS_STEADY);
			}
			ds_hump = ds;
			hsum[0] = hsum[1] = 0;
			hn = 0;
		}

		if((k + 1) % REPORT_SAMPLES != 0) continue;

		if(n > 0) {
			W	mx = sum[0] / n, my = sum[1] / n;
			/* mm per REPORT_SAMPLES window, as mm/s. */
			W	v = motion_ds_delta_mm(ds, ds_win)
				    * (1000 / (REPORT_SAMPLES * SAMPLE_MS));
			UW	t10 = tilt_deg10(imu_isqrt((UW)(mx * mx
							      + my * my)));
			HUMP_STATE hs = hump_get_state();

			/* Still and level, nothing wrong: one line in
			   IDLE_REPORTS, so moving lines stand out. */
			if(v != 0 || hs != HUMP_NONE || rderr != 0
			   || lost != 0 || ++idle_n >= IDLE_REPORTS) {
				idle_n = 0;
				/* One tm_printf per line: built up first. */
				p = line;
				p += tm_sprintf(p, (UB *)"[imu] %-6s tilt %2u.%u"
						" deg  %4d mm/s  turn %+4d dps",
						(v > 0) ? "moving"
						: (v < 0) ? "back" : "still",
						t10 / 10, t10 % 10, v,
						motion_ds_turn_dps(ds));
				if(hs != HUMP_NONE) {
					p += tm_sprintf(p, (UB *)"  | %s %d mm",
							hump_state_name(hs),
							hump_height_mm());
				}
				p += tm_sprintf(p, (UB *)"  | humps %u  last %u mm"
						"  max %u mm",
						hump_get_count(),
						hump_get_last_mm(),
						hump_get_max_mm());
				if(rderr != 0 || lost != 0) {
					p += tm_sprintf(p, (UB *)"  | rderr %u"
							"  lost %u", rderr, lost);
				}
				tm_printf((UB *)"%s\n", line);
			}
#if IMU_VERBOSE
			tm_printf((UB *)"[imu]   acc mg x=%5d y=%5d z=%5d"
				       "  z %5d..%5d |a|max %5u  %s  ds %2u\n",
				  mx, my, sum[2] / n, zmin, zmax, amax,
				  ds_state_name[motion_ds_state(ds)], ds_pub);
#endif
		} else {
			tm_printf((UB *)"[imu] NO GOOD READS in %d ms,"
				       " rderr %u\n",
				  REPORT_SAMPLES * SAMPLE_MS, rderr);
		}
#if IMU_VERBOSE
		if(has_mag) {
			tm_printf((UB *)"[imu]   mag mG x=%6d y=%6d z=%6d\n",
				  m[0], m[1], m[2]);
		}
#else
		(void)has_mag; (void)zmin; (void)zmax; (void)amax;
		(void)ds_pub;
#endif

		ds_win = ds;
		ds_pub = 0;
		lost = 0;
		rderr = 0;
		n = 0;
		sum[0] = sum[1] = sum[2] = 0;
		zmin = 32767; zmax = -32768; amax = 0;
	}
}
