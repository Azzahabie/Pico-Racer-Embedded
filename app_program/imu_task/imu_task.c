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
 *	Output, every READ_MS:
 *	    [imu] acc mg x=.. y=.. z=.. |a|=..   mag mG x=.. y=.. z=..
 *
 *	Lying flat and still, |a| should be close to 1000 mg and z should
 *	carry most of it.  Turning the board should swing the mag values.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../../device/include/dev_i2c.h"

#include "imu_task.h"
#include "../common/messages.h"
#include "../usb_console_compat.h"

#define I2C_DEV		"iica"		/* I2C0: GP16 SDA, GP17 SCL          */
#define READ_MS		250
#define AUTO_INC	0x80		/* ST sub-address MSB: auto-increment */

typedef enum {
	CHIP_NONE, CHIP_LSM303D, CHIP_DLHC, CHIP_AGR, CHIP_ADXL345, CHIP_MMA845X
} CHIP;

LOCAL const char * const chip_name[] = {
	"none", "LSM303D", "LSM303DLHC", "LSM303AGR", "ADXL345", "MMA845x"
};

LOCAL ID	dd;			/* I2C device descriptor            */
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

/* Integer square root, for |a| without pulling in libm. */
LOCAL UW isqrt(UW v)
{
	UW	r = 0, bit = 1UL << 30;

	while(bit > v) bit >>= 2;
	while(bit != 0) {
		if(v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
		else		 { r >>= 1; }
		bit >>= 2;
	}
	return r;
}

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
		write_reg(addr, 0x21, 0x00);	/* CTRL2: +/-2 g            */
		write_reg(addr, 0x24, 0x64);	/* CTRL5: mag hi-res 6.25 Hz*/
		write_reg(addr, 0x25, 0x20);	/* CTRL6: mag +/-4 gauss    */
		write_reg(addr, 0x26, 0x00);	/* CTRL7: mag continuous    */
		break;
	case CHIP_DLHC:
	case CHIP_AGR:
		write_reg(0x19, 0x20, 0x57);	/* 100 Hz, XYZ              */
		write_reg(0x19, 0x23, 0x88);	/* BDU, hi-res, +/-2 g      */
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
		write_reg(addr, 0x31, 0x08);	/* full res, +/-2 g         */
		write_reg(addr, 0x2D, 0x08);	/* measure                  */
		break;
	case CHIP_MMA845X:
		write_reg(addr, 0x2A, 0x00);	/* standby to configure     */
		write_reg(addr, 0x0E, 0x00);	/* +/-2 g                   */
		write_reg(addr, 0x2A, 0x19);	/* 100 Hz, active           */
		break;
	default:
		break;
	}
}

/* Fill acceleration in mg and, when there is a magnetometer, field in mG.
   Returns a negative error if the accel read failed. */
LOCAL ER read_sample(W a[3], W m[3], BOOL *has_mag)
{
	UB	r[6];
	ER	er;
	INT	i;

	*has_mag = FALSE;

	switch(chip) {
	case CHIP_LSM303D:
		er = read_regs(addr, 0x28 | AUTO_INC, r, 6);
		if(er < E_OK) return er;
		for(i = 0; i < 3; i++) a[i] = (W)le16(&r[2 * i]) * 61 / 1000;
		if(read_regs(addr, 0x08 | AUTO_INC, r, 6) >= E_OK) {
			/* 0.160 mG/LSB at +/-4 gauss */
			for(i = 0; i < 3; i++) m[i] = (W)le16(&r[2 * i]) * 4 / 25;
			*has_mag = TRUE;
		}
		return E_OK;

	case CHIP_DLHC:
	case CHIP_AGR:
		er = read_regs(0x19, 0x28 | AUTO_INC, r, 6);
		if(er < E_OK) return er;
		/* 12-bit left-justified, ~1 mg/LSB */
		for(i = 0; i < 3; i++) a[i] = le16(&r[2 * i]) >> 4;
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
		/* 3.9 mg/LSB in full resolution */
		for(i = 0; i < 3; i++) a[i] = (W)le16(&r[2 * i]) * 39 / 10;
		return E_OK;

	case CHIP_MMA845X:
		er = read_regs(addr, 0x01, r, 6);
		if(er < E_OK) return er;
		/* left-justified, so 1 g = 16384 whatever the resolution */
		for(i = 0; i < 3; i++) a[i] = (W)be16(&r[2 * i]) * 1000 / 16384;
		return E_OK;

	default:
		return E_NOEXS;
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	W	a[3], m[3];
	BOOL	has_mag;
	UW	amag;
	ER	er;
	INT	i;

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

	while(1) {
		er = read_sample(a, m, &has_mag);
		if(er < E_OK) {
			tm_printf((UB *)"[imu] read failed, er=%d\n", er);
			tk_dly_tsk(1000);
			continue;
		}
		amag = isqrt((UW)(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));

		if(has_mag) {
			tm_printf((UB *)"[imu] acc mg x=%6d y=%6d z=%6d |a|=%5u"
				       "   mag mG x=%6d y=%6d z=%6d\n",
				  a[0], a[1], a[2], amag, m[0], m[1], m[2]);
		} else {
			tm_printf((UB *)"[imu] acc mg x=%6d y=%6d z=%6d |a|=%5u\n",
				  a[0], a[1], a[2], amag);
		}

		tk_dly_tsk(READ_MS);
	}
}
