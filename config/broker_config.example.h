/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.07.B0
 *
 *    Copyright (C) 2006-2023 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2023/11.
 *
 *----------------------------------------------------------------------
 */

/*
 * Local endpoint for the opt-in Phase-9 MQTT profile.
 *
 * Copy this file to config/broker_config.h, point it at the broker the car
 * must publish to, and flip the guard.  The local copy is ignored by Git
 * because the broker differs between development and demo day -- on demo day
 * it is supplied by the instructor, so the target must stay configuration
 * rather than a compiled-in constant.
 */
#ifndef UTK3_BROKER_CONFIG_H
#define UTK3_BROKER_CONFIG_H

/*
 * ---------------------------------------------------------------------------
 * Copy this file to config/broker_config.h, fill in the values below, then
 * change the 0 on the next line to 1.
 *
 * It is a value to flip rather than a line to delete, so that an unedited
 * copy fails loudly at compile time instead of building fine and failing to
 * connect at runtime -- and so a careless edit cannot leave the guard
 * half-removed.
 * ---------------------------------------------------------------------------
 */
#define UTK_MQTT_CONFIG_SET   0

#if !UTK_MQTT_CONFIG_SET
#error "config/broker_config.h: placeholder values. Set the MQTT broker address and port, then set UTK_MQTT_CONFIG_SET to 1."
#endif


#define UTK_MQTT_BROKER_ADDRESS   "192.168.1.100"
#define UTK_MQTT_BROKER_PORT      1883U
#define UTK_MQTT_CLIENT_ID        "pico-racer-01"
#define UTK_MQTT_KEEPALIVE_S      60U

#endif