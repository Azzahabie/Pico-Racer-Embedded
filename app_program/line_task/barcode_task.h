/*
 * barcode_task.h - barcode_task (Buddy 3): decodes the bar/space widths
 * ir_sense_task captures, keeps the letter -> movement table, and holds the
 * one barcode command nav_ctrl_task takes at the next junction.
 * Priority 14, pinned to core 0 [PROPOSED].
 *
 *   ir_sense_task --bar_mbf (4-byte WIDTH/END/ABORT)--> barcode_task
 *   comms_task    --barcode_map_submit() (7-byte map)--> barcode_task
 *   barcode_task  --barcode_cmd_word() (one aligned load)--> nav_ctrl_task
 */

#ifndef BARCODE_TASK_H
#define BARCODE_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>

/* Create bar_mbf and map_mbf.  Call in usermain() before ir_sense_task or
 * barcode_task starts. */
EXPORT ER barcode_task_init(void);

/* bar_mbf, for ir_sense_task's sends (0 before barcode_task_init()). */
EXPORT ID barcode_bar_mbf(void);

/* A validated map update from comms_task (MQTT_SCHEMA.md 8.3): exactly
 * BARCODE_MAP_SIZE bytes, else E_PAR.  Never blocks (E_TMOUT if two are
 * already waiting).  barcode_task applies it between barcodes; a rejected
 * update leaves the working table as it was. */
EXPORT ER barcode_map_submit(const UB *map, INT length);

/* The pending barcode command (barcode_action.h layout), any task, any
 * core: one aligned load.  nav_ctrl_task reads it at a junction. */
EXPORT uint32_t barcode_cmd_word(void);

/* The map version running (S.mpv), and whether any table is loaded. */
EXPORT UB barcode_map_version(void);
EXPORT BOOL barcode_map_loaded(void);

EXPORT void barcode_task(INT stacd, void *exinf);

#endif /* BARCODE_TASK_H */
