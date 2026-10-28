/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <stdint.h>
#include <tk/tkernel.h>

#include "cyw43_utk_mqtt_ids.h"

IMPORT void cyw43_utk_task(INT stacd, void *exinf);

#if TM_WIFI_MQTT
/* Outbound MQTT record queue: any task -> radio task.  The lib owns it and
 * creates it here, before the radio task starts, because the radio task runs
 * from device init -- before usermain() could create anything for it
 * (docs/comms/WIFI_SUBSYSTEM_DESIGN.md section 6.3).
 *
 * One message is one record: byte 0 holds the topic index in its low 2 bits
 * (the rest reserved, zero), followed by the packed payload.  The message
 * length carries the payload length, so no length field is stored. */
#define MQTT_MSG_MAX   (1 + CYW43_MQTT_RECORD_MAX)
/* The message buffer stores a 4-byte header plus the message rounded up to
   4 bytes (kernel/tkernel/messagebuf.h). */
#define MQTT_MBF_SLOT  (4 + ((MQTT_MSG_MAX + 3) & ~3))

LOCAL UW mqtt_mbf_buf[(CYW43_MQTT_QUEUE_DEPTH * MQTT_MBF_SLOT) / sizeof(UW)];
LOCAL ID mqtt_mbfid;
LOCAL volatile UW mqtt_dropped;

LOCAL ER create_mqtt_queue(void)
{
    T_CMBF cmbf = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = sizeof mqtt_mbf_buf;
    cmbf.maxmsz = MQTT_MSG_MAX;
    cmbf.bufptr = mqtt_mbf_buf;
#if USE_OBJECT_NAME
    cmbf.dsname = (UB*)"mqttq";
#endif
    mqtt_mbfid = tk_cre_mbf(&cmbf);
    return (mqtt_mbfid < E_OK) ? mqtt_mbfid : E_OK;
}

EXPORT int32_t cyw43_utk_mqtt_send(uint32_t topic, const uint8_t *payload,
                                   uint32_t length)
{
    UB msg[MQTT_MSG_MAX];
    ER er;

    if(topic > CYW43_MQTT_TOPIC_STATE || payload == NULL || length == 0
       || length > CYW43_MQTT_RECORD_MAX) {
        return E_PAR;
    }
    if(mqtt_mbfid <= 0) return E_NOEXS;

    msg[0] = (UB)topic;
    for(UW i = 0; i < length; i++) msg[1 + i] = payload[i];
    /* TMO_POL: a producer must never wait on the network.  A full queue is
       counted and the record is dropped. */
    er = tk_snd_mbf(mqtt_mbfid, msg, (INT)(1 + length), TMO_POL);
    if(er < E_OK) mqtt_dropped++;
    return er;
}

/* Radio task only: take the next record, or return a negative error
   (E_TMOUT when the queue is empty).  Returns the payload length. */
EXPORT int32_t cyw43_utk_mqtt_take(uint32_t *topic, uint8_t *payload)
{
    UB msg[MQTT_MSG_MAX];
    INT size;

    if(mqtt_mbfid <= 0) return E_NOEXS;
    size = tk_rcv_mbf(mqtt_mbfid, msg, TMO_POL);
    if(size < E_OK) return size;
    if(size < 2) return E_OBJ;   /* cannot happen: send checks the length */

    *topic = msg[0] & 0x03U;
    for(INT i = 1; i < size; i++) payload[i - 1] = msg[i];
    return size - 1;
}

EXPORT uint32_t cyw43_utk_mqtt_dropped(void)
{
    return mqtt_dropped;
}
#else
EXPORT int32_t cyw43_utk_mqtt_send(uint32_t topic, const uint8_t *payload,
                                   uint32_t length)
{
    (void)topic; (void)payload; (void)length;
    return E_NOEXS;
}
#endif

/* USB CDC drains at priority 3 on the same physical core.  Keep radio polling
 * below it so firmware download and the 2 ms poll cadence cannot overflow the
 * console ring, while remaining well above the qualification task. */
#define CYW43_TASK_PRIORITY  4
#define CYW43_TASK_STACK     (6 * 1024)

EXPORT ER cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

#if TK_SUPPORT_SMP
    /* Pin to processor 1 (RP2040 core 0): the radio owns PIO, DMA and the
       USBCTRL IRQ on that core, and must not migrate. */
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if TK_SUPPORT_SMP
    ctsk.assprc = TP_PRC1;
#endif
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

#if TM_WIFI_MQTT
    {
        ER er = create_mqtt_queue();
        if(er < E_OK) return er;
    }
#endif

    task = tk_cre_tsk(&ctsk);
    if(task < E_OK) return task;
    return tk_sta_tsk(task, 0);
}
