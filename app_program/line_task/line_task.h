#ifndef LINE_TASK_H
#define LINE_TASK_H

#include <tk/tkernel.h>

#define LINE_STATE_LEFT_ON_DARK       (1u << 0)
#define LINE_STATE_CENTRE_ON_DARK     (1u << 1)
#define LINE_STATE_BARCODE_ON_DARK    (1u << 2)

EXPORT void line_task(INT stacd, void *exinf);
IMPORT UINT line_task_get_state(void);

#endif /* LINE_TASK_H */
