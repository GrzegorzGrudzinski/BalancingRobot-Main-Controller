#include "diag.h"
#include <string.h>

Diag_t diag = {0};

void diagInit(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void diagResetCounters(void) {
    memset(&diag, 0, sizeof(diag));
}