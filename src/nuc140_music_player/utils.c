#include "include/utils.h"

void rough_delay(uint32_t t)
{
    volatile uint32_t d = t;
    while (d-- > 0) { /* busy wait */ }
}

uint32_t u32_le(const void *p)
{
    const uint8_t *b = (const uint8_t *)p;
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

uint16_t u16_le(const void *p)
{
    const uint8_t *b = (const uint8_t *)p;
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

void put_rc(FRESULT rc)
{
    static const TCHAR *p =
        _T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
        _T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
        _T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
        _T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
    UINT i;
    const TCHAR *q = p;
    for (i = 0; (i != (UINT)rc) && *q; i++) { while (*q++) { } }
    printf("rc=%u FR_%s\n", (UINT)rc, q);
}
