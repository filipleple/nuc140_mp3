/*
 * NUC140 + WAU8822 ADPCM player
 *
 * SD (FAT32) -> mono IMA ADPCM WAV -> decode -> I2S -> WAU8822 speaker out
 * Keypad: play/pause, next, prev, volume up/down
 * LCD: track name (row 0), progress bar (row 1), state/volume (row 2/3)
 *
 * Notes
 *  - Robust header parse for RIFF/WAVE with expected 'fmt'+'fact'+'data'
 *  - Double-buffered PCM with simple per-half refills (one ADPCM block = 256 B -> 504 samples)
 *  - Pause resumes at precise sample index
 *  - WAU8822 volume expressed as UI percent, mapped to safe raw range
 */

#include "include/player.h"

/* ===== Entry point ===== */
int32_t main(void)
{
    STR_UART_T sParam;

    UNLOCKREG();
    DrvSYS_Open(50000000);
    LOCKREG();

    sParam.u32BaudRate       = 115200;
    sParam.u8cDataBits       = DRVUART_DATABITS_8;
    sParam.u8cStopBits       = DRVUART_STOPBITS_1;
    sParam.u8cParity         = DRVUART_PARITY_NONE;
    sParam.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
    DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC, 0);
    DrvUART_Open(UART_PORT0, &sParam);
    DrvGPIO_InitFunction(E_FUNC_UART0);

    init_LCD();
    clear_LCD();
    OpenKeyPad();

    printf("\n\nNUC100 Series ADPCM\n");
    printf("Insert SDcard with mono IMA-ADPCM WAVs\n");

    return player_mainloop();
}

