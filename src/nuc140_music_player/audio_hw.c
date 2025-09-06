#include "include/audio_hw.h"

/* ===== Codec + I2S init (8 kHz, 16-bit, I2S, slave) ===== */
void init_codec_i2s(void)
{
    /* Tri-state FS and BCLK while configuring codec */
    DrvGPIO_Open(E_GPC, 0, E_IO_OPENDRAIN); /* FS */
    DrvGPIO_Open(E_GPC, 1, E_IO_OPENDRAIN); /* BCLK */
    DrvGPIO_SetBit(E_GPC, 0);
    DrvGPIO_SetBit(E_GPC, 1);

    /* I2C0 @ 100 kHz */
    DrvGPIO_InitFunction(E_FUNC_I2C0);
    DrvI2C_Open(I2C_PORT0, 100000);
    DrvI2C_EnableInt(I2C_PORT0);

    /* Configure codec */
    wau8822_setup();

    /* I2S setup */
    S_DRVI2S_DATA_T st;
    st.u32SampleRate      = 8000;
    st.u8WordWidth        = DRVI2S_DATABIT_16;
    st.u8AudioFormat      = DRVI2S_STEREO;
    st.u8DataFormat       = DRVI2S_FORMAT_I2S;
    st.u8Mode             = DRVI2S_MODE_SLAVE;
    st.u8TxFIFOThreshold  = DRVI2S_FIFO_LEVEL_WORD_4;
    st.u8RxFIFOThreshold  = DRVI2S_FIFO_LEVEL_WORD_4;
    DrvI2S_SelectClockSource(0);
    DrvI2S_Open(&st);
    DrvGPIO_InitFunction(E_FUNC_I2S);
    DrvI2S_SetMCLKFreq(12000000);
    DrvI2S_EnableMCLK();
}

/* ===== I2S ISR (Tx threshold) ===== */
void Tx_thresholdCallbackfn0(uint32_t status)
{
    /* feed up to 4 samples per interrupt from PCM0 */
    for (uint32_t i = 0; i < 4; i++) {
        if (g_samples_left == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }
        if (g_pcm0_pos < PCM_BUFF_SAMPLES) {
            int32_t s = ((int32_t)g_pcm0[g_pcm0_pos++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            g_samples_left--;
        } else {
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }
    if (g_pcm0_pos >= PCM_BUFF_SAMPLES) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn1);
    }
}

void Tx_thresholdCallbackfn1(uint32_t status)
{
    /* feed up to 4 samples per interrupt from PCM1 */
    for (uint32_t i = 0; i < 4; i++) {
        if (g_samples_left == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }
        if (g_pcm1_pos < PCM_BUFF_SAMPLES) {
            int32_t s = ((int32_t)g_pcm1[g_pcm1_pos++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            g_samples_left--;
        } else {
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }
    if (g_pcm1_pos >= PCM_BUFF_SAMPLES) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    }
}

