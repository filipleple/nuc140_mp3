#include "include/codec_wau8822.h"

/* ===== I2C <-> WAU8822 ===== */
static __IO uint32_t g_i2c_tx_done = 0;
static uint8_t  g_codec_i2c_addr   = 0x1A;  /* 7-bit */
static uint8_t  g_i2c_tx_data[2];
static uint8_t  g_i2c_tx_idx = 0;


/* ===== I2C to WAU8822 ===== */
static void i2c0_callback_tx(uint32_t status)
{
    switch (status) {
    case 0x08: /* START tx */
        DrvI2C_WriteData(I2C_PORT0, (g_codec_i2c_addr << 1));
        DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        break;
    case 0x18: /* SLA+W ACK */
        DrvI2C_WriteData(I2C_PORT0, g_i2c_tx_data[g_i2c_tx_idx++]);
        DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        break;
    case 0x20: /* SLA+W NACK */
        DrvI2C_Ctrl(I2C_PORT0, 1, 1, 1, 0);
        break;
    case 0x28: /* DATA ACK */
        if (g_i2c_tx_idx < 2) {
            DrvI2C_WriteData(I2C_PORT0, g_i2c_tx_data[g_i2c_tx_idx++]);
            DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
        } else {
            DrvI2C_Ctrl(I2C_PORT0, 0, 1, 1, 0);
            g_i2c_tx_done = 1;
        }
        break;
    default:
        while (1) { /* unexpected state */ }
    }
}

static inline uint16_t vol_pct_to_raw(uint8_t pct)
{
    if (pct > 100) pct = 100;
    const uint16_t span = (uint16_t)(VOL_RAW_MAX_SAFE - VOL_RAW_MIN_SAFE);
    uint16_t raw = VOL_RAW_MIN_SAFE + (uint16_t)((pct * (uint32_t)span + 50U) / 100U);
    if (raw < VOL_RAW_MIN_SAFE) raw = VOL_RAW_MIN_SAFE;
    if (raw > VOL_RAW_MAX_SAFE) raw = VOL_RAW_MAX_SAFE;
    return (uint16_t)(raw & VOL_MASK);
}

static void i2c_write_wau8822(uint8_t reg, uint16_t data)
{
    g_i2c_tx_idx = 0;
    g_i2c_tx_done = 0;
    g_i2c_tx_data[0] = (uint8_t)((reg << 1) | (data >> 8));
    g_i2c_tx_data[1] = (uint8_t)(data & 0xFF);

    DrvI2C_InstallCallback(I2C_PORT0, I2CFUNC, i2c0_callback_tx);
    DrvI2C_Ctrl(I2C_PORT0, 1, 0, 0, 0); /* START */
    while (!g_i2c_tx_done) { /* wait */ }
}

void wau8822_setup(void)
{
    i2c_write_wau8822(0,  0x000); /* Reset */
    rough_delay(0x200);

    i2c_write_wau8822(1,  0x02F);
    i2c_write_wau8822(2,  0x1B3); /* HP, ADC mix/boost, ADC */
    i2c_write_wau8822(3,  0x07F); /* main mixer, DAC */
    i2c_write_wau8822(4,  0x010); /* 16-bit, I2S, stereo */
    i2c_write_wau8822(5,  0x000); /* no companding/loopback */
    i2c_write_wau8822(6,  0x1ED); /* 8 kHz */
    i2c_write_wau8822(7,  0x00A); /* 8 kHz coeffs */
    i2c_write_wau8822(10, 0x008); /* DAC softmute off, 128x */
    i2c_write_wau8822(14, 0x108); /* ADC HPF off, 128x */
    i2c_write_wau8822(15, 0x1EF); /* ADC L vol */
    i2c_write_wau8822(16, 0x1EF); /* ADC R vol */
    i2c_write_wau8822(43, 0x010);
    i2c_write_wau8822(44, 0x000);
    i2c_write_wau8822(45, 0x150);
    i2c_write_wau8822(46, 0x150);
    i2c_write_wau8822(47, 0x007);
    i2c_write_wau8822(48, 0x007);
    i2c_write_wau8822(49, 0x047);
    i2c_write_wau8822(50, 0x001); /* DAC -> LMIX */
    i2c_write_wau8822(51, 0x000); /* DAC -> RMIX */

    codec_apply_vol_pct(g_vol_pct); /* ensures VU bit write and UI mapping */

    DrvGPIO_Open(E_GPE, 14, E_IO_OUTPUT);
    DrvGPIO_ClrBit(E_GPE, 14);
}

inline void codec_apply_vol_pct(uint8_t pct)
{
    uint16_t raw = vol_pct_to_raw(pct);
    g_vol_pct = pct;
    g_vol_raw_cached = (uint16_t)(raw | VOL_VU_BIT);
    i2c_write_wau8822(54, g_vol_raw_cached); /* LSPKOUT */
    i2c_write_wau8822(55, g_vol_raw_cached); /* RSPKOUT */
}

inline void volume_change_pct(int delta_pct)
{
    int p = (int)g_vol_pct + delta_pct;
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    codec_apply_vol_pct((uint8_t)p);
    volume_show_ui();
}
