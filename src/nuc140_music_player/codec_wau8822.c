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
    /* Enable L/R headphone path + ADCs; your 0x1B3 was already OK */
    i2c_write_wau8822(2,  0x1B3);
    /* Enable LDAC/RDAC + LMIX/RMIX, biasgen; DISABLE LSPK/RSPK here */
    i2c_write_wau8822(3,  0x01F);   /* was 0x07F which turned on speakers */
    /* I2S: 16-bit, I2S, and MONO=1 so left goes to both L/R */
    i2c_write_wau8822(4,  0x011);   /* was 0x010; bit0 MONO=1 */
    i2c_write_wau8822(5,  0x000);
    i2c_write_wau8822(6,  0x1ED);
    i2c_write_wau8822(7,  0x00A);
    i2c_write_wau8822(10, 0x008);
    i2c_write_wau8822(14, 0x108);
    i2c_write_wau8822(15, 0x1EF);
    i2c_write_wau8822(16, 0x1EF);
    i2c_write_wau8822(43, 0x010);
    i2c_write_wau8822(44, 0x000);
    i2c_write_wau8822(45, 0x150);
    i2c_write_wau8822(46, 0x150);
    i2c_write_wau8822(47, 0x007);
    i2c_write_wau8822(48, 0x007);
    /* Output control: clear SPKBST; keep outputs normal impedance */
    i2c_write_wau8822(49, 0x003);   /* was 0x047 enabling speaker boost */
    /* Route DACs to LMIX/RMIX for headphones on LHP/RHP */
    i2c_write_wau8822(50, 0x001);   /* LDAC -> LMIX */
    i2c_write_wau8822(51, 0x001);   /* RDAC -> RMIX (was 0x000) */
    /* Headphone volumes: set start value; speaker volumes muted */
    i2c_write_wau8822(52, 0x139);   /* LHP Volume (VU set, comfy gain) */
    i2c_write_wau8822(53, 0x139);   /* RHP Volume */
    i2c_write_wau8822(54, 0x180);   /* LSPK mute (VU|MUTE) */
    i2c_write_wau8822(55, 0x180);   /* RSPK mute (VU|MUTE) */
    codec_apply_vol_pct(g_vol_pct); /* ensures VU bit write and UI mapping */

    DrvGPIO_Open(E_GPE, 14, E_IO_OUTPUT);
    DrvGPIO_ClrBit(E_GPE, 14);
}

inline void codec_apply_vol_pct(uint8_t pct)
{
    uint16_t raw = vol_pct_to_raw(pct);
    g_vol_pct = pct;
    g_vol_raw_cached = (uint16_t)(raw | VOL_VU_BIT);

    /* Headphone volumes */
    i2c_write_wau8822(52, g_vol_raw_cached);  /* LHP */
    i2c_write_wau8822(53, g_vol_raw_cached);  /* RHP */
}

inline void volume_change_pct(int delta_pct)
{
    int p = (int)g_vol_pct + delta_pct;
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    codec_apply_vol_pct((uint8_t)p);
    volume_show_ui();
}
