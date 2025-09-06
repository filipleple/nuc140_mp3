#include <stdio.h>
#include <string.h>
#include "UART.h"
#include "GPIO.h"
#include "I2C.h"
#include "I2S.h"
#include "SYS.h"
#include "diskio.h"
#include "ff.h"
#include <Audio.h>
#include "adpcm4bit.h"
#include "LCD.h"
#include "Scankey.h"

typedef enum { STATE_STOPPED=0, STATE_PLAYING=1, STATE_PAUSED=2 } play_state_t;
volatile play_state_t g_state = STATE_STOPPED;

#define KEY_NONE        0
#define KEY_VOLUME_UP   2
#define KEY_PREV        4
#define KEY_PLAY_PAUSE  5
#define KEY_NEXT        6
#define KEY_VOLUME_DOWN 8
#define ADPCM_BLOCK_BYTES    256
#define ADPCM_BLOCK_SAMPLES  504

#define LCD_COLS 16

/* redraw throttling for the bar */
static uint8_t g_last_bar_chars = 0xFF;

/* optional: if u32DataSize is changed in ISRs, make it volatile */
volatile uint32_t u32DataSize;

static void ui_progress_reset(void) { g_last_bar_chars = 0xFF; }

static uint32_t g_data_start = 0;        /* byte offset of ADPCM data */
static uint32_t g_resume_samples = 0;    /* absolute sample index to resume from */

volatile uint8_t g_tx_half = 0; /* 0 = i16PCMBuff, 1 = i16PCMBuff1 */

const int8_t i8WavHeader[]=
{0x52,0x49,0x46,0x46,0x34,0xA1,0x00,0x00,0x57,0x41,0x56,0x45,0x66,0x6D,0x74,0x20
,0x14,0x00,0x00,0x00,0x11,0x00,0x01,0x00,0x40,0x1F,0x00,0x00,0xDF,0x0F,0x00,0x00
,0x00,0x01,0x04,0x00,0x02,0x00,0xF9,0x01,0x66,0x61,0x63,0x74,0x04,0x00,0x00,0x00
,0x99,0x3D,0x01,0x00,0x64,0x61,0x74,0x61,0x00,0xA1,0x00,0x00};

int8_t i8FileName[13]="adpcm.wav";
#define i16PCMBuffSize 504
#define	u8FileBuffSize 512
int16_t i16PCMBuff[i16PCMBuffSize];
int16_t i16PCMBuff1[i16PCMBuffSize];
uint8_t	u8FileBuff[u8FileBuffSize];
int32_t i32AdpcmStatus=0;
uint32_t u32PCMBuffPointer;
uint32_t u32PCMBuffPointer1;
uint32_t u32FileBuffPointer;
AudioHeader WavFile;

FATFS FatFs[_DRIVES];		/* File system object for logical drive */
 
//int8_t gIsPlaying = 0;
/* For I2C transfer */
__IO uint32_t EndFlag0 = 0;
uint8_t Device_Addr0 = 0x1A;	 			/* WAU8822 Device ID */
uint8_t Tx_Data0[2];
uint8_t DataCnt0;


#define BUFF_LEN    32*12
#define REC_LEN     REC_RATE / 1000

/* Recoder Buffer and its pointer */
uint16_t PcmRecBuff[BUFF_LEN] = {0};
uint32_t u32RecPos_Out = 0;
uint32_t u32RecPos_In = 0;
									
/* Player Buffer and its pointer */
uint32_t PcmPlayBuff[BUFF_LEN] = {0};
uint32_t u32PlayPos_Out = 0;
uint32_t u32PlayPos_In = 0;
void Tx_thresholdCallbackfn1(uint32_t status);
void Tx_thresholdCallbackfn0(uint32_t status);
void Rx_thresholdCallbackfn1(uint32_t status);
void Rx_thresholdCallbackfn0(uint32_t status);
static void ui_update_progress_bar(void);

static void RoughDelay(uint32_t t)
{
    volatile int32_t delay;

    delay = t;

    while(delay-- >= 0);
}


/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      I2C0_Callback_Tx()	          	                                                          		   */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The callback function is to send Device ID and data to CODEC with I2C0							   */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_Callback_Tx(uint32_t status)
{
	if (status == 0x08)						/* START has been transmitted */
	{
		DrvI2C_WriteData(I2C_PORT0, Device_Addr0<<1);
		DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
	}	
	else if (status == 0x18)				/* SLA+W has been transmitted and ACK has been received */
	{
		DrvI2C_WriteData(I2C_PORT0, Tx_Data0[DataCnt0++]);
		DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
	}
	else if (status == 0x20)				/* SLA+W has been transmitted and NACK has been received */
	{
		
		DrvI2C_Ctrl(I2C_PORT0, 1, 1, 1, 0);
	}	
	else if (status == 0x28)				/* DATA has been transmitted and ACK has been received */
	{
		if (DataCnt0 != 2)
		{
			DrvI2C_WriteData(I2C_PORT0, Tx_Data0[DataCnt0++]);
			DrvI2C_Ctrl(I2C_PORT0, 0, 0, 1, 0);
		}
		else
		{
			DrvI2C_Ctrl(I2C_PORT0, 0, 1, 1, 0);
			EndFlag0 = 1;
		}		
	}
	else
	{
		while(1);
		//printf("Status 0x%x is NOT processed\n", status);
	}		
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      I2C_WriteWAU8822()	          	                                               			           */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The function is to write 9-bit data to 7-bit address register of WAU8822 with I2C interface. 	   */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{		
	DataCnt0 = 0;
	EndFlag0 = 0;
	
	Tx_Data0[0] = (uint8_t)((u8addr << 1) | (u16data >> 8));
	Tx_Data0[1] = (uint8_t)(u16data & 0x00FF);

	/* Install I2C0 call back function for write data to slave */
	DrvI2C_InstallCallback(I2C_PORT0, I2CFUNC, I2C0_Callback_Tx);
		
	/* I2C0 as master sends START signal */
	DrvI2C_Ctrl(I2C_PORT0, 1, 0, 0, 0);
		
	/* Wait I2C0 Tx Finish */
	while (EndFlag0 == 0);
}


/* Keep a shadow copy; WAU8822 has write-only style regs via I2C here */
static volatile uint16_t g_spkvol = 0x139;   /* VU bit set + a comfy default */
enum { VOL_VU_BIT = 0x100, VOL_MUTE_BIT = 0x080, VOL_MASK = 0x07F };

#define VOL_RAW_MIN_SAFE  0x10   /* floor that’s still audible but clean */
#define VOL_RAW_MAX_SAFE  0x3D   /* ceiling ~ old 48% of 0..0x7F */
#define VOL_UI_STEP_PCT   4      /* UI step per keypress (percent points) */

static volatile uint8_t  g_vol_pct = 50;      /* UI 0..100% */
static volatile uint16_t g_vol_raw_cached = (VOL_VU_BIT | VOL_RAW_MIN_SAFE);
static inline uint16_t map_pct_to_raw(uint8_t pct)
{
    if (pct > 100) pct = 100;
    const uint16_t span = (VOL_RAW_MAX_SAFE - VOL_RAW_MIN_SAFE);
    /* integer linear map: y + round(pct*span/100) */
    uint16_t raw = VOL_RAW_MIN_SAFE + (uint16_t)((pct * (uint32_t)span + 50) / 100);
    if (raw < VOL_RAW_MIN_SAFE) raw = VOL_RAW_MIN_SAFE;
    if (raw > VOL_RAW_MAX_SAFE) raw = VOL_RAW_MAX_SAFE;
    return raw & VOL_MASK;
}

static inline void codec_apply_vol_pct(uint8_t pct)
{
    uint16_t raw = map_pct_to_raw(pct);
    g_vol_pct = pct;
    g_vol_raw_cached = (raw | VOL_VU_BIT);       /* never touch mute here */
    I2C_WriteWAU8822(54, g_vol_raw_cached);      /* L */
    I2C_WriteWAU8822(55, g_vol_raw_cached);      /* R */
}

static inline void volume_show_ui(void)
{
    char line[16];
    snprintf(line, sizeof line, "Vol:%3u%%", (unsigned)g_vol_pct);
    print_Line(3, line);
}

static inline void volume_change_pct(int delta_pct)
{
    int p = (int)g_vol_pct + delta_pct;
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    codec_apply_vol_pct((uint8_t)p);
    volume_show_ui();
}


static inline uint8_t vol_pct_from_raw(uint16_t vraw)
{
    uint16_t v = vraw & VOL_MASK;
    /* Map 0..VOL_MAX linearly to 0..100 */
    return (uint8_t)((v * 100u) / VOL_MASK);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      WAU8822_Setup()	          	                                                  			           */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The function is to configure CODEC WAU8822 with I2C interface. 									   */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void WAU8822_Setup(void)
{
	I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */ 
	RoughDelay(0x200);
		
	I2C_WriteWAU8822(1,  0x02F);        
	I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
	I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
	//I2C_WriteWAU8822(3,  0x1FF);   /* Enable L/R main mixer, DAC */			
	I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */			
	I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
	
	//I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
	I2C_WriteWAU8822(6,  0x1ED);   /* Divide by 12, 8K */
	//I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter cofficients */
	I2C_WriteWAU8822(7,  0x00A);   /* 8K for internal filter cofficients */

	I2C_WriteWAU8822(10, 0x008);   /* DAC softmute is disabled, DAC oversampling rate is 128x */
	I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
	I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
	I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */
		
	I2C_WriteWAU8822(43, 0x010);   

	I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
	I2C_WriteWAU8822(45, 0x150);   /* LLIN connected, and its Gain value */
	I2C_WriteWAU8822(46, 0x150);   /* RLIN connected, and its Gain value */
	I2C_WriteWAU8822(47, 0x007);   /* LLIN connected, and its Gain value */
	I2C_WriteWAU8822(48, 0x007);   /* RLIN connected, and its Gain value */
	I2C_WriteWAU8822(49, 0x047);//006);

	//I2C_WriteWAU8822(50, 0x161);   /* Left DAC connected to LMIX */
	//I2C_WriteWAU8822(51, 0x160);//001);   /* Right DAC connected to RMIX */

	I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
	I2C_WriteWAU8822(51, 0x000);//001);   /* Right DAC connected to RMIX */
 
 	I2C_WriteWAU8822(54, 0x139);   /* LSPKOUT Volume */
	I2C_WriteWAU8822(55, 0x139);   /* RSPKOUT Volume */
  /* Apply current UI volume instead of a fixed raw */
  codec_apply_vol_pct(g_vol_pct);

	DrvGPIO_Open(E_GPE,14, E_IO_OUTPUT);	
	DrvGPIO_ClrBit(E_GPE,14);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      Tx_thresholdCallbackfn()	          	                                                           */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The callback function when Data in Tx FIFO is less than Tx FIFO Threshold Level. 				   */
/*      It is used to transfer data in Play Buffer to Tx FIFO. 											   */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      status    I2S Status register value.                     										   */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Tx_thresholdCallbackfn0(uint32_t status)
{
    uint32_t i; int32_t s;
    g_tx_half = 0;

    for (i = 0; i < 4; i++) {
        if (u32DataSize == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }

        if (u32PCMBuffPointer < i16PCMBuffSize) {
            s = ((int32_t)i16PCMBuff[u32PCMBuffPointer++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            u32DataSize--;
        } else {
            /* buffer not ready yet -> output silence to avoid screech */
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }

    if (u32PCMBuffPointer >= i16PCMBuffSize) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn1);
    }
}

void Tx_thresholdCallbackfn1(uint32_t status)
{
    uint32_t i; int32_t s;
    g_tx_half = 1;

    for (i = 0; i < 4; i++) {
        if (u32DataSize == 0) { DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD); DrvI2S_DisableTx(); return; }

        if (u32PCMBuffPointer1 < i16PCMBuffSize) {
            s = ((int32_t)i16PCMBuff1[u32PCMBuffPointer1++]) << 16;
            _DRVI2S_WRITE_TX_FIFO(s);
            u32DataSize--;
        } else {
            _DRVI2S_WRITE_TX_FIFO(0);
        }
    }

    if (u32PCMBuffPointer1 >= i16PCMBuffSize) {
        DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    }
}



/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      Rx_thresholdCallbackfn()	          	                                                           */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The callback function when Data in Rx FIFO is more than Rx FIFO Threshold Level. 				   */
/*      It is used to transfer data in Rx FIFO to Recode Buffer 										   */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      status    I2S Status register value.                     										   */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void Rx_thresholdCallbackfn0(uint32_t status)
{
	uint32_t  i;
	int32_t i32Data;
	
	for	( i = 0; i < 4; i++)
	{
		i32Data=_DRVI2S_READ_RX_FIFO()>>16;
		i16PCMBuff[u32PCMBuffPointer++]=i32Data;
	}
	if(u32PCMBuffPointer==i16PCMBuffSize)
	{
		DrvI2S_EnableInt(I2S_RX_FIFO_THRESHOLD, Rx_thresholdCallbackfn1);
		if(u32PCMBuffPointer1!=0)
			while(1);
	} 
}
void Rx_thresholdCallbackfn1(uint32_t status)
{
	uint32_t  i;
	int32_t i32Data;
	
	for	( i = 0; i < 4; i++)
	{
		i32Data=_DRVI2S_READ_RX_FIFO()>>16;
		i16PCMBuff1[u32PCMBuffPointer1++]=i32Data;
	}
	if(u32PCMBuffPointer1==i16PCMBuffSize)
	{
		DrvI2S_EnableInt(I2S_RX_FIFO_THRESHOLD, Rx_thresholdCallbackfn0);
		if(u32PCMBuffPointer!=0)
			while(1);
	}
}

void put_rc (FRESULT rc)
{
	const TCHAR *p =
		_T("OK\0DISK_ERR\0INT_ERR\0NOT_READY\0NO_FILE\0NO_PATH\0INVALID_NAME\0")
		_T("DENIED\0EXIST\0INVALID_OBJECT\0WRITE_PROTECTED\0INVALID_DRIVE\0")
		_T("NOT_ENABLED\0NO_FILE_SYSTEM\0MKFS_ABORTED\0TIMEOUT\0LOCKED\0")
		_T("NOT_ENOUGH_CORE\0TOO_MANY_OPEN_FILES\0");
	//FRESULT i;
	uint32_t i;
	for (i = 0; (i != (UINT)rc) && *p; i++) {
		while(*p++) ;
	}
	printf("rc=%u FR_%s\n",(UINT)rc, p);
}

uint32_t u32SwapBuffer(void* address)
{
	uint8_t* u8Char;
	u8Char = (uint8_t*) address;
	return (*u8Char|*(u8Char+1)<<8|*(u8Char+2)<<16|*(u8Char+3)<<24);
				
}
uint16_t u16SwapBuffer(void* address)
{
	uint8_t* u8Char;
	u8Char = (uint8_t*) address;
	return (*u8Char|*(u8Char+1)<<8);
				
}
void InitWAU8822(void)
{
	S_DRVI2S_DATA_T st;
	/* Tri-state for FS and BCLK of CODEC */
	DrvGPIO_Open(E_GPC, 0, E_IO_OPENDRAIN);
	DrvGPIO_Open(E_GPC, 1, E_IO_OPENDRAIN);
	DrvGPIO_SetBit(E_GPC, 0);
	DrvGPIO_SetBit(E_GPC, 1);
	
	/* Set I2C0 I/O */
    DrvGPIO_InitFunction(E_FUNC_I2C0);    	
	/* Open I2C0, and set clock = 100Kbps */
	DrvI2C_Open(I2C_PORT0, 100000);		
	/* Enable I2C0 interrupt and set corresponding NVIC bit */
	DrvI2C_EnableInt(I2C_PORT0);
	/* Configure CODEC */
	WAU8822_Setup();
	printf("Configure WAU8822\n");

	/* Configure I2S */
	st.u32SampleRate 	 = 8000;
    st.u8WordWidth 	 	 = DRVI2S_DATABIT_16;
    st.u8AudioFormat 	 = DRVI2S_STEREO;  		
	st.u8DataFormat  	 = DRVI2S_FORMAT_I2S;   
    st.u8Mode 		 	 = DRVI2S_MODE_SLAVE;
    st.u8TxFIFOThreshold = DRVI2S_FIFO_LEVEL_WORD_4;
    st.u8RxFIFOThreshold = DRVI2S_FIFO_LEVEL_WORD_4;
	DrvI2S_SelectClockSource(0);
	DrvI2S_Open(&st);
	/* Set I2S I/O */	
    DrvGPIO_InitFunction(E_FUNC_I2S); 
	/* Set MCLK and enable MCLK */
	DrvI2S_SetMCLKFreq(12000000);	
	DrvI2S_EnableMCLK();
}


#define MAX_TRACKS 32
static char     g_tracks[MAX_TRACKS][13];   /* 8.3 names */
static uint32_t g_track_count = 0;
static int32_t  g_track_idx   = -1;         /* current index in g_tracks */

static FIL g_file;                           /* current open file */

/* small helpers forward decls */
static void stop_playback(void);
static void start_playback(void);
static void pause_playback(void);
static void resume_playback(void);
static void rebuild_playlist(void);
static int  open_track_by_index(int32_t idx);
static int  parse_and_prepare_current_file(void);
static void handle_end_of_track(void);

/* ===== debounced keypad: 1 event per press ===== */
static int8_t get_key_press(void)
{
    enum { STABLE_N = 5 };               /* ~5 consecutive identical scans */
    static int8_t  last_raw = KEY_NONE;
    static uint8_t stable_cnt = 0;
    static uint8_t pressed_latch = 0;

    int8_t raw = ScanKey();

    if (raw == last_raw) {
        if (stable_cnt < 255) stable_cnt++;
    } else {
        stable_cnt = 0;
        last_raw   = raw;
    }

    if (stable_cnt < STABLE_N) return KEY_NONE;   /* not yet stable */

    /* rising edge -> emit exactly one event */
    if (!pressed_latch && raw != KEY_NONE) {
        pressed_latch = 1;
        return raw;
    }

    /* falling edge -> arm for next press */
    if (pressed_latch && raw == KEY_NONE) {
        pressed_latch = 0;
    }

    return KEY_NONE;
}

static void poll_keys_and_dispatch(void)
{
    int8_t k = get_key_press();
    if (k == KEY_NONE) return;

    switch (k) {
    case KEY_VOLUME_UP:
        volume_change_pct(+VOL_UI_STEP_PCT);
        break;
    case KEY_VOLUME_DOWN:
        volume_change_pct(-VOL_UI_STEP_PCT);
        break;
    case KEY_PLAY_PAUSE:
        if (g_state == STATE_PLAYING)      pause_playback();
        else if (g_state == STATE_PAUSED)  resume_playback();
        else                                start_playback();
        break;
    case KEY_NEXT:
        stop_playback();
        if (g_track_count) {
            g_track_idx = (g_track_idx + 1) % g_track_count;
            start_playback();
        }
        break;
    case KEY_PREV:
        stop_playback();
        if (g_track_count) {
            g_track_idx = (g_track_idx + g_track_count - 1) % g_track_count;
            start_playback();
        }
        break;
    default:
        break;
    }
}

/* ===== control primitives ===== */
static void stop_playback(void)
{
    DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD);
    DrvI2S_DisableTx();
    u32DataSize = 0;
    f_close(&g_file);
    g_state = STATE_STOPPED;
    print_Line(2, "Stopped   ");
    ui_progress_reset();
    print_Line(1, "                ");  /* clear bar line */
}

/* ==== capture precise pause position ==== */
static void pause_playback(void)
{
    DrvI2S_DisableInt(I2S_TX_FIFO_THRESHOLD);
    DrvI2S_DisableTx();
    /* total played so far = total_samples - remaining */
    g_resume_samples = WavFile.u32SampleNumber - u32DataSize;
    g_state = STATE_PAUSED;
    ui_update_progress_bar();
    print_Line(2, "Paused    ");
}

static void lcd_print_track_name(void)
{
    if (g_track_count == 0 || g_track_idx < 0) {
        print_Line(0, "No WAVs        ");
        return;
    }
    char name[13];
    strncpy(name, g_tracks[g_track_idx], 12);
    name[12] = '\0';
    char *dot = strrchr(name, '.');
    if (dot) *dot = '\0';

    char line[17];
    /* “01 ” + up to 13 chars of name = 16 columns total */
    snprintf(line, sizeof line, "%02u %-13.13s", (unsigned)(g_track_idx + 1), name);
    print_Line(0, line);
}

/* ==== small helper to decode exactly one ADPCM block into a PCM buffer ==== */
static int decode_one_block_into(int16_t *dest)
{
    UINT br;
    if (f_read(&g_file, u8FileBuff, ADPCM_BLOCK_BYTES, &br) != FR_OK) return 0;
    if (br < ADPCM_BLOCK_BYTES) return 0;
    AdpcmDec4(u8FileBuff, dest, i16PCMBuffSize);   /* produces 504 samples */
    return 1;
}

/* ==== prefill ping/pong from an absolute sample position ==== */
static int prefill_from_sample(uint32_t sample_pos)
{
    uint32_t blk = sample_pos / ADPCM_BLOCK_SAMPLES;
    uint32_t ofs = sample_pos % ADPCM_BLOCK_SAMPLES;

    if (f_lseek(&g_file, g_data_start + (DWORD)blk * ADPCM_BLOCK_BYTES) != FR_OK) return 0;

    /* fill ping */
    if (!decode_one_block_into(i16PCMBuff)) return 0;
    u32PCMBuffPointer = ofs;                     /* start inside this block */

    /* fill pong with the next block if available */
    if (decode_one_block_into(i16PCMBuff1))
        u32PCMBuffPointer1 = 0;
    else
        u32PCMBuffPointer1 = i16PCMBuffSize;     /* mark empty, main loop will refill */

    g_tx_half = 0;
    return 1;
}


/* ==== start from an absolute sample position using your proven start flow ==== */
static void start_playback_from_sample(uint32_t sample_pos)
{
    if (g_track_count == 0) { print_Line(2, "No WAVs   "); return; }
    if (g_track_idx < 0 || g_track_idx >= (int32_t)g_track_count) g_track_idx = 0;

    /* reopen + parse exactly like start_playback() */
    f_close(&g_file);
    if (!open_track_by_index(g_track_idx) || !parse_and_prepare_current_file()) {
        print_Line(2, "Open/parse ");
        g_state = STATE_STOPPED;
        return;
    }

    /* clamp sample_pos to range */
    if (sample_pos > WavFile.u32SampleNumber) sample_pos = WavFile.u32SampleNumber;

    /* reset stream state and reinit codec/I²S exactly like start */
    u32PCMBuffPointer = u32PCMBuffPointer1 = 0;
    InitWAU8822();
    lcd_print_track_name();
    ui_progress_reset();
    ui_update_progress_bar();
    codec_apply_vol_pct(g_vol_pct);

    /* set remaining sample counter and prefill ping/pong from position */
    u32DataSize = WavFile.u32SampleNumber - sample_pos;
    if (!prefill_from_sample(sample_pos)) { handle_end_of_track(); return; }

    /* arm TX just like start_playback() */
    DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    DrvI2S_EnableTx();

    g_state = STATE_PLAYING;
    print_Line(2, "Playing   ");
}

/* ==== make resume use the cold-start-at-position path ==== */
static void resume_playback(void)
{
    start_playback_from_sample(g_resume_samples);
}

static void start_playback(void)
{
    if (g_track_count == 0) {
        print_Line(2, "No WAVs   ");
        return;
    }
    if (g_track_idx < 0 || g_track_idx >= (int32_t)g_track_count)
        g_track_idx = 0;

    if (!open_track_by_index(g_track_idx)) {
        print_Line(2, "Open fail ");
        g_state = STATE_STOPPED;
        return;
    }
    if (!parse_and_prepare_current_file()) {
        f_close(&g_file);
        print_Line(2, "Bad WAV   ");
        g_state = STATE_STOPPED;
        return;
    }

    /* reset stream pointers and init codec/I2S for this file’s rate */
    u32PCMBuffPointer = u32PCMBuffPointer1 = 0;
    InitWAU8822();
    lcd_print_track_name();              /* show current track on LCD line 0 */
    ui_progress_reset();
    ui_update_progress_bar();
    /* Re-apply UI volume in case codec reset altered it */
    codec_apply_vol_pct(g_vol_pct);
    
    DrvI2S_EnableInt(I2S_TX_FIFO_THRESHOLD, Tx_thresholdCallbackfn0);
    DrvI2S_EnableTx();

    g_state = STATE_PLAYING;
    print_Line(2, "Playing        ");
}

/* ===== playlist build: root directory, .WAV ADPCM (fmt=0x11) only ===== */
static int is_wav_adpcm(const char *name)
{
    /* simple extension check .WAV/.wav */
    const char *dot = strrchr(name, '.');
    if (!dot) return 0;
    if (!strcasecmp(dot, ".WAV")) return 1;
    return 0;
}

static void rebuild_playlist(void)
{
    DIR dir;
    FILINFO fi;
    g_track_count = 0;
    if (f_opendir(&dir, "\\") != FR_OK) return;

    for (;;) {
        if (f_readdir(&dir, &fi) != FR_OK) break;
        if (fi.fname[0] == 0) break;
        if (fi.fattrib & AM_DIR) continue;
        if (!is_wav_adpcm(fi.fname)) continue;

        /* tentatively add; deep parse happens on open */
        if (g_track_count < MAX_TRACKS) {
            strncpy(g_tracks[g_track_count], fi.fname, 12);
            g_tracks[g_track_count][12] = '\0';
            g_track_count++;
        }
    }
    // f_closedir(&dir); not implemented in this BSP
}

/* ===== open by index and parse header the same way your code does ===== */
static int open_track_by_index(int32_t idx)
{
    if (idx < 0 || idx >= (int32_t)g_track_count) return 0;
    if (f_open(&g_file, (TCHAR*)g_tracks[idx], FA_READ) != FR_OK) return 0;
    return 1;
}

/* parse WAV header and set u32DataSize, seek to data; enforce ADPCM fmt=0x11 */
static int parse_and_prepare_current_file(void)
{
    UINT br;
    u32FileBuffPointer = u8FileBuffSize;
    if (f_lseek(&g_file, 0) != FR_OK) return 0;
    if (f_read(&g_file, u8FileBuff, u32FileBuffPointer, &br) != FR_OK) return 0;
    if (br != u8FileBuffSize) return 0;

    u32FileBuffPointer = 0;
    if (memcmp(u8FileBuff, "RIFF", 4)) return 0;
    u32FileBuffPointer += 4;
    WavFile.u32ChunkSize = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;

    if (memcmp(u8FileBuff + u32FileBuffPointer, "WAVEfmt ", 8)) return 0;
    u32FileBuffPointer += 8;
    WavFile.u32Subchunk1Size = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;
    WavFile.u16AudioFormat   = u16SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 2;
    WavFile.u16NumChannels   = u16SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 2;
    WavFile.u32SampleRate    = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;
    WavFile.u32ByteRate      = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;
    WavFile.u16BlockAlign    = u16SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 2;
    WavFile.u16BitsPerSample = u16SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 2;

    /* expect optional 'fact' then 'data' like your original code */
    u32FileBuffPointer += 4;
    if (memcmp(u8FileBuff + u32FileBuffPointer, "fact", 4)) return 0;
    u32FileBuffPointer += 8;
    WavFile.u32SampleNumber = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;

    if (memcmp(u8FileBuff + u32FileBuffPointer, "data", 4)) return 0;
    u32FileBuffPointer += 4;
    WavFile.u8HeaderStatus  = 1;
    WavFile.u32Subchunk2Size = u32SwapBuffer(u8FileBuff + u32FileBuffPointer); u32FileBuffPointer += 4;

    if (WavFile.u16AudioFormat != 0x0011) return 0;   /* IMA ADPCM */

    /* playback parameters */
    u32DataSize = WavFile.u32SampleNumber;
    /* seek to start of ADPCM data */
    u32FileBuffPointer = 0x28 + WavFile.u32Subchunk1Size;
    if (f_lseek(&g_file, u32FileBuffPointer) != FR_OK) return 0;

    g_data_start = u32FileBuffPointer;       /* remember where data begins */

    /* optional: adapt WAU8822 + I2S for sample rate. This demo uses 8 kHz.
       If your files are all 8 kHz, nothing to change. If you mix 8 k/16 k,
       reprogram regs 6/7 and DrvI2S_Open() accordingly in InitWAU8822(). */
    return 1;
}

/* when a song ends, auto-advance to next */
static void handle_end_of_track(void)
{
    stop_playback();
    if (g_track_count == 0) return;
    g_track_idx = (g_track_idx + 1) % g_track_count;
    start_playback();
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* FUNCTION                                                                                                */
/*      UAC_MainProcess()	      	                                                                       */
/*                                                                                                         */
/* DESCRIPTION                                                                                             */
/*      The main loop of UAC funciton.                                                                     */
/*                                                                                                         */
/* INPUTS                                                                                                  */
/*      None                                                                                               */
/*                                                                                                         */
/* OUTPUTS                                                                                                 */
/*      None                            				                                                   */
/*                                                                                                         */
/* RETURN                                                                                                  */
/*                                                                                                         */
/*      None                                                                                               */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UAC_MainProcess(void)
{
    FRESULT res;

    res = (FRESULT)disk_initialize(0);
    if (res) { put_rc(res); printf("SD init failed\n"); }

    res = f_mount(0, &FatFs[0]);
    if (res) { put_rc(res); printf("mount failed\n"); }


    rebuild_playlist();
    if (g_track_count == 0) {
        lcd_print_track_name();          /* prints “No WAVs        ” on line 0 */
        print_Line(2, "Stopped        ");
    } else {
        g_track_idx = 0;
        start_playback();
    }

    print_Line(1, "16bit Mono 8KHz");

    while (1)
    {
        /* keypad and UI */
        poll_keys_and_dispatch();
        ui_update_progress_bar();
        
        /* if paused or stopped, skip decode work but keep polling keys */
        if (g_state != STATE_PLAYING) {
            DrvSYS_Delay(1000);
            continue;
        }

        /* refill ping buffer */
        if (u32PCMBuffPointer >= i16PCMBuffSize) {
            UINT br;
            u32FileBuffPointer = u8FileBuffSize;
            if (f_read(&g_file, u8FileBuff, u32FileBuffPointer, &br) != FR_OK) {
                handle_end_of_track();
                continue;
            }
            if (br == 0) { handle_end_of_track(); continue; }
            AdpcmDec4(u8FileBuff, (short*)i16PCMBuff, i16PCMBuffSize);
            u32PCMBuffPointer = 0;
        }

        /* refill pong buffer */
        if (u32PCMBuffPointer1 >= i16PCMBuffSize) {
            /* you kept this offset; preserve it for your ADPCM packet framing */
            AdpcmDec4(u8FileBuff + (i16PCMBuffSize/2 + 4), (short*)i16PCMBuff1, i16PCMBuffSize);
            u32PCMBuffPointer1 = 0;
        }

        /* end-of-file check mirrors your previous loop condition */
        if (u32DataSize == 0) {
            handle_end_of_track();
            continue;
        }
    }
}


void Delay(uint32_t delayCnt)
{
    while(delayCnt--)
    {
        __NOP();
        __NOP();
    }
}

static void ui_update_progress_bar(void)
{
    if (!WavFile.u32SampleNumber) return;

    /* snapshot once; tiny race is fine for UI */
    uint32_t total  = WavFile.u32SampleNumber;
    uint32_t left   = u32DataSize;
    uint32_t played = (left <= total) ? (total - left) : 0;

    /* scale played -> 0..LCD_COLS */
    uint8_t filled = (uint8_t)(((uint64_t)played * LCD_COLS) / total);
    if (filled > LCD_COLS) filled = LCD_COLS;

    if (filled == g_last_bar_chars) return;

    char line[LCD_COLS + 1];
    for (uint8_t i = 0; i < LCD_COLS; i++)
        line[i] = (i < filled) ? '=' : ' ';
    line[LCD_COLS] = '\0';

    print_Line(1, line);
    g_last_bar_chars = filled;
}


/*---------------------------------------------------------------------------------------------------------*/
/* Main function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
	STR_UART_T sParam;

	//extern uint32_t SystemFrequency;
    /* Unlock the locked registers */
  UNLOCKREG();
	DrvSYS_Open(50000000);
	LOCKREG();
	
	/* UART Setting */
    sParam.u32BaudRate 		= 115200;
    sParam.u8cDataBits 		= DRVUART_DATABITS_8;
    sParam.u8cStopBits 		= DRVUART_STOPBITS_1;
    sParam.u8cParity 		= DRVUART_PARITY_NONE;
    sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;
	/* Select UART Clock Source From 12MHz */
	DrvSYS_SelectIPClockSource(E_SYS_UART_CLKSRC,0); 
	/* Set UART Configuration */
	DrvUART_Open(UART_PORT0,&sParam);	
	/* Set UART Pin */
	DrvGPIO_InitFunction(E_FUNC_UART0);	

  init_LCD();  //call initial pannel function
	clear_LCD();

	OpenKeyPad();	       // initialize 3x3 keypad

	printf("\n\nNUC100 Series ADPCM!\n");
	printf("Please insert SDcard and connect line in with J1\n");  
  UAC_MainProcess();
}
