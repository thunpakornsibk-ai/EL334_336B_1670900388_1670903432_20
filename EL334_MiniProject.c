/*
 * EL334 Mini-Project - FRDM-K64F + Multi-Function Shield + I2C LCD
 * VERIFIED pin mapping (FRDM-K64F schematic Rev E4, Arduino header):
 *   A1=PTB3, A2=PTB10, A3=PTB11 | D3=PTA1 | D10=PTD0 D11=PTD2 D12=PTD3 D13=PTD1
 *   D4=PTB23 (seg latch), D7=PTC3 (seg clk), D8=PTC12 (seg data)
 *   LCD (I2C bit-bang): SDA = PTC4 (D9), SCL = PTC2 (D6)  -> shield 3-pin headers ~9 and ~6
 *
 * NORMAL mode : S1 = LED cycle, S2 short = 7-seg edit mode,
 *               S2 hold >= 1 s = TIMER mode, S3 = Happy Birthday melody ON/OFF
 * 7-SEG mode  : S1 short = shift left, S2 = digit +1, S3 = melody ON/OFF,
 *               hold S1 >= 1 s = EXIT mode (hold S2 >= 1 s = backup exit)
 * TIMER mode  : LCD shows HH:MM:SS
 *   EDIT  : S1 = hour+1, S2 = minute+1, S3 = second+1, hold S3 >= 1 s = START
 *   COUNT : PIT2 1 Hz interrupt counts down, reaching 0 -> ALARM
 *   ALARM : melody plays, S3 = silence + reset
 *   any state : hold S1 >= 1 s = EXIT timer mode
 *
 * Timers: PIT0 = 1 ms tick, PIT1 = buzzer tone, PIT2 = 1 s countdown (only while counting)
 * Interrupts: PORTB + PIT0 + PIT1 + PIT2
 */

#include <MK64F12.h>
#include <stdint.h>
#include "system_MK64F12.h"

#define MASK(x) (1UL << (x))

/* ---------------- Pins ---------------- */
#define SW1_PIN     3U    /* PTB3  = A1 */
#define SW2_PIN     10U   /* PTB10 = A2 */
#define SW3_PIN     11U   /* PTB11 = A3 */

#define LED1_PIN    1U    /* PTD1 = D13 */
#define LED2_PIN    3U    /* PTD3 = D12 */
#define LED3_PIN    2U    /* PTD2 = D11 */
#define LED4_PIN    0U    /* PTD0 = D10 */
/* If your shield LEDs light opposite, change 1 to 0 */
#define LED_ACTIVE_LOW 1

#define BUZ_PIN     1U    /* PTA1 = D3 */

#define SEG_LAT_PIN 23U   /* PTB23 = D4 */
#define SEG_CLK_PIN 3U    /* PTC3  = D7 */
#define SEG_DAT_PIN 12U   /* PTC12 = D8 */

/* I2C is bit-banged, so any free GPIO pins work. The shield's own SCL/SDA header pins
 * (PTE24/PTE25) are not usable, so the LCD goes on the 3-pin sensor headers instead:
 *   SDA = D9 (PTC4), SCL = D6 (PTC2).  Power the LCD from the +5 and GND pins of the same headers.
 * To go back to PTE24/PTE25: LCD_GPIO=PTE, LCD_PORT=PORTE, LCD_SCGC5_MASK=SIM_SCGC5_PORTE_MASK,
 * SDA=25U, SCL=24U. */
#define LCD_GPIO        PTC
#define LCD_PORT        PORTC
#define LCD_SCGC5_MASK  SIM_SCGC5_PORTC_MASK
#define LCD_SDA_PIN     4U    /* PTC4 = D9 */
#define LCD_SCL_PIN     2U    /* PTC2 = D6 */
/* PCF8574 backpack address: try 0x27 first, if the LCD stays blank use 0x3F */
#define LCD_ADDR    0x27U

/* ---------------- Constants ---------------- */
#define DEBOUNCE_MS     40U
#define LONG_PRESS_MS   1000U
#define RELEASE_CNT     5U
#define EXIT_RELEASE_MS 30U    /* after hold-S1 exit, ignore S1/S2 until released this long */
#define TB_DEBOUNCE_MS  20U    /* timer-mode button debounce */

/* Timer mode states */
#define T_EDIT  0U
#define T_COUNT 1U
#define T_ALARM 2U

/* Melody tempo: 1 = original speed, 2 = 2x faster, 3 = 3x faster ... */
#define MELODY_SPEED    2U
/* 1 = melody repeats until S3 pressed; 0 = play once then stop */
#define MELODY_LOOP     1U

/* Note frequencies (Hz x 10) */
#define N_REST  0U
#define N_G4    3920U
#define N_A4    4400U
#define N_B4    4939U
#define N_C5    5233U
#define N_D5    5873U
#define N_E5    6593U
#define N_F5    6985U
#define N_G5    7840U

/* Note lengths (ms, before MELODY_SPEED is applied) */
#define QTR 500U     /* 1 beat */
#define HLF 1000U    /* 2 beats */
#define DEG 375U     /* dotted eighth */
#define SXT 125U     /* sixteenth */

typedef struct { uint16_t fx10; uint16_t ms; } note_t;

/* Happy Birthday to You */
static const note_t MEL[] = {
    /* Happy birthday to you */
    {N_G4,DEG},{N_G4,SXT},{N_A4,QTR},{N_G4,QTR},{N_C5,QTR},{N_B4,HLF},
    /* Happy birthday to you */
    {N_G4,DEG},{N_G4,SXT},{N_A4,QTR},{N_G4,QTR},{N_D5,QTR},{N_C5,HLF},
    /* Happy birthday dear ... */
    {N_G4,DEG},{N_G4,SXT},{N_G5,QTR},{N_E5,QTR},{N_C5,QTR},{N_B4,QTR},{N_A4,HLF},
    /* Happy birthday to you */
    {N_F5,DEG},{N_F5,SXT},{N_E5,QTR},{N_C5,QTR},{N_D5,QTR},{N_C5,HLF}
#if MELODY_LOOP
    ,{N_REST,HLF}      /* pause before repeating */
#endif
};
#define MEL_LEN (sizeof(MEL)/sizeof(MEL[0]))

#define NOTE_GAP_MAX_MS 25U    /* short silence at end of each note */

static const uint8_t SEG7[10] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
static const uint8_t DIGSEL[4] = {0xF1,0xF2,0xF4,0xF8};

/* ---------------- State ---------------- */
static volatile uint32_t g_tick = 0U;
static volatile uint8_t  g_flag = 0U;                 /* bit0 S1, bit1 S2, bit2 S3 */
static volatile uint8_t  g_deb1 = 0U, g_deb2 = 0U, g_deb3 = 0U;
static volatile uint8_t  g_led_step = 0U;             /* 0 off, 1..4 */
static volatile uint8_t  g_mode = 0U;                 /* 0 normal, 1 = 7-seg edit */
static volatile uint8_t  g_digits[4] = {0U,0U,0U,0U};
static volatile uint8_t  g_pos = 0U;                  /* blinking position */
static volatile uint8_t  g_mux = 0U;
static volatile uint8_t  g_mel_on = 0U;
static volatile uint8_t  g_note_i = 0U;
static volatile uint16_t g_note_ms = 0U;
static volatile uint8_t  g_s2_pend = 0U;              /* 0 idle,1 timing */
static volatile uint32_t g_s2_t0 = 0U;
static volatile uint8_t  g_s2_hi = 0U;
static volatile uint8_t  g_s1_pend = 0U;              /* 7-seg mode only: 0 idle, 1 timing */
static volatile uint32_t g_s1_t0 = 0U;
static volatile uint8_t  g_s1_hi = 0U;
static volatile uint8_t  g_lock = 0U;                 /* 1 = waiting for S1 and S2 to be released */
static volatile uint8_t  g_lock_rel = 0U;
static volatile uint32_t g_bus_hz = 0U;

/* ---- Timer mode + LCD state (added) ---- */
static volatile uint8_t  g_tmode = 0U;                /* 0 = off, 1 = timer mode */
static volatile uint8_t  g_tmr_state = T_EDIT;
static volatile uint8_t  g_tmr_h = 0U, g_tmr_m = 0U, g_tmr_s = 0U;
static volatile uint8_t  g_lcd_dirty = 1U;
static uint8_t  g_bl = 1U;                            /* LCD backlight */
static uint32_t g_i2c_dly = 60U;                      /* I2C half-bit delay (set in init_lcd_gpio) */

static const uint32_t TB_PIN[3] = { MASK(SW1_PIN), MASK(SW2_PIN), MASK(SW3_PIN) };
static uint8_t  tb_stable[3] = {0U,0U,0U};            /* debounced pressed state */
static uint8_t  tb_cnt[3]    = {0U,0U,0U};
static uint8_t  tb_used[3]   = {0U,0U,0U};            /* action already done on press */
static uint8_t  tb_fired[3]  = {0U,0U,0U};            /* long-press action already done */
static uint16_t tb_hold[3]   = {0U,0U,0U};
static uint8_t  tb_s2_armed  = 0U;                    /* S2 released since normal mode began */
static uint16_t tb_s2_hold   = 0U;
static uint8_t  tb_last_sig  = 0xFFU;

/* ---------------- Prototypes ---------------- */
uint32_t get_bus_clock_hz(void);
void init_gpio(void);
void init_buttons_irq(void);
void init_pit(uint32_t bus_hz);
void led_apply(void);
void seg_shift_byte(uint8_t b);
void seg_show(uint8_t pos, uint8_t num);
void seg_refresh(void);
void tone_start(uint16_t fx10);
void tone_stop(void);
void note_begin(uint8_t i);
void melody_start(void);
void melody_stop(void);
void melody_tick(void);
void do_s1(void);
void do_s3(void);
void s2_short(void);
void s2_long(void);
void s1_short(void);
void s1_long(void);
void poll_lock(void);
void poll_s1_hold(void);
void process_buttons(void);
void deb_rearm(void);
void poll_s2_hold(void);
void tick_1ms(void);
void PORTB_IRQHandler(void);
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);

/* Timer mode + LCD (added) */
void delay_busy_ms(uint32_t ms);
void delay_i2c(void);
void init_lcd_gpio(void);
void init_pit2(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t b);
void lcd_write_nibble(uint8_t nib, uint8_t rs);
void lcd_write_byte(uint8_t b, uint8_t rs);
void lcd_cmd(uint8_t c);
void lcd_init(void);
void lcd_put2(char *p, uint8_t v);
void lcd_copy(char *dst, const char *src);
void lcd_line(uint8_t row, const char *s);
void lcd_refresh(void);
void timer_enter(void);
void timer_exit(void);
void timer_start(void);
void timer_poll(void);
void PIT2_IRQHandler(void);

/* ---------------- Init ---------------- */
uint32_t get_bus_clock_hz(void)
{
    uint32_t d = (SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT;
    return SystemCoreClock / (d + 1U);
}

void init_gpio(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    /* Buzzer FIRST. The shield drives it through a PNP transistor from 5 V, so a
     * 3.3 V "high" from the MCU does NOT switch it off (it stays on = long beep).
     * Use open-drain (ODE): output 0 = buzzer ON (pin pulled low),
     *                       output 1 = buzzer OFF (pin released / hi-Z).
     * Write the OFF level before enabling the output so it is never driven low. */
    PORTA->PCR[BUZ_PIN] = PORT_PCR_MUX(1) | PORT_PCR_ODE_MASK;
    PTA->PSOR  = MASK(BUZ_PIN);          /* idle = silent */
    PTA->PDDR |= MASK(BUZ_PIN);

    PTB->PDDR &= ~(MASK(SW1_PIN)|MASK(SW2_PIN)|MASK(SW3_PIN));
    PORTB->PCR[SW1_PIN] = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
    PORTB->PCR[SW2_PIN] = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
    PORTB->PCR[SW3_PIN] = PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;

    PORTD->PCR[LED1_PIN]=PORT_PCR_MUX(1); PORTD->PCR[LED2_PIN]=PORT_PCR_MUX(1);
    PORTD->PCR[LED3_PIN]=PORT_PCR_MUX(1); PORTD->PCR[LED4_PIN]=PORT_PCR_MUX(1);
    PTD->PDDR |= MASK(LED1_PIN)|MASK(LED2_PIN)|MASK(LED3_PIN)|MASK(LED4_PIN);
    led_apply();

    PORTB->PCR[SEG_LAT_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[SEG_CLK_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[SEG_DAT_PIN] = PORT_PCR_MUX(1);
    PTB->PDDR |= MASK(SEG_LAT_PIN);
    PTC->PDDR |= MASK(SEG_CLK_PIN)|MASK(SEG_DAT_PIN);
    PTB->PSOR  = MASK(SEG_LAT_PIN);
    PTC->PCOR  = MASK(SEG_CLK_PIN);
}

void init_buttons_irq(void)
{
    PORTB->PCR[SW1_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->PCR[SW2_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->PCR[SW3_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->ISFR = 0xFFFFFFFFU;