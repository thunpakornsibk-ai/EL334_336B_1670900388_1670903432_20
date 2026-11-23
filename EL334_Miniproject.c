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
    NVIC_ClearPendingIRQ(PORTB_IRQn);
    NVIC_EnableIRQ(PORTB_IRQn);
}

void init_pit(uint32_t bus_hz)
{
    g_bus_hz = bus_hz;
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0U;
    PIT->CHANNEL[0].LDVAL = (bus_hz/1000U)-1U;
    PIT->CHANNEL[0].TFLG  = PIT_TFLG_TIF_MASK;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK|PIT_TCTRL_TEN_MASK;
    PIT->CHANNEL[1].TCTRL = 0U;
    PIT->CHANNEL[1].TFLG  = PIT_TFLG_TIF_MASK;
    NVIC_ClearPendingIRQ(PIT0_IRQn); NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_ClearPendingIRQ(PIT1_IRQn); NVIC_EnableIRQ(PIT1_IRQn);
}

/* ---------------- LEDs ---------------- */
void led_apply(void)
{
    uint8_t a = (g_led_step==1U)?1U:0U, b = (g_led_step==2U)?1U:0U;
    uint8_t c = (g_led_step==3U)?1U:0U, d = (g_led_step==4U)?1U:0U;
#if LED_ACTIVE_LOW
    a^=1U; b^=1U; c^=1U; d^=1U;
#endif
    if(a) PTD->PSOR=MASK(LED1_PIN); else PTD->PCOR=MASK(LED1_PIN);
    if(b) PTD->PSOR=MASK(LED2_PIN); else PTD->PCOR=MASK(LED2_PIN);
    if(c) PTD->PSOR=MASK(LED3_PIN); else PTD->PCOR=MASK(LED3_PIN);
    if(d) PTD->PSOR=MASK(LED4_PIN); else PTD->PCOR=MASK(LED4_PIN);
}

/* S1 : normal mode = LED cycle (on press)
 *      7-seg mode  = short press: shift left (on release), hold >= 1 s: exit */
void do_s1(void)
{
    if (g_mode) {
        g_s1_pend = 1U; g_s1_t0 = g_tick; g_s1_hi = 0U;   /* decide short/long in poll_s1_hold() */
        return;
    }
    g_led_step++;
    if (g_led_step > 4U) g_led_step = 0U;
    led_apply();
}

/* S3 : melody ON/OFF in every mode */
void do_s3(void)
{
    if (g_mel_on) melody_stop(); else melody_start();
}

void s2_short(void)
{
    if (!g_mode) {                       /* enter 7-seg mode */
        g_mode = 1U;
        g_pos  = 0U;
    } else {
        g_digits[g_pos] = (uint8_t)((g_digits[g_pos]+1U)%10U);
    }
}

void s2_long(void)
{
    if (g_mode) g_mode = 0U;             /* backup exit: hold S2 >= 1 s */
}

/* 7-seg mode: S1 short press = shift left */
void s1_short(void)
{
    if (g_mode) g_pos = (uint8_t)((g_pos+3U)&3U);
}

/* 7-seg mode: hold S1 >= 1 s = exit */
void s1_long(void)
{
    g_mode     = 0U;
    g_lock     = 1U;                     /* ignore S1/S2 until both are released */
    g_lock_rel = 0U;
    g_flag    &= (uint8_t)~3U;           /* drop pending S1/S2 edges */
}

/* Release the lock once S1 and S2 have both been up for EXIT_RELEASE_MS */
void poll_lock(void)
{
    if (!g_lock) return;
    if (((PTB->PDIR & MASK(SW1_PIN)) != 0U) && ((PTB->PDIR & MASK(SW2_PIN)) != 0U)) {
        g_lock_rel++;
        if (g_lock_rel >= EXIT_RELEASE_MS) { g_lock = 0U; g_lock_rel = 0U; }
    } else {
        g_lock_rel = 0U;
    }
}

/* ---------------- 7-segment ---------------- */
void seg_shift_byte(uint8_t b)
{
    int8_t i;
    for (i=7; i>=0; i--) {
        PTC->PCOR = MASK(SEG_CLK_PIN);
        if (b & (1U<<i)) PTC->PSOR = MASK(SEG_DAT_PIN);
        else             PTC->PCOR = MASK(SEG_DAT_PIN);
        PTC->PSOR = MASK(SEG_CLK_PIN);
    }
    PTC->PCOR = MASK(SEG_CLK_PIN);
}

void seg_show(uint8_t pos, uint8_t num)
{
    uint8_t seg = (num>9U) ? 0xFFU : SEG7[num];
    PTB->PCOR = MASK(SEG_LAT_PIN);
    seg_shift_byte(seg);
    seg_shift_byte(DIGSEL[pos]);
    PTB->PSOR = MASK(SEG_LAT_PIN);
}

void seg_refresh(void)
{
    uint8_t val;
    if (!g_mode) {
        seg_show(g_mux, 10U);                       /* blank when mode off */
    } else {
        val = g_digits[g_mux];
        if ((g_mux==g_pos) && ((g_tick%500U)>=250U)) val = 10U;  /* blink */
        seg_show(g_mux, val);
    }
    g_mux = (uint8_t)((g_mux+1U)&3U);
}

/* ---------------- Melody ---------------- */
void tone_start(uint16_t fx10)
{
    uint32_t half = (g_bus_hz*10U)/(2U*(uint32_t)fx10);
    PIT->CHANNEL[1].TCTRL = 0U;
    PIT->CHANNEL[1].LDVAL = half-1U;
    PIT->CHANNEL[1].TFLG  = PIT_TFLG_TIF_MASK;
    PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK|PIT_TCTRL_TEN_MASK;
}

void tone_stop(void)
{
    PIT->CHANNEL[1].TCTRL = 0U;
    PTA->PSOR = MASK(BUZ_PIN);
}

/* Start note i (a rest = silence) */
void note_begin(uint8_t i)
{
    if (MEL[i].fx10 == N_REST) tone_stop();
    else                       tone_start(MEL[i].fx10);
}

void melody_start(void)
{
    g_mel_on = 1U; g_note_i = 0U; g_note_ms = 0U;
    note_begin(0U);
}

void melody_stop(void)
{
    g_mel_on = 0U;
    tone_stop();
}

void melody_tick(void)
{
    uint16_t dur, gap;
    if (!g_mel_on) return;

    dur = (uint16_t)(MEL[g_note_i].ms / MELODY_SPEED);  /* faster tempo */
    if (dur == 0U) dur = 1U;

    g_note_ms++;
    if (g_note_ms >= dur) {
        g_note_ms = 0U;
        g_note_i++;
        if (g_note_i >= MEL_LEN) {
#if MELODY_LOOP
            g_note_i = 0U;                          /* repeat until S3 */
#else
            melody_stop(); return;
#endif
        }
        note_begin(g_note_i);
    } else if (MEL[g_note_i].fx10 != N_REST) {
        gap = (uint16_t)(dur / 4U);                 /* silence at note end so */
        if (gap > NOTE_GAP_MAX_MS) gap = NOTE_GAP_MAX_MS;   /* repeated notes are distinct */
        if (g_note_ms == (uint16_t)(dur - gap)) tone_stop();
    }
}

/* ---------------- Buttons ---------------- */
void process_buttons(void)
{
    if (g_lock) g_flag &= (uint8_t)~3U;   /* ignore S1/S2 until both released after S1+S2 exit */

    if (g_flag & 1U) {
        g_flag &= (uint8_t)~1U;
        if ((g_deb1==0U) && ((PTB->PDIR & MASK(SW1_PIN))==0U) && (g_s1_pend==0U)) { g_deb1=DEBOUNCE_MS; do_s1(); }
    }
    if (g_flag & 4U) {
        g_flag &= (uint8_t)~4U;
        if ((g_deb3==0U) && ((PTB->PDIR & MASK(SW3_PIN))==0U)) { g_deb3=DEBOUNCE_MS; do_s3(); }
    }
    if (g_flag & 2U) {
        g_flag &= (uint8_t)~2U;
        if ((g_deb2==0U) && ((PTB->PDIR & MASK(SW2_PIN))==0U) && (g_s2_pend==0U)) {
            g_deb2=DEBOUNCE_MS; g_s2_pend=1U; g_s2_t0=g_tick; g_s2_hi=0U;
        }
    }
}

/* Keep the debounce timers loaded while a button is held, so they only start counting
 * down after the button is RELEASED. This blocks contact bounce on release from being
 * seen as a new press (one press must never give two steps). */
void deb_rearm(void)
{
    if ((PTB->PDIR & MASK(SW1_PIN)) == 0U) g_deb1 = DEBOUNCE_MS;
    if ((PTB->PDIR & MASK(SW2_PIN)) == 0U) g_deb2 = DEBOUNCE_MS;
    if ((PTB->PDIR & MASK(SW3_PIN)) == 0U) g_deb3 = DEBOUNCE_MS;
}

void poll_s2_hold(void)
{
    if (g_s2_pend != 1U) return;
    if ((PTB->PDIR & MASK(SW2_PIN)) == 0U) {          /* still held */
        g_s2_hi = 0U;
        if ((g_tick - g_s2_t0) >= LONG_PRESS_MS) { g_s2_pend=0U; s2_long(); }
    } else {                                          /* released */
        g_s2_hi++;
        if (g_s2_hi >= RELEASE_CNT) { g_s2_pend=0U; s2_short(); }
    }
}

void poll_s1_hold(void)
{
    if (g_s1_pend != 1U) return;
    if ((PTB->PDIR & MASK(SW1_PIN)) == 0U) {          /* still held */
        g_s1_hi = 0U;
        if ((g_tick - g_s1_t0) >= LONG_PRESS_MS) { g_s1_pend=0U; s1_long(); }
    } else {                                          /* released */
        g_s1_hi++;
        if (g_s1_hi >= RELEASE_CNT) { g_s1_pend=0U; s1_short(); }
    }
}

void tick_1ms(void)
{
    g_tick++;
    if (g_deb1) g_deb1--;
    if (g_deb2) g_deb2--;
    if (g_deb3) g_deb3--;
    poll_lock();
    timer_poll();          /* added: timer mode / LCD hook */
    process_buttons();
    deb_rearm();           /* debounce on release too */
    poll_s2_hold();
    poll_s1_hold();
    melody_tick();
    seg_refresh();
}

/* ---------------- ISRs ---------------- */
void PORTB_IRQHandler(void)
{
    uint32_t f = PORTB->ISFR;
    if (f & MASK(SW1_PIN)) { PORTB->ISFR = MASK(SW1_PIN); g_flag |= 1U; }
    if (f & MASK(SW2_PIN)) { PORTB->ISFR = MASK(SW2_PIN); g_flag |= 2U; }
    if (f & MASK(SW3_PIN)) { PORTB->ISFR = MASK(SW3_PIN); g_flag |= 4U; }
}

void PIT0_IRQHandler(void)
{
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
        tick_1ms();
    }
}

void PIT1_IRQHandler(void)
{
    if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
        PTA->PTOR = MASK(BUZ_PIN);
    }
}

/* ====================================================================
 *  TIMER MODE + I2C LCD  (added)
 *  - Timer mode owns the buttons while it is active (timer_poll), so the
 *    existing S1/S2/S3 functions above are not used and not changed.
 *  - LCD is refreshed from main(); countdown runs in PIT2 interrupt.
 * ==================================================================== */

/* ---------- busy-wait delays (scaled with core clock) ---------- */
void delay_busy_ms(uint32_t ms)
{
    volatile uint32_t j;
    uint32_t n = SystemCoreClock / 4000U;        /* assume >= 4 cycles per pass */
    while (ms--) {
        for (j = 0U; j < n; j++) { __NOP(); }
    }
}

void delay_i2c(void)
{
    volatile uint32_t i;
    for (i = 0U; i < g_i2c_dly; i++) { __NOP(); }
}

/* ---------- init ---------- */
void init_lcd_gpio(void)
{
    SIM->SCGC5 |= LCD_SCGC5_MASK;

    /* I2C bit-bang: open-drain with internal pull-up */
    LCD_PORT->PCR[LCD_SDA_PIN] = PORT_PCR_MUX(1)|PORT_PCR_ODE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
    LCD_PORT->PCR[LCD_SCL_PIN] = PORT_PCR_MUX(1)|PORT_PCR_ODE_MASK|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
    LCD_GPIO->PSOR  = MASK(LCD_SDA_PIN)|MASK(LCD_SCL_PIN);   /* idle high (released) */
    LCD_GPIO->PDDR |= MASK(LCD_SDA_PIN)|MASK(LCD_SCL_PIN);

    g_i2c_dly = SystemCoreClock / 800000U;               /* ~5 us per half bit (<= 100 kHz) */
    if (g_i2c_dly < 8U) g_i2c_dly = 8U;
}

void init_pit2(void)
{
    /* PIT clock + MCR are already set by init_pit(). PIT2 is started only while counting. */
    PIT->CHANNEL[2].TCTRL = 0U;
    PIT->CHANNEL[2].TFLG  = PIT_TFLG_TIF_MASK;
    NVIC_ClearPendingIRQ(PIT2_IRQn);
    NVIC_EnableIRQ(PIT2_IRQn);
}

/* ---------- I2C bit-bang + LCD (HD44780 + PCF8574) ---------- */
void i2c_start(void)
{
    LCD_GPIO->PSOR = MASK(LCD_SDA_PIN); delay_i2c();
    LCD_GPIO->PSOR = MASK(LCD_SCL_PIN); delay_i2c();
    LCD_GPIO->PCOR = MASK(LCD_SDA_PIN); delay_i2c();
    LCD_GPIO->PCOR = MASK(LCD_SCL_PIN); delay_i2c();
}

void i2c_stop(void)
{
    LCD_GPIO->PCOR = MASK(LCD_SDA_PIN); delay_i2c();
    LCD_GPIO->PSOR = MASK(LCD_SCL_PIN); delay_i2c();
    LCD_GPIO->PSOR = MASK(LCD_SDA_PIN); delay_i2c();
}

void i2c_write(uint8_t b)
{
    uint8_t i;
    for (i = 0U; i < 8U; i++) {
        if (b & 0x80U) LCD_GPIO->PSOR = MASK(LCD_SDA_PIN);
        else           LCD_GPIO->PCOR = MASK(LCD_SDA_PIN);
        b <<= 1;
        delay_i2c();
        LCD_GPIO->PSOR = MASK(LCD_SCL_PIN); delay_i2c();
        LCD_GPIO->PCOR = MASK(LCD_SCL_PIN); delay_i2c();
    }
    /* ACK clock (ACK bit ignored) */
    LCD_GPIO->PSOR = MASK(LCD_SDA_PIN); delay_i2c();
    LCD_GPIO->PSOR = MASK(LCD_SCL_PIN); delay_i2c();
    LCD_GPIO->PCOR = MASK(LCD_SCL_PIN); delay_i2c();
}

void lcd_write_nibble(uint8_t nib, uint8_t rs)
{
    uint8_t b = (uint8_t)((nib & 0xF0U) | (rs ? 0x01U : 0x00U) | (g_bl ? 0x08U : 0x00U));
    i2c_start();
    i2c_write((uint8_t)(LCD_ADDR << 1));
    i2c_write((uint8_t)(b | 0x04U));        /* E high */
    i2c_write((uint8_t)(b & 0xFBU));        /* E low  */
    i2c_stop();
}

void lcd_write_byte(uint8_t b, uint8_t rs)
{
    lcd_write_nibble((uint8_t)(b & 0xF0U), rs);
    lcd_write_nibble((uint8_t)(b << 4), rs);
}

void lcd_cmd(uint8_t c) { lcd_write_byte(c, 0U); }

void lcd_init(void)
{
    delay_busy_ms(50);
    lcd_write_nibble(0x30U, 0U); delay_busy_ms(5);
    lcd_write_nibble(0x30U, 0U); delay_busy_ms(1);
    lcd_write_nibble(0x30U, 0U); delay_busy_ms(1);
    lcd_write_nibble(0x20U, 0U); delay_busy_ms(1);   /* 4-bit mode */
    lcd_cmd(0x28U); delay_busy_ms(2);                /* 2 lines, 5x8 */
    lcd_cmd(0x0CU); delay_busy_ms(2);                /* display on */
    lcd_cmd(0x06U); delay_busy_ms(2);                /* entry inc */
    lcd_cmd(0x01U); delay_busy_ms(3);                /* clear */
}

void lcd_put2(char *p, uint8_t v)
{
    p[0] = (char)('0' + (v / 10U));
    p[1] = (char)('0' + (v % 10U));
}

void lcd_copy(char *dst, const char *src)
{
    uint8_t i;
    for (i = 0U; (i < 16U) && (src[i] != '\0'); i++) dst[i] = src[i];
}

void lcd_line(uint8_t row, const char *s)
{
    uint8_t i;
    lcd_cmd((uint8_t)(row == 0U ? 0x80U : 0xC0U));
    for (i = 0U; i < 16U; i++) {
        lcd_write_byte((uint8_t)((s[i] != '\0') ? s[i] : ' '), 1U);
    }
}

void lcd_refresh(void)
{
    char l1[17], l2[17];
    uint8_t i, h, m, s, st, tm, sg, mo;

    __disable_irq();                                  /* consistent snapshot */
    h = g_tmr_h; m = g_tmr_m; s = g_tmr_s; st = g_tmr_state;
    tm = g_tmode; sg = g_mode; mo = g_mel_on;
    __enable_irq();

    for (i = 0U; i < 16U; i++) { l1[i] = ' '; l2[i] = ' '; }
    l1[16] = '\0'; l2[16] = '\0';

    if (tm) {
        lcd_copy(l1, "TIME 00:00:00   ");
        lcd_put2(&l1[5],  h);
        l1[7] = ':';
        lcd_put2(&l1[8],  m);
        l1[10] = ':';
        lcd_put2(&l1[11], s);
        if      (st == T_EDIT)  lcd_copy(l2, "EDIT S1h S2m S3s");
        else if (st == T_COUNT) lcd_copy(l2, "COUNTING DOWN...");
        else                    lcd_copy(l2, "!! TIME UP !!   ");
    } else if (sg) {
        lcd_copy(l1, "7-SEG EDIT MODE ");
        lcd_copy(l2, "EXIT: HOLD S1   ");
    } else {
        lcd_copy(l1, "EL334 LCD TIMER ");
        lcd_copy(l2, mo ? "NORMAL  MUS:ON  " : "NORMAL  MUS:OFF ");
    }

    lcd_line(0U, l1);
    lcd_line(1U, l2);
}

/* ---------- timer mode actions ---------- */
void timer_enter(void)
{
    uint8_t b;
    melody_stop();
    g_tmode     = 1U;
    g_tmr_state = T_EDIT;
    g_s1_pend   = 0U;                    /* make sure the old button logic is idle */
    g_s2_pend   = 0U;
    g_flag     &= (uint8_t)~7U;
    for (b = 0U; b < 3U; b++) {
        tb_stable[b] = (uint8_t)(((PTB->PDIR & TB_PIN[b]) == 0U) ? 1U : 0U);
        tb_cnt[b]    = 0U;
        tb_hold[b]   = 0U;
        tb_fired[b]  = 0U;
        tb_used[b]   = tb_stable[b];     /* button still held from entering: ignore until released */
    }
    g_lcd_dirty = 1U;
}

void timer_exit(void)
{
    PIT->CHANNEL[2].TCTRL = 0U;          /* stop countdown */
    melody_stop();
    g_tmode     = 0U;
    g_tmr_state = T_EDIT;
    g_lock      = 1U;                    /* ignore S1/S2 until released (existing lock) */
    g_lock_rel  = 0U;
    g_flag     &= (uint8_t)~7U;
    g_lcd_dirty = 1U;
}

void timer_start(void)
{
    if (g_tmr_state != T_EDIT) return;
    if ((g_tmr_h | g_tmr_m | g_tmr_s) == 0U) return;   /* nothing set */

    g_tmr_state = T_COUNT;
    PIT->CHANNEL[2].TCTRL = 0U;                        /* first tick comes exactly 1 s after start */
    PIT->CHANNEL[2].LDVAL = g_bus_hz - 1U;
    PIT->CHANNEL[2].TFLG  = PIT_TFLG_TIF_MASK;
    PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
    g_lcd_dirty = 1U;
}

/* ---------- timer mode buttons (b: 0 = S1, 1 = S2, 2 = S3) ---------- */
static void tb_press(uint8_t b)          /* debounced press */
{
    if ((b == 2U) && (g_tmr_state == T_ALARM)) {        /* S3 in ALARM: silence + reset */
        melody_stop();
        g_tmr_h = 0U; g_tmr_m = 0U; g_tmr_s = 0U;
        g_tmr_state = T_EDIT;
        tb_used[2] = 1U;
        g_lcd_dirty = 1U;
    } else if ((b == 1U) && (g_tmr_state == T_EDIT)) {  /* S2: minute + 1 */
        g_tmr_m = (uint8_t)((g_tmr_m + 1U) % 60U);
        tb_used[1] = 1U;
        g_lcd_dirty = 1U;
    }
}

static void tb_release(uint8_t b)        /* short press finished */
{
    if (tb_used[b] || tb_fired[b]) return;
    if (g_tmr_state != T_EDIT) return;
    if (b == 0U) {                                      /* S1: hour + 1 */
        g_tmr_h = (uint8_t)((g_tmr_h + 1U) % 100U);
        g_lcd_dirty = 1U;
    } else if (b == 2U) {                               /* S3: second + 1 */
        g_tmr_s = (uint8_t)((g_tmr_s + 1U) % 60U);
        g_lcd_dirty = 1U;
    }
}

static void tb_hold_check(uint8_t b)     /* called every ms while the button is held */
{
    if (tb_used[b] || tb_fired[b] || (tb_hold[b] < LONG_PRESS_MS)) return;
    if (b == 0U) {                                      /* hold S1: exit timer mode */
        tb_fired[0] = 1U;
        timer_exit();
    } else if ((b == 2U) && (g_tmr_state == T_EDIT)) {  /* hold S3: start countdown */
        tb_fired[2] = 1U;
        timer_start();
    }
}

/* Called every 1 ms from tick_1ms(), before process_buttons() */
void timer_poll(void)
{
    uint8_t b;
    uint8_t sig = (uint8_t)((g_tmode ? 4U : 0U) | (g_mode ? 2U : 0U) | (g_mel_on ? 1U : 0U));

    if (sig != tb_last_sig) { tb_last_sig = sig; g_lcd_dirty = 1U; }   /* keep LCD in sync */

    if (!g_tmode) {
        /* NORMAL mode: hold S2 >= 1 s enters timer mode (S2 short press still = 7-seg mode) */
        if ((g_mode != 0U) || g_lock) { tb_s2_armed = 0U; tb_s2_hold = 0U; return; }
        if ((PTB->PDIR & MASK(SW2_PIN)) != 0U) { tb_s2_armed = 1U; tb_s2_hold = 0U; return; }
        if (!tb_s2_armed) return;        /* S2 was already down when normal mode began */
        tb_s2_hold++;
        if (tb_s2_hold >= LONG_PRESS_MS) { tb_s2_hold = 0U; tb_s2_armed = 0U; timer_enter(); }
        return;
    }

    /* TIMER mode owns the buttons: hide the edges from process_buttons() */
    g_flag &= (uint8_t)~7U;

    for (b = 0U; b < 3U; b++) {
        uint8_t raw = (uint8_t)(((PTB->PDIR & TB_PIN[b]) == 0U) ? 1U : 0U);

        if (raw != tb_stable[b]) {
            tb_cnt[b]++;
            if (tb_cnt[b] >= TB_DEBOUNCE_MS) {
                tb_cnt[b]    = 0U;
                tb_stable[b] = raw;
                if (raw) { tb_hold[b] = 0U; tb_used[b] = 0U; tb_fired[b] = 0U; tb_press(b); }
                else     { tb_release(b); }
            }
        } else {
            tb_cnt[b] = 0U;
            if (tb_stable[b]) {
                if (tb_hold[b] < 0xFFFFU) tb_hold[b]++;
                tb_hold_check(b);
                if (!g_tmode) return;    /* timer_exit() was called */
            }
        }
    }
}

/* 1-second interrupt: drives the LCD timer countdown (PIT2 runs only while counting) */
void PIT2_IRQHandler(void)
{
    if (PIT->CHANNEL[2].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[2].TFLG = PIT_TFLG_TIF_MASK;

        if (g_tmode && (g_tmr_state == T_COUNT)) {
            if (g_tmr_s > 0U) {
                g_tmr_s--;
            } else if (g_tmr_m > 0U) {
                g_tmr_m--; g_tmr_s = 59U;
            } else if (g_tmr_h > 0U) {
                g_tmr_h--; g_tmr_m = 59U; g_tmr_s = 59U;
            }

            if ((g_tmr_h == 0U) && (g_tmr_m == 0U) && (g_tmr_s == 0U)) {
                g_tmr_state = T_ALARM;
                PIT->CHANNEL[2].TCTRL = 0U;     /* stop counting */
                melody_start();                 /* time up -> melody ON */
            }
            g_lcd_dirty = 1U;
        }
    }
}

int main(void)
{
    init_gpio();
    init_lcd_gpio();                     /* added */
    lcd_init();                          /* added */
    init_buttons_irq();
    init_pit(get_bus_clock_hz());
    init_pit2();                         /* added */

    while (1) {
        if (g_lcd_dirty) {               /* added: refresh LCD outside the interrupts */
            g_lcd_dirty = 0U;
            lcd_refresh();
        }
        __WFI();
    }
}