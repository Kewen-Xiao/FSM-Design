#include "stm32l476xx.h"
#include <stdint.h>

/* =========================================================
 * STM32L476RG Nucleo — Pedestrian Crossing Controller
 * Register-level C, no HAL
 *
 * Inputs (active-low, internal pull-up):
 *   PA1  BUT1  pedestrian request button 1
 *   PA4  BUT2  pedestrian request button 2
 *   PC6  SYS   system ON/OFF toggle (press to toggle)
 *   PC8  SEL0  crossing-time select bit 0 (LSB)
 *   PC9  SEL1  crossing-time select bit 1 (MSB)
 *
 * Outputs:
 *   PB10  car GREEN     PB4  car AMBER    PB5  car RED
 *   PB6   ped GREEN     PC7  ped RED
 *   PC10  buzzer  (LOW = ON, active-low MH-FMD module)
 *   PA2   USART2 TX via ST-Link VCP, 9600 baud 8N1
 *   PB8   I2C1 SCL — SSD1306 OLED display
 *   PB9   I2C1 SDA — SSD1306 OLED display
 *
 * OLED display layout (128x64 SSD1306):
 *   Line 1 (large): state name  e.g. "IDLE" / "CROSSING"
 *   Line 2 (large): countdown   e.g. "  4.7s" (S1/S2/S3)
 *                                or  "PRESS BTN" (S0)
 *   Line 3 (small): system status "SYS: ON " / "SYS: OFF"
 *
 * Crossing time (SEL1:SEL0):
 *   00 = 3 s    01 = 5 s    10 = 7 s    11 = 10 s
 * ========================================================= */

#define DEBOUNCE_MS          30U
#define AMBER_TIME_MS      1000U
#define CROSS_TIME_00_MS   3000U
#define CROSS_TIME_01_MS   5000U
#define CROSS_TIME_10_MS   7000U
#define CROSS_TIME_11_MS  10000U

/* SSD1306 I2C address (7-bit = 0x3C, write byte = 0x78) */
#define OLED_ADDR            0x78U
/* SSD1306 control bytes */
#define OLED_CMD             0x00U  /* next byte is a command */
#define OLED_DATA            0x40U  /* next byte(s) are data  */

/* =========================================================
 * SysTick — 1 ms timebase
 * ========================================================= */
volatile uint32_t g_ms_ticks = 0U;
void SysTick_Handler(void) { g_ms_ticks++; }
static inline uint32_t millis(void) { return g_ms_ticks; }

/* =========================================================
 * FSM types
 * ========================================================= */
typedef enum
{
    STATE_IDLE = 0,
    STATE_AMBER_TO_STOP,
    STATE_CROSSING,
    STATE_AMBER_TO_GO
} traffic_state_t;

typedef struct
{
    uint8_t  stable;
    uint8_t  last_raw;
    uint32_t changed_at;
} debounce_t;

/* =========================================================
 * Global FSM state
 * ========================================================= */
static traffic_state_t g_state           = STATE_IDLE;
static uint32_t        g_state_enter_ms  = 0U;
static uint32_t        g_cross_time_ms   = 0U;
static uint8_t         g_request_pending = 0U;
static uint8_t         g_system_enabled  = 0U;

static debounce_t g_db_btn1, g_db_btn2, g_db_sys, g_db_sel0, g_db_sel1;
static uint8_t    g_prev_btn1 = 0U, g_prev_btn2 = 0U, g_prev_sys = 0U;

/* =========================================================
 * GPIO helpers
 * ========================================================= */
static void gpio_output_init(GPIO_TypeDef *port, uint32_t pin)
{
    port->MODER  &= ~(3UL << (pin * 2U));
    port->MODER  |=  (1UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR &= ~(3UL << (pin * 2U));
    port->PUPDR  &= ~(3UL << (pin * 2U));
}

static void gpio_input_pullup_init(GPIO_TypeDef *port, uint32_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
    port->PUPDR |=  (1UL << (pin * 2U));
}

static inline void gpio_write(GPIO_TypeDef *port, uint32_t pin, uint8_t val)
{
    if (val) port->BSRR = (1UL << pin);
    else     port->BSRR = (1UL << (pin + 16U));
}

static inline uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (uint8_t)((port->IDR >> pin) & 0x1U);
}

/* =========================================================
 * USART2 — ST-Link VCP (PA2 = TX, AF7, 9600 baud)
 * ========================================================= */
static void uart_init(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    (void)RCC->APB1ENR1;
    GPIOA->MODER  &= ~(3UL << (2U * 2U));
    GPIOA->MODER  |=  (2UL << (2U * 2U));
    GPIOA->OSPEEDR |= (3UL << (2U * 2U));
    GPIOA->PUPDR  &= ~(3UL << (2U * 2U));
    GPIOA->AFR[0] &= ~(0xFUL << (2U * 4U));
    GPIOA->AFR[0] |=  (7UL   << (2U * 4U));
    SystemCoreClockUpdate();
    USART2->CR1 = 0U;
    USART2->BRR = (uint32_t)(SystemCoreClock / 9600U);
    USART2->CR2 = 0U; USART2->CR3 = 0U;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
    for (volatile uint32_t i = 1000U; i; i--) { __NOP(); }
}

static void uart_send_byte(uint8_t c)
{
    while (!(USART2->ISR & USART_ISR_TXE)) { }
    USART2->TDR = (uint32_t)c;
}

static void uart_send_state(traffic_state_t s)
{
    uart_send_byte((uint8_t)('0' + (uint8_t)s));
    uart_send_byte('\r'); uart_send_byte('\n');
}

/* =========================================================
 * I2C1 — hardware peripheral driver (PB8=SCL, PB9=SDA, AF4)
 *
 * Uses I2C1 in fast mode (400 kHz) with the STM32L476 timing
 * register value for 80 MHz PCLK1.
 * TIMINGR = 0x00D00E28 gives 400 kHz with 80 MHz source.
 * ========================================================= */
static void i2c_init(void)
{
    /* Enable GPIOB and I2C1 clocks */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
    (void)RCC->APB1ENR1;

    /* PB8 = I2C1_SCL, AF4, open-drain, no pull (OLED module has pull-ups) */
    GPIOB->MODER  &= ~(3UL << (8U * 2U));
    GPIOB->MODER  |=  (2UL << (8U * 2U));   /* alternate function */
    GPIOB->OTYPER |=  (1UL << 8U);           /* open-drain */
    GPIOB->OSPEEDR |= (3UL << (8U * 2U));   /* high speed */
    GPIOB->PUPDR  &= ~(3UL << (8U * 2U));   /* no pull */
    GPIOB->AFR[1] &= ~(0xFUL << ((8U-8U) * 4U));
    GPIOB->AFR[1] |=  (4UL   << ((8U-8U) * 4U)); /* AF4 = I2C1 */

    /* PB9 = I2C1_SDA, AF4, open-drain, no pull */
    GPIOB->MODER  &= ~(3UL << (9U * 2U));
    GPIOB->MODER  |=  (2UL << (9U * 2U));
    GPIOB->OTYPER |=  (1UL << 9U);
    GPIOB->OSPEEDR |= (3UL << (9U * 2U));
    GPIOB->PUPDR  &= ~(3UL << (9U * 2U));
    GPIOB->AFR[1] &= ~(0xFUL << ((9U-8U) * 4U));
    GPIOB->AFR[1] |=  (4UL   << ((9U-8U) * 4U));

    /* Configure I2C1 */
    I2C1->CR1     = 0U;                /* disable while configuring */
    I2C1->TIMINGR = 0x00D00E28U;       /* 400 kHz @ 80 MHz PCLK1  */
    I2C1->CR2     = 0U;
    I2C1->CR1     = I2C_CR1_PE;        /* enable peripheral */
}

/* Send one byte over I2C to the given 8-bit address (already shifted).
   Returns 0 on success, 1 on NACK or timeout. */
static uint8_t i2c_write(uint8_t addr8, const uint8_t *buf, uint8_t len)
{
    uint32_t timeout;

    /* Set slave address, byte count, write direction, autoend */
    I2C1->CR2 = ((uint32_t)(addr8 & 0xFEU) << 0U)  /* SADD[7:1] in bits [7:1] */
              | ((uint32_t)len << 16U)               /* NBYTES */
              | I2C_CR2_AUTOEND;                     /* auto STOP after last byte */

    /* Generate START */
    I2C1->CR2 |= I2C_CR2_START;

    for (uint8_t i = 0U; i < len; i++)
    {
        /* Wait for TXIS (transmit register empty) or NACK */
        timeout = 100000U;
        while (!(I2C1->ISR & I2C_ISR_TXIS))
        {
            if (I2C1->ISR & I2C_ISR_NACKF) { I2C1->ICR = I2C_ICR_NACKCF; return 1U; }
            if (--timeout == 0U)            { return 1U; }
        }
        I2C1->TXDR = buf[i];
    }

    /* Wait for STOP condition (AUTOEND sets this) */
    timeout = 100000U;
    while (!(I2C1->ISR & I2C_ISR_STOPF))
    {
        if (--timeout == 0U) { return 1U; }
    }
    I2C1->ICR = I2C_ICR_STOPCF;   /* clear STOP flag */
    return 0U;
}

/* =========================================================
 * SSD1306 128×64 OLED driver
 *
 * The SSD1306 command/data format over I2C:
 *   Byte 0: control byte  0x00 = command stream
 *                         0x40 = data stream
 *   Byte 1+: command or pixel data bytes
 *
 * We use a 1-bit-per-pixel framebuffer (128 × 64 / 8 = 1024 bytes).
 * Pages: 8 pages of 8 rows each. Each data byte covers 8 vertical
 * pixels in one column. Bit 0 = topmost row of the page.
 * ========================================================= */
#define OLED_W   128U
#define OLED_H    64U
#define OLED_PAGES (OLED_H / 8U)   /* 8 pages */

/* Framebuffer: [page][column], 1 byte = 8 vertical pixels */
static uint8_t g_fb[OLED_PAGES][OLED_W];

/* -- SSD1306 initialisation sequence ---------------------- */
static void oled_init(void)
{
    static const uint8_t init_cmds[] = {
        OLED_CMD,
        0xAE,        /* display OFF                          */
        0xD5, 0x80,  /* clock divide ratio / oscillator freq */
        0xA8, 0x3F,  /* multiplex ratio = 64                 */
        0xD3, 0x00,  /* display offset = 0                   */
        0x40,        /* start line = 0                       */
        0x8D, 0x14,  /* charge pump ON                       */
        0x20, 0x00,  /* horizontal addressing mode           */
        0xA1,        /* segment remap (col 127 -> SEG0)      */
        0xC8,        /* COM scan direction: remapped         */
        0xDA, 0x12,  /* COM pins hardware config             */
        0x81, 0xCF,  /* contrast = 207                       */
        0xD9, 0xF1,  /* pre-charge period                    */
        0xDB, 0x40,  /* VCOMH deselect level                 */
        0xA4,        /* display from RAM (not all-on)        */
        0xA6,        /* normal display (not inverted)        */
        0xAF         /* display ON                           */
    };
    i2c_write(OLED_ADDR, init_cmds, sizeof(init_cmds));
}

/* -- Push framebuffer to OLED ----------------------------- */
static void oled_flush(void)
{
    /* Set column address 0..127 and page address 0..7 */
    uint8_t addr_cmd[] = {
        OLED_CMD,
        0x21, 0x00, 0x7F,   /* column start=0, end=127 */
        0x22, 0x00, 0x07    /* page   start=0, end=7   */
    };
    i2c_write(OLED_ADDR, addr_cmd, sizeof(addr_cmd));

    /* Send all 1024 framebuffer bytes in one transaction.
       First byte is the data control byte 0x40. */
    uint8_t tx[OLED_W + 1U];
    for (uint8_t page = 0U; page < OLED_PAGES; page++)
    {
        tx[0] = OLED_DATA;
        for (uint8_t col = 0U; col < OLED_W; col++)
            tx[col + 1U] = g_fb[page][col];
        i2c_write(OLED_ADDR, tx, OLED_W + 1U);
    }
}

/* -- Clear framebuffer ------------------------------------ */
static void oled_clear(void)
{
    for (uint8_t p = 0U; p < OLED_PAGES; p++)
        for (uint8_t c = 0U; c < OLED_W; c++)
            g_fb[p][c] = 0x00U;
}

/* -- Draw one pixel --------------------------------------- */
static void oled_pixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= OLED_W || y >= OLED_H) return;
    uint8_t page = y / 8U;
    uint8_t bit  = y % 8U;
    if (on) g_fb[page][x] |=  (1U << bit);
    else    g_fb[page][x] &= ~(1U << bit);
}

/* =========================================================
 * 5×7 bitmap font — ASCII 0x20 (space) to 0x5A (Z)
 * Each character is 5 columns wide, each column is 7 bits tall.
 * Bit 0 = top row, bit 6 = bottom row.
 * Only uppercase + digits + space + punctuation needed here.
 * ========================================================= */
static const uint8_t FONT5X7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20  (space) */
    {0x00,0x00,0x5F,0x00,0x00}, /* !     */
    {0x00,0x07,0x00,0x07,0x00}, /* "     */
    {0x14,0x7F,0x14,0x7F,0x14}, /* #     */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $     */
    {0x23,0x13,0x08,0x64,0x62}, /* %     */
    {0x36,0x49,0x55,0x22,0x50}, /* &     */
    {0x00,0x05,0x03,0x00,0x00}, /* '     */
    {0x00,0x1C,0x22,0x41,0x00}, /* (     */
    {0x00,0x41,0x22,0x1C,0x00}, /* )     */
    {0x14,0x08,0x3E,0x08,0x14}, /* *     */
    {0x08,0x08,0x3E,0x08,0x08}, /* +     */
    {0x00,0x50,0x30,0x00,0x00}, /* ,     */
    {0x08,0x08,0x08,0x08,0x08}, /* -     */
    {0x00,0x60,0x60,0x00,0x00}, /* .     */
    {0x20,0x10,0x08,0x04,0x02}, /* /     */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0     */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1     */
    {0x42,0x61,0x51,0x49,0x46}, /* 2     */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3     */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4     */
    {0x27,0x45,0x45,0x45,0x39}, /* 5     */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6     */
    {0x01,0x71,0x09,0x05,0x03}, /* 7     */
    {0x36,0x49,0x49,0x49,0x36}, /* 8     */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9     */
    {0x00,0x36,0x36,0x00,0x00}, /* :     */
    {0x00,0x56,0x36,0x00,0x00}, /* ;     */
    {0x08,0x14,0x22,0x41,0x00}, /* <     */
    {0x14,0x14,0x14,0x14,0x14}, /* =     */
    {0x00,0x41,0x22,0x14,0x08}, /* >     */
    {0x02,0x01,0x51,0x09,0x06}, /* ?     */
    {0x32,0x49,0x79,0x41,0x3E}, /* @     */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A     */
    {0x7F,0x49,0x49,0x49,0x36}, /* B     */
    {0x3E,0x41,0x41,0x41,0x22}, /* C     */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D     */
    {0x7F,0x49,0x49,0x49,0x41}, /* E     */
    {0x7F,0x09,0x09,0x09,0x01}, /* F     */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G     */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H     */
    {0x00,0x41,0x7F,0x41,0x00}, /* I     */
    {0x20,0x40,0x41,0x3F,0x01}, /* J     */
    {0x7F,0x08,0x14,0x22,0x41}, /* K     */
    {0x7F,0x40,0x40,0x40,0x40}, /* L     */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M     */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N     */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O     */
    {0x7F,0x09,0x09,0x09,0x06}, /* P     */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q     */
    {0x7F,0x09,0x19,0x29,0x46}, /* R     */
    {0x46,0x49,0x49,0x49,0x31}, /* S     */
    {0x01,0x01,0x7F,0x01,0x01}, /* T     */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U     */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V     */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W     */
    {0x63,0x14,0x08,0x14,0x63}, /* X     */
    {0x07,0x08,0x70,0x08,0x07}, /* Y     */
    {0x61,0x51,0x49,0x45,0x43}, /* Z     */
};

/* -- Draw a single 5×7 character at pixel position (x, y_page)
      scale=1 draws 5×7 pixels, scale=2 draws 10×14 pixels etc. */
static void oled_char(uint8_t x, uint8_t y_px, char c, uint8_t scale)
{
    if (c < 0x20 || c > 0x5A) c = 0x20;   /* map unknown to space */
    const uint8_t *col_data = FONT5X7[(uint8_t)(c - 0x20)];

    for (uint8_t col = 0U; col < 5U; col++)
    {
        uint8_t bits = col_data[col];
        for (uint8_t row = 0U; row < 7U; row++)
        {
            uint8_t on = (bits >> row) & 0x01U;
            for (uint8_t sy = 0U; sy < scale; sy++)
                for (uint8_t sx = 0U; sx < scale; sx++)
                    oled_pixel(x + col * scale + sx,
                               y_px + row * scale + sy, on);
        }
    }
}

/* -- Draw a null-terminated string.
      scale=1 ? 6px wide per char (5+1 gap), scale=2 ? 12px etc. */
static void oled_str(uint8_t x, uint8_t y_px, const char *s, uint8_t scale)
{
    while (*s)
    {
        oled_char(x, y_px, *s, scale);
        x += (uint8_t)(6U * scale);
        s++;
    }
}

/* -- Draw a horizontal divider line ----------------------- */
static void oled_hline(uint8_t y_px)
{
    for (uint8_t x = 0U; x < OLED_W; x++)
        oled_pixel(x, y_px, 1U);
}

/* =========================================================
 * Simple integer-to-string helpers (no printf needed)
 * ========================================================= */

/* Write a 1-digit number (0-9) at position in buf */
static void itoa1(char *buf, uint8_t v)
{
    buf[0] = (char)('0' + (v % 10U));
    buf[1] = '\0';
}

/* Write a 2-digit zero-padded number (0-99) */
static void itoa2(char *buf, uint8_t v)
{
    buf[0] = (char)('0' + (v / 10U));
    buf[1] = (char)('0' + (v % 10U));
    buf[2] = '\0';
}

/* =========================================================
 * OLED display update
 *
 * Called from the main loop. Rebuilds the framebuffer from
 * the current FSM state and elapsed time, then flushes.
 *
 * Layout (128×64):
 *   y=0..1   thin top bar (state name background)
 *   y=2..17  large state label (scale=2, 14 px tall)
 *   y=18     divider line
 *   y=20..47 large countdown or message (scale=4, 28 px tall)
 *   y=48     divider line
 *   y=50..56 small system status line (scale=1, 7 px tall)
 * ========================================================= */
static void oled_update(void)
{
    oled_clear();

    /* -- Row 1: state name (scale 2 = 14 px tall, y=2) -- */
    const char *state_str;
    if      (!g_system_enabled)           state_str = "SYSTEM OFF";
    else if (g_state == STATE_IDLE)        state_str = "IDLE";
    else if (g_state == STATE_AMBER_TO_STOP) state_str = "AMBER";
    else if (g_state == STATE_CROSSING)    state_str = "CROSSING";
    else                                   state_str = "AMBER";
    oled_str(0U, 2U, state_str, 2U);

    /* -- Divider -- */
    oled_hline(18U);

    /* -- Row 2: countdown or message (scale 4 = 28 px, y=20) -- */
    if (!g_system_enabled)
    {
        /* System off — show "OFF" in large text */
        oled_str(14U, 20U, "OFF", 4U);
    }
    else if (g_state == STATE_IDLE)
    {
        /* Idle — prompt the user */
        oled_str(4U, 20U, "PRESS", 2U);
        oled_str(10U, 36U, "BTN", 2U);
    }
    else
    {
        /* S1 / S2 / S3 — show remaining time as  X.Xs  */
        uint32_t elapsed_ms  = (uint32_t)(millis() - g_state_enter_ms);
        uint32_t total_ms    = (g_state == STATE_CROSSING)
                               ? g_cross_time_ms : AMBER_TIME_MS;
        uint32_t remain_ms   = (elapsed_ms < total_ms)
                               ? (total_ms - elapsed_ms) : 0U;

        /* Convert to tenths of a second */
        uint32_t tenths = (remain_ms + 99U) / 100U;   /* round up */
        uint8_t  secs   = (uint8_t)(tenths / 10U);
        uint8_t  frac   = (uint8_t)(tenths % 10U);

        /* Build string "X.Xs" — up to "10.0s" (5 chars) */
        char tbuf[8];
        uint8_t i = 0U;
        if (secs >= 10U) tbuf[i++] = '1';
        tbuf[i++] = (char)('0' + (secs % 10U));
        tbuf[i++] = '.';
        tbuf[i++] = (char)('0' + frac);
        tbuf[i++] = 'S';
        tbuf[i]   = '\0';

        /* Centre the text horizontally.
           Each char at scale 3 = 18 px wide (6×3).
           Total width = strlen * 18, offset = (128 - width) / 2 */
        uint8_t str_len = i;
        uint8_t x_start = (uint8_t)((128U - (uint32_t)str_len * 18U) / 2U);
        oled_str(x_start, 20U, tbuf, 3U);
    }

    /* -- Divider -- */
    oled_hline(48U);

    /* -- Row 3: system status (scale 1, y=50) -- */
    oled_str(0U, 50U, "SYS:", 1U);
    oled_str(30U, 50U, g_system_enabled ? "ON " : "OFF", 1U);

    /* -- Row 3 right: crossing time setting -- */
    uint8_t code = (uint8_t)((g_db_sel1.stable << 1U) | g_db_sel0.stable);
    const char *dur_str = (code == 0U) ? " 3S" :
                          (code == 1U) ? " 5S" :
                          (code == 2U) ? " 7S" : "10S";
    oled_str(72U, 50U, "DUR:", 1U);
    oled_str(102U, 50U, dur_str, 1U);

    oled_flush();
}

/* =========================================================
 * Active-low input helpers
 * ========================================================= */
static inline uint8_t ped_btn1_raw(void){ return gpio_read(GPIOA,1U)==0U?1U:0U; }
static inline uint8_t ped_btn2_raw(void){ return gpio_read(GPIOA,4U)==0U?1U:0U; }
static inline uint8_t sys_btn_raw(void) { return gpio_read(GPIOC,6U)==0U?1U:0U; }
static inline uint8_t sel0_raw(void)    { return gpio_read(GPIOC,8U)==0U?1U:0U; }
static inline uint8_t sel1_raw(void)    { return gpio_read(GPIOC,9U)==0U?1U:0U; }
static inline void buzzer_write(uint8_t on){ gpio_write(GPIOC,10U,on?0U:1U); }

/* =========================================================
 * Debounce
 * ========================================================= */
static void debounce_init(debounce_t *db, uint8_t initial_raw)
{
    db->stable = db->last_raw = initial_raw;
    db->changed_at = millis();
}

static uint8_t debounce_update(debounce_t *db, uint8_t raw)
{
    if (raw != db->last_raw) { db->last_raw = raw; db->changed_at = millis(); }
    if ((uint32_t)(millis() - db->changed_at) >= DEBOUNCE_MS) db->stable = raw;
    return db->stable;
}

/* =========================================================
 * Timing helpers
 * ========================================================= */
static uint32_t selected_cross_time_ms(void)
{
    uint8_t code = (uint8_t)((g_db_sel1.stable << 1U) | g_db_sel0.stable);
    switch (code)
    {
        case 0U: return CROSS_TIME_00_MS;
        case 1U: return CROSS_TIME_01_MS;
        case 2U: return CROSS_TIME_10_MS;
        case 3U: return CROSS_TIME_11_MS;
        default: return CROSS_TIME_01_MS;
    }
}

static inline uint8_t elapsed(uint32_t start_ms, uint32_t duration_ms)
{
    return ((uint32_t)(millis() - start_ms) >= duration_ms) ? 1U : 0U;
}

/* =========================================================
 * Output control
 * ========================================================= */
static void set_outputs(uint8_t car_green, uint8_t car_amber,
                        uint8_t car_red,   uint8_t ped_green,
                        uint8_t ped_red,   uint8_t buzzer_on)
{
    gpio_write(GPIOB, 10U, car_green);
    gpio_write(GPIOB,  4U, car_amber);
    gpio_write(GPIOB,  5U, car_red);
    gpio_write(GPIOB,  6U, ped_green);
    gpio_write(GPIOC,  7U, ped_red);
    buzzer_write(buzzer_on);
}

static void apply_outputs(void)
{
    if (!g_system_enabled) { set_outputs(0,0,0,0,0,0); return; }
    switch (g_state)
    {
        case STATE_IDLE:          set_outputs(1,0,0,0,1,0); break;
        case STATE_AMBER_TO_STOP: set_outputs(0,1,0,0,1,0); break;
        case STATE_CROSSING:      set_outputs(0,0,1,1,0,1); break;
        case STATE_AMBER_TO_GO:   set_outputs(0,1,0,0,1,0); break;
        default:                  set_outputs(1,0,0,0,1,0); break;
    }
}

/* =========================================================
 * Input polling
 * ========================================================= */
static void poll_inputs(void)
{
    uint8_t btn1_now = debounce_update(&g_db_btn1, ped_btn1_raw());
    uint8_t btn2_now = debounce_update(&g_db_btn2, ped_btn2_raw());
    uint8_t sys_now  = debounce_update(&g_db_sys,  sys_btn_raw());
    (void)debounce_update(&g_db_sel0, sel0_raw());
    (void)debounce_update(&g_db_sel1, sel1_raw());

    if (g_system_enabled)
    {
        if ((btn1_now != 0U) && (g_prev_btn1 == 0U)) g_request_pending = 1U;
        if ((btn2_now != 0U) && (g_prev_btn2 == 0U)) g_request_pending = 1U;
    }

    if ((sys_now != 0U) && (g_prev_sys == 0U))
    {
        g_system_enabled  = !g_system_enabled;
        g_state           = STATE_IDLE;
        g_state_enter_ms  = millis();
        g_request_pending = 0U;
        if (g_system_enabled)
            { uart_send_byte('O'); uart_send_byte('N'); }
        else
            { uart_send_byte('O'); uart_send_byte('F'); uart_send_byte('F'); }
        uart_send_byte('\r'); uart_send_byte('\n');
    }

    g_prev_btn1 = btn1_now;
    g_prev_btn2 = btn2_now;
    g_prev_sys  = sys_now;
}

/* =========================================================
 * FSM
 * ========================================================= */
static void enter_state(traffic_state_t next_state)
{
    g_state          = next_state;
    g_state_enter_ms = millis();
    apply_outputs();
    uart_send_state(next_state);
}

static void fsm_update(void)
{
    if (!g_system_enabled) return;

    switch (g_state)
    {
        case STATE_IDLE:
            if (g_request_pending != 0U)
            {
                g_request_pending = 0U;
                g_cross_time_ms   = selected_cross_time_ms();
                enter_state(STATE_AMBER_TO_STOP);
            }
            break;
        case STATE_AMBER_TO_STOP:
            if (elapsed(g_state_enter_ms, AMBER_TIME_MS))
                enter_state(STATE_CROSSING);
            break;
        case STATE_CROSSING:
            if (elapsed(g_state_enter_ms, g_cross_time_ms))
                enter_state(STATE_AMBER_TO_GO);
            break;
        case STATE_AMBER_TO_GO:
            if (elapsed(g_state_enter_ms, AMBER_TIME_MS))
                enter_state(STATE_IDLE);
            break;
        default:
            enter_state(STATE_IDLE);
            break;
    }
}

/* =========================================================
 * Initialisation
 * ========================================================= */
static void gpio_init_all(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN |
                    RCC_AHB2ENR_GPIOBEN |
                    RCC_AHB2ENR_GPIOCEN;
    (void)RCC->AHB2ENR;

    gpio_input_pullup_init(GPIOA, 1U);
    gpio_input_pullup_init(GPIOA, 4U);
    gpio_input_pullup_init(GPIOC, 6U);
    gpio_input_pullup_init(GPIOC, 8U);
    gpio_input_pullup_init(GPIOC, 9U);

    gpio_output_init(GPIOB, 10U);
    gpio_output_init(GPIOB,  4U);
    gpio_output_init(GPIOB,  5U);
    gpio_output_init(GPIOB,  6U);
    gpio_output_init(GPIOC,  7U);
    gpio_output_init(GPIOC, 10U);

    set_outputs(0,0,0,0,0,0);
    gpio_write(GPIOC, 10U, 1U);
}

static void systick_init_1ms(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000U);
}

static void inputs_init(void)
{
    debounce_init(&g_db_btn1, ped_btn1_raw());
    debounce_init(&g_db_btn2, ped_btn2_raw());
    debounce_init(&g_db_sys,  sys_btn_raw());
    debounce_init(&g_db_sel0, sel0_raw());
    debounce_init(&g_db_sel1, sel1_raw());
    g_prev_btn1 = g_db_btn1.stable;
    g_prev_btn2 = g_db_btn2.stable;
    g_prev_sys  = g_db_sys.stable;
}

/* =========================================================
 * Main
 * ========================================================= */
int main(void)
{
    gpio_init_all();
    systick_init_1ms();
    uart_init();
    i2c_init();
    inputs_init();

    /* Boot message over UART */
    uart_send_byte('O'); uart_send_byte('F'); uart_send_byte('F');
    uart_send_byte('\r'); uart_send_byte('\n');

    /* Initialise and clear OLED */
    oled_init();
    oled_clear();
    oled_flush();

    /* OLED update throttle — refresh at ~10 Hz (every 100 ms) */
    uint32_t oled_last_ms = 0U;

    while (1)
    {
        poll_inputs();
        fsm_update();
        apply_outputs();

        /* Update OLED at 10 Hz — fast enough to show smooth countdown,
           slow enough not to overwhelm the I2C bus in the main loop */
        if ((uint32_t)(millis() - oled_last_ms) >= 100U)
        {
            oled_last_ms = millis();
            oled_update();
        }
    }
}
