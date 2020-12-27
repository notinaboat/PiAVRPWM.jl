//==============================================================================
// Copright Sam O'Connor 2009
//
// References:
//
// [MMC]: SanDisk MultiMediaCard and Reduced-Size MultiMediaCard
//        Product Manual, Version 1.3, Document No. 80-36-00320
//        April 2005 (MMCv1.3.pdf)
//
// [M8]:  ATmega8 8-bit AVR Microcontroller data sheet.
//        2486V–AVR–05/09 (doc2486.pdf)
//
// [M32]: ATmega32 8-bit AVR Microcontroller data sheet.
//        2503O–AVR–07/09 (doc2503.pdf)
//
// [FNV]: Fowler / Noll / Vo (FNV) Hash
//        http://www.isthe.com/chongo/tech/comp/fnv/index.html
//        http://www.isthe.com/chongo/src/fnv/hash_32.c
//        http://en.wikipedia.org/wiki/Fowler-Noll-Vo_hash_function
//
//==============================================================================


#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>


void lcd_clear(void);
void lcd_line2(void);
void lcd_home(void);
void lcd_dec(uint32_t v, uint32_t pad);
void lcd_hex(uint8_t v);
void lcd_text(const char* text);

uint16_t rtc16(void);

static void monitor_tx(uint32_t v);



//==============================================================================
// GPIO
//==============================================================================

#ifndef DDRA
#define DDRA DDRB
#define PORTA PORTB
#define PINA PINB
#endif


static const typeof(&DDRB) dirport[] = {
    [1] =  &DDRA,
    [2] =  &DDRB,
    [3] =  &DDRC,
    [4] =  &DDRD
};


static const typeof(&PORTB) outport[] = {
    [1] = &PORTA,
    [2] = &PORTB,
    [3] = &PORTC,
    [4] = &PORTD
};


static const typeof(&PINB) inport[] = {
    [1] =  &PINA,
    [2] =  &PINB,
    [3] =  &PINC,
    [4] =  &PIND
};


// Encode port & pin to a single 8 bit value...
#define pin_code(port,pin)              \
  ((&(port) == &PORTA ? (1 << 4) :      \
    &(port) == &PORTB ? (2 << 4) :      \
    &(port) == &PORTC ? (3 << 4) :      \
    &(port) == &PORTD ? (4 << 4) :      \
    1/0) | (pin))


// Extract pin number from encoded pin value...
#define pin_number(pin) ((pin) & 0x0F)


// Extract output port from encoded pin value...
#define pin_outport(pin) (*(outport[(pin) >> 4]))


// Extract input port from encoded pin value...
#define pin_inport(pin) (*(inport[(pin) >> 4]))


// Extract direction port from encoded pin value...
#define pin_dirport(pin) (*(dirport[(pin) >> 4]))


// Extract pin mask from encoded pin value...
#define pin_mask(pin) (1 << ((pin) & 0x0F))


// Pin Direction...
#define pin_enable_output(pin) pin_dirport(pin) |= pin_mask(pin)
#define pin_enable_input(pin) pin_dirport(pin) &= (uint8_t)~pin_mask(pin)


// Read pin...
#define pin_is_high(pin) (pin_inport(pin) & pin_mask(pin))
#define pin_is_low(pin) (!pin_is_high(pin))


// Write pin...
#define pin_set_high(pin) ({pin_outport(pin) |= pin_mask(pin);})
#define pin_set_low(pin) ({pin_outport(pin) &= (uint8_t)~pin_mask(pin);})
#define pin_set(pin,high) ({if(high) pin_set_high(pin); else pin_set_low(pin);})


// Input mode pullups...
#define pin_enable_pullup(pin) pin_set_high(pin)
#define pin_disable_pullup(pin) pin_set_low(pin)


// Pulse pin...
#define pin_pulse_ms(pin,ms,count) \
({                                                                             \
    int _i;                                                                    \
    for (_i = 0 ; _i < (count) ; _i++) {                                       \
        pin_set_high(pin);                                                     \
        _delay_ms(ms);                                                         \
        pin_set_low(pin);                                                      \
        _delay_ms(ms);                                                         \
    }                                                                          \
})



//==============================================================================
// Debug
//==============================================================================


static void error(uint16_t code) __attribute__ ((noreturn));
static void error(uint16_t code)
{
    for(;;) {
#ifdef LCD_RW
        lcd_clear();
        lcd_text("     ERR:");
        lcd_dec(code, 1);
#endif
        _delay_ms(1000);
#ifdef MONITOR
        monitor_tx(0xEEEE0000UL | code);
#endif
    }
}

#define get_pc() \
({ \
    uint16_t pc; \
    asm volatile("rcall next%= \n" \
                 "next%=:      \n" \
                 "pop %B0      \n" \
                 "pop %A0      \n" \
                 : "=w" (pc)); \
    pc; \
})

#define assert(test) \
({ \
    if ((__builtin_expect(!!(!(test)), 0))) { \
        error(get_pc()); \
    } \
})

#define println(m) monitor_tx(get_pc())

#define printn(m, d) monitor_tx( (uint32_t)(0x8000 | get_pc()) << 16 \
                               | (uint16_t)(d))



//==============================================================================
// Events
//==============================================================================


#define EVENT_COUNT 16


volatile uint16_t _event_flags;

struct event {
    uint8_t count;
};


#define EVENT_DECLARE(name) \
    struct event name __attribute__ ((section (".bss.events")))

EVENT_DECLARE(EVENT_MIN);
EVENT_DECLARE(EVENT_IDLE);

#define EVENT_FLAG(event) ((uint16_t)(event - &EVENT_MIN))


void event_signal(struct event* event)
{
    uint16_t flag = EVENT_FLAG(event);
    cli();
    event->count++;
    _event_flags |= flag;
    sei();
}


void isr_event_signal(struct event* event)
{
    uint16_t flag = EVENT_FLAG(event);
    event->count++;
    _event_flags |= flag;
}


uint16_t event_flags()
{
    cli();
    uint16_t flags = _event_flags;
    _event_flags = 0;
    sei();
    return flags;
}


void event_sleep()
{
    for(;;) {
        // Attomic event check and sleep...                     [avr/sleep.h:81]
        cli();
        if (!_event_flags) {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        } else {
            sei();
            break;
        }
    }
}


uint16_t event_next()
{
    //static uint8_t idle = 0;

    // Sleep if idle...
//    if (idle) {
        event_sleep();
//        idle = 0;
//    }

    // Get queued events...
    uint16_t flags = event_flags();

    // If there are no events signal idle...
//    if (!flags) {
//        flags = EVENT_FLAG(&EVENT_IDLE);
//        idle = 1;
//    }

    return flags;
}


uint8_t event_count_reset(struct event* event)
{
    cli();
    uint8_t count = event->count;
    event->count = 0;
    sei();

    return count;
}


void event_init()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
}



//==============================================================================
// Event Handlers
//==============================================================================


#define EVENT_HANDLER_COUNT 6

struct {
    uint16_t event_mask;
    void(*event_handler)(uint16_t flags);
} event_handlers[EVENT_HANDLER_COUNT];


void event_handler_register(uint16_t event_mask,
                            void(*event_handler)(uint16_t flags))
{
    int i;
    for (i = 0 ; i < EVENT_HANDLER_COUNT ; i++) {
        if (!event_handlers[i].event_handler) {
            event_handlers[i].event_mask = event_mask;
            event_handlers[i].event_handler = event_handler;
            return;
        }
    }
}


#define EVENT_HANDLER(event, name) \
    void name (uint16_t flags); \
    void init_ ## name () __attribute__ ((naked)) \
                          __attribute__ ((used)) \
                          __attribute__ ((section (".init8"))); \
    void init_ ## name () { event_handler_register(EVENT_FLAG(event), name); } \
    void name (uint16_t flags)


void event_dispatch(uint16_t flags)
{
    int i;
    for (i = 0 ; i < EVENT_HANDLER_COUNT ; i++) {
        if (event_handlers[i].event_handler
        && (flags & event_handlers[i].event_mask)) {
            event_handlers[i].event_handler(flags);
        }
    }
}


void event_dispatch_next()
{
    event_dispatch(event_next());
}


void event_run(void) __attribute__ ((noreturn));
void event_run()
{
    for(;;) {
        event_dispatch_next();
    }
}


void event_delay_ms(uint16_t ms)
{
    uint16_t deadline = rtc16() + (ms * 10);
    do {
        event_dispatch_next();
    } while (rtc16() < deadline);
}



//==============================================================================
// ADC
//==============================================================================

// Values from 2486V–AVR–05/09, p206...
#define ADC_REF_VCC (1 << REFS0)
#define ADC_LEFT_ADJUST (1 << ADLAR)

// Values from 2486V–AVR–05/09, p207...
#define ADC_ENABLE (1 << ADEN)
#define ADC_START (1 << ADSC)

#define ADC_MAX 1024
#define ADC_VREF_MV 5000

uint16_t adc_sample(uint8_t pin)
{
    // See 2486V–AVR–05/09, p197.

    // Enable the ADC...
    ADCSRA = ADC_ENABLE;

    // Select reference, input pin and output format...
    ADMUX = ADC_REF_VCC
          | pin_number(pin);
//          | ADC_LEFT_ADJUST;

    // Start the conversion...
    ADCSRA |= ADC_START;

    // Wait for the conversion to complete...
    while (ADCSRA & ADC_START);

    // Fetch the result...
    uint16_t v = ADC;

    // Turn ADC off...
//    ADCSRA = 0;

    return v;
}


uint16_t adc_sample_mv(uint8_t pin)
{
    uint32_t v = adc_sample(pin);
    v *= ADC_VREF_MV;
    v /= ADC_MAX;
    return (uint16_t)v;
}



//==============================================================================
// Counter
//==============================================================================

#ifdef ENABLE_COUNTER

uint32_t counter0_total;

#define counter0_count TCNT0


ISR(TIMER0_OVF_vect)
{
    counter0_total += 256;
}


void counter0_init()
{
    TCNT0 = 0;

    // External clock source on rising edge of T0 pin...
    TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);    // [M32, Table-42, p82]

    // Enable overflow interrupt...
    TIMSK |= (1 << TOIE0);
}

#endif // ENABLE_COUNTER


//==============================================================================
// Monitor Serial Data TX
//==============================================================================

#ifdef MONITOR

volatile struct __attribute__ ((packed)) {
    uint8_t byte;
    union {
        uint32_t buffer;
        uint16_t buffer16[2];
    };
} _m_buf;
volatile int8_t _m_i;


int monitor_buffer_busy(void)
{
    return (_m_i != 0);
}


void monitor_tx(uint32_t v)
{
    while(monitor_buffer_busy()) {}

    _m_buf.byte = 0;
    _m_buf.buffer = v;
    _m_i =  1  // start bit
         + 32  // data bits
         +  1; // stop bit (32N1)
}


void monitor_tx_16(uint16_t v1, uint16_t v2)
{
    while(monitor_buffer_busy()) {}

    _m_buf.byte = 0;
    _m_buf.buffer16[1] = v1;
    _m_buf.buffer16[0] = v2;
    _m_i =  1  // start bit
         + 32  // data bits
         +  1; // stop bit (32N1)
}


#endif // MONITOR



//==============================================================================
// RTC
//==============================================================================

EVENT_DECLARE(RTC_TICK);

volatile uint8_t _rtc100;
volatile uint8_t _rtc;

uint32_t rtc32()
{
    cli();
    uint32_t v = _rtc;
    sei();
    return v;
}


uint16_t rtc16()
{
    cli();
    uint16_t v = (uint16_t) _rtc;
    sei();
    return v;
}


uint32_t rtc8()
{
    return (uint8_t) _rtc;
}


void rtc_sync()
{
    uint8_t v = rtc8();
    while(v == rtc8());
}

void rtc100_sync()
{
    uint8_t v = _rtc100;
    while(v == _rtc100);
}


void rtc_delay_ms(uint16_t ms)
{
    ms *= 10;
    do {
        rtc_sync();
    } while(--ms);
}


#ifndef CUSTOM_RTC_ISR
// @16 MHz:
// 1.5 us (_m_i == 0)
// 2.3 us (_m_i > 0)
// 2.3 us (_m_i > 0)
// 3.0 us (_m_i % 8 == 0)
ISR(TIMER2_COMPA_vect,ISR_NAKED)
{
    asm volatile(
    "               cbi  0x12, 6             \n" // DEBUG LED

    "               push r24                 \n" // Save state.
    "               in   r24, __SREG__       \n"
    "               push r24                 \n"

    "               lds  r24, _rtc100        \n"
    "               dec  r24                 \n" // Decrement _rtc100.

    "               brne .sts_rtc100%=       \n" // When _rtc100 is zero...
    "                   lds  r24, _rtc       \n"
    "                   inc  r24             \n" // increment _rtc and
    "                   sts _rtc, r24        \n"
    "                   ldi  r24, 100        \n" // reset _rtc100 to 100.

    ".sts_rtc100%=: sts _rtc100, r24         \n" // Store updated _rtc100 value.

    "               lds  r24, _m_i           \n" // Decrement _m_i,
    "               dec  r24                 \n"
    "               brmi .return%=           \n" // Do nothing if _m_i < 0
    "               sts  _m_i, r24           \n"
    "               breq .tx_high%=          \n" // Send stop bit if _m_i is 0.

    "               andi r24, 0b111          \n" // Get next char.
    "               breq .next_char%=        \n"

    ".load_m_buf%=: lds  r24, _m_buf         \n" // Shift _m_buf right.
    ".do_shift%=:   lsr  r24                 \n"
    "               sts _m_buf, r24          \n"

    "               brcc .tx_low%=           \n" // Send LSB based on Cary Flag.
    ".tx_high%=:    sbi  %[tx_port], %[pin]  \n"
    "               cpse r0, r0              \n"
    ".tx_low%=:     cbi  %[tx_port], %[pin]  \n"

    ".return%=:     pop  r24                 \n" // Restore state.
    "               out __SREG__, r24        \n"
    "               pop  r24                 \n"

    "               sbi  0x12, 6             \n" // DEBUG LED

    "               reti                     \n"

    ".next_char%=:  lds  r24, _m_buf+1       \n" // Shift right by 8 bits
    "               push r24                 \n"
    "               lds  r24, _m_buf+2       \n"
    "               sts _m_buf+1, r24        \n"
    "               lds  r24, _m_buf+3       \n"
    "               sts _m_buf+2, r24        \n"
    "               lds  r24, _m_buf+4       \n"
    "               sts _m_buf+3, r24        \n"
    "               pop r24                  \n"
    "               rjmp .do_shift%=         \n"
    :
    : [tx_port] "I" (_SFR_IO_ADDR(MONITOR_PORT)),
      [pin]     "I" (MONITOR_PIN)
    :
    );
}
#endif


void rtc_init()
{
    // CTC mode, 16MHz / 8 = 2000kHz...
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS21) | (0 << CS20);

    // 2000kHz / 200 = 10kHz...
    OCR2A = 199;

    TCNT2 = 0;

    // Clear counter match bit...                              [M32, p116, p130]
    TIFR2 |= (1 << OCF2A);


    TIMSK2 |= (1 << OCIE2A);
}


//==============================================================================
// PWM
//==============================================================================

#ifdef ENABLE_PWM

#define TCCR1A_FAST_8_BIT_PWM (1 << WGM10)
#define TCCR1B_FAST_8_BIT_PWM (1 << WGM12)
#define TCCR1A_INVERTING_PWM ((1 << COM1A0) | (1 << COM1A1) \
                            | (1 << COM1B0) | (1 << COM1B1))
#define TCCR1A_NON_INVERTING_PWM ((1 << COM1A1) \
                                | (1 << COM1B1))
#define TCCR1B_PRESCALER_1 (1 << CS10)

void timer1_pwm_init()
{
    OCR1A = 0;
    OCR1B = 0;

    TCCR1A = TCCR1A_FAST_8_BIT_PWM
           | TCCR1A_NON_INVERTING_PWM;
    TCCR1B = TCCR1B_FAST_8_BIT_PWM
           | TCCR1B_PRESCALER_1;
}


void timer1_pwm_set(uint8_t pin, uint8_t v)
{
#ifdef __AVR_ATmega32__
    switch(pin) {
        case pin_code(PORTD, 5): OCR1AL = v; break;
        case pin_code(PORTD, 4): OCR1BL = v; break;
        default: assert(0);
    }
#else
    if (pin == pin_code(PORTB, 1)) {
        OCR1AL = v;
    } else {
        OCR1BL = v;
    }
#endif
}


#define TCCR2_FAST_PWM ((1 << WGM21) | (1 << WGM20))
#define TCCR2_INVERTING_PWM ((1 << COM20) | (1 << COM21))
#define TCCR2_NON_INVERTING_PWM (1 << COM21)
#define TCCR2_PRESCALER_1 (1 << CS20)
#define TCCR2_PRESCALER_8 (1 << CS21)
#define TCCR2_PRESCALER_32 ((1 << CS20) | (1 << CS21))


void timer2_pwm_init()
{
    OCR2 = 0;

    TCCR2 = TCCR2_FAST_PWM
          | TCCR2_INVERTING_PWM
          | TCCR2_PRESCALER_1;
}


void timer2_pwm_set(uint8_t v)
{
    OCR2 = v;
}

#endif // ENABLE_PWM


//==============================================================================
// Soft PWM
//==============================================================================

#ifdef ENABLE_SOFT_PWM

#define SOFT_PWM_TICK_COUNT 10
#define SOFT_PWM_COUNT_MAX 4

uint8_t soft_pwm_tick;

struct {
    uint8_t pin;
    uint8_t duty;
} soft_pwm_pins[SOFT_PWM_COUNT_MAX];


EVENT_HANDLER(&RTC_TICK, soft_pwm_update)
{
    int i;
    uint8_t pin;

    if (soft_pwm_tick == 0) {
        for (i = 0 ; i < SOFT_PWM_COUNT_MAX ; i++) {
            if ((pin = soft_pwm_pins[i].pin)) {
                pin_set_high(pin);
            }
        }
    } else {
        for (i = 0 ; i < SOFT_PWM_COUNT_MAX ; i++) {
            if (soft_pwm_pins[i].duty == soft_pwm_tick) {
                pin_set_low(soft_pwm_pins[i].pin);
            }
        }
    }

    soft_pwm_tick++;
    if (soft_pwm_tick == SOFT_PWM_TICK_COUNT) {
        soft_pwm_tick = 0;
    }
}


void soft_pwm_set(uint8_t pin, uint8_t duty)
{
    int i;
    for (i = 0 ; i < SOFT_PWM_COUNT_MAX ; i++) {
        if (soft_pwm_pins[i].pin == pin) {
            soft_pwm_pins[i].duty = duty;
            if (duty == 0) {
                pin_set_low(soft_pwm_pins[i].pin);
                soft_pwm_pins[i].pin = 0;
            }
            return;
        }
    }
    for (i = 0 ; i < SOFT_PWM_COUNT_MAX ; i++) {
        if (soft_pwm_pins[i].pin == 0) {
            soft_pwm_pins[i].pin = pin;
            soft_pwm_pins[i].duty = duty;
            return;
        }
    }
    assert(0);
}

#endif // ENABLE_SOFT_PWM


//==============================================================================
// USART
//==============================================================================

#ifdef USART_RX

#ifndef AVR_UTIL_CUSTOM_USART_ISR
EVENT_DECLARE(USART_RX);

ISR(USART_RXC_vect)
{
    isr_event_signal(&USART_RX);
}
#endif


void usart_init()
{
    pin_enable_input(USART_RX);
    pin_disable_pullup(USART_RX);

    // 9600 baud (with 16Mhz clock)...
    UBRRH = 0;
    UBRRL = 103;

    // Enable RX...
    UCSRB = (1 << RXEN) | (1 << RXCIE);

    // N81...
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}


uint8_t usart_rx_ready()
{
    return UCSRA & (1 << RXC);
}


uint8_t usart_data()
{
    return UDR;
}


uint8_t usart_rx()
{
    while (!usart_rx_ready());
    return usart_data();
}

#endif


//==============================================================================
// SPI
//==============================================================================


#ifdef SPI_SCK


void spi_init()
{
    pin_enable_output(SPI_SCK);
    pin_enable_output(SPI_MOSI);
    pin_enable_input(SPI_MISO);

    SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) |
           (1 << SPR1) | (1 << SPR0); //             f/128 [M32, Table-58, p137]

    SPSR = 0;
}


uint8_t spi_tx(uint8_t out)
{
    SPDR = out;
    while(!(SPSR & (1 << SPIF)));
    volatile uint8_t in = SPDR;
    return in;
}


uint8_t spi_rx()
{
    return spi_tx(0xFF);
}

#endif


//==============================================================================
// MMC (over SPI)
//==============================================================================

#ifdef AVR_UTIL_MMC

#define MMC_RETRY_COUNT 100

#define MMC_START_BLOCK_TOKEN 0b11111110                 // [MMC Table 5-8, p81]

// R1 Response Format Error codes... [MMC Figure 5-13, p78]

#define MMC_R1_OK          0b00000000
#define MMC_R1_IDLE        0b00000001
#define MMC_R1_ERASE_RESET 0b00000010
#define MMC_R1_ILLEGAL     0b00000100
#define MMC_R1_CRC         0b00001000
#define MMC_R1_ERASE_SEQ   0b00010000
#define MMC_R1_ADDRESS     0b00100000
#define MMC_R1_PARAMETER   0b01000000


void mmc_command_begin(uint8_t command)
{
    assert(pin_is_high(MMC_nSS));
    spi_tx(0xFF); //                                 Ncs [MMC, Figure 5-20, p83]
    pin_set_low(MMC_nSS);

    spi_tx(0b01000000 | command);
}


uint8_t mmc_read_response()
{
    uint8_t r;
    uint8_t ncr = 8; //                    [MMC, Figure 5-20 p83 Table 5-11 p86]

    do {
        r = spi_rx();
    } while((r & 0b10000000) && --ncr);

    return r;
}


void mmc_command_end()
{
    pin_set_high(MMC_nSS);
    spi_tx(0xFF); //                                             [MMC, 5.12 p37]
}


void mmc_go_idle_state()
{
    // Select SPI mode...                                         [MMC, 5,5 p68]
    // CMD0: GO_IDLE_STATE...                               [MMC, Table 5-5 p75]
    mmc_command_begin(0);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0x95); //                                              [MMC, 5.6 p69]

    mmc_read_response();
    mmc_command_end();
}


void mmc_send_op_cond()
{
    uint8_t retry = MMC_RETRY_COUNT;
    uint8_t r1;

    // Wait for initialisation...                                [MMC, 5.11 p72]
    do {
        _delay_ms(1);

        // CMD1: SEND_OP_COND...                            [MMC, Table 5-5 p75]
        mmc_command_begin(1);
        spi_tx(0);
        spi_tx(0);
        spi_tx(0);
        spi_tx(0);
        spi_tx(0b00000001);

        r1 = mmc_read_response();
        mmc_command_end();
    } while (r1 & MMC_R1_IDLE && retry--);
}


void mmc_set_block_len(uint16_t block_len)
{
    // CMD16: SET_BLOCKLEN...                               [MMC, Table 5-5 p76]
    mmc_command_begin(16);
    spi_tx(0);
    spi_tx(0);
    spi_tx((uint8_t)(block_len >> 8));
    spi_tx((uint8_t)(block_len >> 0));
    spi_tx(0b00000001);

    mmc_read_response();
    mmc_command_end();
}


uint16_t mmc_block_len;

void mmc_init(uint16_t block_len)
{
    mmc_block_len = block_len;

    pin_enable_output(MMC_nSS);
    pin_set_high(MMC_nSS);

    spi_init();

    _delay_ms(10);

    // Run clock a bit to wake up card...
    int i;
    for (i = 0 ; i < 10 ; i++) spi_rx();

    // MMC reset sequence...                                     [MMC, 5.11 p72]
    mmc_go_idle_state();
    mmc_send_op_cond();
    mmc_set_block_len(mmc_block_len);
}


static uint16_t mmc_write_count;

void mmc_write_begin(uint32_t address)
{
    uint8_t r1;

    mmc_write_count = 0;

    // Single Block Write...                                      [MMC, 5.8 p70]
    // CMD24: WRITE_BLOCK...                                [MMC, Table 5-5 p76]
    mmc_command_begin(24);
    spi_tx((uint8_t)(address >> 24));
    spi_tx((uint8_t)(address >> 16));
    spi_tx((uint8_t)(address >> 8));
    spi_tx((uint8_t)(address >> 0));
    spi_tx(0b00000001);

    r1 = mmc_read_response();
// FIXME lcd_home(); lcd_text("     R1:"); lcd_hex(r1);
//    assert(r1 == 0);

    spi_tx(MMC_START_BLOCK_TOKEN); //                     [MMC, Figure 5.7, p71]
}


void mmc_write(uint8_t* data, int len)
{
    mmc_write_count += len;
    assert(mmc_write_count <= mmc_block_len);

    // Send data...
    while(len--) {
        spi_tx(*data++);
    }
}


void mmc_write_end()
{
    uint8_t retry = MMC_RETRY_COUNT;

    // Pad to block size...
    while(mmc_write_count++ <= mmc_block_len) {
        spi_tx(0xFF);
    }

    // Dummy CRC...
    spi_tx(0xFF);
    spi_tx(0xFF);

    // Read response...
    uint8_t r = mmc_read_response();

    // Wait while card is busy...
    while (r != 0 && retry--) {
        r = spi_rx();
    }

    mmc_command_end();
}


static uint16_t mmc_read_count;


void mmc_read_begin(uint32_t address)
{
    uint8_t r1;

    // Multiple Block Read...                                     [MMC, 5.7 p69]
    // CMD18: READ_MULTIPLE_BLOCK...                        [MMC, Table 5-5 p76]
    mmc_command_begin(18);
    spi_tx((uint8_t)(address >> 24));
    spi_tx((uint8_t)(address >> 16));
    spi_tx((uint8_t)(address >> 8));
    spi_tx((uint8_t)(address >> 0));
    spi_tx(0b00000001);

    r1 = mmc_read_response();
    assert(r1 == 0);

    mmc_read_count = 0;
}


uint8_t mmc_read_byte()
{
    uint8_t retry = MMC_RETRY_COUNT;

    if (mmc_read_count % mmc_block_len == 0) {
        while (spi_rx() != MMC_START_BLOCK_TOKEN && retry--);
    }

    uint8_t byte = spi_rx();

    // Ignore 2-byte CRC at end of each block...
    if (++mmc_read_count % mmc_block_len == 0) {;
        spi_rx();
        spi_rx();
    }

    return byte;
}


void mmc_read_end()
{
    // CMD12: STOP_TRANSMISSION...                          [MMC, Table 5-5 p76]
    spi_tx(0b01000000 | 12);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0);
    spi_tx(0b00000001);
    spi_rx(); //                                          [MMC, Figure 5-25 p84]
    spi_rx();

    mmc_read_response();

    mmc_command_end();
}


void mmc_read(uint32_t address, uint8_t* buf, uint16_t len)
{
    mmc_read_begin(address);
    while(len--) {
        *buf++ = mmc_read_byte();
    }
    mmc_read_end();
}


#endif


//==============================================================================
// LCD (KS0066U, HD44780)
//==============================================================================


#ifdef LCD_RW


void lcd_nibble(uint8_t nibble, uint8_t rs)
{
    pin_set_low(LCD_RW);
    pin_set(LCD_RS, rs);

    // Set LCD pins...
    pin_set(LCD_DB7, nibble & 0b1000);
    pin_set(LCD_DB6, nibble & 0b0100);
    pin_set(LCD_DB5, nibble & 0b0010);
    pin_set(LCD_DB4, nibble & 0b0001);

    // Clock...
    pin_set_high(LCD_E);
    _delay_us(1);
    pin_set_low(LCD_E);
    _delay_us(1);
}


void lcd_byte(uint8_t byte, uint8_t rs)
{
    lcd_nibble(byte >> 4, rs);
    lcd_nibble(byte & 0b00001111, rs);
    _delay_us(50);
}


void lcd_cmd(uint8_t byte)
{
    lcd_byte(byte, /* RS: */ 0);
}


uint8_t lcd_cursor;

void lcd_write(uint8_t byte)
{
    lcd_byte(byte, /* RS: */ 1);
    lcd_cursor++;
}


void lcd_clear()
{
    lcd_cmd(0b00000001);
    _delay_ms(2);
    lcd_cursor = 0;
}


void lcd_home()
{
    lcd_cmd(0b00000010);
    _delay_ms(2);
    lcd_cursor = 0;
}


void lcd_increment_mode() { lcd_cmd(0b00000110); }
void lcd_on()             { lcd_cmd(0b00001100); }
void lcd_2_line_mode()    { lcd_cmd(0b00101000); }
void lcd_shift()          { lcd_cmd(0b00011100); }


void lcd_set_cursor(uint8_t address)
{
   lcd_cmd(0b10000000 | address);
   lcd_cursor = address;
}


void lcd_set_gfx(uint8_t address)
{
   lcd_cmd(0b01000000 | address);
}


void lcd_line2()
{
    lcd_set_cursor(0x40);
}


void lcd_init()
{
    pin_enable_output(LCD_RS);
    pin_enable_output(LCD_E);
    pin_enable_output(LCD_DB5);
    pin_enable_output(LCD_DB7);
    pin_enable_output(LCD_DB6);
    pin_enable_output(LCD_DB4);
    pin_enable_output(LCD_RW);

    pin_set_low(LCD_E);

    // Vcc == 4.5V + 30ms...
    _delay_ms(30);

    // 4 bit IO mode...
    lcd_nibble(0b0010, 0);

    lcd_2_line_mode();
    lcd_2_line_mode();

    lcd_on();
    lcd_clear();
    lcd_increment_mode();
}


void lcd_dec(uint32_t v, uint32_t pad)
{
    assert(pad > 0) ;
    uint32_t m;

    for (m = 100000000 ; m > pad && m > v ; m /= 10);
    while (m) {
        uint32_t n = v / m;
        lcd_write('0' + (uint8_t)n);
        v -= n * m;
        m /= 10;
    }
}


void lcd_hex(uint8_t v)
{
    uint8_t n = v >> 4;
    n += n > 9 ? ('A' - 10) : '0';
    lcd_write(n);

    n = v & 0b00001111;
    n += n > 9 ? ('A' - 10) : '0';
    lcd_write(n);

    if (lcd_cursor < 0x40 && lcd_cursor > 7) {
        lcd_set_cursor(0x40);
    }

    if (lcd_cursor > 0x47) {
        lcd_set_cursor(0);
    }

    lcd_write(':');
    lcd_set_cursor(lcd_cursor-1);
}


void lcd_hex16 (uint16_t v)
{
    lcd_hex((uint8_t)(v >> 8));
    lcd_hex((uint8_t)(v & 0xFF));
}


void lcd_text (const char* text)
{
    uint8_t c;
    while((c = *text++)) {
        lcd_write(c);
    }
}


#if 0
void lcd_gfx_char (char c)
{
    // Extract glyph from font...
    uint8_t glyph[8];
    memcpy_P (glyph, font_5x8h[c], sizeof(glyph));

    uint8_t i;
    for (i = 0 ; i < 8 ; i++) {
        lcd_set_gfx(i):
        lcd_write(glyph[i]);
    }
}
#endif


#if 0
#include "avr_util_font.h"

void lcd_text (const char* text)
{
    static uint8_t mask = 0b11111111;

    uint8_t glyph[5];

    uint8_t c;
    while((c = *text++)) {
        // Draw space character one pixel wide...
        if (c == ' ') {
            lcd_write(0);
            continue;
        }

        // Extract glyph from font...
        memcpy_P (glyph, font_5x8[c], sizeof(glyph));

        // Ignore leading whitespace...
        uint8_t i;
        for (i = 0 ; !glyph[i] ; i++);

        // Insert a gap if this glyph touches last one...
        if (glyph[i] & mask) {
            lcd_write(0);
        }

        // Draw glyph...
        while(i < 5) {
            lcd_write(glyph[i]);
        }

        // Keep mask of last pixels in glyph...
        mask = glyph[i];
        mask = mask << 1 | mask | mask >> 1;
    }
}
#endif


#endif


//==============================================================================
// FNV Sum
//==============================================================================

uint32_t fnv(const uint8_t* data, int len)
{
    // See [FNV, Parameters of the FNV-1 hash]...
    uint32_t h = 2166136261UL;
    while(len--) {
        h = (h * 0x01000193UL) ^ *data++;
    }
    return h;
}


//==============================================================================
//End of file
//==============================================================================
