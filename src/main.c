/*

Multi-channel 0-5V soft PWM (PDM) Output Module

main.c for ATMega328p.

*/

// Monitor Serial Data TX pin.
#define MONITOR_PORT    PORTB
#define MONITOR_PIN     4
#define MONITOR         pin_code(MONITOR_PORT, MONITOR_PIN)

#ifndef UNIT_TEST
    #include <stdlib.h>
    #include "avr_util.h"
#else
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <assert.h>

    #define pin_enable_output(pin) ({})
    #define pin_code(port,pin) (pin)
    #define pin_set(pin, state) ({})
    #define printn(m, n) ({})

    void print_channel(uint8_t channel);

    #define DO_PWM_CHECK
#endif

#define PWM_BITS 10
#define PWM_MAX 512
#define PWM_BUF_LEN (PWM_MAX / 8)


// SPI RX.
#ifndef UNIT_TEST
#define SPI_MOSI        pin_code(PORTB, 3)
#define SPI_MISO        pin_code(PORTB, 4)
#define SPI_SCK         pin_code(PORTB, 5)
#include "bbspi.c"
#endif


#include "pwm_pins.h"


// PWM bit patterns.
static uint8_t pwm_buf[PWM_PIN_COUNT][PWM_BUF_LEN];

// PWM bit pattern cursor.
static uint8_t pwm_i;
static uint8_t pwm_mask;

static void pwm_set(uint8_t channel, uint16_t target);

// Configure GPIO pins for output and set bit patterns to zero.
static void pwm_init()
{
    for (uint8_t i = 0 ; i < PWM_PIN_COUNT ; i++) {
        pin_enable_output(pwm_pins[i]);
        pin_set(pwm_pins[i], 0);
        pwm_set(i, PWM_MAX/2);
    }
    pwm_i = 0;
    pwm_mask = 1;
}


// Move cursor to next position.
static void increment_pwm()
{
    pwm_mask <<= 1;
    if (pwm_mask == 0) {
        pwm_mask = 1;
        pwm_i ++;
    }
    if (pwm_i == PWM_BUF_LEN) {
        pwm_i = 0;
    }
}


// PWM state for `channel` at current cursor position.
static uint8_t pwm_state(uint8_t channel)
{
    return pwm_buf[channel][pwm_i] & pwm_mask;
}


// Write PWM state for each channel to the GPIO pins.
static void pwm_output()
{
    for (uint8_t i = 0 ; i < PWM_PIN_COUNT ; i++) {
        pin_set(pwm_pins[i], pwm_state(i));
    }
    increment_pwm();
}


// Set bit `i` in a `channel`'s PWM buffer to `value` (0 or 1).
static void pwm_set_bit(uint8_t channel, uint16_t i, uint8_t value)
{
    assert(channel < PWM_PIN_COUNT);
    assert(i < PWM_MAX);
    uint8_t mask = 1 << (i % 8);
    i >>= 3;
    uint8_t byte = pwm_buf[channel][i];
    if (value) {
        byte |= mask;
    } else {
        byte &= ~mask;
    }
    pwm_buf[channel][i] = byte;
}


#ifdef DO_PWM_CHECK
// Value at index `i` is the number of bits set in binary representation of `i`.
const int8_t _bit_count[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

// Number of bits set in the binary representation of `v`.
static int8_t bit_count(uint8_t v)
{
    return _bit_count[v & 0b1111]
         + _bit_count[v >> 4];
}
#endif


// Set PWM `channel` to `target` output value.
static void pwm_set(uint8_t channel, uint16_t target)
{
    assert(target <= PWM_MAX);
    assert(channel < PWM_PIN_COUNT);

    // Pulse Density Modulation
    // https://en.wikipedia.org/wiki/Pulse-density_modulation
    int16_t sum = 0;
    for (uint16_t i = 0 ; i < PWM_MAX ; i++) {
        sum += target;
        if (sum < PWM_MAX) {
            pwm_set_bit(channel, i, 0);
        } else {
            pwm_set_bit(channel, i, 1);
            sum -= PWM_MAX;
        }
    }

//    printn("pwm_set", (channel << PWM_BITS) | target);

#ifdef DO_PWM_CHECK
    // Check that the target was reached.
    uint16_t count = 0;
    for (uint8_t i = 0 ; i < PWM_BUF_LEN ; i++) {
        count += bit_count(pwm_buf[channel][i]);
    }
    assert(count == target);
#endif

    return;
}


#ifndef UNIT_TEST

ISR(TIMER1_COMPA_vect)
{
    pwm_output();
}

int main(void) __attribute((noreturn));
int main()
{
    // Configure Monitor Serial Data TX pin...
    pin_enable_output(MONITOR);
    pin_set_high(MONITOR);

    // Initialise SPI RX...
    bbspi_init();

    rtc_init();

    pwm_init();

    // Configure ISR(TIMER1_COMPA_vect)
    // CTC mode, 16MHz / 8 = 2000kHz
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (0 << CS10);
    OCR1A = 40; // 2000 / 40 = 50kHz
    TIFR1 |= (1 << OCF1A);
    TIMSK1 |= (1 << OCIE1A);

    // Enable interrupts..
    sei();

    uint8_t channel = 0;
    uint16_t target = 0;

    //println("PiAVRPWM Started.");

    for(;;) {
        uint8_t byte = bbspi_rx();
        if ((byte & 0b10000000) == 0) {
            channel = byte >> 3;
            target = (uint16_t)(byte & 0b00000111) << 7;
        } else {
            target |= (byte & 0b01111111);
            pwm_set(channel, target);
        }
    }
}

#endif // ndef UNIT_TEST



#ifdef UNIT_TEST

void print_channel(uint8_t channel)
{
    int16_t count = 0;
    for (uint8_t i = 0 ; i < PWM_BUF_LEN ; i++) {
        uint8_t byte = pwm_buf[channel][i];
        if (i % 4 == 0) {
            printf("%02X ", byte);
        }
        count += bit_count(byte);
    }
    printf("(%d/%d)\n", count, PWM_MAX);
}

int main()
{
    for (int16_t i = 0 ; i <= 0xFF ; i++) {
        assert(bit_count(i) == __builtin_popcount(i));
    }
    print_channel(0);
    pwm_set(0, PWM_MAX);
    print_channel(0);
    pwm_set(0, 0);
    print_channel(0);
    pwm_set(0, PWM_MAX/2);
    print_channel(0);
    for (uint16_t i = 1 ; i <= PWM_MAX ; i += 10) {
        pwm_set(0, i);
        print_channel(0);
    }
    for (uint16_t i = 1 ; i <= 1000; i += 1) {
        pwm_set(0, (rand() * (uint64_t)PWM_MAX) / RAND_MAX);
        print_channel(0);
    }
}

#endif // UNIT_TEST

/* End of file. */
