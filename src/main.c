/*

17-channel soft-PWM.

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
    #include <stdlib.h>
    #include <assert.h>

    #define pin_enable_output(pin) ({})
    #define pin_code(port,pin) (pin)
    #define pin_set(pin, state) ({})
    #define printn(m, n) ({})

    void print_channel(uint8_t channel);
#endif

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


// PWM output level setting.
static int16_t pwm_level[PWM_PIN_COUNT];

// PWM bit patterns.
static uint8_t pwm_buf[PWM_PIN_COUNT][PWM_BUF_LEN];

// PWM bit pattern cursor.
static uint8_t pwm_i;
static uint8_t pwm_mask;

// Configure GPIO pins for output and zero bit patterns.
static void pwm_init()
{
    for (uint8_t i = 0 ; i < PWM_PIN_COUNT ; i++) {
        pin_enable_output(pwm_pins[i]);
        pin_set(pwm_pins[i], 0);
        pwm_level[i] = 0;
        for (uint8_t j = 0 ; j < PWM_BUF_LEN ; j++) {
            pwm_buf[i][j] = 0;
        }
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
// Return `1` if the bit was changed from `0` to `1`.
// Return `-1` if the bit was changed from `1` to `0`.
// Return `0` if the bit was unchanged.
static int8_t pwm_set_bit(uint8_t channel, uint16_t i, uint8_t value)
{
    assert(channel < PWM_PIN_COUNT);
    assert(i < PWM_MAX);
    int8_t result = 0;
    uint8_t mask = 1 << (i % 8);
    i >>= 3;
    uint8_t byte = pwm_buf[channel][i];
    if (value) {
        if ((byte & mask) == 0) {
            byte |= mask;
            result = 1;
        }
    } else {
        if (byte & mask) {
            byte &= ~mask;
            result = -1;
        }
    }
    pwm_buf[channel][i] = byte;
    return result;
}


// Set a random bit in the PWM pattern buffer for `channel`.
static void pwm_set_random_bit(uint8_t channel)
{
    assert(channel < PWM_PIN_COUNT);

    uint32_t i = rand() % PWM_MAX;
    for (;;) {
        if (pwm_set_bit(channel, i, 1)) {
            return;
        }
        i++;
        if (i == PWM_MAX) {
            i = 0;
        }
    }
}


// Value at index `i` is the number of bits set in binary representation of `i`.
const int8_t _bit_count[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

// Number of bits set in the binary representation of `v`.
static int8_t bit_count(uint8_t v)
{
    return _bit_count[v & 0b1111]
         + _bit_count[v >> 4];
}


// Set PWM `channel` to `target` output value.
static void pwm_set(uint8_t channel, uint16_t target)
{
    assert(target <= PWM_MAX);
    assert(channel < PWM_PIN_COUNT);

    // Find a ratio `a:b` that approximates `target:PWM_MAX`...
    uint16_t a = target;
    uint16_t b = PWM_MAX;
    int16_t err = 0;
    while(a > 1) {
        uint16_t _a = a / 2;
        uint16_t _b = b / 2;
        err = target - _a * (PWM_MAX / _b);
        if (err > (PWM_MAX / _b)) {
            break;
        }
        a = _a;
        b = _b;
    }
    assert(err >= 0);

    // Create a bit pattern using the ratio `a:b`...
    for (uint16_t i = 0 ; i < PWM_MAX ; i ++) {
        pwm_set_bit(channel, i, (i % b) < a);
    }

    // Fill in the remaining bits randomly...
    while (err--) {
        pwm_set_random_bit(channel);
    }

    // Check that the target was reached.
    uint16_t count = 0;
    for (uint8_t i = 0 ; i < PWM_BUF_LEN ; i++) {
        count += bit_count(pwm_buf[channel][i]);
    }
    assert(count == target);

    pwm_level[channel] = target;
    printn("err: ", target - count);

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
    // Configure Moinitor Serial Data TX pin...
    pin_enable_output(MONITOR);
    pin_set_high(MONITOR);

    // Initialise SPI RX...
    bbspi_init();

    rtc_init();

    pwm_init();

    // CTC mode, 16MHz / 8 = 2000kHz...
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (0 << CS10);
     
    // Enable TIMER1_COMPA ISR...
    OCR1A = 200;
    TIFR1 |= (1 << OCF1A);
    TIMSK1 |= (1 << OCIE1A);

    // Enable interrupts..
    sei();

    uint8_t channel = 0;
    uint16_t target = 0;

    println("PiAVRPWM Started.");

    for(;;) {
        uint8_t byte = bbspi_rx();
        uint8_t flag = (byte & 0b11000000) >> 6;
        byte &= 0b00111111;
        if (flag == 0) {
            channel = byte;
        } else if (flag == 1) {
            target = (uint16_t)byte << 6;
        } else if (flag == 2) {
            target |= byte;
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
