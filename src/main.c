/*

17-channel soft-PWM.


                 ATMega328p "Nano"
             ┌───────────────────────┐
             │      RST SCK MISO     │
             │      [ ] [ ] [ ]      │
             │      [ ] [ ] [ ]      │
             │      GND MOSI 5V      │
             │                       │          
       PWM1  │[ ] TX  D1      VIN [ ]│
       PWM0  │[ ] RX  D0      GND [ ]│ 
             │[ ] RST      C6 RST [ ]│  (RST)
             │[ ] GND          5V [ ]│
       PWM2  │[ ] D2  D2       A7 [ ]│
       PWM3  │[ ] D3  D3       A6 [ ]│
       PWM4  │[ ] D4  D4   C5  A5 [ ]│  PWM16
       PWM5  │[ ] D5  D5   C4  A4 [ ]│  PWM15
       PWM6  │[ ] D6  D6   C3  A3 [ ]│  PWM14
       PWM7  │[ ] D7  D7   C2  A2 [ ]│  PWM13
       PWM8  │[ ] D8  B0   C1  A1 [ ]│  PWM12
       PWM9  │[ ] D9  B1   C0  A0 [ ]│  PWM11
      PWM10  │[ ] D10 B2      REF [ ]│
     (MOSI)  │[ ] D11 B3      3V3 [ ]│
     (MISO)  │[ ] D12 B4   B5 D13 [ ]│  (SCK)
             │      ┌─────────┐      │
             └──────┤   USB   ├──────┘
                    │         │
                    └─────────┘
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

#define PWM_BUF_LEN 64
#define PWM_MAX (PWM_BUF_LEN * 8)
#define PWM_CHANNEL_COUNT 17


// SPI RX.
#define SPI_MOSI        pin_code(PORTB, 3)
#define SPI_MISO        pin_code(PORTB, 4)
#define SPI_SCK         pin_code(PORTB, 5)

const uint8_t pwm_pins[] = {
    pin_code(PORTD, 0), //  0
    pin_code(PORTD, 1), //  1
    pin_code(PORTD, 2), //  2
    pin_code(PORTD, 3), //  3
    pin_code(PORTD, 4), //  4
    pin_code(PORTD, 5), //  5
    pin_code(PORTD, 6), //  6
    pin_code(PORTD, 7), //  7
    pin_code(PORTB, 0), //  8
    pin_code(PORTB, 1), //  9
    pin_code(PORTB, 2), // 10
    pin_code(PORTC, 0), // 11
    pin_code(PORTC, 1), // 12
    pin_code(PORTC, 2), // 13
    pin_code(PORTC, 3), // 14
    pin_code(PORTC, 4), // 15
    pin_code(PORTC, 5), // 16
};


#ifndef UNIT_TEST
// SPI bit-bang RX.
volatile uint8_t rx_t0;
volatile uint8_t rx_i;
volatile uint8_t rx_buf;
#define RX_TIMEOUT 2

ISR(PCINT0_vect)
{
    if (pin_is_high(SPI_SCK)) {
        if (rx_i == 0 || (_rtc - rx_t0) > RX_TIMEOUT) {
            rx_buf = 0;
            rx_i = 0;
            rx_t0 = _rtc;
        }
        if (rx_i < 8) {
            uint8_t b = rx_buf;
            if (pin_is_high(SPI_MOSI)) {
                b |= 1;
            }
            rx_i++;
            if (rx_i < 8) {
                b <<= 1;
            }
            rx_buf = b;
        }
    }
}

static void bbspi_init()
{
    rx_i = 0;
    rx_buf = 0;

    // Configure SPI RX pins...
    pin_enable_input(SPI_SCK);
    pin_disable_pullup(SPI_SCK);
    pin_enable_input(SPI_MOSI);
    pin_disable_pullup(SPI_MOSI);

    // Connect PORTB bit 5 to PCINT0_vect
    PCMSK0 |= (1 << PCINT5);                     // [ATMega328.pdf, 10.2.8, p74]
    PCICR |= (1 << PCIE0);                       // [ATMega328.pdf, 10.2.4, p73]
}
#endif


// Value at index `i` is the number of bits set in binary representation of `i`.
const int8_t _bit_count[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

// Number of bits set in the binary representation of `v`.
static int8_t bit_count(uint8_t v)
{
    return _bit_count[v & 0b1111]
         + _bit_count[v >> 4];
}


static int16_t pwm_level[PWM_CHANNEL_COUNT];
static uint8_t pwm_buf[PWM_CHANNEL_COUNT][PWM_BUF_LEN];

static uint8_t pwm_i;
static uint8_t pwm_mask;

static void pwm_init()
{
    for (uint8_t i = 0 ; i < PWM_CHANNEL_COUNT ; i++) {
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

static void set_pwm_bit(uint8_t channel, uint16_t i, uint8_t value)
{
    uint8_t mask = 1 << (i % 8);
    i >>= 4;
    if (value) {
        pwm_buf[channel][i] |= mask;
    } else {
        pwm_buf[channel][i] &= ~mask;
    }
}

static uint8_t pwm_state(uint8_t channel)
{
    return pwm_buf[channel][pwm_i] & pwm_mask;
}

static void pwm_output()
{
    for (uint8_t i = 0 ; i < PWM_CHANNEL_COUNT ; i++) {
        pin_set(pwm_pins[i], pwm_state(i));
    }
    increment_pwm();
}

static int8_t pwm_random_tweak(uint8_t channel, uint8_t tweak_upwards)
{
    assert(channel < PWM_CHANNEL_COUNT);

    // Random index and mask.
    uint16_t r = rand();
    uint8_t i = r % PWM_BUF_LEN;
    uint8_t mask = r >> 8;

    // Count the number of one bits in byte.
    uint8_t byte = pwm_buf[channel][i];
    int8_t b_count = bit_count(byte);

    // Tweak the byte.
    if (tweak_upwards) {
        byte |= mask;
    } else {
        byte &= mask;
    }
    pwm_buf[channel][i] = byte;

    // Count the number bits added or removed.
    int8_t result = bit_count(byte) - b_count;
    if (tweak_upwards) {
        assert(result >= 0);
    } else {
        assert(result <= 0);
    }
    return result;
}

static uint8_t reverse_bits(uint8_t byte) 
{ 
    return ((byte >> 1) & 0x55) | ((byte << 1) & 0xAA); 
}

static void pwm_tweak(uint8_t channel, int16_t err)
{
    for (uint8_t i = 0 ; i < PWM_BUF_LEN && err != 0 ; i++) {
        uint8_t j = reverse_bits(i);
        uint8_t byte = pwm_buf[channel][j];
        int8_t b_count = bit_count(byte);
        uint8_t mask = 1;
        while(b_count < 8 && err > 0 && mask != 0) {
            if ((byte & mask) == 0) {
                byte |= mask;
                b_count++;
                err--;
            }
            mask <<= 1;
        }
        while(b_count > 0 && err < 0 && mask != 0) {
            if ((byte & mask) != 0) {
                byte &= ~mask;
                b_count--;
                err++;
            }
            mask <<= 1;
        }
        pwm_buf[channel][j] = byte;
    }
}


static void pwm_set(uint8_t channel, uint16_t target)
{
    assert(target <= PWM_MAX);
    assert(channel < PWM_CHANNEL_COUNT);

    int16_t level = pwm_level[channel];

    // How far from the target is the current level?
    int16_t err = target - level;
    if (err == 0) {
        return;
    }

    // Apply limited number of random tweaks.
    uint16_t tweak_limit = abs(err/2);
    //printn("tweak_limit:", tweak_limit);
    for (uint8_t i = 0 ; i < tweak_limit ; i++) {
        level += pwm_random_tweak(channel, level < target);
        if (level == target || (err > 0 && level > target)) {
            break;
        }
    }

    // Correct the remaining error.
    pwm_tweak(channel, target - level);

    // Check that target was reached.
    int16_t count = 0;
    for (uint8_t i = 0 ; i < PWM_BUF_LEN ; i++) {
        count += bit_count(pwm_buf[channel][i]);
    }
    //printn("count: ", count);
    //printn("target: ", target);
    assert(count == target);

    pwm_level[channel] = target;
    printn("target: ", target);
}



#ifndef UNIT_TEST

uint16_t counter = 0;

ISR(TIMER2_COMPB_vect)
{
//    counter++;
//    if (counter % 5 == 0) {
        pwm_output();
//    }
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

    // Enable TIMER2_COMPB ISR...
    OCR2B = 100;
    TIFR2 |= (1 << OCF2B);
    TIMSK2 |= (1 << OCIE2B);

    // Enable interrupts..
    sei();

    uint8_t channel = 0;
    uint16_t target = 0;

    println("Hello!!");

    for(;;) {

        if (rx_i == 8) {
            cli();
                uint8_t byte = rx_buf;
                rx_i = 0;
            sei();
            //printn("byte: ", byte);
            uint8_t flag = (byte & 0b11000000) >> 6;
            //printn("flag: ", flag);
            byte &= 0b00111111;
            if (flag == 0) {
                channel = byte;
            } else if (flag == 1) {
                target = (uint16_t)byte << 6;
            } else if (flag == 2) {
                target |= byte;
                //printn("channel: ", channel);
                //printn("target: ", target);
                pwm_set(channel, target);
            }
        }

    }
}

#endif // ndef UNIT_TEST



#ifdef UNIT_TEST

void print_channel(uint8_t channel)
{
    int16_t count = 0;
    for (uint8_t i = 0 ; i < PWM_BUF_LEN ; i += 4) {
        uint8_t byte = pwm_buf[channel][i];
        printf("%02X ", byte);
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
