/*
Bit-bash SPI.
*/

volatile uint8_t bbspi_rx_t0;
volatile uint8_t bbspi_rx_i;
volatile uint8_t bbspi_rx_buf;
#define RX_TIMEOUT 2


uint8_t bbspi_buffer_is_full() { return bbspi_rx_i == 8;}

uint8_t bbspi_rx()
{
    while(!bbspi_buffer_is_full());
    uint8_t byte = bbspi_rx_buf;
    bbspi_rx_i = 0;
    return byte;
}


ISR(PCINT0_vect)
{
    if (bbspi_buffer_is_full()) {
        return;
    }
    if (!pin_is_high(SPI_SCK)) {
        return;
    }

    if (bbspi_rx_i == 0 || (_rtc - bbspi_rx_t0) > RX_TIMEOUT) {
        bbspi_rx_buf = 0;
        bbspi_rx_i = 0;
        bbspi_rx_t0 = _rtc;
    }
    if (bbspi_rx_i < 8) {
        uint8_t b = bbspi_rx_buf;
        if (pin_is_high(SPI_MOSI)) {
            b |= 1;
        }
        bbspi_rx_i++;
        if (bbspi_rx_i < 8) {
            b <<= 1;
        }
        bbspi_rx_buf = b;
    }
}


static void bbspi_init()
{
    bbspi_rx_i = 0;
    bbspi_rx_buf = 0;

    // Configure SPI RX pins...
    pin_enable_input(SPI_SCK);
    pin_disable_pullup(SPI_SCK);
    pin_enable_input(SPI_MOSI);
    pin_disable_pullup(SPI_MOSI);

    // Connect PORTB bit 5 to PCINT0_vect
    PCMSK0 |= (1 << PCINT5);                     // [ATMega328.pdf, 10.2.8, p74]
    PCICR |= (1 << PCIE0);                       // [ATMega328.pdf, 10.2.4, p73]
}


/* End of file. */
