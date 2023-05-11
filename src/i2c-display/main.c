#include <stdbool.h>
#include <stdint.h>

// Specifies the CPU frequency to be considered by the delay macros
// Must be defined before including <util/delay>
#define F_CPU 1000000UL

// #include <avr/io.h>
#include <util/delay.h>

// When addressing I/O Registers as data space using LD and ST
// instrucrutions, 0x20 must be added to these addresses.
#define OFFSET 0x20

// Macro helper for registers
#define REG(addr) *((volatile unsigned char*)(addr+OFFSET))

// Port B Data Direction Register
#define DDRB REG(0x17)

// Port B Data Register
#define PORTB REG(0x18)

// Port B Input Pins Address
#define PINB REG(0x16)

// USICR - USI Control Register
#define USICR REG(0x0D)

// USISR – USI Status Register
#define USISR REG(0x0E)

// USIDR – USI Data Register
#define USIDR REG(0x0F)

// PORTB pins
#define PB2_PIN 2 // SCL
#define PB0_PIN 0 // SDA
#define SCL_PIN PB2_PIN
#define SDA_PIN PB0_PIN

// USICR pins
#define USISIE_PIN 7 // Start Condition Interrupt Enable
#define USIOIE_PIN 6 // Counter Overflow Interrupt Enable
#define USIWM1_PIN 5 // Wire Mode
#define USIWM0_PIN 4 // Wire Mode
#define USICS1_PIN 3 // Clock Source Select
#define USICS0_PIN 2 // Clock Source Select
#define USICLK_PIN 1 // Clock Strobe
#define USITC_PIN 0  // Toggle Clock Port Pin

// USISR pins
#define USISIF_PIN 7  // Start Condition Interrupt Flag
#define USIOIF_PIN 6  // Counter Overflow Interrupt Flag
#define USIPF_PIN 5   // Stop Condition Flag
#define USIDC_PIN 4   // Data Output Collision
#define USICNT3_PIN 3 // Counter Value
#define USICNT2_PIN 2 // Counter Value
#define USICNT1_PIN 1 // Counter Value
#define USICNT0_PIN 0 // Counter Value

// USISR mask
#define USISR_CLOCK_8_BIT 0b11110000
#define USISR_CLOCK_1_BIT 0b11111110

#define WRITE_BIT 0
#define READ_BIT 1

// Creates a bitmask for a specific bit position. The input (bit)
// represents the bit position you want to create a bitmask for.
#define BV_MASK(bit) (1 << (bit))

void i2c_init()
{
  // Set data direction for PB0_PIN (SDA) and PB2_PIN (SCL)
  DDRB |= (1 << SDA_PIN);
  DDRB |= (1 << SCL_PIN);

  PORTB |= (1 << SDA_PIN); // Pull SDA high
  PORTB |= (1 << SCL_PIN); // Pull SCL high

  USIDR = 0xFF;            // Preload data register

  USICR = (1 << USIWM1_PIN) | (1 << USICS1_PIN) | (1 << USICLK_PIN);

  USISR = 1 << USISIF_PIN | 1 << USIOIF_PIN | 1 << USIPF_PIN | 1 << USIDC_PIN |    // Clear flags
    0x0 << USICNT0_PIN; // Reset counter
}

uint8_t i2c_transfer(uint8_t usisr_mask)
{
  USISR = usisr_mask; // Set USI Status Register according to mask

  // I2C transfer
  do {
    _delay_us(5);
    USICR |= (1 << USITC_PIN);            // Generate positive clock edge
    while (!(PINB & 1 << SCL_PIN));    // Wait for SCL to go high
    _delay_us(4);
    USICR |= (1 << USITC_PIN);            // Generate negative clock edge
  } while (!(USISR & 1 << USIOIF_PIN)); // Repeat clock generation at SCL until the counter overflows and a byte is transferred

  _delay_us(5);

  uint8_t data = USIDR;                   // Read data

  USIDR = 0xFF;

  DDRB |= (1 << SDA_PIN);

  return data;
}

bool i2c_start()
{
  PORTB |= (1 << SCL_PIN); // Pull SCL high

  while (!(PORTB & 1 << SCL_PIN)); // Verify that SCL goes high.

  _delay_us(5);

  // Generate start condition
  PORTB &= ~(1 << SDA_PIN); // Pull SDA low

  _delay_us(4);

  PORTB &= ~(1 << SCL_PIN); // Pull SCL low
  PORTB |= (1 << SDA_PIN); // Pull SDA high

  if (!(USISR & 1 << USISIF_PIN)) {
    return false;
  }

  return true;
}

uint8_t i2c_write_byte(uint8_t data)
{
  PORTB &= ~(1 << SCL_PIN); // Pull SCL low
  USIDR = data;
  i2c_transfer(USISR_CLOCK_8_BIT);

  // Verify ack
  DDRB &= ~(1 << SDA_PIN); // Set SDA data direction to input to recieve ack
  uint8_t ack = i2c_transfer(USISR_CLOCK_1_BIT);
  DDRB |= (1 << SDA_PIN); // Set SDA data direction back to output

  return ack;
}

void i2c_stop()
{
  // Pull SDA low
	PORTB &= ~(1 << SDA_PIN);

	// Pull SCL high
	PORTB |= (1 << SCL_PIN);
	while (! (PORTB & (1 << SCL_PIN)));

	_delay_us(5);

	// Pull SDA high
	PORTB |= (1 << SDA_PIN);

	_delay_us(4);
}

int main()
{
  _delay_ms(4000);

  i2c_init();
  bool success = i2c_start();

  uint8_t i2c_addr = 0x27 << 1 | WRITE_BIT;
  i2c_write_byte(i2c_addr);

  i2c_stop();

  return 0;
}
