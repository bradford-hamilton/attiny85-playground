#include <stdbool.h>
#include <stdint.h>

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

// USICR pins
#define USIWM1_PIN 5 // Wire Mode
#define USICS1_PIN 3 // Clock Source Select
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

// Creates a bitmask for a specific bit position. The input (bit)
// represents the bit position you want to create a bitmask for.
#define BV_MASK(bit) (1 << (bit))

void _delay_us(uint16_t us)
{
  while (us--) { __asm__ __volatile__ ("nop"); }
}

void _delay_ms(uint16_t ms)
{
  while (ms--) { _delay_us(1000); }
}

void i2c_init()
{
  // I2C start condition
  PORTB |= (1 << PB0_PIN); // Pull SDA high
  // delay_ms(5);
  PORTB |= (1 << PB2_PIN); // Pull SCL high
  // delay_ms(5);

  // Set data direction for PB0_PIN (SDA) and PB2_PIN (SCL)
  DDRB |= BV_MASK(PB0_PIN);
  DDRB |= BV_MASK(PB2_PIN);
  
  USIDR = 0xFF;                 // Preload data register

  USICR |= BV_MASK(USIWM1_PIN); // Set two-wire mode (I2C)
  USICR |= BV_MASK(USICS1_PIN); // Software stobe as counter clock source
  USICR |= BV_MASK(USICLK_PIN); // Software stobe as counter clock source

  USISR = 1 << USISIF_PIN | 1 << USIOIF_PIN | 1 << USIPF_PIN | 1 << USIDC_PIN | // Clear flags,
    0x0 << USICNT0_PIN; // reset counter.
}

bool i2c_start(uint8_t address, int32_t read_count)
{

}

int main()
{
  i2c_init();

  // --------------------------------------------------------------------

  while (!(USISR & (1 << USISIF_PIN)));
  
  // Set data register to slave address and perform write op
  USIDR = (0x27 << 1) & ~0x01;

  // I2C transfer
  do {
    USICR |= (1 << USITC_PIN);        // Clock signal from SCL
    while ((PINB & (1 << PB2_PIN)));  // Wait for SCL to get pulled high
    USICR |= (1 << USITC_PIN);        // Toggle SCL low TODO: try PORTB bit write

  } while (!(USISR & (1 << USIOIF_PIN))); // Repeat clock generation at SCL until the counter overflows and a byte is transferred
  USISR |= (1 << USIOIF_PIN);             // Clear overflow flag

  // I2C ack
  DDRB &= ~(1 << PB0_PIN); // Change direction of SDA to receive acknowledge bit
  USISR |= (1 << USICNT3_PIN) | (1 << USICNT2_PIN) | (1 << USICNT1_PIN); // Set counter to 1110 to force it to overflow when ACK bit is received

  // I2C transfer
  do {
    USICR |= (1 << USITC_PIN);
    while ((PINB & (1 << PB2_PIN)));
    USICR |= (1 << USITC_PIN);

  } while (!(USISR & (1 << USIOIF_PIN)));
  USISR |= (1 << USIOIF_PIN);

  char i2c_data;
  i2c_data = USIDR;

  if (i2c_data & 0x01) {
    DDRB |= BV_MASK(PB0_PIN);

    char data_to_send[] = "testing";
    short int i = 0;
    
    while (data_to_send[i] != "\0") {
      USIDR = data_to_send[i]; // Place byte in data register

      // I2C transfer
      do {
        USICR |= (1 << USITC_PIN);
        while ((PINB & (1 << PB2_PIN)));
        USICR |= (1 << USITC_PIN);

      } while (!(USISR & (1 << USIOIF_PIN)));
      USISR |= (1 << USIOIF_PIN);

      i++;
    }
  }

  // I2C stop
  PORTB |= (0 << PB2_PIN); // Pulling SDA low 
  delay_ms(5);
  PORTB |= (0 << PB0_PIN); // Pulling SCL low
  delay_ms(5);

  return 0;
}
