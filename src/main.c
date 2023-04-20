// Port B data direction register
#define DDRB *((volatile unsigned char*)0x37)

// Port B data register
#define PORTB *((volatile unsigned char*)0x38)

int main()
{
  // Set data direction for Port B bit 0 (PB0 - pin 5)
  DDRB |= (1 << 0); // 00000001

  while (1) {
    // Set data for Port B bit 0 (PB0 - pin 5)
    PORTB |= (1 << 0);
    for (long i = 0; i < 100000; i++) { PORTB |= (1 << 0); }

    PORTB = 0x0;
    for (long i = 0; i < 100000; i++) { PORTB = 0x0; }
  }
}
