// Port B Data Direction Register. (0x17 + __SFR_OFFSET)
#define DDRB *((volatile unsigned char*)0x37)

// Port B Data Register (0x18 + __SFR_OFFSET)
#define PORTB *((volatile unsigned char*)0x38)

// Timer/Counter Control Register B (0x33 + __SFR_OFFSET)
#define TCCR0B *((volatile unsigned char*)0x53)

// Timer/Counter Interrupt Mask Register (0x39 + __SFR_OFFSET)
#define TIMSK *((volatile unsigned char*)0x59)

// Creates a bitmask for a specific bit position. The input (bit)
// represents the bit position you want to create a bitmask for.
#define BV_MASK(bit) (1 << (bit))

// Handler function for the TIMER0_OVF (Timer/Counter0 Overflow) interrupt
// (vector #5 from datasheet). The naming (__vector_+vector#) is ultimately
// what the compiler needs in order to know this is an Interrupt Service
// Routine handler func and to patch the vector table, etc.
void __vector_5(void) __attribute__ ((signal, used, externally_visible));
void __vector_5(void)
{
  PORTB ^= BV_MASK(0);
  __asm__ __volatile__ ("reti" ::: "memory");
}

int main()
{
  // Set data direction for Port B bit 0 (PB0 - pin 5)
  DDRB |= BV_MASK(0);

  // Set prescale timer to 1/1024th the clock rate
  TCCR0B = BV_MASK(2) | BV_MASK(0);

  // Enable timer overflow interrupt
  TIMSK |= BV_MASK(1);

  // Enable global interrupts
  __asm__ __volatile__ ("sei" ::: "memory");

  while (1) {}
}
