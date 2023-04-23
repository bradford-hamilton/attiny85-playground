// When addressing I/O Registers as data space using LD and ST
// instrucrutions, 0x20 must be added to these addresses.
#define OFFSET 0x20

// Macro helper for registers
#define REGISTER(addr) *((volatile unsigned char*)(addr+OFFSET))

// Port B Data Direction Register
#define DDRB REGISTER(0x17)

// Port B Data Register
#define PORTB REGISTER(0x18)

// Timer/Counter Control Register B
#define TCCR0B REGISTER(0x33)

// Timer/Counter Interrupt Mask Register
#define TIMSK REGISTER(0x39)

// Creates a bitmask for a specific bit position. The input (bit)
// represents the bit position you want to create a bitmask for.
#define BV_MASK(bit) (1 << (bit))

// Inline assembly for SEI instruction
#define ASM_SEI() __asm__ __volatile__ ("sei" ::: "memory")

// Inline assembly for RETI instruction
#define ASM_RETI() __asm__ __volatile__ ("reti" ::: "memory")

// Handler function for the TIMER0_OVF (Timer/Counter0 Overflow) interrupt
// (vector #5 from datasheet). The naming (__vector_+vector#) is ultimately
// what the compiler needs in order to know this is an Interrupt Service
// Routine handler func and to patch the vector table, etc.
void __vector_5(void) __attribute__ ((signal, used, externally_visible));
void __vector_5(void)
{
  PORTB ^= BV_MASK(0);
  ASM_RETI();
}

int main()
{
  // Set data direction for Port B bit 0 (PB0)
  DDRB |= BV_MASK(0);

  // Set prescale timer to 1/1024th the clock rate
  TCCR0B = BV_MASK(2) | BV_MASK(0);

  // Enable timer overflow interrupt
  TIMSK |= BV_MASK(1);

  // Enable global interrupts
  ASM_SEI();

  while (1) {}
}
