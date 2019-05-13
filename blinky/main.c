//------------------------------------------------------------------------
// blinky - msp430m0 arm m0 instructions running on msp430fr6989 launchpad
//
#include <inttypes.h>

#define BIT0 (1<<0)
#define BIT3 (4<<0)

//------------------------------------------------------------------------
// GPIO_Type - fake GPIO peripheral wired to 8 bit P1,P2,P3 ports
typedef struct {
    volatile unsigned INR;      // 8 bit input pin register
    volatile unsigned ODR;      // 8 bit output pin register
    volatile unsigned MODER;    // 8 bit pin mode register
    volatile unsigned BSRR;     // 16 bit bit set / reset register 
    volatile unsigned TOGGLER;  // 8 bit toggle pin register
} GPIO_Type ;

#define PERIPHERAL_BASE         0x40000000
#define P1_BASE                 0x40001000
#define P2_BASE                 (PERIPHERAL_BASE+0x2000)

GPIO_Type * const P1 = (GPIO_Type * const)P1_BASE;

//------------------------------------------------------------------------

int main ( void )
{
  P1->MODER |= BIT0 | BIT3;     // P1.0 and P1.3 outputs
  P1->BSRR = 0xff00;            // turn all pins off (8..15 clr) (0..7 set)
  P1->BSRR = 0x0001;            // P1.0 on

  while(1) {
    P1->TOGGLER = BIT0 | BIT3;  // toggle P1.0 and P1.3

    volatile unsigned x=1000; do{} while(--x); // delay
  }

  return(0);
}

// vim: set ts=2 sw=2 expandtab :
