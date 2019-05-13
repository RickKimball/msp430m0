//------------------------------------------------------------------------
#include <inttypes.h>

//------------------------------------------------------------------------
// GPIO_Type - fake GPIO peripheral wired to 16 bit PA port pins

typedef struct {
    volatile unsigned GPIO_IN;
    volatile unsigned GPIO_OUT;
    volatile unsigned GPIO_DIR; /* 0-in 1-pushpull out */
} GPIO_Type ;

#define PERIPHERAL_BASE         0x40000000
#define P1_BASE                 0x40001000
#define P2_BASE                 (PERIPHERAL_BASE+0x2000)

GPIO_Type * const P1 = (GPIO_Type * const)P1_BASE;

//------------------------------------------------------------------------

int main ( void )
{
  P1->GPIO_DIR |= (1<<0) | (1<<3);    // P1.0 and P1.3 outputs
  P1->GPIO_OUT &= ~0xffff;            // turn all pins off
  P1->GPIO_OUT |= (1<<0);             // P1.0 on
  while(1) {
    P1->GPIO_OUT ^= (1<<0) | (1<<3);  // toggle P1.0 and P1.3

    volatile unsigned x=1000; do{} while(--x); // delay
  }

  return(0);
}

// vim: set ts=2 sw=2 expandtab :

