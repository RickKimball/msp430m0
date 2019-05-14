//--------------------------------------------------------------------------
// msp430m0.c - turn a fast msp430fr6989 into a slow msp430m0 w/thumbulator
/*--------------------------------------------------------------------------
   based on: https://github.com/dwelch67/thumbulator
   see copyright at bottom of file
 */

#include <msp430.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F_CPU 16095000L  /* measured */

uint32_t read32 ( uint32_t );
uint32_t read_register ( uint32_t );

#define ROMADDMASK 0x7fffL /* 32k */
#define RAMADDMASK 0x7fffL /* 32k */

#define ROMSIZE (ROMADDMASK+1)
#define RAMSIZE (RAMADDMASK+1)

__attribute__((section(".m0rom.bss")))
uint16_t rom[ROMSIZE>>1];

__attribute__((section(".m0ram.bss")))
uint16_t ram[RAMSIZE>>1];

#define CPSR_N (1L<<31)
#define CPSR_Z (1L<<30)
#define CPSR_C (1L<<29)
#define CPSR_V (1L<<28)
#define CPSR_Q (1L<<27)

uint32_t vcdcount;
uint32_t output_vcd;
FILE *fpvcd;

uint32_t systick_ctrl;
uint32_t systick_reload;
uint32_t systick_count;
uint32_t systick_calibrate;

uint32_t halfadd;
uint32_t cpsr;
uint32_t handler_mode;
uint32_t reg_norm[16]; //normal execution mode, do not have a thread mode

uint32_t instructions;
uint32_t fetches;
uint32_t reads;
uint32_t writes;
uint32_t systick_ints;

//-------------------------------------------------------------------
void dump_counters ( void )
{
#if 0
    printf("\n\n");
    printf("instructions %lu\n",instructions);
    printf("fetches      %lu\n",fetches);
    printf("reads        %lu\n",reads);
    printf("writes       %lu\n",writes);
    printf("memcycles    %lu\n",fetches+reads+writes);
    printf("systick_ints %lu\n",systick_ints);
#endif
}
//-------------------------------------------------------------------
uint32_t fetch16 ( uint32_t addr )
{
    uint32_t data;

    fetches++;

    switch(addr&0xF0000000)
    {
        case 0x00000000: //ROM
            addr&=ROMADDMASK;
            addr>>=1;
            data=rom[addr];
            return(data);

        case 0x20000000: //RAM
            addr&=RAMADDMASK;
            addr>>=1;
            data=ram[addr];
            return(data);
    }
    while(1); // fprintf(stderr,"fetch16(0x%08lX), abort pc = 0x%04lX\n",addr,read_register(15));
}
//-------------------------------------------------------------------
uint32_t fetch32 ( uint32_t addr )
{
    uint32_t data;

    _NOP();

    switch(addr&0xF0000000)
    {
        case 0x00000000: //ROM
            if(addr<0x50)
            {
                data=read32(addr);
                if(addr==0x00000000) return(data);
                if(addr==0x00000004) return(data);
                if(addr==0x0000003C) return(data);
                while(1); // fprintf(stderr,"fetch32(0x%08lX), abort pc = 0x%04lX\n",addr,read_register(15));
            }
        case 0x20000000: //RAM
            //data=fetch16(addr+0);
            //data|=((uint32_t)fetch16(addr+2))<<16;
            data=read32(addr);
            return(data);
    }
    while(1); //fprintf(stderr,"fetch32(0x%08lX), abort pc 0x%04lX\n",addr,read_register(15));
}

//-------------------------------------------------------------------
void write16 ( uint32_t addr, uint32_t data )
{

    writes++;


    switch(addr&0xF0000000)
    {
        case 0x20000000: //RAM
            addr&=RAMADDMASK;
            addr>>=1;
            ram[addr]=data&0xFFFF;
            return;
    }
    while(1); // fprintf(stderr,"write16(0x%08lX,0x%04lX), abort pc 0x%04lX\n",addr,data,read_register(15));
}

//-------------------------------------------------------------------

typedef struct {
  volatile uint32_t CR;         // control register
                                // 31 en, 30,29 mode, 8..16 port, 0..7 pin
  volatile uint32_t PERIODR;    // period register
  volatile uint32_t DUTYR;      // duty register 0..100
} PWM_Type;

PWM_Type PWM0;

void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
{
    uint8_t _P1 = P1OUT;

    if ( PWM0.DUTYR == 0) {
        P1OUT = _P1 & ~BIT0;
        TA0CCR0 += PWM0.PERIODR;
        goto ISRExit;
    }
    else if ( PWM0.DUTYR >= PWM0.PERIODR ) {
        P1OUT = _P1 | BIT0;
        TA0CCR0 += PWM0.PERIODR;
        goto ISRExit;
    }
    
    if ( P1OUT & BIT0 ) {
        TA0CCR0 += PWM0.PERIODR-PWM0.DUTYR;
    }
    else {
        TA0CCR0 += PWM0.DUTYR;
    }
    P1OUT = _P1 ^ BIT0; // toggle it

ISRExit:
    asm("nop");
}
 

//-------------------------------------------------------------------
void write32 ( uint32_t addr, uint32_t data )
{
    switch(addr&0xF0000000) {
    case 0xF0000000: //halt
        dump_counters();
        exit(0);

    case 0xE0000000: // periph
        if ( 0 )
            ;
        else if ( addr == 0xE000E010L ) {
            uint32_t old;

            old=systick_ctrl;
            systick_ctrl = data&0x00010007;
            if(((old&1)==0)&&(systick_ctrl&1))
            {
            //timer started, load count
            systick_count=systick_reload;
            }
        }
        else if ( addr == 0xE000E014UL ) {
            systick_reload=data&0x00FFFFFF;
        }
        else if ( addr == 0xE000E018UL ) {
            systick_count=data&0x00FFFFFF;
        }
        else if ( addr == 0xE000E01CUL ) {
            systick_calibrate=data&0x00FFFFFF;
        }
        return;

#if 0
    case 0xD0000000: //debug
        switch(addr&0xFF) {
        case 0x00:
            {
            fprintf(stderr,"[0x%08lX][0x%08lX] 0x%08lX\n",read_register(14),addr,data);
            return;
            }
        case 0x10:
            {
            printf("0x%08lX ",data);
            return;
            }
        case 0x20:
            {
            printf("0x%08lX\n",data);
            return;
            }
        }
 #endif

    case 0x20000000UL: // RAM
        write16(addr+0,(data>> 0)&0xFFFF);
        write16(addr+2,(data>>16)&0xFFFF);
        return;

    case 0x40000000UL:  // peripherals
        if ( 0 )
            ;
        else if ( addr == 0x40001000UL ) {  // GPIO IDR
            return;
        }
        else if ( addr == 0x40001004UL ) {  // GPIO ODR
            uint8_t set = data & 0xff;
            P1OUT = set;
            return;
        }
        else if ( addr == 0x40001008UL ) { // GPIO MODER
            uint8_t set = data & 0xff;
            P1DIR = set;
            return;
        }
        else if ( addr == 0x4000100CUL ) { // GPIO BSRR 16 bit 8..15 clr / 0..8 set
            uint8_t clr = (data >> 8) & 0xff;
            uint8_t set = data & 0xff;
            
            P1OUT = (P1OUT & ~clr) | set;
            return;
        }
        else if ( addr == 0x40001010UL ) { // GPIO TOGGLER 8 bit
            uint8_t set = data & 0xff;
            P1OUT ^= set;
            return;
        }
        else if ( addr == 0x40010000UL ) { // PWM CR
            if ( data & ( 1L<<31) ) {
                P1OUT |= (data & 0xff);
                P1DIR |= (data & 0xff);
            }
            else {
                P1OUT &= ~(data & 0xff);
                P1DIR &= ~(data & 0xff);
            }
            return;
        }
        else if ( addr == 0x40010004UL ) { // PWM PERIOD
            uint32_t T_PRE = (F_CPU / data / 65535L) + 1;
            PWM0.PERIODR = (F_CPU / T_PRE / data);
            PWM0.DUTYR = PWM0.PERIODR >> 1;

            TA0CCTL0 = CCIE;
            TA0CCR0 = PWM0.PERIODR;
            TA0CTL = TASSEL__SMCLK | MC__CONTINOUS | TACLR;

            return;
        }
        else if ( addr == 0x40010008UL ) { // PWM DUTY
            PWM0.DUTYR = (PWM0.PERIODR * data * 10 ) / 1000UL;
            return;
        }
    }

    while(1); //fprintf(stderr,"write32(0x%08lX,0x%08lX), abort pc 0x%04lX\n",addr,data,read_register(15));
}

//-----------------------------------------------------------------
uint32_t read16 ( uint32_t addr )
{
    uint32_t data;

    reads++;

    switch(addr&0xF0000000)
    {
        case 0x00000000: //ROM
            addr&=ROMADDMASK;
            addr>>=1;
            data=rom[addr];
            return(data);
        case 0x20000000: //RAM
            addr&=RAMADDMASK;
            addr>>=1;
            data=ram[addr];
            return(data);
    }
    while(1); //fprintf(stderr,"read16(0x%08lX), abort pc 0x%04lX\n",addr,read_register(15));
}

//-------------------------------------------------------------------
uint32_t read32 ( uint32_t addr )
{
    uint32_t data;

    _NOP();

    switch(addr&0xF0000000)
    {
        case 0x00000000: //ROM
        case 0x20000000: //RAM
            data =read16(addr+0);
            data|=((uint32_t)read16(addr+2))<<16;
            return(data);

        case 0x40000000:  // peripherals
            if ( 0 )
                ;
            else if ( addr == 0x40001000UL ) { // GPIO P1 IDR  8 bits
                data = P1IN;
                return data;
            }
            else if ( addr == 0x40001004UL ) { // GPIO P1 ODR  8 bits
                data = P1OUT;
                return data;
            }
            else if ( addr == 0x40001008UL ) { // GPIO P1 MODER 8 bits 
                data = P1DIR;
                return data;
            }
            else if ( addr == 0x4000100CUL ) { // GPIO P1 BSRR  - write only
                return 0;
            }
            else if ( addr == 0x40001010UL ) { // GPIO P1 TOGGLER - write only
                return 0;
            }
            else if ( addr == 0x40010000UL ) { // PWM CR
                return 0;
            }
            else if ( addr == 0x40010004UL ) { // PWM PERIOD
                return 0;
            }
            else if ( addr == 0x40010008UL ) { // PWM DUTY
                return 0;
            }

            break;

        case 0xE0000000:
        {
            if ( 0 )
                ;
            else if ( addr == 0xE000E010UL ) 
            {
                data = systick_ctrl;
                systick_ctrl&=(~0x00010000UL);
                return(data);
            }
            else if ( addr == 0xE000E014UL )
            {
                data=systick_reload;
                return(data);
            }
            else if ( addr == 0xE000E018UL )
            {
                data=systick_count;
                return(data);
            }
            else if ( addr == 0xE000E01CUL )
            {
                data=systick_calibrate;
                return(data);
            }
        }
        break;

    }
    while(1); // fprintf(stderr,"read32(0x%08lX), abort pc 0x%04lX\n",addr,read_register(15));
}

//-------------------------------------------------------------------
uint32_t read_register ( uint32_t reg )
{
    uint32_t data;

    reg&=0xF;
    data=reg_norm[reg];
    if(reg==15)
    {
        if(data&1)
        {
            while(1); // fprintf(stderr,"pc has lsbit set 0x%08lX\n",data);
        }
        data&=~1;
    }
    return(data);
}

//-------------------------------------------------------------------
void write_register ( uint32_t reg, uint32_t data )
{
    reg&=0xF;
    if(reg==15) data&=~1;
    reg_norm[reg]=data;

#if 0
if(output_vcd)
{
    uint32_t vv;
    fprintf(fpvcd,"b");
    for(vv=0x80000000;vv;vv>>=1)
    {
        if(vv&data) fprintf(fpvcd,"1"); else fprintf(fpvcd,"0");
    }
    fprintf(fpvcd," r%lu\n",reg);
}
#endif

}

//-------------------------------------------------------------------
void do_zflag ( uint32_t x )
{
    if(x==0) cpsr|=CPSR_Z; else cpsr&=~CPSR_Z;
}

//-------------------------------------------------------------------
void do_nflag ( uint32_t x )
{
    if(x&0x80000000) cpsr|=CPSR_N; else cpsr&=~CPSR_N;
}

//-------------------------------------------------------------------
void do_cflag ( uint32_t a, uint32_t b, uint32_t c )
{
    uint32_t rc;

    cpsr&=~CPSR_C;
    rc=(a&0x7FFFFFFF)+(b&0x7FFFFFFF)+c; //carry in
    rc = (rc>>31)+(a>>31)+(b>>31);  //carry out
    if(rc&2) cpsr|=CPSR_C;
}

//-------------------------------------------------------------------
void do_vflag ( uint32_t a, uint32_t b, uint32_t c )
{
    uint32_t rc;
    uint32_t rd;

    cpsr&=~CPSR_V;
    rc=(a&0x7FFFFFFF)+(b&0x7FFFFFFF)+c; //carry in
    rc>>=31; //carry in in lsbit
    rd=(rc&1)+((a>>31)&1)+((b>>31)&1); //carry out
    rd>>=1; //carry out in lsbit
    rc=(rc^rd)&1; //if carry in != carry out then signed overflow
    if(rc) cpsr|=CPSR_V;
}

//-------------------------------------------------------------------
void do_cflag_bit ( uint32_t x )
{
   if(x) cpsr|=CPSR_C; else cpsr&=~CPSR_C;
}

//-------------------------------------------------------------------
void do_vflag_bit ( uint32_t x )
{
   if(x) cpsr|=CPSR_V; else cpsr&=~CPSR_V;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
int execute ( void )
{
    uint32_t pc;
    uint32_t sp;
    uint32_t inst;

    uint32_t ra,rb,rc;
    uint32_t rm,rd,rn,rs;
    uint32_t op;

    pc=read_register(15);

    if(handler_mode)
    {
        if((pc&0xF0000000)==0xF0000000)
        {
            uint32_t sp;

            handler_mode = 0;
//fprintf(stderr,"--leaving handler\n");
            sp=read_register(13);
            write_register(0,read32(sp)); sp+=4;
            write_register(1,read32(sp)); sp+=4;
            write_register(2,read32(sp)); sp+=4;
            write_register(3,read32(sp)); sp+=4;
            write_register(12,read32(sp)); sp+=4;
            write_register(14,read32(sp)); sp+=4;
            pc=read32(sp); sp+=4;
            cpsr=read32(sp); sp+=4;
            write_register(13,sp);
        }
    }
    if(systick_ctrl&1)
    {
        if(systick_count)
        {
            systick_count--;
        }
        else
        {
            systick_count=systick_reload;
            systick_ctrl|=0x00010000;
        }
    }

    if((systick_ctrl&3)==3)
    {
        if(systick_ctrl&0x00010000)
        {
            if(handler_mode==0)
            {
                uint32_t sp;

                systick_ints++;
//fprintf(stderr,"--- enter systick handler\n");
                sp=read_register(13);
                sp-=4; write32(sp,cpsr);
                sp-=4; write32(sp,pc);
                sp-=4; write32(sp,read_register(14));
                sp-=4; write32(sp,read_register(12));
                sp-=4; write32(sp,read_register(3));
                sp-=4; write32(sp,read_register(2));
                sp-=4; write32(sp,read_register(1));
                sp-=4; write32(sp,read_register(0));
                write_register(13,sp);
                pc=fetch32(0x0000003C); //systick vector
                pc+=2;
                //write_register(14,0xFFFFFF00);
                write_register(14,0xFFFFFFF9);

                handler_mode=1;
            }
        }
    }

    inst=fetch16(pc-2);
    pc+=2;
    write_register(15,pc);

    instructions++;

    //ADC
    if((inst&0xFFC0)==0x4140)
    {
        rd=(inst>>0)&0x07;
        rm=(inst>>3)&0x07;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra+rb;
        if(cpsr&CPSR_C) rc++;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        if(cpsr&CPSR_C) { do_cflag(ra,rb,1); do_vflag(ra,rb,1); }
        else            { do_cflag(ra,rb,0); do_vflag(ra,rb,0); }
        return(0);
    }

    //ADD(1) small immediate two registers
    if((inst&0xFE00)==0x1C00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rb=(inst>>6)&0x7;
        if(rb)
        {
            ra=read_register(rn);
            rc=ra+rb;
//fprintf(stderr,"0x%08lX = 0x%08lX + 0x%08lX\n",rc,ra,rb);
            write_register(rd,rc);
            do_nflag(rc);
            do_zflag(rc);
            do_cflag(ra,rb,0);
            do_vflag(ra,rb,0);
            return(0);
        }
        else
        {
            //this is a mov
        }
    }

    //ADD(2) big immediate one register
    if((inst&0xF800)==0x3000)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x7;
        ra=read_register(rd);
        rc=ra+rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,rb,0);
        do_vflag(ra,rb,0);
        return(0);
    }

    //ADD(3) three registers
    if((inst&0xFE00)==0x1800)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra+rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,rb,0);
        do_vflag(ra,rb,0);
        return(0);
    }

    //ADD(4) two registers one or both high no flags
    if((inst&0xFF00)==0x4400)
    {
        if((inst>>6)&3)
        {
            //UNPREDICTABLE
        }
        rd=(inst>>0)&0x7;
        rd|=(inst>>4)&0x8;
        rm=(inst>>3)&0xF;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra+rb;
        if(rd==15)
        {
            if((rc&1)==0)
            {
                while(1); // fprintf(stderr,"add pc,... produced an arm address 0x%08lX 0x%08lX\n",pc,rc);
            }
            rc&=~1; //write_register may do this as well
            rc+=2; //The program counter is special
        }
//fprintf(stderr,"0x%08lX = 0x%08lX + 0x%08lX\n",rc,ra,rb);
        write_register(rd,rc);
        return(0);
    }

    //ADD(5) rd = pc plus immediate
    if((inst&0xF800)==0xA000)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x7;
        rb<<=2;
        ra=read_register(15);
        rc=(ra&(~3))+rb;
        write_register(rd,rc);
        return(0);
    }

    //ADD(6) rd = sp plus immediate
    if((inst&0xF800)==0xA800)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x7;
        rb<<=2;
        ra=read_register(13);
        rc=ra+rb;
        write_register(rd,rc);
        return(0);
    }

    //ADD(7) sp plus immediate
    if((inst&0xFF80)==0xB000)
    {
        rb=(inst>>0)&0x7F;
        rb<<=2;
        ra=read_register(13);
        rc=ra+rb;
        write_register(13,rc);
        return(0);
    }

    //AND
    if((inst&0xFFC0)==0x4000)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra&rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //ASR(1) two register immediate
    if((inst&0xF800)==0x1000)
    {
        rd=(inst>>0)&0x07;
        rm=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rc=read_register(rm);
        if(rb==0)
        {
            if(rc&0x80000000)
            {
                do_cflag_bit(1);
                rc=~0;
            }
            else
            {
                do_cflag_bit(0);
                rc=0;
            }
        }
        else
        {
            do_cflag_bit(rc&(1<<(rb-1)));
            ra=rc&0x80000000;
            rc>>=rb;
            if(ra) //asr, sign is shifted in
            {
                rc|=(~0)<<(32-rb);
            }
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //ASR(2) two register
    if((inst&0xFFC0)==0x4100)
    {
        rd=(inst>>0)&0x07;
        rs=(inst>>3)&0x07;
        rc=read_register(rd);
        rb=read_register(rs);
        rb&=0xFF;
        if(rb==0)
        {
        }
        else if(rb<32)
        {
            do_cflag_bit(rc&(1<<(rb-1)));
            ra=rc&0x80000000;
            rc>>=rb;
            if(ra) //asr, sign is shifted in
            {
                rc|=(~0)<<(32-rb);
            }
        }
        else
        {
            if(rc&0x80000000)
            {
                do_cflag_bit(1);
                rc=(~0);
            }
            else
            {
                do_cflag_bit(0);
                rc=0;
            }
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //B(1) conditional branch
    if((inst&0xF000)==0xD000)
    {
        rb=(inst>>0)&0xFF;
        if(rb&0x80) rb|=(~0)<<8;
        op=(inst>>8)&0xF;
        rb<<=1;
        rb+=pc;
        rb+=2;
        switch(op)
        {
            case 0x0: //b eq  z set
                if(cpsr&CPSR_Z)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x1: //b ne  z clear
                if(!(cpsr&CPSR_Z))
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x2: //b cs c set
                if(cpsr&CPSR_C)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x3: //b cc c clear
                if(!(cpsr&CPSR_C))
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x4: //b mi n set
                if(cpsr&CPSR_N)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x5: //b pl n clear
                if(!(cpsr&CPSR_N))
                {
                    write_register(15,rb);
                }
                return(0);


            case 0x6: //b vs v set
                if(cpsr&CPSR_V)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x7: //b vc v clear
                if(!(cpsr&CPSR_V))
                {
                    write_register(15,rb);
                }
                return(0);


            case 0x8: //b hi c set z clear
                if((cpsr&CPSR_C)&&(!(cpsr&CPSR_Z)))
                {
                    write_register(15,rb);
                }
                return(0);

            case 0x9: //b ls c clear or z set
                if((cpsr&CPSR_Z)||(!(cpsr&CPSR_C)))
                {
                    write_register(15,rb);
                }
                return(0);

            case 0xA: //b ge N == V
                ra=0;
                if(  (cpsr&CPSR_N) &&  (cpsr&CPSR_V) ) ra++;
                if((!(cpsr&CPSR_N))&&(!(cpsr&CPSR_V))) ra++;
                if(ra)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0xB: //b lt N != V
                ra=0;
                if((!(cpsr&CPSR_N))&&(cpsr&CPSR_V)) ra++;
                if((!(cpsr&CPSR_V))&&(cpsr&CPSR_N)) ra++;
                if(ra)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0xC: //b gt Z==0 and N == V
                ra=0;
                if(  (cpsr&CPSR_N) &&  (cpsr&CPSR_V) ) ra++;
                if((!(cpsr&CPSR_N))&&(!(cpsr&CPSR_V))) ra++;
                if(cpsr&CPSR_Z) ra=0;
                if(ra)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0xD: //b le Z==1 or N != V
                ra=0;
                if((!(cpsr&CPSR_N))&&(cpsr&CPSR_V)) ra++;
                if((!(cpsr&CPSR_V))&&(cpsr&CPSR_N)) ra++;
                if(cpsr&CPSR_Z) ra++;
                if(ra)
                {
                    write_register(15,rb);
                }
                return(0);

            case 0xE:
                //undefined instruction
                break;
            case 0xF:
                //swi
                break;
        }
    }

    //B(2) unconditional branch
    if((inst&0xF800)==0xE000)
    {
        rb=(inst>>0)&0x7FF;
        if(rb&(1<<10)) rb|=(~0)<<11;
        rb<<=1;
        rb+=pc;
        rb+=2;
        write_register(15,rb);
        return(0);
    }

    //BIC
    if((inst&0xFFC0)==0x4380)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra&(~rb);
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //BKPT
    if((inst&0xFF00)==0xBE00)
    {
        rb=(inst>>0)&0xFF;
        // fprintf(stderr,"bkpt 0x%02lX\n",rb);
        return(1);
    }

    //BL/BLX(1)
    if((inst&0xE000)==0xE000) //BL,BLX
    {
        if((inst&0x1800)==0x1000) //H=b10
        {
            rb=inst&((1<<11)-1);
            if(rb&1<<10) rb|=(~((1<<11)-1)); //sign extend
            rb<<=12;
            rb+=pc;
            write_register(14,rb);
            return(0);
        }
        else
        if((inst&0x1800)==0x1800) //H=b11
        {
            //branch to thumb
            rb=read_register(14);
            rb+=(inst&((1<<11)-1))<<1;;
            rb+=2;

            write_register(14,(pc-2)|1);
            write_register(15,rb);
            return(0);
        }
        else
        if((inst&0x1800)==0x0800) //H=b01
        {
            //fprintf(stderr,"cannot branch to arm 0x%08lX 0x%04lX\n",pc,inst);
            //return(1);
            //branch to thumb
            rb=read_register(14);
            rb+=(inst&((1<<11)-1))<<1;;
            rb&=0xFFFFFFFC;
            rb+=2;

//printf("hello\n");

            write_register(14,(pc-2)|1);
            write_register(15,rb);
            return(0);



        }
    }

    //BLX(2)
    if((inst&0xFF87)==0x4780)
    {
        rm=(inst>>3)&0xF;
        rc=read_register(rm);
//fprintf(stderr,"blx r%lu 0x%lX 0x%lX\n",rm,rc,pc);
        rc+=2;
        if(rc&1)
        {
            write_register(14,(pc-2)|1);
            rc&=~1;
            write_register(15,rc);
            return(0);
        }
        else
        {
            while(1); // fprintf(stderr,"cannot branch to arm 0x%08lX 0x%04lX\n",pc,inst);
            return(1);
        }
    }

    //BX
    if((inst&0xFF87)==0x4700)
    {
        rm=(inst>>3)&0xF;
        rc=read_register(rm);
        rc+=2;
//fprintf(stderr,"bx r%lu 0x%lX 0x%lX\n",rm,rc,pc);
        if(rc&1)
        {
            rc&=~1;
            write_register(15,rc);
            return(0);
        }
        else
        {
            while(1); // fprintf(stderr,"cannot branch to arm 0x%08lX 0x%04lX\n",pc,inst);
            return(1);
        }
    }

    //CMN
    if((inst&0xFFC0)==0x42C0)
    {
        rn=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra+rb;
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,rb,0);
        do_vflag(ra,rb,0);
        return(0);
    }

    //CMP(1) compare immediate
    if((inst&0xF800)==0x2800)
    {
        rb=(inst>>0)&0xFF;
        rn=(inst>>8)&0x07;
        ra=read_register(rn);
        rc=ra-rb;
//fprintf(stderr,"0x%08lX 0x%08lX\n",ra,rb);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //CMP(2) compare register
    if((inst&0xFFC0)==0x4280)
    {
        rn=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra-rb;
//fprintf(stderr,"0x%08lX 0x%08lX\n",ra,rb);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //CMP(3) compare high register
    if((inst&0xFF00)==0x4500)
    {
        if(((inst>>6)&3)==0x0)
        {
            //UNPREDICTABLE
        }
        rn=(inst>>0)&0x7;
        rn|=(inst>>4)&0x8;
        if(rn==0xF)
        {
            //UNPREDICTABLE
        }
        rm=(inst>>3)&0xF;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra-rb;
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //CPS
    if((inst&0xFFE8)==0xB660)
    {
        return(1);
    }

    //CPY copy high register
    if((inst&0xFFC0)==0x4600)
    {
        //same as mov except you can use both low registers
        //going to let mov handle high registers
        rd=(inst>>0)&0x7; //mov handles the high registers
        rm=(inst>>3)&0x7; //mov handles the high registers
        rc=read_register(rm);
        //if(rd==15) //mov handles the high registers like r15
        //{
            //rc&=~1;
            //rc+=2; //The program counter is special
        //}
        write_register(rd,rc);
        return(0);
    }

    //EOR
    if((inst&0xFFC0)==0x4040)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra^rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //LDMIA
    if((inst&0xF800)==0xC800)
    {
        rn=(inst>>8)&0x7;
        sp=read_register(rn);
        for(ra=0,rb=0x01;rb;rb=(rb<<1)&0xFF,ra++)
        {
            if(inst&rb)
            {
                write_register(ra,read32(sp));
                sp+=4;
            }
        }
        //there is a write back exception.
        if((inst&(1<<rn))==0) write_register(rn,sp);
        return(0);
    }

    //LDR(1) two register immediate
    if((inst&0xF800)==0x6800)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb<<=2;
        rb=read_register(rn)+rb;
        rc=read32(rb);
        write_register(rd,rc);
        return(0);
    }

    //LDR(2) three register
    if((inst&0xFE00)==0x5800)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read32(rb);
        write_register(rd,rc);
        return(0);
    }

    //LDR(3)
    if((inst&0xF800)==0x4800)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x07;
        rb<<=2;
        ra=read_register(15);
        ra&=~3;
        rb+=ra;
        rc=read32(rb);
        write_register(rd,rc);
        return(0);
    }

    //LDR(4)
    if((inst&0xF800)==0x9800)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x07;
        rb<<=2;
        ra=read_register(13);
        //ra&=~3;
        rb+=ra;
        rc=read32(rb);
        write_register(rd,rc);
        return(0);
    }

    //LDRB(1)
    if((inst&0xF800)==0x7800)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb=read_register(rn)+rb;
        rc=read16(rb&(~1));
        if(rb&1)
        {
            rc>>=8;
        }
        else
        {
        }
        write_register(rd,rc&0xFF);
        return(0);
    }

    //LDRB(2)
    if((inst&0xFE00)==0x5C00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read16(rb&(~1));
        if(rb&1)
        {
            rc>>=8;
        }
        else
        {
        }
        write_register(rd,rc&0xFF);
        return(0);
    }

    //LDRH(1)
    if((inst&0xF800)==0x8800)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb<<=1;
        rb=read_register(rn)+rb;
        rc=read16(rb);
        write_register(rd,rc&0xFFFF);
        return(0);
    }

    //LDRH(2)
    if((inst&0xFE00)==0x5A00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read16(rb);
        write_register(rd,rc&0xFFFF);
        return(0);
    }

    //LDRSB
    if((inst&0xFE00)==0x5600)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read16(rb&(~1));
        if(rb&1)
        {
            rc>>=8;
        }
        else
        {
        }
        rc&=0xFF;
        if(rc&0x80) rc|=((~0)<<8);
        write_register(rd,rc);
        return(0);
    }

    //LDRSH
    if((inst&0xFE00)==0x5E00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read16(rb);
        rc&=0xFFFF;
        if(rc&0x8000L) rc|=((~0L)<<16);
        write_register(rd,rc);
        return(0);
    }

    //LSL(1)
    if((inst&0xF800)==0x0000)
    {
        rd=(inst>>0)&0x07;
        rm=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rc=read_register(rm);
        if(rb==0)
        {
            //if immed_5 == 0
            //C unnaffected
            //result not shifted
        }
        else
        {
            //else immed_5 > 0
            do_cflag_bit(rc&(1<<(32-rb)));
            rc<<=rb;
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //LSL(2) two register
    if((inst&0xFFC0)==0x4080)
    {
        rd=(inst>>0)&0x07;
        rs=(inst>>3)&0x07;
        rc=read_register(rd);
        rb=read_register(rs);
        rb&=0xFF;
        if(rb==0)
        {
        }
        else if(rb<32)
        {
            do_cflag_bit(rc&(1<<(32-rb)));
            rc<<=rb;
        }
        else if(rb==32)
        {
            do_cflag_bit(rc&1);
            rc=0;
        }
        else
        {
            do_cflag_bit(0);
            rc=0;
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //LSR(1) two register immediate
    if((inst&0xF800)==0x0800)
    {
        rd=(inst>>0)&0x07;
        rm=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rc=read_register(rm);
        if(rb==0)
        {
            do_cflag_bit(rc&0x80000000);
            rc=0;
        }
        else
        {
            do_cflag_bit(rc&(1<<(rb-1)));
            rc>>=rb;
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //LSR(2) two register
    if((inst&0xFFC0)==0x40C0)
    {
        rd=(inst>>0)&0x07;
        rs=(inst>>3)&0x07;
        rc=read_register(rd);
        rb=read_register(rs);
        rb&=0xFF;
        if(rb==0)
        {
        }
        else if(rb<32)
        {
            do_cflag_bit(rc&(1<<(rb-1)));
            rc>>=rb;
        }
        else if(rb==32)
        {
            do_cflag_bit(rc&0x80000000);
            rc=0;
        }
        else
        {
            do_cflag_bit(0);
            rc=0;
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //MOV(1) immediate
    if((inst&0xF800)==0x2000)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x07;
        write_register(rd,rb);
        do_nflag(rb);
        do_zflag(rb);
        return(0);
    }

    //MOV(2) two low registers
    if((inst&0xFFC0)==0x1C00)
    {
        rd=(inst>>0)&7;
        rn=(inst>>3)&7;
        rc=read_register(rn);
//fprintf(stderr,"0x%08lX\n",rc);
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag_bit(0);
        do_vflag_bit(0);
        return(0);
    }

    //MOV(3)
    if((inst&0xFF00)==0x4600)
    {
        rd=(inst>>0)&0x7;
        rd|=(inst>>4)&0x8;
        rm=(inst>>3)&0xF;
        rc=read_register(rm);
        if((rd==14)&&(rm==15))
        {
            //printf("mov lr,pc warning 0x%08lX\n",pc-2);
            //rc|=1;
        }
        if(rd==15)
        {
            //if((rc&1)==0)
            //{
                //fprintf(stderr,"cpy or mov pc,... produced an ARM address 0x%08lX 0x%08lX\n",pc,rc);
                //exit(1);
            //}
            rc&=~1; //write_register may do this as well
            rc+=2; //The program counter is special
        }
        write_register(rd,rc);
        return(0);
    }

    //MUL
    if((inst&0xFFC0)==0x4340)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra*rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //MVN
    if((inst&0xFFC0)==0x43C0)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=(~ra);
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //NEG
    if((inst&0xFFC0)==0x4240)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=0-ra;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(0,~ra,1);
        do_vflag(0,~ra,1);
        return(0);
    }

    //ORR
    if((inst&0xFFC0)==0x4300)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra|rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }


    //POP
    if((inst&0xFE00)==0xBC00)
    {
        sp=read_register(13);
        for(ra=0,rb=0x01;rb;rb=(rb<<1)&0xFF,ra++)
        {
            if(inst&rb)
            {
                write_register(ra,read32(sp));
                sp+=4;
            }
        }
        if(inst&0x100)
        {
            rc=read32(sp);
            if((rc&1)==0)
            {
                while(1); // fprintf(stderr,"pop {rc} with an ARM address pc 0x%08lX popped 0x%08lX\n",pc,rc);
                //exit(1);
                rc&=~1;
            }
            rc+=2;
            write_register(15,rc);
            sp+=4;
        }
        write_register(13,sp);
        return(0);
    }

    //PUSH
    if((inst&0xFE00)==0xB400)
    {
        sp=read_register(13);
        for(ra=0,rb=0x01,rc=0;rb;rb=(rb<<1)&0xFF,ra++)
        {
            if(inst&rb)
            {
                rc++;
            }
        }
        if(inst&0x100) rc++;
        rc<<=2;
        sp-=rc;
        rd=sp;
        for(ra=0,rb=0x01;rb;rb=(rb<<1)&0xFF,ra++)
        {
            if(inst&rb)
            {
                write32(rd,read_register(ra));
                rd+=4;
            }
        }
        if(inst&0x100)
        {
            rc=read_register(14);
            write32(rd,rc); //read_register(14));

            if((rc&1)==0)
            {
                while(1); // fprintf(stderr,"push {lr} with an ARM address pc 0x%08lX popped 0x%08lX\n",pc,rc);
//                exit(1);
            }


        }
        write_register(13,sp);
        return(0);
    }

    //REV
    if((inst&0xFFC0)==0xBA00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        ra=read_register(rn);
        rc =((ra>> 0)&0xFF)<<24;
        rc|=((ra>> 8)&0xFF)<<16;
        rc|=((ra>>16)&0xFF)<< 8;
        rc|=((ra>>24)&0xFF)<< 0;
        write_register(rd,rc);
        return(0);
    }

    //REV16
    if((inst&0xFFC0)==0xBA40)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        ra=read_register(rn);
        rc =((ra>> 0)&0xFF)<< 8;
        rc|=((ra>> 8)&0xFF)<< 0;
        rc|=((ra>>16)&0xFF)<<24;
        rc|=((ra>>24)&0xFF)<<16;
        write_register(rd,rc);
        return(0);
    }

    //REVSH
    if((inst&0xFFC0)==0xBAC0)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        ra=read_register(rn);
        rc =((ra>> 0)&0xFF)<< 8;
        rc|=((ra>> 8)&0xFF)<< 0;
        if(rc&0x8000) rc|=0xFFFF0000;
        else          rc&=0x0000FFFF;
        write_register(rd,rc);
        return(0);
    }

    //ROR
    if((inst&0xFFC0)==0x41C0)
    {
        rd=(inst>>0)&0x7;
        rs=(inst>>3)&0x7;
        rc=read_register(rd);
        ra=read_register(rs);
        ra&=0xFF;
        if(ra==0)
        {
        }
        else
        {
            ra&=0x1F;
            if(ra==0)
            {
                do_cflag_bit(rc&0x80000000);
            }
            else
            {
                do_cflag_bit(rc&(1<<(ra-1)));
                rb=rc<<(32-ra);
                rc>>=ra;
                rc|=rb;
            }
        }
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //SBC
    if((inst&0xFFC0)==0x4180)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rd);
        rb=read_register(rm);
        rc=ra-rb;
        if(!(cpsr&CPSR_C)) rc--;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        if(cpsr&CPSR_C)
        {
            do_cflag(ra,~rb,1);
            do_vflag(ra,~rb,1);
        }
        else
        {
            do_cflag(ra,~rb,0);
            do_vflag(ra,~rb,0);
        }
        return(0);
    }

    //SETEND
    if((inst&0xFFF7)==0xB650)
    {
        while(1); // fprintf(stderr,"setend not implemented\n");
        return(1);
    }

    //STMIA
    if((inst&0xF800)==0xC000)
    {
        rn=(inst>>8)&0x7;
        sp=read_register(rn);
        for(ra=0,rb=0x01;rb;rb=(rb<<1)&0xFF,ra++)
        {
            if(inst&rb)
            {
                write32(sp,read_register(ra));
                sp+=4;
            }
        }
        write_register(rn,sp);
        return(0);
    }

    //STR(1)
    if((inst&0xF800)==0x6000)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb<<=2;
        rb=read_register(rn)+rb;
        rc=read_register(rd);
        write32(rb,rc);
        return(0);
    }

    //STR(2)
    if((inst&0xFE00)==0x5000)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read_register(rd);
        write32(rb,rc);
        return(0);
    }

    //STR(3)
    if((inst&0xF800)==0x9000)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x07;
        rb<<=2;
        rb=read_register(13)+rb;
//fprintf(stderr,"0x%08lX\n",rb);
        rc=read_register(rd);
        write32(rb,rc);
        return(0);
    }

    //STRB(1)
    if((inst&0xF800)==0x7000)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb=read_register(rn)+rb;
        rc=read_register(rd);
        ra=read16(rb&(~1));
        if(rb&1)
        {
            ra&=0x00FF;
            ra|=rc<<8;
        }
        else
        {
            ra&=0xFF00;
            ra|=rc&0x00FF;
        }
        write16(rb&(~1),ra&0xFFFF);
        return(0);
    }

    //STRB(2)
    if((inst&0xFE00)==0x5400)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read_register(rd);
        ra=read16(rb&(~1));
        if(rb&1)
        {
            ra&=0x00FF;
            ra|=rc<<8;
        }
        else
        {
            ra&=0xFF00;
            ra|=rc&0x00FF;
        }
        write16(rb&(~1),ra&0xFFFF);
        return(0);
    }

    //STRH(1)
    if((inst&0xF800)==0x8000)
    {
        rd=(inst>>0)&0x07;
        rn=(inst>>3)&0x07;
        rb=(inst>>6)&0x1F;
        rb<<=1;
        rb=read_register(rn)+rb;
        rc=read_register(rd);
        write16(rb,rc&0xFFFF);
        return(0);
    }

    //STRH(2)
    if((inst&0xFE00)==0x5200)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        rb=read_register(rn)+read_register(rm);
        rc=read_register(rd);
        write16(rb,rc&0xFFFF);
        return(0);
    }

    //SUB(1)
    if((inst&0xFE00)==0x1E00)
    {
        rd=(inst>>0)&7;
        rn=(inst>>3)&7;
        rb=(inst>>6)&7;
        ra=read_register(rn);
        rc=ra-rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //SUB(2)
    if((inst&0xF800)==0x3800)
    {
        rb=(inst>>0)&0xFF;
        rd=(inst>>8)&0x07;
        ra=read_register(rd);
        rc=ra-rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //SUB(3)
    if((inst&0xFE00)==0x1A00)
    {
        rd=(inst>>0)&0x7;
        rn=(inst>>3)&0x7;
        rm=(inst>>6)&0x7;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra-rb;
        write_register(rd,rc);
        do_nflag(rc);
        do_zflag(rc);
        do_cflag(ra,~rb,1);
        do_vflag(ra,~rb,1);
        return(0);
    }

    //SUB(4)
    if((inst&0xFF80)==0xB080)
    {
        rb=inst&0x7F;
        rb<<=2;
        ra=read_register(13);
        ra-=rb;
        write_register(13,ra);
        return(0);
    }

    //SWI
    if((inst&0xFF00)==0xDF00)
    {
        rb=inst&0xFF;

        if((inst&0xFF)==0xCC)
        {
            write_register(0,cpsr);
            return(0);
        }
        else
        {
            // fprintf(stderr,"\n\nswi 0x%02lX\n",rb);
            return(1);
        }
    }

    //SXTB
    if((inst&0xFFC0)==0xB240)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=ra&0xFF;
        if(rc&0x80) rc|=(~0)<<8;
        write_register(rd,rc);
        return(0);
    }

    //SXTH
    if((inst&0xFFC0)==0xB200)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=ra&0xFFFF;
        if(rc&0x8000L) rc|=(~0L)<<16;
        write_register(rd,rc);
        return(0);
    }

    //TST
    if((inst&0xFFC0)==0x4200)
    {
        rn=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rn);
        rb=read_register(rm);
        rc=ra&rb;
        do_nflag(rc);
        do_zflag(rc);
        return(0);
    }

    //UXTB
    if((inst&0xFFC0)==0xB2C0)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=ra&0xFF;
        write_register(rd,rc);
        return(0);
    }

    //UXTH
    if((inst&0xFFC0)==0xB280)
    {
        rd=(inst>>0)&0x7;
        rm=(inst>>3)&0x7;
        ra=read_register(rm);
        rc=ra&0xFFFF;
        write_register(rd,rc);
        return(0);
    }

    while(1); // fprintf(stderr,"invalid instruction 0x%08lX 0x%04lX\n",pc-4,inst);
    return(1);
}

//-------------------------------------------------------------------
int reset ( void )
{
    memset(ram,0xFF,sizeof(ram));

    systick_ctrl=0x00000004;
    systick_reload=0x00000000;
    systick_count=0x00000000;
    systick_calibrate=0x00ABCDEF;
    handler_mode=0;
    cpsr=0;

    reg_norm[13]=fetch32(0x00000000); //cortex-m
    reg_norm[14]=0xFFFFFFFF;
    reg_norm[15]=fetch32(0x00000004); //cortex-m
    if((reg_norm[15]&1)==0)
    {
        while(1); // fprintf(stderr,"reset vector with an ARM address 0x%08lX\n",reg_norm[15]);
    }
    reg_norm[15]&=~1;
    reg_norm[15]+=2;

    instructions=0;
    fetches=0;
    reads=0;
    writes=0;

    return(0);
}

//-------------------------------------------------------------------
int run ( void )
{
    reset();
    while(1)
    {
        if(output_vcd)
        {
            // fprintf(fpvcd,"#%lu\n",vcdcount++);
        }
        if(execute()) break;
    }
    dump_counters();
    return(0);
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
int main ()
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL1 = DCOFSEL_4 | DCORSEL;             // Set DCO to 16MHz

    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL0_H = 0;                             // Lock CS registers

#if 0
    P7DIR |= 1<<4;
    P7SEL1 |= 1 << 4;
    P7SEL0 |= 1 << 4;
    // LCD_C_BASE  &= ~(1<<4);
#endif
    

#if 0
    PADIR |= 0xffff;
    PAOUT |= 0x1;
    unsigned n=4;
    do {
        volatile unsigned long x=10000L/4;
        while(--x);

        PAOUT ^= 0x1;
    } while(--n);

    memset(rom,0xFF,sizeof(rom)); 
#endif
    memset(ram,0x00,sizeof(ram));
    __enable_interrupt();
    run();
    return(0);
}

//-------------------------------------------------------------------------
//
// Copyright (c) 2010 David Welch dwelch@dwelch.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------

/* vim: set ts=2 sw=2 expandtab : */

