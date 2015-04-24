//ADC pin: P2.1

#include "msp430x22x4.h"
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "vlo_rand.h"
voidlinkTo(void);
voidcreateRandomAddress();
__no_init volatile char Flash_Addr[4] @ 0x10F0; // Flash address set randomly
void main (void)
{
addr_tlAddr;
  WDTCTL = WDTPW + WDTHOLD;                 // Stop Watchdog timer
                                            // delay function to ensure proper startup before SimpliciTI increases DCO
  __delay_cycles(1000);
  // SimpliciTI will change port pin settings as well
  P1DIR = 0xFF;
  P1OUT = 0x00;
  P2DIR = 0x27;
  P2OUT = 0x00;
  P3DIR = 0xC0;
  P3OUT = 0x00;
  P4DIR = 0xFF;
  P4OUT = 0x00;
BSP_Init();
if(Flash_Addr[0] == 0xFF &&
Flash_Addr[1] == 0xFF &&
Flash_Addr[2] == 0xFF &&
Flash_Addr[3] == 0xFF )
  {
createRandomAddress();  //Set Random device address at initial startup
  }

lAddr.addr[0]=Flash_Addr[0];
lAddr.addr[1]=Flash_Addr[1];
lAddr.addr[2]=Flash_Addr[2];
lAddr.addr[3]=Flash_Addr[3];
SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
BCSCTL1 = CALBC1_8MHZ;
DCOCTL = CALDCO_8MHZ;
BCSCTL3 |= LFXT1S_2;
TACCTL0 = CCIE;
TACCR0 = 12000;
TACTL = TASSEL_1 + MC_1; 

// keep trying to join until successful. Toggle LED red
while (SMPL_NO_JOIN == SMPL_Init((uint8_t (*)(linkID_t))0))
  {
    BSP_TOGGLE_LED1();     //red
    __bis_SR_register(LPM3_bits + GIE);     // LPM3 with interrupts enabled
  }
  // unconditional link to Access Point (Receiver)
  //which is listening due to successful join.
linkTo();
}
voidcreateRandomAddress()
{
unsignedint rand, rand2;

do {
rand = TI_getRandomIntegerFromVLO(); // first byte cannot be 0x00 of 0xFF
  }
while( (rand & 0xFF00)==0xFF00 || (rand & 0xFF00)==0x0000 );
  rand2 = TI_getRandomIntegerFromVLO();
  BCSCTL1 = CALBC1_1MHZ;
  DCOCTL = CALDCO_1MHZ;
  FCTL2 = FWKEY + FSSEL0 + FN1;
  FCTL3 = FWKEY + LOCKA;
  FCTL1 = FWKEY + WRT;
Flash_Addr[0]=(rand>>8) & 0xFF;
Flash_Addr[1]=rand & 0xFF;
Flash_Addr[2]=(rand2>>8) & 0xFF;
Flash_Addr[3]=rand2 & 0xFF;
  FCTL1 = FWKEY;
  FCTL3 = FWKEY + LOCKA + LOCK;
}

voidlinkTo()
{
linkID_t linkID1;
  uint8_t  adc_val[1];
 //ADC setup
  ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE; // ADC10ON, interrupt enabled
  ADC10CTL1 = INCH_1; // A1 Input
  ADC10AE0 |= 0x02;
  // keep trying to link to Receiver
while (SMPL_SUCCESS != SMPL_Link(&linkID1))
  {
    __bis_SR_register(LPM3_bits + GIE);   // LPM3 with interrupts enabled
    BSP_TOGGLE_LED1();      //red LED toggle
  }
// When linked, Turn off red LED and Turn on Green LED
if (BSP_LED1_IS_ON())
{ }
while (1)
BSP_TOGGLE_LED1();
BSP_TURN_ON_LED2();
//red LED toggle
//green LED ON
  
  {
    ADC10CTL0 |= ENC + ADC10SC; // ADC Sampling and conversion start
adc_val[0] = (uint8_t)(ADC10MEM/4);
SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, "" );
    __bis_SR_register(LPM3_bits+GIE);       // LPM3 with interrupts enabled
SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, "" );
  }
if (SMPL_SUCCESS == SMPL_Send(linkID1,adc_val,sizeof(adc_val)))
    {
BSP_TOGGLE_LED2();
BSP_TOGGLE_LED1();
//Toggle green LED on successful transmission
//Toggle red LED if sending failed
    } 
else
{}
// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)

{
  __bic_SR_register_on_exit(LPM3_bits);
// Clear LPM3 bit from 0(SR)
// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
__bic_SR_register_on_exit(CPUOFF);
}
// Clear CPUOFF bit from 0(SR)