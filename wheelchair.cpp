// Ultrasonic sensor: Trigger = P4.3, Echo1 = P4.4, Echo2 = P4.5
// MOTOR_PORT
// left_motor
// right_motor
// motor enable
// buzzer
PORT2
P2.0,P2.1
P2.2,P2.3
P2.4
P4.6
#include "msp430x22x4.h"
#include "bsp.h"
#include "mrfi.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "vlo_rand.h"
#define US_OUT P4OUT
#define US_IN  P4IN
#define US_DIR  P4DIR
#define US_Trig BIT3
#define US_Echo1 BIT4
#define US_Echo2 BIT5


voidInitialise_Ports(void)
{
}
P2DIR = BIT0 | BIT1 | BIT2 | BIT3 | BIT4;
P4DIR = BIT3 | BIT6;
P4DIR &= ~BIT4;
P4DIR &= ~BIT5;
P2OUT = 0x00;
voidlmotor_for(void)
{
       P2OUT |= BIT0;
       P2OUT &=~ BIT1;
}
voidrmotor_for(void)
{
       P2OUT |= BIT2;
       P2OUT &=~ BIT3;
}
voidlmotor_bac(void)
{
       P2OUT |= BIT1;
       P2OUT &=~ BIT0;
}
voidrmotor_bac(void)
{
       P2OUT |= BIT3;
       P2OUT &=~ BIT2;
}
voidlmotor_stop(void)
{
       P2OUT &=~ BIT0;
       P2OUT &=~ BIT1;
}
voidrmotor_stop(void)
{
       P2OUT &=~ BIT2;
       P2OUT &=~ BIT3;
}
unsignedint find_dist1()
{

long unsigned inti,result;
  US_OUT &= (~(US_Trig));//Low
  __delay_cycles(10);
  //Give the US pin a 15us High Pulse
  US_OUT |= (US_Trig);   //High
  __delay_cycles(15);
  US_OUT &= (~(US_Trig));//Low
  __delay_cycles(20);
  //Wait for the rising edge
for(i=0;i<600000;i++)
  {
if(!(US_IN & US_Echo1)) continue; else break;
  }
        //High Edge Found
  TA0CTL = TASSEL_2 + MC_2;
  //Now wait for the falling edge
for(i=0;i<600000;i++)
  {
if(US_IN & US_Echo1)
    {
if(TA0R > 60000)
break;
else continue;
} else
{
      TA0CTL = MC_0;
break;
} }
  //Falling edge found
result=TA0R;
//Stop Timer
  TA0CTL = TACLR;
//Start Timer, SMCLK~1MHz, continuous mode
//Stop Timer
//Clear Timer value
if(result > 60000)
return 150; //No obstacle if pulse width is longer
else
return (unsigned int)((result>>1)/58);  //Convert to cm
}
unsignedint find_dist2()
{
long unsigned inti,result;
  US_OUT &= (~(US_Trig));//Low
  __delay_cycles(10);
  //Give the US pin a 15us High Pulse
  US_OUT |= (US_Trig);   //High
  __delay_cycles(15);
  US_OUT &= (~(US_Trig));//Low
  __delay_cycles(20);
  //Wait for the rising edge
for(i=0;i<600000;i++)
  {
if(!(US_IN & US_Echo2)) continue; else break;
  }
  //High Edge Found

TA0CTL = TASSEL_2 + MC_2;      //Start Timer, SMCLK~1MHz, continuous mode
  //Now wait for the falling edge
for(i=0;i<600000;i++)
  {
if(US_IN & US_Echo2)
    {
if(TA0R > 60000)
break;
else continue;
} else
{
      TA0CTL = MC_0;
break;
} }
  //Falling edge found
result=TA0R;
//Stop Timer
  TA0CTL = TACLR;
//Stop Timer
//Clear Timer value
if(result > 60000)
return 150; //No obstacle
else
return (unsigned int)((result>>1)/58);  //Convert to cm
}
voidTXString( char* string, int length );
voidMCU_Init(void);
voidcreateRandomAddress();
voidEEG_to_DSP(void);
__no_init volatile char Flash_Addr[4] @ 0x10F0; // Flash address set randomly
// reserve space for the maximum possible peer Link IDs
staticlinkID_tsLID[NUM_CONNECTIONS];
static uint8_t  sNumCurrentPeers;
// callback handler
static uint8_t sCB(linkID_t);
// work loop semaphores
static uint8_t sPeerFrameSem;
static uint8_t sJoinSem;
static uint8_t sSelfMeasureSem;
char rec = 0;   //variable to store wheelchair control info from DSP
void main (void)
{
unsignedint d1=0,d2=0;
addr_tlAddr;
bspIState_tintState;
  WDTCTL = WDTPW + WDTHOLD;
  __delay_cycles(1000);
BSP_Init();
if(Flash_Addr[0] == 0xFF &&
Flash_Addr[1] == 0xFF &&
Flash_Addr[2] == 0xFF &&
Flash_Addr[3] == 0xFF )
{

// Stop WDT
30
createRandomAddress();      // set Random device address at initial startup
  }
lAddr.addr[0]=Flash_Addr[0];
lAddr.addr[1]=Flash_Addr[1];
lAddr.addr[2]=Flash_Addr[2];
lAddr.addr[3]=Flash_Addr[3];
SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
  //Configure the UART
MCU_Init();
  //Transmit splash screen and network init notification
SMPL_Init(sCB);
Initialise_Ports();
  P2OUT |= BIT4;
  // main work loop
while (1)
//Initialise the general purpose I/O port
  //Enable the Motor driver
  {
EEG_to_DSP();  //Transmit EEG data to DSP
    d1 = find_dist1();//Find distance from Ultrasound sensor 1(Obstacle detect)
    d2 = find_dist2();//Find distance from Ultrasound sensor 2(Pit detect)
if(d1<10 || d2>15)
    {
      P4OUT |= BIT6;   //Obstacle or Pit detected. Sound buzzer
rmotor_stop();   //Stop both the left and right motor
lmotor_stop();
} 
else
{
  P4OUT &=~BIT6;


//Wheelchair motor control from rec received from DSP

if(rec==1) {
       rmotor_for();
       lmotor_for();        //straight
}
else if(rec==2)
{
       rmotor_stop();
       lmotor_for();        //right
}
else if(rec==3)
{
       rmotor_for();
       lmotor_stop();       //left
}
else if(rec==4)
{
       rmotor_bac();
       lmotor_for();        //sharp right
}
else if(rec==5)
{
       rmotor_for();
       lmotor_bac();        //sharp left
}
else if(rec==6)
{
  rmotor_stop();
  lmotor_stop();            //stop
}
else if(rec==7)
  P4OUT |= BIT6;          //On Buzzer
else if(rec==8)
  P4OUT &=~ BIT6;         //OFF Buzzer

} }
voidEEG_to_DSP()
{
  // Wait for the Join semaphore to be set by the receipt
    // of a Join frame from a device that supports and End Device.
if (sJoinSem&& (sNumCurrentPeers< NUM_CONNECTIONS))
    {
      // listen for a new connection
SMPL_LinkListen(&sLID[sNumCurrentPeers]);
sNumCurrentPeers++;
      BSP_ENTER_CRITICAL_SECTION(intState);
if (sJoinSem)
      {
sJoinSem--;
}
      BSP_EXIT_CRITICAL_SECTION(intState);
    }
    // Have we received a frame on one of the ED connections?
    // No critical section -- it doesn't really matter much if we miss a poll
if (sPeerFrameSem)
    {
uint8_tmsg[1], len, i;
      // process all frames waiting
for (i=0; i<sNumCurrentPeers; ++i)
      {
        //Receive data(EEG) from the End Device(Transmitter)
if (SMPL_Receive(sLID[i], msg, &len) == SMPL_SUCCESS)
        {
if(start==1)
          {
TXString(msg,sizeof(msg)); //Send received EEG data to UART(to DSP)
          }
          BSP_TOGGLE_LED2(); //Toggle green LED on successfull reception of EEG
          BSP_ENTER_CRITICAL_SECTION(intState);
sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);
        }
else
{
} }
} }
voidcreateRandomAddress()
{
unsignedint rand, rand2;
do
BSP_TOGGLE_LED1();
//Toggle red LED if reception failed
  {
rand = TI_getRandomIntegerFromVLO();  // first byte can not be 0x00 of 0xFF
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

// Set DCO to 1MHz
// MCLK/3 for Flash Timing Generator
// Clear LOCK & LOCKA bits
// Set WRT bit for write operation

}
FCTL1 = FWKEY;
FCTL3 = FWKEY + LOCKA + LOCK;
// Clear WRT bit
// Set LOCK & LOCKA bit
voidTXString( char* string, int length )
{
    int pointer;
    for( pointer = 0; pointer < length; pointer++)
      {
    volatileinti;
        UCA0TXBUF = string[pointer];
    while (!(IFG2&UCA0TXIFG));
} }
voidMCU_Init()
{
  BCSCTL1 = CALBC1_8MHZ;
  DCOCTL = CALDCO_8MHZ;
  BCSCTL3 |= LFXT1S_2;
  TACCTL0 = CCIE;
  TACCR0 = 12000;
  TACTL = TASSEL_1 + MC_1;
  P3SEL |= 0x30;
  UCA0CTL1 = UCSSEL_2;
  UCA0BR0 = 0x41;
  UCA0BR1 = 0x3;
  UCA0MCTL = UCBRS_2;
  UCA0CTL1 &= ~UCSWRST;
  IE2 |= UCA0RXIE;
  __enable_interrupt();
}
static uint8_t sCB(linkID_t lid)
{
if (lid)
  {
sPeerFrameSem++;
} else
  {
sJoinSem++;
}
  // leave frame to be read by application.
return 0;
}
// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
sSelfMeasureSem = 1;
}
// USCIA interrupt service routine
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
rec = UCA0RXBUF;
}

// USCI_A0 TX buffer ready?
// Set DCO
    // LFXT1 = VLO
    // TACCR0 interrupt enabled
    // ~1 second
    // ACLK, upmode
    // P3.4,5 = USCI_A0 TXD/RXD
    // SMCLK
    // 9600 from 8Mhz
    // Initialize USCI state machine
    // Enable USCI_A0 RX interrupt
