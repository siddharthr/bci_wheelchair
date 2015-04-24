
#include "stdio.h"
#include "stdlib.h"
#include "usbstk5515.h"
#include "usbstk5515_led.h"
#include "usbstk5515_i2c.h"
#include "usbstk5515_gpio.h"
#include "lcd.h"
#include "math.h"
#include "imagelib.h"
#include "wavelet.h"
#include "csl_uart.h"
#include "csl_uartAux.h"
#include "csl_intc.h"
#include "csl_general.h"
#include "cls.h" //with 5 stage, 55 classifier cascade
#define OSD9616_I2C_ADDR 0x3C #define CSL_TEST_FAILED #define CSL_TEST_PASSED #define CSL_UART_BUF_LEN
 // OSD9616 I2C address
(1U)
(0)
                                        (4U)
/* String length to be received and transmitted */
/* Global constants */
#define WR_STR_LEN
#define RD_STR_LEN
#define CSL_PLL_DIV_000
#define CSL_PLL_DIV_001
#define CSL_PLL_DIV_002
#define CSL_PLL_DIV_003
#define CSL_PLL_DIV_004
#define CSL_PLL_DIV_005
#define CSL_PLL_DIV_006
#define CSL_PLL_DIV_007
#define CSL_PLL_CLOCKIN
80 10
                           (0)
                           (1u)
                           (2u)
                           (3u)
                           (4u)
                           (5u)
                           (6u)
                           (7u)
(32768u)
                         *(ioport volatile unsigned *)0x1C20
                         *(ioport volatile unsigned *)0x1C21
                         *(ioport volatile unsigned *)0x1C22
                         *(ioport volatile unsigned *)0x1C23
       /* Input clock freq in MHz */
    60000000,
       /* Baud rate */
    9600,
       /* Word length of 8 */
    CSL_UART_WORD8,
       /* To generate 1 stop bit */
    0,
       /* Disable the parity */
    CSL_UART_DISABLE_PARITY,
       /* Enable trigger 14 fifo */
       CSL_UART_FIFO_DMA1_DISABLE_TRIG14,
       /* Loop Back enable */
       CSL_UART_NO_LOOPBACK,
       /* No auto flow control*/
       CSL_UART_NO_AFE ,
       /* No RTS */
       CSL_UART_NO_RTS ,
};
// CSL UART data structures
CSL_UartObjuartObj;
CSL_UartHandlehUart;

#define PLL_CNTL1
#define PLL_CNTL2
#define PLL_CNTL3
#define PLL_CNTL4
// Global data definition
// UART setup structure
CSL_UartSetupmySetup =
{
// CSL UART data buffers
Uint16    uartIntcWriteBuff[CSL_UART_BUF_LEN ];
Uint16    uartIntcReadBuff[CSL_UART_BUF_LEN ];
char      buffer[50]={0};
chareeg[50]={0};
chartemp_wksp[50]
char      b=0;
Uint16    mutex = 0;
Uint16    setbit;
char      *ptr = buffer;
volatile Bool    endOf50 = FALSE;
/* Interrupt vector start address */
extern void VECSTART(void);
//Function to calculate the system clock
Uint32 getSysClk(void);
//Interrupt Service Routine to handle UART Line Status Interrupt
voiduart_lsiIsr(void)
{
       Uint16 reg;
       reg=hUart->uartRegs->LSR;
       reg= reg;
}
//Interrupt Service Routine to handle UART Character Timeout Interrupt
voiduart_ctoIsr(void)
{
       UART_read(hUart,buffer[b],0,0);
       UART_eventEnable(hUart,CSL_UART_XMITOR_REG_EMPTY_INTERRUPT);
}
//Interrupt Service Routine to handle UART Receive Interrupt
voiduart_rxIsr(void)
{
}
UART_read(hUart,buffer[b],0,0);
b=b+1;
if(b==50)
{
       endOf50 = TRUE;
       b=0;
       eeg=buffer;
}
UART_eventEnable(hUart,CSL_UART_XMITOR_REG_EMPTY_INTERRUPT);
//brief  Interrupt Dispatcher to identify interrupt source
//This function identify the type of UART interrupt generated and
//calls concerned ISR to handle the interrupt
interrupt void UART_intrDispatch(void)
{
}

Uint16 eventId = 0;
IRQ_disable(UART_EVENT);
/* Get the event Id which caused interrupt */
eventId = UART_getEventId(hUart);
if (((void (*)(void))(hUart->UART_isrDispatchTable[eventId])))
{
       ((void (*)(void))(hUart->UART_isrDispatchTable[eventId]))();
}
IRQ_enable(UART_EVENT);
return;

//UART interrupt initialize function
CSL_Statusuart_initialize(void)
{
       CSL_UartIsrAddrisrAddr;
       CSL_Status         status;
       Uint32            sysClk;
       sysClk = getSysClk();
       mySetup.clkInput = sysClk;
    /* Loop counter and error flag */
status = UART_init(&uartObj,CSL_UART_INST_0,UART_INTERRUPT);
if(CSL_SOK != status)
    {
page_select(0);
       write_char("UART INIT FAILED");
return(status);
}
else
{
}
page_select(0);
write_char("UART INIT SUCCESS");
    /* Handle created */
hUart = (CSL_UartHandle)(&uartObj);
    /* Configure UART registers using setup structure */
status = UART_setup(hUart,&mySetup);
if(CSL_SOK != status)
    {
page_select(0);
       write_char("UART SETUP FAILED");
return(status);
}
else
{
}
page_select(0);
write_char("UART SETUP SUCCESS");
       /* Configure and Register the UART interrupts */
       isrAddr.rbiAddr  =uart_rxIsr;
       isrAddr.tbeiAddr = uart_txIsr;
       isrAddr.ctoi     = uart_ctoIsr;
       isrAddr.lsiAddr  =uart_lsiIsr;
    /* Disable interrupt */
IRQ_globalDisable();
    /* Clear any pending interrupts */
       IRQ_clearAll();
       /* Disable all the interrupts */
       IRQ_disableAll();
       IRQ_setVecs((Uint32)(&VECSTART));
       /* Configuring Interrupt */
       IRQ_plug (UART_EVENT, &UART_intrDispatch);
       /* Enabling Interrupt */
       IRQ_enable(UART_EVENT);
       IRQ_globalEnable();
       /* Set the UART callback function */
       status = UART_setCallback(hUart,&isrAddr);
       if(status != CSL_SOK)
       {

}
       page_select(0);
       write_char("UART SETCALLBACK FAIL");
       return(status);
} else {
}
page_select(0);
write_char("UART SETCALLBACK SUCCESS");
/* Enable the UART Events */
status = UART_eventEnable(hUart, CSL_UART_XMITOR_REG_EMPTY_INTERRUPT);
if(status != CSL_SOK)
{
       page_select(0);
       write_char("UART EVENT ENABLE FAIL");
       return(status);
}
status = UART_eventEnable(hUart, CSL_UART_RECVOR_REG_DATA_INTERRUPT);
if(status != CSL_SOK)
{
       page_select(0);
       write_char("UART EVENT ENABLE FAIL");
       return(status);
}
status = UART_eventEnable(hUart, CSL_UART_RECVOR_LINE_STATUS_INTERRUPT);
if(status != CSL_SOK)
{
       page_select(0);
       write_char("UART EVENT ENABLE FAIL");
       return(status);
}
return(CSL_SOK);
voidoled_initialize();
Int16 printLetter(Uint16,Uint16,Uint16,Uint16);
voidpage_select(Uint16);
Int16 OSD9616_multiSend(Uint8*,Uint16);
Int16 OSD9616_send(Uint16,Uint16);
voidwrite_char(char*);
#define LENGTH      50
void main()
{
CSL_Status status;
charmsg[1] = {0};
int result=0;
    USBSTK5515_init();
oled_initialize();
page_select(0);
write_char("  BCI WHEELCHAIR  ");
       page_select(1);
       write_char("FOR QUADRIPLEGICS ");
       OSD9616_send(0x00,0x2e);
       USBSTK5515_waitusec(5000);
       page_select(0);
       write_char("       BY          ");
       page_select(1);
       write_char("SIDDHARTH & SAHEEM ");

return 1;
return 0;
} else {
else


OSD9616_send(0x00,0x2e);
       USBSTK5515_waitusec(500000);
       uart_initialize();
       page_select(0);
       write_char("SIGNAL PROCESSING");
while(1) {
       page_select(0);
       write_char("COLLECTING FIFTY SAMPLES");
       while(endOf50 == FALSE)
       {}           //wait loop
       page_select(0);
       write_char("WAVELET DECOMPOSITION");
       IMG_wave_decom_one_dim(eeg,temp_wksp,db4,50,5);//5 level dwt with db4
       thr=sqrt(2*log(50)); // select threshold
       //Thresholding
       for(i = (LENGTH>>1); i< LENGTH; i++ )
       {
             if(signal[i] > thr ) signal[i] = thr;
             if(signal[i] < -thr ) signal[i] = -thr;
}
       page_select(0);
       write_char("APPLY ADABOOST MODEL");
result=RunCascade((int)eeg)();
       if(result)
             msg[0]=1;    //move forward
       else
             msg[0]=6;    //stop
       status = UART_write(hUart,msg,1,0);
       if(CSL_SOK != status)
       {
       page_select(0);
             write_char("UART WRITE FAIL");
} else {
}
} }
} }
page_select(0);
write_char("UART WRITE SUCCESS");
intApplyClassifier(WeakClassifiercls, int *sig)
{
doublewynik = ApplyFeature(cls.f,sig);
if (cls.parity == 1)
        {
if(wynik<cls.theta)
                {
                {
if(-1*wynik> -1*cls.theta)
return 1;
return 0;
}
{
} else { }
}
intApplyStrongClassifier (StrongClassifier strong, int *sig )
{
inti,j,clsf;
double sum=0.0;
doubleth=0;
for (i=0; i<strong.count; i++)
th+=strong.sc[i].alpha * ApplyClassifier(strong.sc[i], sig);
         //printf("klas o feat = %d theta = %fmowi, ze f=%f i H=%d\n",strong.count,
strong.theta,th,(th>strong.theta)?1:-1);
if(th<strong.theta) return -1;
return 1;
        //return (th<sc.theta)?0:1;
}
intRunCascade (int *sig)
{
inti=0;
int T=0;
intclsf = 0;
for(i=0; i<num_stages; i++) {
                //T = stages[i];
if ( ApplyStrongClassifier(st[i],sig) == -1)
{
return 0;
} }
return 1; }
Int16 OSD9616_send( Uint16 comdat, Uint16 data )
{
    Uint8 cmd[2];
cmd[0] = comdat& 0x00FF;     // Specifies whether data is Command or Data
cmd[1] = data;                // Command / Data
return USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 2 );
}
Int16 OSD9616_multiSend( Uint8* data, Uint16 len )
{
Uint16 x;
    Uint8 cmd[10];
for(x=0;x<len;x++)
    {
       cmd[x] = data[x];
// Command / Data
    }
return USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, len );
}
Int16 printLetter(Uint16 l1,Uint16 l2,Uint16 l3,Uint16 l4)
{
       OSD9616_send(0x40,l1);
       OSD9616_send(0x40,l2);
       OSD9616_send(0x40,l3);
       OSD9616_send(0x40,l4);
       OSD9616_send(0x40,0x00);

return 0; }
voidoled_initialize()
{
       Uint8 cmd[10];
// For multibyte commands
    /* Initialize I2C */
    USBSTK5515_I2C_init( );
    /* Initialize LCD power */
    USBSTK5515_GPIO_setDirection( 12, 1 );  // Output
    USBSTK5515_GPIO_setOutput( 12, 1 );     // Enable 13V
    /* Initialize OSD9616 display */
    OSD9616_send(0x00,0x00); // Set low column address
    OSD9616_send(0x00,0x10); // Set high column address
    OSD9616_send(0x00,0x40); // Set start line address
cmd[0] = 0x00 & 0x00FF;  // Set contrast control register
cmd[1] = 0x81;
cmd[2] = 0x7f;
    USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 3 );
    OSD9616_send(0x00,0xa1); // Set segment re-map 95 to 0
    OSD9616_send(0x00,0xa6); // Set normal display
cmd[0] = 0x00 & 0x00FF;  // Set multiplex ratio(1 to 16)
cmd[1] = 0xa8;
cmd[2] = 0x0f;
    USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 3 );
    OSD9616_send(0x00,0xd3); // Set display offset
    OSD9616_send(0x00,0x00); // Not offset
    OSD9616_send(0x00,0xd5); // Set display clock divide ratio/oscillator frequency
    OSD9616_send(0x00,0xf0); // Set divide ratio
cmd[0] = 0x00 & 0x00FF;  // Set pre-charge period
cmd[1] = 0xd9;
cmd[2] = 0x22;
    USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 3 );
cmd[0] = 0x00 & 0x00FF;  // Set com pins hardware configuration
cmd[1] = 0xda;
cmd[2] = 0x02;
    USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 3 );
    OSD9616_send(0x00,0xdb); // Set vcomh
    OSD9616_send(0x00,0x49); // 0.83*vref
cmd[0] = 0x00 & 0x00FF;  //--set DC-DC enable
cmd[1] = 0x8d;
cmd[2] = 0x14;
    USBSTK5515_I2C_write( OSD9616_I2C_ADDR, cmd, 3 );
    OSD9616_send(0x00,0xaf); // Turn on oled panel
}
voidpage_select(Uint16 page)
{
Uint16 i;
    /* Fill page */
    OSD9616_send(0x00,0x00);
    OSD9616_send(0x00,0x10);
    OSD9616_send(0x00,0xb0+page); // Set page for page 0 to page 5
for(i=0;i<128;i++)
    {
       OSD9616_send(0x40,0x00);
    }
    /* Write to page*/
    OSD9616_send(0x00,0x00);

// Set low column address
// Set low column address
// Set high column address

    OSD9616_send(0x00,0x10);   // Set high column address
    OSD9616_send(0x00,0xb0+page); // Set page for page 0 to page 5
}
Uint8 string_length(char *pointer)
{
       Uint8 c = 0;
while( *(pointer+c) != '\0' )
c++;
return c;
}
voidwrite_char(char *data)
{
       intlength,c;
       char *begin,*end,temp;
       length = string_length(data);
       begin = data;
       end = data;
for (c=0;c<(length-1);c++)
       end++;
       for (c=0;c<length/2;c++)
    {
temp = *end;
       *end = *begin;
       *begin = temp;
begin++;
end--; }
{

while(*data != '\0')
switch(*data)
{
       case 'A':
       case 'B':
       case 'C':
       case 'D':
       case 'E':
       case 'F':
       case 'G':
       case 'H':
       case 'I':
       case 'J':
       case 'K':
       case 'L':
printLetter(0x7E,0x09,0x0A,0x7C);  // A
break;
printLetter(0x36,0x49,0x49,0x7F);  // B
break;
printLetter(0x22,0x41,0x41,0x3E);  // C
break;
printLetter(0x3E,0x41,0x41,0x7F);  // D
break;
printLetter(0x41,0x49,0x49,0x7F);  // E
break;
printLetter(0x01,0x09,0x09,0x7F);  // F
break;
printLetter(0x72,0x51,0x41,0x3E);  // G
break;
printLetter(0x7F,0x08,0x08,0x7F);  // H
break;
printLetter(0x00,0x7F,0x00,0x00);  // I
break;
printLetter(0x01,0x3F,0x41,0x21);  // J
break;
printLetter(0x41,0x22,0x14,0x7F);  // K
break;
printLetter(0x40,0x40,0x40,0x7F);  // L
}

             case 'M':
             case 'N':
             case 'O':
             case 'P':
             case 'Q':
             case 'R':
             case 'S':
             case 'T':
             case 'U':
             case 'V':
             case 'W':
             case 'X':
             case 'Y':
             case 'Z':
             case '&':
             default :
data++; }
printLetter(0x7F,0x06,0x06,0x7F);  // M
break;
printLetter(0x7F,0x30,0x0E,0x7F);  // N
break;
printLetter(0x3E,0x41,0x41,0x3E);  // O
break;
printLetter(0x06,0x09,0x09,0x7F);  // P
break;
printLetter(0x5E,0x31,0x21,0x1E);  // Q
break;
printLetter(0x46,0x29,0x19,0x7F);  // R
break;
printLetter(0x32,0x49,0x49,0x26);  // S
break;
printLetter(0x01,0x7F,0x01,0x01);  // T
break;
printLetter(0x3F,0x40,0x40,0x3F);  // U
break;
printLetter(0x3F,0x40,0x20,0x1F);  // V
break;
printLetter(0x7F,0x30,0x30,0x7F);  // W
break;
printLetter(0x63,0x1C,0x1C,0x63);  // X
break;
printLetter(0x0F,0x70,0x08,0x07);  // Y
break;
printLetter(0x43,0x4D,0x51,0x61);  // Z
break;
printLetter(0x48,0x34,0x54,0x28);  //&
break;
printLetter(0x00,0x00,0x00,0x00);
break;
}