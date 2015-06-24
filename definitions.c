/*
 * File:   definitions.c
 * Author: Emilio Ferrari, Simone Faccini
 *
 */

// *****************************************************************************
// ******************************* INCLUDED FILES ******************************
// *****************************************************************************

#include "definitions.h"


// *****************************************************************************
// ***************************** GLOBAL VARIABLES ******************************
// *****************************************************************************

unsigned int MSB=0, LSB=0, UART_cmd=0;
unsigned char temp=0;
extern unsigned char AudioBuffer[AudioBufferSize];
extern unsigned int read, write, audioTX;
extern char USB_command_rcv, BT_command_rcv;


// *****************************************************************************
// ************** DSPIC33EP256MU806 Configuration Bit Settings *****************
// *****************************************************************************

//-----------------------------------------------------------------------------
//Macros for Configuration Fuse Registers:
//-----------------------------------------------------------------------------
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device header files.
//The configuration fuse registers reside in Flash memory.
//'C' source line config statements:

//// FICD
//#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
//#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)
//
//// FPOR
//#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
//#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
////#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
//
//// FWDT
//#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
//#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
//#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
//#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
//#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)
//
//// FOSC
//#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
//#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
//#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
//#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
//
//// FOSCSEL
//#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
//#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)
//
//// FGS
//#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)


// -----------------------------------------------------------------------------
// ------------------------- FUNCTIONS DECLARATIONS ----------------------------
// -----------------------------------------------------------------------------


/******************************************************************************/
// ***************************** DELAY FUNCTION ********************************
/******************************************************************************/

void Delay_us(unsigned int delay)
{
    int  i;
    for (i = 0; i < delay; i++){
    __asm__ volatile ("repeat #39");
    __asm__ volatile ("nop");
    }
}


/******************************************************************************/
// ************************ INTERRUPT INITIALIZATION ***************************
/******************************************************************************/

void Interrupt_Init(void) {

    INTCON1bits.NSTDIS=0; // Interrupt nesting enable
    INTCON2bits.GIE = 1; // Global Interrupt Enable
    IEC0bits.AD1IE = 1; // ADC Interrupt Enable 
}

/******************************************************************************/
// ************************* SYSTEM INITIALIZATION *****************************
/******************************************************************************/

//void InitializeSystem(void) {
//
//    /**************************************************************************/
//    /*************************** Ports Configuration **************************/
//    /**************************************************************************/
//
//    // Set pin as Analog=1/Digital=0 (PORTF is only digital)
//    // RP100(RF4) on UART1_RX and RP101(RF5) on UART1_TX are digital
////  ANSELA = 0x0000;
//    ANSELB = 0x0001; // Ensure AN0(RB0) on ADC_CH0 is analog
//    ANSELC = 0x0000;
//    ANSELD = 0x0000;
//    ANSELE = 0x0000;
//    ANSELG = 0x0000;
//
//    // Set pin as Input=1/Output=0
////  TRISA = 0x0000;
//    TRISB = 0x0001; // AN0(RB0) as input
//    TRISC = 0x0000;
//    TRISD = 0x0000;
//    TRISE = 0x0000;
//    TRISF = 0x0010; // RF4 as input and RF5 as output
//    TRISG = 0x0000;
//
//    /**************************************************************************/
//    /************************ Oscillator Configuration ************************/
//    /**************************************************************************/
//
//    // The crystal frequency is 8 MHz
//    // The oscillator frequency Fosc=Fin*M/(N1*N2)=8MHz
//    // The CPU clock frequency is Fcy=Fosc/2=4MHz
//
//    // PLL Feedback Divisor Register
//    // PLL Feedback Divisor bits (also denoted as M, PLL multiplier)
//    PLLFBD = 2;                    /* M  = 4 */
//    // Clock Divisor Register
//    // PLL VCO Output Divider Select bits (also denoted as N2, PLL postscaler)
//    CLKDIVbits.PLLPOST = 0;         /* N2 = 2  */
//    // PLL Phase Detector Input Divider Select bits (also denoted as N1, PLL prescaler)
//    CLKDIVbits.PLLPRE = 0;          /* N2 = 2  */
//    // FRC Oscillator Tuning Register
//    OSCTUN = 0; // Tune FRC oscillator center frequency (7.373 MHz nominal), if FRC is used
//
//    // OSCCON: Oscillator Control Register
//    // OSCCONH contains COSC<2:0> (Current Oscillator Selection bits (read-only)) and NOSC<2:0> (New Oscillator Selection bits)
//    __builtin_write_OSCCONH( 0x03 );  // New Oscillator Selection ( Primary Oscillator with PLL (XTPLL, HSPLL, ECPLL) )
//    __builtin_write_OSCCONL(0x01);    // Oscillator Switch Enable bit (Request oscillator switch to selection specified by the NOSC<2:0> bits in OSCCONH)
//
//    while (OSCCONbits.COSC != 0x3);     // Wait for clock switching (COSC = 0x3, i.e. COSC<2:0> = NOSC<2:0>)
//    while (_LOCK == 0);                 /* Wait for PLL lock at 40 MIPS, i.e. PLL is in lock or PLL start-up timer is satisfied */
//
//}


/******************************************************************************/
// *************************** TIMER1 INITIALIZATION ***************************
/******************************************************************************/

void Timer_Init(void) {

    //Timer1
    T1CONbits.TON=0; // Timer On bit (Stops the timer)
    T1CONbits.TSIDL=0; // Stop in Idle Mode bit (Continue timer operation in Idle mode)
    T1CONbits.TCS=0; // Timer Clock Source Select bit (Internal clock (FOSC/2))
    T1CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable bit (Gated time accumulation disabled)
    T1CONbits.TCKPS = 0; // Timerx Input Clock Prescale Select bits (1:1 prescale value)

    TMR1 = 0x0000; // Clear timer register
    PR1 = 0x0EA5; // Load the period value (with a system clock of 15MHz, timer interrupt generated every 2*TSIM=250us)

    IPC0bits.T1IP = 0x01; //Interrupt Priority
    IFS0bits.T1IF = 0; // Reset Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Interrupt Enable

    // T1CONbits.TON = 1; // Timer Enable
}


/******************************************************************************/
// ************************* TIMER_1 Interrupt Routine *************************
/******************************************************************************/

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    // Sample_1
    // Trasmission of 1st byte
    MSB = (ADC1BUF0 & 0x0FFF)>>6 ;
    temp = MSB | 0x0080 ;
    AudioBuffer[write++]=temp;
    write = write%AudioBufferSize;
//  while(U1STAbits.UTXBF==1);
//  U1TXREG = temp;
    // Trasmission of 2nd byte
    LSB = (ADC1BUF0 & 0x003F)<<2 ;
    temp = LSB | 0x0001 ;
    AudioBuffer[write++]=temp;
    write = write%AudioBufferSize;
//    while(U1STAbits.UTXBF==1);
//    U1TXREG = temp;

    // Sample_2
    // Trasmission of 1st byte
    MSB = (ADC1BUF1 & 0x0FFF)>>6 ;
    temp = MSB | 0x0080 ;
    AudioBuffer[write++]=temp;
    write = write%AudioBufferSize;
//    while(U1STAbits.UTXBF==1);
//    U1TXREG = temp;
    // Trasmission of 2nd byte
    LSB = (ADC1BUF1 & 0x003F)<<2 ;
    temp = LSB | 0x0001 ;
    AudioBuffer[write++]=temp;
    write = write%AudioBufferSize;
//    while(U1STAbits.UTXBF==1);
//    U1TXREG = temp;

    IFS0bits.T1IF = 0;
}


/******************************************************************************/
// *************************** TIMER2 INITIALIZATION ***************************
/******************************************************************************/

void Timer2_Init(void) {

    //Timer2
    T2CONbits.TON=0; // Timer On bit (Stops the timer)
    T2CONbits.TSIDL=0; // Stop in Idle Mode bit (Continue timer operation in Idle mode)
    T2CONbits.TCS=0; // Timer Clock Source Select bit (Internal clock (FOSC/2))
    T2CONbits.T32=0; // 32-Bit Timer Mode Select bit (Timerx and Timery act as two 16-bit timers)
    T2CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable bit (Gated time accumulation disabled)
    T2CONbits.TCKPS = 0; // Timerx Input Clock Prescale Select bits (1:1 prescale value)

    TMR2 = 0x0000; // Clear timer register
    PR2 = 0x03A9; // Load the period value (with a system clock of 15MHz, timer interrupt generated every 62.5us
    //PR2 = 0x0270; // Load the period value (with a system clock of 4MHz/8, timer interrupt generated every 2*TSIM=250us)

    IPC1bits.T2IP = 0x01; //Interrupt Priority
    IFS0bits.T2IF = 0; // Reset Interrupt Flag
    IEC0bits.T2IE = 1; // Enable Interrupt Enable

    //T2CONbits.TON = 1; // Timer Enable
}


/******************************************************************************/
// ************************* TIMER_2 Interrupt Routine *************************
/******************************************************************************/

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    // Sent 4 byte
    if(audioTX==1 && write!=read)
    {
        if(USB_command_rcv & 0x02)
        {
            while(U1STAbits.UTXBF==1);
            U1TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U1STAbits.UTXBF==1);
            U1TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U1STAbits.UTXBF==1);
            U1TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U1STAbits.UTXBF==1);
            U1TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;
        }

        // Sent 4 byte
        if(BT_command_rcv & 0x02)
        {
            while(U2STAbits.UTXBF==1);
            U2TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U2STAbits.UTXBF==1);
            U2TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U2STAbits.UTXBF==1);
            U2TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;

            while(U2STAbits.UTXBF==1);
            U2TXREG = AudioBuffer[read++];
            read = read%AudioBufferSize;
        }
    }
    IFS0bits.T2IF = 0;
}



/******************************************************************************/
// ***************************** ADC INITIALIZATION ****************************
/******************************************************************************/

void InitAdc1(void)
{
    /* Initialize ADC module */

    // AD1CON1: ADC1 Control Register 1
    AD1CON1bits.ADON=0;     // ADC Operating Mode bit (ADC is off)
    AD1CON1bits.ADSIDL=0;   // ADC Stop in Idle Mode bit (Continues module operation in Idle mode)
    AD1CON1bits.AD12B=1;    // ADC 10-Bit or 12-Bit Operation Mode bit(12-bit, 1-channel ADC operation)
    AD1CON1bits.FORM=1;     // Data Output Format bits (Signed Integer (DOUT = ssss sddd dddd dddd))
    AD1CON1bits.SSRC=7;     // Sample Clock Source Select bits(Internal counter ends sampling and starts conversion (auto-convert))
    AD1CON1bits.SSRCG=0;    // Sample Clock Source Group bit
//  AD1CON1bits.SIMSAM=0;   // Simultaneous Sample Select bit (Not used when CHPS<1:0> = 00)
    AD1CON1bits.ASAM=1;     // ADC Sample Auto-Start bit (Sampling begins immediately after last conversion; SAMP bit is auto-set)
    AD1CON1bits.SAMP=0;     // ADC Sample Enable bit (Automatically set by hardware if ASAM = 1)
    AD1CON1bits.DONE=0;     // ADC Conversion Status bit (Automatically set/clear by hardware when Analog-to-Digital conversion is complete/started)

    // AD1CON2: ADC1 Control Register 2
    AD1CON2bits.VCFG=0;     // ADC Converter Voltage Reference Configuration bits (VrefH=AVdd, VrefL=AVss)
    AD1CON2bits.CSCNA=0;    // Input Scan Select bit (Does not scan inputs)
    AD1CON2bits.CHPS=0;     // Channel Select bits (Converts CH0)
    AD1CON2bits.SMPI=1;     // Sample and Conversion Operation bits (Generates interrupt after completion of every 2nd sample/conversion operation)
    AD1CON2bits.BUFM=0;     // Buffer Fill Mode Select bit (Always starts filling the buffer from the Start address)
    AD1CON2bits.ALTS=0;     // Alternate Input Sample Mode Select bit (Always uses channel input selects for Sample MUXA)

    // AD1CON3: ADC1 Control Register 3
    AD1CON3bits.ADRC=0;     // ADC Conversion Clock Source bit (Clock Derived From System Clock)
    AD1CON3bits.SAMC=1;     // Auto-Sample Time bits (TSMP = SAMC<4:0> * TAD, with minimum TSMP = 2 * TAD)
    AD1CON3bits.ADCS=124;    // ADC Conversion Clock Select bits ( TAD = TCY * (ADCS<7:0> + 1) )
                            // 12-bit ADC conversion time: Tconv = 14 * TAD [datasheet pag.562]
                            // For one sampling and one conversion, TSIM = TSMP + TCONV (SAMC=1 and ADCS=124 ensure TSIM = 125us)

    // AD1CON4: ADC1 Control Register 4
    AD1CON4 = 0x0000;    // ADC DMA Enable bit (Conversion results stored in ADCxBUF0 through ADCxBUFF registers; DMA is not used)

    // AD1CSSH: ADC1 INPUT SCAN SELECT REGISTER HIGH
    AD1CSSH = 0x0000;
    // AD1CSSL: ADC1 INPUT SCAN SELECT REGISTER LOW
    AD1CSSL = 0x0000;

    /* Assign MUXA inputs */

    // AD1CHS0: ADC1 Input Channel 0 Select Register [ADC Reference Manual pag.62]
    AD1CHS0bits.CH0SA = 0; // Channel 0 Positive Input Select for Sample A bits (Channel 0 positive input is AN0)
    AD1CHS0bits.CH0NA = 0; // Channel 0 Negative Input Select for Sample A bit (Select VrefL for CH0)
    // AD1CHS123: ADC1 Input Channel 1, 2, 3 Select Register
//  AD1CHS123bits.CH123SA = 0; // Channels 1, 2, 3 Positive Input Select for Sample A bits (Not used when in 12-bit mode)
//  AD1CHS123bits.CH123NA = 0; // Channels 1, 2, 3 Negative Input Select for Sample A bits (Not used when in 12-bit mode)

    /* Enable ADC module and provide ADC stabilization delay */
    //AD1CON1bits.ADON = 1;
    //Delay_us(20);
}


/******************************************************************************/
// ************************** ADC Interrupt Routine ****************************
/******************************************************************************/

void __attribute__((interrupt,no_auto_psv)) _AD1Interrupt(void)
{     
    IFS0bits.AD1IF=0;
}


/******************************************************************************/
// ***************************** UART INITIALIZATION ***************************
/******************************************************************************/

void InitUART1(void)
{ 
    // U1MODE: UART1 Mode Register
    U1MODEbits.UARTEN=0; // UARTx Enable bit (UARTx is disabled; all UARTx pins are controlled by port latches; UARTx power consumption is minimal)
    U1MODEbits.USIDL=0; // UARTx Stop in Idle Mode bit (Continues operation in Idle mode)
    U1MODEbits.IREN=0; // IrDA® Encoder and Decoder Enable bit (IrDA encoder and decoder are disabled)
    U1MODEbits.RTSMD=1; // Mode Selection for UxRTS Pin bit (U1RTS pin in Simplex mode)
    U1MODEbits.UEN=0; // UARTx Pin Enable bits (UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins are controlled by port latches)
    U1MODEbits.WAKE=0; // Wake-up on Start Bit Detect During Sleep Mode Enable bit (No wake-up is enabled)
    U1MODEbits.LPBACK=0; //UARTx Loopback Mode Select bit (Loopback mode is disabled)
    U1MODEbits.ABAUD=0; // Auto-Baud Enable bit (Baud rate measurement is disabled or has completed)
    U1MODEbits.URXINV=0; // Receive Polarity Inversion bit (UxTX Idle state is '1')
    U1MODEbits.BRGH=1; // High Baud Rate Enable bit (BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode))
    U1MODEbits.PDSEL=0; // Parity and Data Selection bits (No Parity, 8-Data bits)
    U1MODEbits.STSEL=0; // Stop Bit Selection bit (One Stop bit)
    // Enable UART1
    U1MODEbits.UARTEN=1; // UARTx Enable bit (UARTx is enabled; UARTx pins are controlled by UARTx as defined by the UEN<1:0> and UTXEN control bits)

    // U1STA: UART1 STATUS AND CONTROL REGISTER
    U1STAbits.UTXISEL0=0; // UARTx Transmission Interrupt Mode Selection bits (Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty)
    U1STAbits.UTXISEL1=1;
    U1STAbits.UTXINV=0; // UARTx Transmit Polarity Inversion bit (UxTX Idle state is '1')
    U1STAbits.UTXBRK=0; // UARTx Transmit Break bit (Sync Break transmission is disabled or completed)
    U1STAbits.UTXEN=0; // UARTx Transmit Enable bit (Transmit is disabled, any pending transmission is aborted and the buffer is reset; UxTX pin controlled by port)
    U1STAbits.URXISEL=0; // UARTx Receive Interrupt Mode Selection bits (Interrupt is set when any character is received and transferred from the UxRSR to the receive buffer; receive buffer has one or more characters)
    U1STAbits.ADDEN=0;  // Address Character Detect bit [bit 8 of received data = 1] (Address Detect mode is disabled)
    // Enable UART1 TX
    U1STAbits.UTXEN=1; // UARTx Transmit Enable bit (Transmit is enabled, UxTX pin is controlled by UARTx)

    // Baud-Rate=Fp/[4*(UxBRG +1)], with Fp=10MHz
    // Start + Stop bit for every byte trasmitted
//  #define FP 4000000
//  #define BAUDRATE 140000 // 250us to transfer 4 data byte + (start+stop)*4 bit
//  #define BRGVAL ((FP/BAUDRATE)/16)-1
//  U1BRG = BRGVAL; // maximum Baud-Rate corresponds to minimum UxBRG
    U1BRG = 3;

    // Enable UART Interrupts
    IFS0bits.U1TXIF = 0;    // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1;    // Enable UART Transmit Interrupts
    IFS0bits.U1RXIF = 0;    // Clear the Receieve Interrupt Flag
    IEC0bits.U1RXIE = 1;    // Enable UART Receieve Interrupts

    // Set input/output UART1 pins
    RPINR18bits.U1RXR = 70; // Set UART1 RX to RP70 pin, that corresponds to RD6 pin (see "pin table")
    TRISDbits.TRISD6 = 1;
    RPOR2bits.RP69R = 1; // Set UART1 TX to RP69 pin, that corresponds to RD5 pin
    TRISDbits.TRISD5 = 0;

//    RPINR18bits.U1RXR = 96;      //Set UART1 RX to RP96 = RF0
//    TRISFbits.TRISF0=1;
//    RPOR7bits.RP97R = 0b000001;       // Set UART1 TX to RP97 = RF1
//    TRISFbits.TRISF1=0;
      
    /* Wait at least 105 microseconds (1/9600) before sending first char */
    #define DELAY_105uS asm volatile ("REPEAT, #4201"); Nop(); // 105uS delay
    DELAY_105uS
}


/******************************************************************************/
// ************************* UART1_TX Interrupt Routine ************************
/******************************************************************************/

void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{        
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
}

/******************************************************************************/
// ************************* UART2_TX Interrupt Routine ************************
/******************************************************************************/

void __attribute__((interrupt,no_auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag
}

/******************************************************************************/
// ************************** UART_RX Interrupt Routine ************************
/******************************************************************************/

//void __attribute__ ((interrupt,no_auto_psv)) _U1RXInterrupt(void)
//{
//    unsigned int temp=0;
//    temp=U1RXREG;
//
//    if( (temp>>5)==0x7 && (temp & 0x7)==0x0 )
//        UART_cmd=(temp>>3) & (0x03);
//
//    switch (UART_cmd){
//        case 0x0 :
//            /* Disable ADC module */
//            AD1CON1bits.ADON = 0;
//            // Disable UART1 TX
//            //U1STAbits.UTXEN=0; // UARTx Transmit Enable bit (Transmit is disabled)
//            // Timer Disable
//            T1CONbits.TON = 0;
//            break;
//
//        case 0x1 :
//            /* Enable ADC module and provide ADC stabilization delay */
//            AD1CON1bits.ADON = 1;
//            Delay_us(20);
//            // Enable UART1 TX
//            //U1STAbits.UTXEN=1; // UARTx Transmit Enable bit (Transmit is enabled, UxTX pin is controlled by UARTx)
//            // Timer Enable
//            T1CONbits.TON = 1;
//            break;
//
//        default : break;
//    }
//
//    IFS0bits.U1RXIF = 0; // Clear TX Interrupt flag
//}