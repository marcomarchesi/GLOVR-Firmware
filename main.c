/******************************************************************************
  File Name    : main.c
  Version      : 1.0
  Description  : GLOVE controller's firmware
  Author(s)    : Andrea Verdecchia
  Target(s)    : dsPIC33EP256MU806
  Compiler     : Microchip MPLAB XC16 C Compiler v1.10
  IDE          : Microchip MPLAB X IDE v1.41
  Programmer   : PICkit3
  Last Updated : 10/04/2013
******************************************************************************/

#include "main.h"

/** Hardware Configuration ***************************************************/
#define GetSystemClock()        20000000
#define GetPeripheralClock()    (GetSystemClock() / 2)
#define GetInstructionClock()   (GetSystemClock() / 2)

/** CONFIGURATION **************************************************/
_FOSCSEL(FNOSC_FRC); // select start-up oscillator as Internal Fast RC
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT); // select primary oscillator as xt crystal, no osc2, clock switching
_FWDT(FWDTEN_OFF); // watchdog on/off by user
_FPOR(ALTI2C1_ON);  // set use of ASCL and ASDA pins for I2C Module

/** GLOBAL VARIABLES ***********************************************/
char USB_command_rcv=0x00, BT_command_rcv=0x00;

unsigned char AudioBuffer[AudioBufferSize];
unsigned int read, write, audioTX = 0;

//#define ECHO

int main(void)
{
    signed int i;
    float imu_data[IMU_DATA_SIZE];
    unsigned char *TXbyte = (unsigned char *)imu_data;
    float rho;

    InitializeSystem();

    TRISBbits.TRISB8 = 1; //Data Ready input
    TRISBbits.TRISB9 = 0; //FSYNC output
    BT_RST = 1;
    SYS_ON_OFF = 1;
    FSYNC = 0;
    __delay_ms(30);

    Interrupt_init();
    InitAdc1();
    InitUART1();
    UART_bt_init();
    Timer_Init();
    Timer2_Init();
    I2C_init();
    MPU9250_AK8963_Init();

    USB_command_rcv = 0; 
    BT_command_rcv = 0;

    for(i=0;i<IMU_DATA_SIZE;i++)
        imu_data[i] = 0;

    for(i=0;i<AudioBufferSize;i++)
        AudioBuffer[i] = 0;
    
    while(1)
    {
        __delay_ms(10);
#ifdef ECHO

#else
        if(USB_command_rcv & 0x01)
        {
            //printf("1");
            Inertial_Read_Measure_Real(imu_data);
            //printf("2");
            MadgwickAHRSupdate(imu_data[3],imu_data[4],imu_data[5],imu_data[0],imu_data[1],imu_data[2],imu_data[6],imu_data[7],imu_data[8]);
            rho = acos(q0);
            imu_data[9] = rho*2;
            imu_data[10] = q1 / sin(rho);
            imu_data[11] = q2 / sin(rho);
            imu_data[12] = q3 / sin(rho);

            // MIC
            if(audioTX==1)
                T2CONbits.TON = 0; // Timer Disable

            //printf("\n%3.3f    %3.3f    %3.3f    %3.3f    %3.3f    %3.3f    %3.3f    %3.3f    %3.3f    %3.3f",imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],imu_data[6],imu_data[7],imu_data[8],imu_data[9]);

            while(U1STAbits.UTXBF==1)
                ;
            U1TXREG = SOH;
            while(U1STAbits.UTXBF==1)
                ;
            U1TXREG = STX;
            for(i=0;i<(IMU_DATA_SIZE*sizeof(float));i++)
            {
                while(U1STAbits.UTXBF==1)
                    ;
                U1TXREG = *(TXbyte + i);
//                TEST DATA
//                if(i%2!=0)
//                    U1TXREG = 0x41;
//                else
//                    U1TXREG = 0x8b;

            }
            while(U1STAbits.UTXBF==1)
                ;
            U1TXREG = ETX;

            // MIC
            if(audioTX==1){
                T2CONbits.TON = 1; // Timer Enable
            }
        }

        if(BT_command_rcv & 0x01)
        {
            Inertial_Read_Measure_Real(imu_data);
            MadgwickAHRSupdate(imu_data[3],imu_data[4],imu_data[5],imu_data[0],imu_data[1],imu_data[2],imu_data[6],imu_data[7],imu_data[8]);
            rho = acos(q0);
            imu_data[9] = rho*2;
            imu_data[10] = q1 / sin(rho);
            imu_data[11] = q2 / sin(rho);
            imu_data[12] = q3 / sin(rho);

            // MIC
            if(audioTX==1)
                T2CONbits.TON = 0; // Timer Disable

            while(U2STAbits.UTXBF==1)
                ;
            U2TXREG = SOH;
            while(U2STAbits.UTXBF==1)
                ;
            U2TXREG = STX;
            for(i=0;i<(IMU_DATA_SIZE*sizeof(float));i++)
            {
                while(U2STAbits.UTXBF==1)
                    ;
                U2TXREG = *(TXbyte + i);
            }
            while(U2STAbits.UTXBF==1)
                ;
            U2TXREG = ETX;

            // MIC
            if(audioTX==1){
                T2CONbits.TON = 1; // Timer Enable
            }
        }

#endif
    }
}

// <editor-fold defaultstate="collapsed" desc="UART USB INITIALIZZATION">

void __attribute__ ((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    #ifdef ECHO
    while(U1STAbits.URXDA /*&& U2STAbits.UTXBF*/) // Aspetto che il dato sia disponibile in RX1 e che il TX2 sia vuoto
    {
        while(U2STAbits.UTXBF)
            ;
        U2TXREG = U1RXREG;
    }
    U1STAbits.OERR=0;
    IFS0bits.U1RXIF = 0;
    #else
    
    char temp;
    
    if(U1RXREG==SOH)
    {
        
        while(U1STAbits.URXDA!=1);
        
        if(U1RXREG==STX)
        {
             
            while(U1STAbits.URXDA!=1);
            temp=U1RXREG;
             
             while(U1STAbits.URXDA!=1);
            if(U1RXREG!=ETX)
            {
                IFS0bits.U1RXIF = 0;
                return;
            }
            else
            {
                USB_command_rcv = temp;
                BT_command_rcv=0x00;
                if((USB_command_rcv & 0x02)==0x00)
                {
                    /* Disable ADC module */
                    AD1CON1bits.ADON = 0;
                    // Disable UART1 TX
                    //U1STAbits.UTXEN=0; // UARTx Transmit Enable bit (Transmit is disabled)
                    // Timer Disable
                    T1CONbits.TON = 0;
                    // MIC
                    T2CONbits.TON = 0; // Timer Disable
                    audioTX=0;
                }
                if((USB_command_rcv & 0x02))
                {
                    /* Enable ADC module and provide ADC stabilization delay */
                    AD1CON1bits.ADON = 1;
                    Delay_us(20);
                    // Enable UART1 TX
                    //U1STAbits.UTXEN=1; // UARTx Transmit Enable bit (Transmit is enabled, UxTX pin is controlled by UARTx)
                    // Timer Enable
                    T1CONbits.TON = 1;
                     // MIC
                    T2CONbits.TON = 1; // Timer Disable
                    audioTX=1;
                }

                IFS0bits.U1RXIF = 0;
                return;
            }
        }
        else
        {
            IFS0bits.U1RXIF = 0;
            return;
        }
    }
    else
    {
        IFS0bits.U1RXIF = 0;
        return;
    }
    
    U1STAbits.OERR=0;
    IFS0bits.U1RXIF = 0;
    #endif
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART BLUETOOTH INITIALIZZATION">

void UART_bt_init(void)
{
//    RPINR19bits.U2RXR = 67; // Set UART2 RX to RP67 pin, that corresponds to RD3 pin (see "pin table")
//    TRISDbits.TRISD3 = 1;
//    RPOR2bits.RP68R = 1; // Set UART2 TX to RP68 pin, that corresponds to RD4 pin
//    TRISDbits.TRISD4 = 0;

    // Set input/output UART2 pins
    RPINR19bits.U2RXR = 100; //Set UART2 RX to RP100 = RF4
    TRISFbits.TRISF4=1;
    RPOR9bits.RP101R = 0b000011; // Set UART2 TX to RP101 = RF5
    TRISFbits.TRISF5=0;

    TRISEbits.TRISE0 = 0; //reset bluetooth

    U2BRG = 3;
    /* Baud rate for 60MIP (See FRM Tables). Baud Rate = FCY/(16 * (BRG + 1))  (389 for 9600)*/
    /* 3 for 921600 kbps */
    /* 7 for 460800 kbps */
    /* 15 for 230400 kbps */
    /* 31 for 115200 kbps */
    /* 64 for 57600 kbps */
    /* 194 for 19200 kbps */
    /* Change U1BRG to suit your clock frequency */

    U2MODE = 0x0808; // Enable, 8-bit data, no parity, 1 stop bit, high speed
    U2STAbits.UTXISEL0=0; // UARTx Transmission Interrupt Mode Selection bits (Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty)
    U2STAbits.UTXISEL1=1;
    
    // Enable UART Interrupts
    IFS1bits.U2TXIF = 0;    // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 1;    // Enable UART Transmit Interrupts
    IFS1bits.U2RXIF = 0;    // Clear the Receieve Interrupt Flag
    IEC1bits.U2RXIE = 1;    // Enable UART Receieve Interrupts

    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1; // Enable TX
}

void __attribute__ ((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
#ifdef ECHO
    while (U2STAbits.URXDA /*&& U1STAbits.UTXBF*/) // Aspetto che il dato sia disponibile in RX2 e che il TX1 sia vuoto
    {
        while(U1STAbits.UTXBF)
            ;
        U1TXREG = U2RXREG;
    }
    U2STAbits.OERR=0;
    IFS1bits.U2RXIF = 0;
#else
    char temp;
    if(U2RXREG==SOH)
    {
        while(U2STAbits.URXDA!=1);

        if(U2RXREG==STX)
        {
            while(U2STAbits.URXDA!=1);
            temp=U2RXREG;
            
            while(U2STAbits.URXDA!=1);
            if(U2RXREG!=ETX)
            {
                IFS1bits.U2RXIF = 0;
                return;
            }
            else
            {
                BT_command_rcv = temp;
                USB_command_rcv=0x00;
                if((BT_command_rcv & 0x02)==0x00)
                {
                    /* Disable ADC module */
                    AD1CON1bits.ADON = 0;
                    // Disable UART1 TX
                    //U1STAbits.UTXEN=0; // UARTx Transmit Enable bit (Transmit is disabled)
                    // Timer Disable
                    T1CONbits.TON = 0;
                    // MIC
                    T2CONbits.TON = 0; // Timer Disable
                    audioTX=0;
                }
                if((BT_command_rcv & 0x02))
                {
                    /* Enable ADC module and provide ADC stabilization delay */
                    AD1CON1bits.ADON = 1;
                    Delay_us(20);
                    // Enable UART1 TX
                    //U1STAbits.UTXEN=1; // UARTx Transmit Enable bit (Transmit is enabled, UxTX pin is controlled by UARTx)
                    // Timer Enable
                    T1CONbits.TON = 1;
                     // MIC
                    T2CONbits.TON = 1; // Timer Disable
                    audioTX=1;

                }
                IFS1bits.U2RXIF = 0;
                return;
            }
        }
        else
        {
            IFS1bits.U2RXIF = 0;
            return;
        }
    }
    else
    {
        IFS1bits.U2RXIF = 0;
        return;
    }

    U2STAbits.OERR=0;
    IFS1bits.U2RXIF = 0;
#endif
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="SYSTEM INITIALIZZATION">


void InitializeSystem(void)
{
    // set all ports as digital (PORTF is only digital)
    ANSELB = 0x0001;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELG = 0x0000;

    // Set Input=1/Output=0
    TRISB = 0x0001;
    TRISC = 0x0000;
    TRISD = 0x0007;
    TRISE = 0x0000;
    TRISF = 0x0000;
    TRISG = 0x0000;

    /****** OSC Configuration ******/
    OSCTUN = 0x00; // FRC oscillator tuning register to nominal value (7.37 MHz)

    // Crystal frequency = 8MHz. Divide by 2, multiply by 15 and divide by 2.
    // Fosc = 30MHz. The CPU clock frequency is Fcy = Fosc/2 = 15MHz (15 MIPS)
    PLLFBD = 13; /* M  = 15	*/
    CLKDIVbits.PLLPOST = 0; /* N2 = 2	*/
    CLKDIVbits.PLLPRE = 0; /* N1 = 2	*/
    __builtin_write_OSCCONH(0x03); // set Clock Switch to Primary Osc with PLL (NOSC = 0x3)
    __builtin_write_OSCCONL(0x01); // switching request
    while (OSCCONbits.COSC != 3); // wait for clock switching (COSC = 0x3)
    while (OSCCONbits.LOCK != 1); // Wait for PLL to lock
}
//end InitializeSystem
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERRUPT INITIALIZZATION">
void Interrupt_init(void)
{
    INTCON1bits.NSTDIS=0; // Interrupt nesting enable
    INTCON2bits.GIE = 1; // Global Interrupt Enable
    IEC0bits.AD1IE = 1; // ADC Interrupt Enable
}

//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="I2C INITIALIZZATION">
void I2C_init(void)
{
    ODCDbits.ODCD9=0;       //Configre SCA/SDA pin as open-drain
    ODCDbits.ODCD10=0;

    I2C1CONbits.A10M=0;     //I2C2ADD register is a 7-bit slave address
    I2C1CONbits.ACKDT=0;
    I2C1BRG=23;            //Baud Rate Generator Reload Register
    /* Baud rate for 10MIP (See FRM Tables). Baud Rate = 1 / (((1 / FCY) * (BRG + 1)) * e-7) */
    /* 98 for 100000 kbps */
    /* 23 for 400000 kHz */
    /* Change I2C1BRG to suit your clock frequency */
    I2C1CONbits.DISSLW=0;

    I2C1ADD=0x0000;
    I2C1MSK=0x0000;

    IFS1bits.MI2C1IF = 0;
    IEC1bits.MI2C1IE = 1;

    I2C1CONbits.I2CEN = 1;
}

extern unsigned int Done;
void __attribute__((__interrupt__,__auto_psv__)) _MI2C1Interrupt(void)
{
    Done=1;
    IFS1bits.MI2C1IF=0;
}
//</editor-fold>