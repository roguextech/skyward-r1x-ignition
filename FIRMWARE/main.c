/****************************************************************************************************************************************************************************
 *  FILE NAME     : main.c
 *  Version       : 1.0
 *  Description   : Ignition Firmware for ROCKSANNE I-X
 *  Coder         : Matteo Franceschini
 *  Email         : matteo.franceschini@skywarder.eu
 *  Target        : PIC 16F1824
 *  Compilator    : HI-TECH C PRO v 9.83
 *  IDE           : Microchip MPLAB X  v1.95
 *  Programmer    : PICKIT 3
 *  Creation date : 20.03.2014
 *  Copyright     : SKYWARD EXPERIMENTAL ROCKETRY
 *
 * DESCRIPTION
 * The program uses the TIMER 1 with clock input on pin RA5 to recognize the frequency of a PWM signal from the main
 * board (YodaBoard). It supports two different frequencies:
 * -> The IGNITION_MIN~IGNITION_MAX range defines that we are ready to launch and we will activate the gate of the MOS that ignites the spark to start the rocket.
 * -> The YODA_MIN~YODA_MAX range is a "link test" frequency that turns on the green led on the board to let the user know that everything is fine
 *   on the link between the YodaBoard and this circuit.
 *
 * The program is kept very simple for fast usage. A future version may use interrupts with a timer instead of a blocking delay function in main (highly inaccurate)..
 *
 * The PCB is designed by Stefano  (stefano.marino@skywarder.eu), with revision 3.3.
 * 
 *
 * RELEASE HISTORY:
 * //--> 23.05.2014  VERSION 1.0  ==> First setup and test. Everything seems to work.
*/

/***************************************************************************************************************
 *                                                 LIBRARIES                                                   *
 ***************************************************************************************************************/
#include <htc.h> //default library for basic registers defines

/***************************************************************************************************************
 *                                                   DEFINE                                                    *
 ***************************************************************************************************************/

//PORTS
#define LED_IGNITION    LATCbits.LATC0 ///> Led for ignition signalling pin
#define LED_LINK        LATCbits.LATC1 ///> Led for succesful linkage with the main board pin
#define MOS_GATE        LATCbits.LATC5 ///> MOS gate pin
#define INPUT_DISABLE   LATAbits.LATA4 ///> Pin to keep the counter stopped by hardware (shortcircuited to the RC5 input)

//COSTANTS
#define _XTAL_FREQ      16000000    ///> Necessary for hi-tech c delay routines
#define IGNITION_MIN    300         ///> minimum frequency in hertz accepted for the ignition of the spark plug
#define IGNITION_MAX    600         ///> maximum frequency in hertz accepted for the ignition of the spark plug
#define YODA_MIN        4500        ///> minimum frequency in hertz accepted for the link check
#define YODA_MAX        5500        ///> maximum frequency in hertz accepted for the link check

//GENERAL UTILITY
#define ON          1
#define OFF         0
#define TRUE        1
#define FALSE       0
#define SET         1
#define CLEAR       0
#define INPUT       1
#define OUTPUT      0

/***************************************************************************************************************
 *                                          CONFIGURATION WORDS                                                *
 ***************************************************************************************************************/

//! The first Configuration Word.
    /*!
      \param FOSC_INTOSC        Uses the internal oscillator, the CLKIN pin and is set as I/O.
      \param WDTE_OFF           WATCHDOG disabled
      \param PWRTE_OFF          POWER UP TIMER disabled. Waits for the oscillator to stabilize before starting the program.
      \param MCLRE_OFF          MCLR Pin is digital input
      \param CP_OFF             CODE PROTECTION disabled
      \param CPD_OFF            DATA MEMORY PROTECTION disabled
      \param BOREN_OFF          BROWN OUT RESET disabled
      \param CLKOUTEN_OFF       CLOCKOUT disabled on CLKOUT
      \param IESO_OFF           INTERNAL-EXTERNAL SWITCHOVER disabled
      \param FCMEN_OFF          FAIL-SAFE MONITOR disabled

    */

__CONFIG(FOSC_INTOSC & WDTE_SWDTEN & PWRTE_OFF & MCLRE_OFF & CP_OFF & CPD_OFF & BOREN_OFF & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);


__CONFIG(WRT_OFF & PLLEN_OFF & STVREN_OFF & BORV_HI & LVP_OFF);

//! The second Configuration Word.
    /*!
      \param WRT_OFF            FLASH MEMORY WRITE PROTECTION disabled
      \param PLLEN_OFF          4x PLL disabled
      \param STVREN_OFF         STACK OVERFLOW/UNDERFLOW RESET disabled
      \param BORV_HI            BROWN OUT RESET VOLTAGE SELECTED: 2.5V
      \param LVP_OFF            LOW VOLTAGE PROGRAMMING disabled, if enabled MCLR is enabled by default.

    */


/***************************************************************************************************************
 *                                          GLOBAL VARIABLES                                                   *
 ***************************************************************************************************************/

unsigned int freq = 0; //Variable that has the last ridden frequency

/***************************************************************************************************************
 *                                                 FUNCTIONS                                                   *
 ***************************************************************************************************************/

//\brief Initialization function
void init()
{
    //--OSCILLATOR--//------------------------------------------------------------------------------------------------

    OSCCON=0b01111010;   // 0       --> spll disabled (it works only if activated in the configuration word)
                         // 1111    --> 16 Mhz
                         // 0       --> not used
                         // 1x      --> System Clock Select, internal clock

    //--OPTION REGISTER--//-------------------------------------------------------------------------------------------

    OPTION_REG=0b10001000; // 1   --> Weak pull up disabled
                           // 0   --> Interrupt on rising edge on RA2 disabled
                           // 0   --> TMR0 uses internal clock
                           // 0   --> TMR0 increments with low-to-high
                           // 1   --> prescaler to WDT
                           // 000 --> prescaler is 1:2

    //--WATCHDOG--//--------------------------------------------------------------------------------------------------

    WDTCON = 0b000000000;  // 00        --> not used
                           // 00000     --> 1ms prescaler
                           // 00        --> Watchdog off

   //--INPUT/OUTPUT--//----------------------------------------------------------------------------------------------

    ANSELA = 0b00000000; // we only use digital logics
    ANSELC = 0b00000000;

    INLVLA = 0b00000000; //every input is TTL, we have 2v as logic "1". With schmitt trigger it would be 0.8VDD
    TRISA = 0b00111000;  //details follow below
    TRISC = 0b00000000;  //details follow below



//    TRISAbits.TRISA0=OUTPUT; //PIN DAC not used
//    TRISAbits.TRISA1=OUTPUT; //PIN not used
//    TRISAbits.TRISA2=OUTPUT;  //PIN not used
//    TRISAbits.TRISA3=INPUT;  //VPP not used (only for programming, input only pin)
//    TRISAbits.TRISA4=INPUT; //PIN for optional clock bypass (it "disables" the clock from the YodaBoard by forcing 0V on RA5).
                                //we're not using it right now, we keep it as input.
//    TRISAbits.TRISA5=INPUT;  //PIN for clock input from YodaBoard
//
//    ANSELAbits.ANSA0 = DIGITAL;//PIN  DIGITAL
//    ANSELAbits.ANSA1 = DIGITAL;//PIN DIGITAL
//    ANSELAbits.ANSA2 = DIGITAL;//PIN DIGITAL
//    ANSELAbits.ANSA4 = DIGITAL;//PIN DIGITAL   

//    TRISCbits.TRISC0=OUTPUT; //PIN LED 1
//    TRISCbits.TRISC1=OUTPUT;  //PIN LED 2
//    TRISCbits.TRISC2=OUTPUT; //PIN not used
//    TRISCbits.TRISC3=OUTPUT;  //PIN not used
//    TRISCbits.TRISC4=OUTPUT; //PIN not used
//    TRISCbits.TRISC5=OUTPUT;  //PIN MOS GATE
//
//    ANSELCbits.ANSC0 = DIGITAL; //PIN DIGITAL
//    ANSELCbits.ANSC1 = DIGITAL; //PIN DIGITAL
//    ANSELCbits.ANSC2 = DIGITAL;//PIN DIGITAL
//    ANSELCbits.ANSC3 = DIGITAL;//PIN DIGITAL


    //PORTS RESET.
    PORTA = 0;
    PORTC = 0;

    //LATCHS RESET
    LATA = 0;
    LATC = 0;


    //--CAPACITIVE SENSING--//

    CPSCON0 = 0b00000000; //disabled

    //--COMPARATORS--//

    CM1CON0 = 0b00000000; //disabling first comparator
    CM1CON1 = 0b00000000;
    CM2CON0 = 0b00000000; //disabling second comparator
    CM2CON1 = 0b00000000;

    //--FIXED VOLTAGE REFERENCE--//
    FVRCON = 0b00000000; //we're not using it

    //--DATA SIGNAL MODULATOR--//
    MDCON = 0b00000000; //disabled

    
    //--A/D CONVERTER--//-------------------------------------------------------------------------------------------

/*  ________________________________________________________________________________
    |   PORTA   |      ANALOG PORT      |   ADCON0   |         FUNCTION            |
    -------------------------------------------------------------------------------|
    |   RA1     |           AN1         | 0b00000111 |  debug only, not used       |
    |------------------------------------------------------------------------------|
*/

    ADCON1=0b00100000;   //0        --> Left justified
                         //010      --> FOSC/32
                         //0        --> not used
                         //0        --> negative ref is VSS
                         //00       --> positive ref is VDD

    //--D/A CONVERTER--//-------------------------------------------------------------------------------------------


    DACCON0=0b01000000;  // 0   --> DAC is disabled
                         // 1   --> DAC Positive reference source selected
                         // 0   --> DAC logics are off
                         // 0   --> not used
                         // 01  --> positive source is FVR BUFFER 2
                         // 00  --> not used
    DACCON1=0b00000000;  // 5 bit with the DAC value


    //--PWM--//--------------------------------------------------------------------------------------------------------

    CCP1CON = 0b00000000;  // 00      --> PWM MODE, single output, P1A modulated, P1B,C,D are I/O.
                           // 00      --> LSBs of PWM duty cycle (called DC1B 1 and 2)
                           // 0000    --> PWM OFF

    CCPR1L = 0;            // PWM duty cycle = 0%

    
    PSTR1CON = 0b00000000; // we're using default pins



    //--TIMER1--//-----------------------------------------------------------------------------------------------------

    T1CON = 0b10000100;  // 10      --> TMR1CS Timer1 clock source is pin
                         // 00      --> T1CKPS 1:1 prescaler
                         // 0       --> T1OSCEN TMR1 dedicated oscillator disabled
                         // 1       --> T1SYNC do not sync TMR1 with FOSC
                         // 0       --> Not used
                         // 0       --> TMR1ON timer off
    
    T1GCON = 0b01000100; // 0       --> TMR1GE  gate function ignored
                         // 1       --> T1GPOL  TMR1 counts if gate is high (not used bc TMR1GE is off)
                         // 0       --> T1GTM timer1 toggle mode disabled
                         // 0       --> T1GSPM single pulse mode disabled
                         // 0       --> T1GGO/nDONE single pulse acquisition not started
                         // 0       --> T1GVAL gate current state bit
                         // 00      --> T1GSS timer1 gate select: gate pin
    


    //--TIMER2 --//-------------------------------------------------------------------------------------------

    PR2=0xFF; //defines the PWM period (not used!)

    T2CON = 0b00000001;  // 0       --> not used
                         // 0000    --> Postscaler  1:1
                         // 0       --> TMR2 OFF
                         // 01      --> Prescaler set to 1:4 (with 16 Mhz clock and PR2=110 --> 9kHz PWM)

    //--TIMER4 --//--------------------------------------------------------------------------------------

    PR4 = 0xFF;          //defines the PWM period (not used!)

    T4CON = 0b00000000;  // 0       --> not used
                         // 0000    --> Postscaler  1:1
                         // 0       --> TMR4 OFF
                         // 01      --> Prescaler set to 1:4 (with 16 Mhz clock and PR2=110 --> 9kHz PWM)

    //--TIMER6--//--------------------------------------------------------------------------------------

    PR6 = 0xFF; //definisce a che valore avviene l'interrupt
    T6CON = 0b00000010;  // 0       --> not used
                         // 0000    --> Postscaler  1:1
                         // 0       --> TMR2 OFF
                         // 01      --> Prescaler set to 1:4 (with 16 Mhz clock and PR2=110 --> 9kHz PWM)
    //--INTERRUPT--//--------------------------------------------------------------------------------------------------

    INTCON = 0b00000000; // 0       --> Global interrupt disabled  (GIE)
                         // 0       --> Peripheral interrupt disabled (PEIE)
                         // 0       --> Interrupt di TMR0 disabled (TMR0IE)
                         // 0       --> External Interrupt disabled (define in OPTION_REG if pullup or pulldown) on pin INT (RA2) (INTE)
                         // 0       --> Interrupt on change disabled  (IOCIE)
                         // 0       --> Flag  TMR0 Overflow (TMR0IF)
                         // 0       --> Flag  External Interrupt (INTF)
                         // 0       --> Flag  interrupt on change (IOCIF)

    PIR1 = 0;              // Reset PIE1 interrupts flags
    PIR2 = 0;              // Reset PIE2 interrupts flags
    PIR3 = 0;              // Reset PIE3 interrupts flags

    PIE1 = 0b00000000;     // 0       --> TMR1 Gate Interrupt disabled
                           // 0       --> A/D converter interrupt disabled
                           // 0       --> USART RECEIVE interrupt disabled
                           // 0       --> USART TRANSMIT interrupt disabled
                           // 0       --> Serial Port interrupt disabled
                           // 0       --> CCP1 interrupt disabled
                           // 0       --> TMR2 to PR2 Match interrupt disabled
                           // 0       --> TMR1 overflow interrupt disabled

    PIE2 = 0b00000000;     // 0       --> Oscillator fail interrupt disabled
                           // 0       --> Interrupt comparator C2 disabled
                           // 0       --> Interrupt comparator C1 disabled
                           // 0       --> Interrupt EEPROM scrittura completata disabled
                           // 0       --> Serial Port Collision interrupt disabled
                           // 000     --> not used

    PIE3 = 0b00000000;     // 00      --> not used
                           // 0       --> Interrupt comparator C4 disabled
                           // 0       --> Interrupt comparator C3 disabled
                           // 0       --> Interrupt TMR6 to PR6 disabled
                           // 0       --> not used
                           // 0       --> TMR4 to PR4 disabled
                           // 0       --> not used

}


//\brief Delay function
// Input the number of desired delay milliseconds. MAX 65535.
void delayerMs(unsigned int delay)
{
        unsigned int i = 0; //variabile per il ciclo for
        
        for(i=0;i<delay;i++) //ritardo di delay ms
                 __delay_ms(1); 
}


/***************************************************************************************************************
 *                                                   MAIN                                                      *
 ***************************************************************************************************************/

void main(void)
 {
   unsigned int i = 0; //temp variable used in for cycle
   
   init(); // initializing the system
         
   for(i=0;i<5;i++) //a fast led cycle to visually check they're working at startup
   {
           LED_IGNITION = ON;
           LED_LINK = OFF;
           delayerMs(50);
           LED_IGNITION = OFF;
           LED_LINK = ON;
           delayerMs(50);
   }
   
   TMR1H = 0; //resetting the TMR1 values (it's a 16 bit number, in two registers!)
   TMR1L = 0;
  
   INPUT_DISABLE=OFF; //This disables the input if later we set TRIS-A4 bit to output (debug only)

   while (TRUE) //infinite loop, almost once a second it checks the actual frequency.
   {
        TMR1ON = ON; //we turn on the timer
        delayerMs(1000); //we wait (about) a second
        TMR1ON = OFF; //we turn off the timer
        //TRISAbits.TRISA4=OUTPUT; //Debug only, this makes impossible for the clock to reach the TMR1 counter pin.
            
        freq = TMR1H; //These are the higher 8 bits
        freq = ((freq<<8)|(TMR1L));  //we shift the higher bits by eight places up, and OR it with the lower eight. (to recover the 16bit word for easier use in code)
        
       if(freq>= IGNITION_MIN && freq<= IGNITION_MAX) //Checking if it's the frequency for ignition
       {
                LED_IGNITION = ON; //turning on the ignition led
                LED_LINK = OFF; //turning off the link led (because is for test only)
                MOS_GATE = ON; //Turning on the MOSFET (giving power to the spark plug)
       }
       else if(freq>=YODA_MIN && freq<=YODA_MAX) //if it's not for ignition, maybe it's for signal check
       {
            LED_IGNITION = OFF; //if it's for link check, this led should be off.
            LED_LINK = ~LED_LINK; //when link checking, this led blinks like a heartbeat (constantly on means only that the board is powered on!)
            MOS_GATE = OFF; //the spark plug must be off, it's a good thing to remember it!
       }
       else //if it's none of the above, I'm just waiting for connection
       {
           LED_IGNITION = OFF; //ignition led is off because
           MOS_GATE = OFF; //the MOSFET (and spark plug) is off
           LED_LINK = ON; // but the led link is CONSTANTLY on, indicating that the processor is succesfully powered on and waiting.
       }
        TMR1H = 0; //resetting the values, for the next readings.
        TMR1L = 0;
       
   }
      
 }