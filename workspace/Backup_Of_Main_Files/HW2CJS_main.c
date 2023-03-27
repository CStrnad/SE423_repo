//This code is being commented by Chris Strnad. Search for CJS.
//#############################################################################
// FILE:   HW2CJS_main.c
//
// TITLE:  HW2 Code for SE423. Strnad, Christopher
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCA_isr(void);    //CJS Predefinition of ADCA ISR Void function

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint16_t updown = 0;    //CJS global variable to control the count for CMPA.
int16_t rawADCINA4 = 0; //CJS global variable for raw ADCINA4 data
float scaledADCINA4 = 0;    ///CJS global variable for scaled voltage conversion of Raw ADCINA4 data.
int32_t countADA1 = 0;


//LED Stack Control Function
void SetLEDsOnOff(int16_t LEDvalue)
{
    if((LEDvalue & 0x80) == 0x80) {
        GpioDataRegs.GPDSET.bit.GPIO111 = 1;
        GpioDataRegs.GPASET.bit.GPIO27 = 1;
        GpioDataRegs.GPESET.bit.GPIO159 = 1;
        GpioDataRegs.GPFSET.bit.GPIO160 = 1;
    }
    else {
        GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
        GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;
    }
    if((LEDvalue & 0x100) == 0x100) {
        GpioDataRegs.GPDSET.bit.GPIO97 = 1;
        GpioDataRegs.GPASET.bit.GPIO26 = 1;
        GpioDataRegs.GPESET.bit.GPIO158 = 1;
    }
    else {
        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
    }
    if((LEDvalue & 0x200) == 0x200) {
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;
        GpioDataRegs.GPASET.bit.GPIO25 = 1;
        GpioDataRegs.GPESET.bit.GPIO157 = 1;
    }
    else {
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
    }
    if((LEDvalue & 0x400) == 0x400) {
       GpioDataRegs.GPCSET.bit.GPIO94 = 1;
       GpioDataRegs.GPESET.bit.GPIO131 = 1;
       GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    }
    else {
       GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
       GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
       GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    }
    if((LEDvalue & 0x800) == 0x800) {
        GpioDataRegs.GPESET.bit.GPIO130 = 1;
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
    }
    else {
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
    }
}


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCA1_INT   = &ADCA_isr;           //CJS Create interrupt mapping for ADCA1. HW2 Exercise 3.

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    //CJS setting bitfield registers for PWM in Exercise 2.
    //CJS Set TBCTL options
    EPwm12Regs.TBCTL.bit.CTRMODE = 0;       //CJS - Counter Mode to Count Up
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 3;    //CJS - Free soft emulation to Free Run
    EPwm12Regs.TBCTL.bit.PHSEN = 0;         //CJS - Disable time-base counter @ phase register
    EPwm12Regs.TBCTL.bit.CLKDIV =  0;     //CJS - Set clock div to div by 1.

    //CJS Set TBCTR options
    EPwm12Regs.TBCTR = 0;         //CJS Start time @ zero.

    //CJS Set TBPRD options
    EPwm12Regs.TBPRD = 10000;      //CJS - Period freq set to 5KHz (of 200msec)

    //CJS Set CMPA options
    EPwm12Regs.CMPA.bit.CMPA = 0; //CJS - Start duty cycle at 0%
    //Was originally 4951, but changed to 0.

    //CJS Set AQCTLA options
    EPwm12Regs.AQCTLA.bit.CAU = 1; //
    EPwm12Regs.AQCTLA.bit.ZRO = 2; //CJS - set PWM12A out-pin to LOW when CMPA reached. Pin HIGH when TBCTR register = zero.

    //CJS Set TBPHS options
    EPwm12Regs.TBPHS.bit.TBPHS = 0; //CJS - Phase = zero.

    //CJS Override GPIO22 to use EPWM12A - HW2 Exercise 2
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);   //CJS Set pin 22 to use PWM instead of GPIO

    //CJS Setting protected registers such that the pull-up resistor is disabled - HW2 Exercise 2
    EALLOW; //CJS Set protected registers (Below are protected registers)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; //CJS Disable pull-up resistor.
    EDIS;   //CJS end of protected registers.

    //--------------------------- Start of HW 2 Exercise 3 Register code ----------------------
    //CJS Register setup for HW2 Exercise 3 - Setting up ADCA for Photo Resistor polling.
    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD //CJS Set register to 010 to enable time-base counter equal to period (TBCTR = TBPRD)
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”) //CJS Geenrate the EPWMxSOCA pulse/trigger on the first event: ETPS[SOCACNT]=0.1
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode //CJS Set to 0x00 to set Counter mode to "Up-count mode"
    EDIS;

    //CJS Register configuration code for ADCA in HW2 Exercise 3/4
    EALLOW;
    //write configurations for ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;//SOC0 will convert Channel you choose Does not have to be A0 //CJS Set SOC0 to sample channel 0x1.
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11;// EPWM4 ADCSOCA // CJS Set TRIGSEL trig source to ADCTRIG11 - ePWM4, ADCSOCA (0B in hex).
    //AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2;//SOC1 will conv Channel you choose Does not have to be A1    //CJS Set ADC CHSEL to convert channel ADCIN1 (0x1 in hex).
    //AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM4 ADCSOCA
    //AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3;//SOC2 will conv Channel you choose Does not have to be A2
    //AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 4;// EPWM4 ADCSOCA
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL= 0;//set to last or only SOC that is converted and it will set INT1 flag ADCA1  //CJS Set INT1SEL to 0x0B so that the trigger is set to OEC11, just like CHSEL?
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    //--------------------------- END of HW 2 Exercise 3 Register code --------------------------

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;  //CJS INT1 already enabled for both Timer0 and ADCA1.
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // CJS Enable TINT0 in the PIE: Group 1 interrupt 1.
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
//				serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            serial_printf(&SerialA, "ADC Voltage Value: %.2f\r\n", scaledADCINA4);
            UARTPrint = 0;
        }
    }
}

//CJS Create interrupt function for ADCA
__interrupt void ADCA_isr(void){
    //CJS Read the value of ADCINA4. Values will range from 0.0 to 3.0 divided by 4095.

    rawADCINA4 = AdcaResultRegs.ADCRESULT0;   //CJS - Use global var int16_t rawADCINA4 to store and get reading.

    scaledADCINA4 = rawADCINA4 / 4095.0 * 3.0;    //CJS - Convert raw value to voltage value and write to scaledADCINA4.

    if (countADA1 % 100 == 1) {
        UARTPrint = 1; //CJS Trigger UARTPrint Function
        EPwm12Regs.CMPA.bit.CMPA = scaledADCINA4 / 3 * EPwm12Regs.TBPRD; //CJS Set dim based on PT.
        SetLEDsOnOff(rawADCINA4);
    }
    countADA1++;    //CJS Incriment counter for ADA1.

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //CJS Clear interrupt flag.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //CJS Not sure what this does YET.

//    EPwm12Regs.CMPA.bit.CMPA = scaledADCINA4 / 3 * EPwm12Regs.TBPRD; //CJS Set dim based on PT.
//
//    SetLEDsOnOff(rawADCINA4);
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
//        displayLEDletter(LEDdisplaynum);  //CJS Commented out for HW2.
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	
	
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    //if ((CpuTimer2.InterruptCount % 2 == 0)){ //CJS Every 0.1 seconds do something
//    if (updown == 1) {
//        EPwm12Regs.CMPA.bit.CMPA++;  //CJS Incriment CPMA until it reached TBPRD
//        if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) updown = 0;
//    }
//    if (updown == 0) {
//        EPwm12Regs.CMPA.bit.CMPA--; //CJS Decrement CPMA until it reaches Zero
//        if (EPwm12Regs.CMPA.bit.CMPA == 0) updown = 1;
//    }
   // }
	
	if ((CpuTimer2.InterruptCount % 50) == 0) {
//		UARTPrint = 1;
	}
}

