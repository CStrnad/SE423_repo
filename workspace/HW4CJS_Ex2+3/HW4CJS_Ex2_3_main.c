//#############################################################################
// FILE:   HW4CJS_Ex2+3_main.c
//
// TITLE:  HW4 Code for Exercise 3 regarding Servo Motor control.
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

volatile uint32_t Xint1Count=0;
volatile uint32_t Xint2Count=0;

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void xint1_isr(void);
__interrupt void xint2_isr(void);

// Other Function predefinitions.
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//Servo Control Variables
uint16_t desiredComparatorA = 0;
uint16_t desiredComparatorB = 0;
float desiredAngleA = 0;
float desiredAngleB = 0;
uint16_t zeroLock = 0;
uint16_t speed = 10;
uint16_t countUpA = 0;
uint16_t countUpB = 0;




// CJS Function to display numerically the button pressed using LEDs on the Green Board
uint16_t readbuttons(void) {
    uint16_t returnvalue = 0;

    if (GpioDataRegs.GPADAT.bit.GPIO4 == 0) {   //CJS Button 1
        returnvalue |= 1;
    }
    if (GpioDataRegs.GPADAT.bit.GPIO5 == 0) {   //CJS Button 2
        returnvalue |= 2;
    }
    if (GpioDataRegs.GPADAT.bit.GPIO6 == 0) {   //CJS Button 3
        returnvalue |= 4;
    }
    if (GpioDataRegs.GPADAT.bit.GPIO7 == 0) {   //CJS Button 4
        returnvalue |= 8;
    }

    return(returnvalue);
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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.XINT1_INT = &xint1_isr;
    PieVectTable.XINT2_INT = &xint2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 5000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIB(&SerialB,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    //------------ Servo EPWM Setup --------------------
    //EPWM 8 A & B
    EALLOW;
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;       //CJS - Counter Mode to Count Up
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 3;    //CJS - Free soft emulation to Free Run
    EPwm8Regs.TBCTL.bit.PHSEN = 0;         //CJS - Disable time-base counter @ phase register
    EPwm8Regs.TBCTL.bit.CLKDIV =  6;     //CJS - Set clock div to div by 64.(6 is decimal for binary of 110b)
    EPwm8Regs.TBCTR = 0;         //CJS Start time @ zero.
    EPwm8Regs.TBPRD = 15625;      //CJS - Period freq set to 50Hz
    EPwm8Regs.CMPA.bit.CMPA = 1250; //CJS - Start duty cycle at 8% (Such that Servo is at 0 degrees)
    EPwm8Regs.AQCTLA.bit.CAU = 1; //
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //CJS - set PWM12A out-pin to LOW when CMPA reached. Pin HIGH when TBCTR register = zero.
    EPwm8Regs.TBPHS.bit.TBPHS = 0; //CJS - Phase = zero.

    EPwm8Regs.CMPB.bit.CMPB = 1250; //CJS - Start duty cycle at 0%
//    EPwm8Regs.AQCTLB.bit.CAU = 1; //
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //CJS - set PWM8B out-pin to LOW when TBCTR = 0.
    EPwm8Regs.AQCTLB.bit.CBU = 1; //CJS - Set PWM8B high when tbctr = cmpb
    EDIS;

    //CJS Override Servo Pins to use EPWM8 A and B
//    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);   //CJS Set pin 22 to use PWM instead of GPIO
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //CJS Set pin 14 (Servo 1) to use EPWM8A
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); //CJS Set pin 15 (Servo 2) to use EPWM8B

    //CJS Setting protected registers such that the pull-up resistor is disabled
    EALLOW; //CJS Set protected registers (Below are protected registers)
//    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; //CJS Disable pull-up resistor.
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; //CJS Disable pull-up resistor.
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; //CJS Disable pull-up resistor.
    EDIS;   //CJS end of protected registers.

    //------------ END SERVO EPWM Setup --------------------

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4  Xint1
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5  Xint2
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    EALLOW;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2;        //CJS set XINT1 to trigger on PB3 .
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO4 = 2;        // XINT1 Qual using 6 samples
    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;   // Each sampling window

    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2;        //CJS set XINT2 to trigger on PB4
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 2;        // XINT2 Qual using 6 samples
    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;   // Each sampling window is 510*SYSCLKOUT
    EDIS;

    // GPIO4 is XINT1, GPIO5 is XINT2
//    GPIO_SetupXINT1Gpio(4);
//    GPIO_SetupXINT2Gpio(5);
    GPIO_SetupXINT1Gpio(6);
    GPIO_SetupXINT2Gpio(7);

    // Configure XINT1 XINT2
    XintRegs.XINT1CR.bit.POLARITY = 0;          // Falling edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 0;          // Falling edge interrupt

    // Enable XINT1 and XINT2
    XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
    XintRegs.XINT2CR.bit.ENABLE = 1;            // Enable XINT2

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
//            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            serial_printf(&SerialA,"Servo A angle: %.3f\tCMPA: %ld ,\tServo B angle: %.3f\tCMPB: %ld\r\n",desiredAngleA, desiredAngleB, desiredComparatorA, desiredComparatorB);
            serial_printf(&SerialA,"NumXInt1=%ld,NumXInt2=%ld\r\n",Xint1Count,Xint2Count);
            UARTPrint = 0;
        }
    }
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

    if ((numTimer0calls%10) == 0) {
        UARTPrint = 1;
        //Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

        if (readbuttons()==0) {
            displayLEDletter(LEDdisplaynum);
            LEDdisplaynum++;
            if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
                LEDdisplaynum = 0;
            }
        } else {
            displayLEDletter(readbuttons());
        }
    }

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


//    if ((CpuTimer2.InterruptCount % speed) == 0) {
//        if(zeroLock == 1) {
//                setEPWM8A_RCServo(0);
//        }
//        else{
//            if(countUpA == 1){
//                desiredAngleA = desiredAngleA + 2;
//                setEPWM8A_RCServo(desiredAngleA);
//                if(desiredAngleA > 90) countUpA = 0;
//            }
//            if(countUpA == 0){
//                desiredAngleA = desiredAngleA - 2;
//                setEPWM8A_RCServo(desiredAngleA);
//                if(desiredAngleA < -90) countUpA = 1;
//            }
//        }
//
//    }
//    if ((CpuTimer2.InterruptCount % speed) == 0) {
//        if(zeroLock == 1) {
//            setEPWM8B_RCServo(0);
//        }
//        else{
//            if(countUpB == 0){
//                desiredAngleB = desiredAngleB + 2;
//                setEPWM8B_RCServo(desiredAngleB);
//                if(desiredAngleB > 90) countUpB = 1;
//            }
//            if(countUpB == 1){
//                desiredAngleB = desiredAngleB - 2;
//                setEPWM8B_RCServo(desiredAngleB);
//                if(desiredAngleB < -90) countUpB = 0;
//            }
//        }
//    }


//    if ((CpuTimer2.InterruptCount % 50) == 0) {
//        if(servoLoopMarker == 1){
//            setEPWM8A_RCServo(90);
//            setEPWM8B_RCServo(-90);
//            servoLoopMarker = 0;
//        }
//        if(servoLoopMarker == 0){
//            setEPWM8A_RCServo(-90);
//            setEPWM8B_RCServo(90);
//            servoLoopMarker = 1;
//        }
//    }
}

//Center Angle is 0* at 1250
//+90* at 1875
//-90* at 625
//Use slope y = 6.94x + 1250

//CJS Function to set the angle of Servo A
void setEPWM8A_RCServo(float angle) {
    //CJS "Saturate" input first to prevent over-turning servo motor.
    if(angle < -90.0) angle = -90.0;
    if(angle >  90.0) angle =  90.0;

    //Get CMPA value conversion.
    desiredComparatorA = 6.94 * angle + 1250.0;

    //Set CMPA to calculated value.
    EPwm8Regs.CMPA.bit.CMPA = desiredComparatorA;
}

//CJS Function to set the angle of Servo B
void setEPWM8B_RCServo(float angle) {
    //CJS "Saturate" input first to prevent over-turning servo motor.
    if(angle < -90.0) angle = -90.0;
    if(angle >  90.0) angle =  90.0;

    //Get CMPB value conversion.
    desiredComparatorB = 6.94 * angle + 1250.0;

    //Set CMPB to calculated value.
    EPwm8Regs.CMPB.bit.CMPB = desiredComparatorB;
}

// xint1_isr - External Interrupt 1 ISR
interrupt void xint1_isr(void)
{
    Xint1Count++;
    desiredAngleA = desiredAngleA + 10;     //CJS Increment the angle of Servo 1 by 10 degrees.
    setEPWM8A_RCServo(desiredAngleA);
    // Acknowledge this interrupt to get more from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// xint2_isr - External Interrupt 2 ISR
interrupt void xint2_isr(void)
{
    Xint2Count++;
    desiredAngleA = desiredAngleA - 10;    //CJS Decrement the angle of Servo 1 by 10 degrees.
    setEPWM8A_RCServo(desiredAngleA);

    // Acknowledge this interrupt to get more from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

