#include "F28x_Project.h"
void Gpio_select(void);
void Setup_ePWM(void);
//void Setup_ePWM7(void);
volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs};
float adca_vload=0;
float adcb_vload=0;
void Setup_ePWM6(void);
interrupt void adcb1_isr(void);
void Setup_ADC_conf(void);
// Function Prototype
float vdc1=0;
float vdc2=0;
float vdc=0;
float vdc_ref=30;
float vdcerr=0;
float ib_ref;
float ib1=0;
float ib2=0;
float ib=0;
float t_s = 0;
float iberr=0;
float Ib_ref_limit=0;
int PWM6_CMP = 0;
#define DACA                  1
#define REFERENCE_VREF        1
#define REFERENCE             REFERENCE_VREF
#define DAC_NUM               DACA
void configureDAC(Uint16 dac_num);


void main(void)
{
InitSysCtrl(); // Initialize System Control
// Clear all interrupts and initialize PIE vector table
DINT; // Disable all interrupts
Gpio_select(); // GPIO9, GPIO11, GPIO34 and GPIO49 as output
// to 4 LEDs at Peripheral Explorer)
Setup_ePWM(); // init of ePWM1A
//Setup_ePWM7();
Setup_ePWM6();
Setup_ADC_conf();
configureDAC(DAC_NUM);
InitPieCtrl(); // Initialize the PIE control registers to their default state
// Clear all interrupts and initialize PIE vector table:
IER = 0x0000;
IFR = 0x0000;
InitPieVectTable(); // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR)
EALLOW;
PieVectTable.ADCB1_INT = &adcb1_isr; //function for ADCA interrupt
EDIS;
PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
IER |= 1; // enable INT1 for ADC
// Enable global Interrupts and higher priority real-time debug events:
EINT; // Enable Global interrupt INTM
ERTM; // Enable Global realtime interrupt DBGM
}


void Gpio_select(void)
{
EALLOW;
GpioCtrlRegs.GPAMUX1.all = 0; // GPIO15 ... GPIO0 = General Puropse I/O
GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // ePWM2A active
GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1; // ePWM7A active
GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1; // ePWM7B active
GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; //epwm6A active
GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // ePWM6B active
EALLOW;
GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;  // GPIO6
GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;

GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // Disable pull-up on GPIO2 (EPWM1A)
GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1; // Disable pull-up on GPIO12 (EPWM7A)
GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1; // Disable pull-up on GPIO13 (EPWM7B)
GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // Disable pull-up on GPIO10 (EPWM8A)
GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1; // Disable pull-up on GPIO11 (EPWM8B)
GpioCtrlRegs.GPAMUX2.all = 0; // GPIO31 ... GPIO16 = General Purpose I/O
GpioCtrlRegs.GPBMUX1.all = 0; // GPIO47 ... GPIO32 = General Purpose I/O
GpioCtrlRegs.GPBMUX2.all = 0; // GPIO63 ... GPIO48 = General Purpose I/O
GpioCtrlRegs.GPCMUX1.all = 0; // GPIO79 ... GPIO64 = General Purpose I/O
GpioCtrlRegs.GPCMUX2.all = 0; // GPIO87 ... GPIO80 = General Purpose I/O
GpioCtrlRegs.GPADIR.all = 0;
GpioCtrlRegs.GPBDIR.all = 0; // GPIO63-32 as inputs
GpioCtrlRegs.GPCDIR.all = 0; // GPIO87-64 as inputs
EDIS;
}
void Setup_ePWM(void)
{
EPwm2Regs.TBCTL.bit.CLKDIV = 0; // CLKDIV = 1
EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1; // HSPCLKDIV = 2
EPwm2Regs.TBCTL.bit.CTRMODE = 2; // up count mode
EPwm2Regs.TBPRD = 1250; // 10KHz - PWM signal
EPwm2Regs.ETSEL.all = 0;
EPwm2Regs.ETSEL.bit.SOCAEN = 1; //Enable SOC on A group
EPwm2Regs.ETSEL.bit.SOCASEL = 2; //Select SOC from CPMA on counter
EPwm2Regs.ETPS.bit.SOCAPRD = 1; //Generate pulse on 1st event
}
void Setup_ADC_conf(void)
{
EALLOW;
AdcaRegs.ADCCTL2.bit.PRESCALE = 2; // set ADCCLK divider to /4
// AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
AdcaRegs.ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0; // single-ended channel conversions (12-bit mode only)
AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Set pulse positions to late
AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up the ADC
AdcbRegs.ADCCTL2.bit.PRESCALE = 2; // set ADCCLK divider to /4
// AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
AdcbRegs.ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0; // single-ended channel conversions (12-bit mode only)
AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Set pulse positions to late
AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up the ADC
DELAY_US(10); // delay for 1ms to allow ADC time to power up
EDIS;
EALLOW;
AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // SOC0 will convert pin A2 (64) vdc
AdcaRegs.ADCSOC0CTL.bit.ACQPS = 21; // sample window is 14 SYSCLK cycles
AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7; // trigger on ePWM2 SOCA
AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // end of SOC0 will set INT1 flag
AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; // enable INT1 flag
AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
AdcbRegs.ADCSOC0CTL.bit.CHSEL = 5; // SOC0 will convert pin B5 (66) ib
AdcbRegs.ADCSOC0CTL.bit.ACQPS = 21; // sample window is 14 SYSCLK cycles
AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7; // trigger on ePWM2 SOCA
AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // end of SOC1 will set INT1 flag
AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; // enable INT1 flag
AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // make sure INT1 flag is cleared
EDIS;
}
interrupt void adcb1_isr(void)
{
 GpioDataRegs.GPASET.bit.GPIO6 = 1;
// Return from interrupt
AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear INT1 flag
PieCtrlRegs.PIEACK.all = 1; // acknowledge PIE group 1 to enable further interrupts
// control logic
vdc1 =AdcaResultRegs.ADCRESULT0;
vdc2 = ((vdc1*3)/4096);
vdc = vdc2/0.093;
AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Clear INT1 flag
ib1 =AdcbResultRegs.ADCRESULT0;
ib2 = ((ib1*3)/4096)-1.5;
EPwm6Regs.CMPA.bit.CMPA = PWM6_CMP;
GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
}
void configureDAC(Uint16 dac_num)
{
    EALLOW;

    DAC_PTR[dac_num]->DACCTL.bit.DACREFSEL = REFERENCE;
    DAC_PTR[dac_num]->DACOUTEN.bit.DACOUTEN = 1;
    DAC_PTR[dac_num]->DACVALS.all = ((vdc2)*4096)/3;

    DELAY_US(10); // Delay for buffered DAC to power up

    EDIS;
 }
void Setup_ePWM6(void)
{
EPwm6Regs.TBPRD = 1250; // Set timer period
EPwm6Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
EPwm6Regs.TBCTR = 0x0000; // Clear counter
// Setup TBCLK
EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2; // Clock ratio to SYSCLKOUT
EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2; // Slow so we can observe on the scope
// Setup compare
EPwm6Regs.CMPA.bit.CMPA = 0;
EPwm6Regs.CMPCTL.bit.SHDWAMODE = 0x0; // enable shadow mode
EPwm6Regs.CMPCTL.bit.LOADAMODE = 0x2; // Change CMPA for CTR = 0 or CTR = PRD
// Set actions Qualifiers
EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM8A on Zero
EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
EPwm6Regs.AQCTLB.bit.CAU = AQ_SET; // Set PWM8B on Zero
EPwm6Regs.AQCTLB.bit.CAD = AQ_CLEAR;
//EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
//EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//EPwm6Regs.DBRED.bit.DBRED = 50;
//EPwm6Regs.DBFED.bit.DBFED = 50;
//EPwm6Regs.TBCTL.bit.SYNCOSEL = 1; // generate a syncout if CTR = 0
//EPwm7_DB_Direction = DB_UP;
// Interrupt where we will change the deadband
//EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
//EPwm6Regs.ETSEL.bit.INTEN = 1; // Enable INT
//EPwm6Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event


//DacaRegs.DACVALS.all = ((vdc2)*4096)/3;
// Return from interrupt
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear INT1 flag
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     // acknowledge PIE group 1 to enable further interrupts
}


