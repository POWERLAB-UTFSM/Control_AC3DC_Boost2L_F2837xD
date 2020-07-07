/*
 * main.c
 *
 *  Created on: 06-07-2020
 *      Author: Matias_L
 *
 */

//
// Included Files
//
#include "task_file.h"
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h" // si no se usa, quitar
#include <CLAmath.h>


//
// Defines
//

#define soc0 0
#define soc1 1
#define soc2 2
#define soc3 3
#define soc4 4
#define soc5 5
#define soc6 6
#define soc7 7

extern uint16_t Cla1ProgRunStart, Cla1ProgLoadStart, Cla1ProgLoadSize;
extern uint16_t CLA1mathTablesRunStart, CLA1mathTablesLoadStart;
extern uint16_t CLA1mathTablesLoadSize;

//
//ADC defines
//definitions for selecting ADC reference
//CAUTION: internal reference mode will drive a voltage onto the VREF pin,
//make sure that this pin is not also being driven externally in this case
//
#define REFERENCE_INTERNAL     0 //internal reference (12-bit only)
#define REFERENCE_EXTERNAL     1 //external reference

//
//EPWM defines
//
#define PWM1_PERIOD            2000
#define PWM1_DUTY_CYCLE        1000
#define PWM2_PERIOD            10000
#define PWM2_DUTY_CYCLE        5000

//
//CLA defines
//
#define WAITSTEP               asm(" RPT #255 || NOP")

//
//Common
//
#define ADC_BUF_LEN            200
#define LOWPASS                1
#define HIGHPASS               (~LOWPASS)

//
// Globals
//------------maquina-de-estados-------------//
uint16_t sw1, sw2;
uint16_t estado   = 1;
uint16_t contador = 0;
uint16_t vg_q     = 0;
uint16_t vdc      = 530;
uint16_t boton_1  = 0;
uint16_t boton_2  = 0;
//-------------------------------------------//
//Task 1 (ASM) Variables
//


//------------Variables-de-prueba------------//
#pragma DATA_SECTION(Num,"CpuToCla1MsgRAM");
float Num;
#pragma DATA_SECTION(Den,"CpuToCla1MsgRAM");
float Den;
#pragma DATA_SECTION(Res,"Cla1ToCpuMsgRAM");
float Res;
#pragma DATA_SECTION(salida_prueba,"Cla1ToCpuMsgRAM");
float salida_prueba;
#pragma DATA_SECTION(voltFilt,"Cla1ToCpuMsgRAM")
float voltFilt;
#pragma DATA_SECTION(a1,"Cla1ToCpuMsgRAM");
float a1;
#pragma DATA_SECTION(a2,"Cla1ToCpuMsgRAM");
float a2;
#pragma DATA_SECTION(a3,"Cla1ToCpuMsgRAM");
float a3;
#pragma DATA_SECTION(a4,"Cla1ToCpuMsgRAM");
float a4;
#pragma DATA_SECTION(a5,"Cla1ToCpuMsgRAM");
float a5;
#pragma DATA_SECTION(a6,"Cla1ToCpuMsgRAM");
float a6;
#pragma DATA_SECTION(a7,"Cla1ToCpuMsgRAM");
float a7;

int aaa  = 0;
int vg_q1 = 10;
//-------------------------------------------//
//---------------variables-pll---------------//
#pragma DATA_SECTION(theta_l,"CpuToCla1MsgRAM");
float theta_l;
#pragma DATA_SECTION(theta_ant_l,"CpuToCla1MsgRAM");
float theta_ant_l;
#pragma DATA_SECTION(integrador_ant_l,"CpuToCla1MsgRAM");
float integrador_ant_l;
#pragma DATA_SECTION(integrador_l,"CpuToCla1MsgRAM");
float integrador_l;

#pragma DATA_SECTION(integrador_g,"Cla1ToCpuMsgRAM");
float integrador_g;
#pragma DATA_SECTION(theta_g,"Cla1ToCpuMsgRAM");
float theta_g;
#pragma DATA_SECTION(theta_ant_g,"Cla1ToCpuMsgRAM");
float theta_ant_g;
#pragma DATA_SECTION(integrador_ant_g,"Cla1ToCpuMsgRAM");
float integrador_ant_g;
//-------------------------------------------//
//----------Controlador-de-voltaje-----------//
#pragma DATA_SECTION(x_ant_v_l,"CpuToCla1MsgRAM");
float x_ant_v_l;
#pragma DATA_SECTION(x_ant_v_g,"Cla1ToCpuMsgRAM");
float x_ant_v_g;
#pragma DATA_SECTION(x_v_l,"CpuToCla1MsgRAM");
float x_v_l;
#pragma DATA_SECTION(x_v_g,"Cla1ToCpuMsgRAM");
float x_v_g;
//-------------------------------------------//

//---------Controlador-de-corriente-d--------//
#pragma DATA_SECTION(x_ant_id_l,"CpuToCla1MsgRAM");
float x_ant_id_l;
#pragma DATA_SECTION(x_ant_id_g,"Cla1ToCpuMsgRAM");
float x_ant_id_g;

#pragma DATA_SECTION(x_id_l,"CpuToCla1MsgRAM");
float x_id_l;
#pragma DATA_SECTION(x_id_g,"Cla1ToCpuMsgRAM");
float x_id_g;
//-------------------------------------------//

//---------Controlador-de-corriente-q--------//
#pragma DATA_SECTION(x_ant_iq_l,"CpuToCla1MsgRAM");
float x_ant_iq_l;
#pragma DATA_SECTION(x_ant_iq_g,"Cla1ToCpuMsgRAM");
float x_ant_iq_g;

#pragma DATA_SECTION(x_iq_l,"CpuToCla1MsgRAM");
float x_iq_l;
#pragma DATA_SECTION(x_iq_g,"Cla1ToCpuMsgRAM");
float x_iq_g;
//-------------------------------------------//
//--------------dq-a-alpha_beta--------------//

//-------------------------------------------//
//--------------alpha_beta-a-abc-------------//

//-------------------------------------------//



//


//
//Task 2 (ASM) Variables
//

//
//Task 3 (ASM) Variables
//

//
//Task 4 (ASM) Variables
//

//
//Task 5 (ASM) Variables
//

//
//Task 6 (ASM) Variables
//

//
//Task 7 (ASM) Variables
//

//
//Task 8 (ASM) Variables
//

//
//Common (ASM) Variables
//

//
// Function Prototypes
//
void CLA_configClaMemory(void);

void configCLAMemory2(void);
void CLA_initCpu1Cla1(void);
void EPWM_initEpwm(void);
void ADC_initAdcA(void);
void ADC_initAdcB(void);

void SetupADCEpwm(void);

void SetupADCEpwm2(void);
void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel);


__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();

//
// Start of main
//
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. enable PWM1 and PWM2 and their GPIOs
//
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;

//
// For this case just init GPIO pins for ePWM1, ePWM2
// These functions are in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();
    InitEPwm2Gpio();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();

//
// Step 4. Configure the CLA memory spaces first followed by
// the CLA task vectors. Force task 8.
//
////    CLA_configClaMemory();
//    configCLAMemory2();
//    CLA_initCpu1Cla1();

//
// Force CLA task 8 using the IACK instruction
// Task 8 will initialize the filter input delay
// line to zero (X[0] - X[4]).
//
// No need to wait, the task will finish by the time
// we configure the ePWM and ADC modules
//
//    Cla1ForceTask8();

//
// The CLA_configClaMemory wipes the message RAM so instead of
// global initialization, init A here
//
//    for(i = 0; i < FILTER_LEN; i++)
//    {
//        A[i] = Coeffs[i];
//    }

//
// Step 5. Configure the ADC to start sampling on an EPWM period
//
    ADC_initAdcA();
//    ADC_initAdcB();
//
// Step 6. Configure EPWM to trigger ADC every 20us
//
    EPWM_initEpwm();
//
// Configurar los ADC para ser disparados con el ePWM
//
//    SetupADCEpwm();
    SetupADCEpwm2();
//
// Step 7. Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// Step 8. Turn on the EPWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1; // Habilita pulso SOCA  (EPWMxSOCA)
    EPwm1Regs.ETSEL.bit.SOCBEN  = 1; // Habilita pulso SOCB  (EPWMxSOCB)
    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode

    EPwm2Regs.TBCTL.bit.CTRMODE = 2; // Contador entra en up-down count mode
    EDIS;


//    CLA_configClaMemory();
    configCLAMemory2();
    CLA_initCpu1Cla1();
//
// Step 9. Buffer till full
//
// The main CPU will receive an interrupt from the
// CLA each time task 7 (CLA FIR filter) completes
//
// In the meantime the main CPU can do other work
//
// After the CLA has fired ADC_BUF_LEN interrupts,
// the main CPU  will halt here
//
// You can view the results by graphing AdcBuf and
// AdcFiltBuf.  Both are unsigned int arrays.
//
//    SampleCount = 0;
//    for(;;)
//    {
//      if(SampleCount == ADC_BUF_LEN-1)
//      {
//          asm(" ESTOP0");
//      }
//    }



    //PLL
    theta_l          = 0;
    theta_ant_l      = 0;
    integrador_ant_l = 0;
    integrador_l     = 0;
    theta_g          = 0;
    theta_ant_g      = 0;
    integrador_ant_g = 0;
    integrador_g     = 0;
    voltFilt         = 0;

//    a4 = 0;
//    a5 = 0;
//----------Controlador-de-voltaje-----------//

    x_v_l       = 0;
    x_ant_v_l   = 0;
    x_id_l      = 0;
    x_ant_id_l  = 0;
    x_ant_iq_l  = 0;

//------------------------------------------//

//---------Controlador-de-corriente-d--------//
    x_ant_id_l = 0;
//------------------------------------------//

//---------Controlador-de-corriente-q--------//
    x_ant_iq_l = 0;
//------------------------------------------//
    boton_1 = 1;
    while(1)
        {
            switch (estado)
            {
                    case idle:
                        sw1 = 0;
                        sw2 = 0;
                        if (boton_1 == 1)
                        {
                            estado = sync_pll;
                        }
                    break;
                    case sync_pll:
                        sw1 = 0;
                        sw2 = 0;
                        if (DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 == 0)
                        {
                            EALLOW;
                            DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 1;
                            DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;
                            EDIS;
                        }
                        if ((vg_q1 > -1)&(vg_q1 <1))
                        {
                            estado = pre_charge;
                            vg_q1 = 10;
                        }
                    break;
                    case pre_charge:
                        sw1 = 1;
                        sw2 = 0;
                        if (vdc > 528)
                        {
                            estado = normal_mode;
                        }
                    break;
                    case normal_mode:
                        sw1 = 1;
                        sw2 = 1;
                        if (DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 == 0)
                        {
                            EALLOW;
                            DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;
                            DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 1;
                            EDIS;
                        }
                        if (boton_2 == 1)
                        {
                            estado = apagado;
                            boton_2 = 0;
                            contador = 0;
                    break;
                    case apagado:
                        sw1 = 0;
                        sw2 = 0;
                        DELAY_US(5000000);
                        estado = idle;
                    break;
                    default:
                    break;
            }
                    if (contador < 0xFFFF)
                    {
                        contador++;
                    }
                    else
                    {
                        contador = 0;
                    }
            }
        }
}






//
// CLA_configClaMemory - Configure CLA memory sections
//
void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;

    EALLOW;
#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //
    // Select LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS5 and then
    // set the space to be a program block
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;
}

void configCLAMemory2(void)
{
#ifdef _FLASH
    //
    // Copy over code and tables from FLASH to RAM
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (uint32_t)&RamfuncsLoadSize);
    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
            (uint32_t)&Cla1ProgLoadSize);

#if !(CLA_MATH_TABLES_IN_ROM)
    //
    // Copy over CLA Math tables from FLASH to RAM
    //
    memcpy((uint32_t *)&CLA1mathTablesRunStart, (uint32_t *)&CLA1mathTablesLoadStart,
            (uint32_t)&CLA1mathTablesLoadSize);
#endif
#endif

    EALLOW;
    //
    // Perform RAM initialization on the message RAMs
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1)
    {
    }

    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1)
    {
    }

    //
    // Select LS0RAM and LS1RAM to be program space for the CLA
    // First, configure the CLA to be the master for LS0 and LS1
    // Second, set the space to be a program block
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 1;

    //
    // Configure RAM blocks LS2-LS5 as data spaces for the CLA
    // First, configure the CLA to be the master for LSx
    // Second, set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS2 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS2 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS3 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS3 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 0;

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
//    Cla1Regs.MCTL.bit.IACKE = 1;
//    Cla1Regs.MIER.all = (M_INT8 | M_INT7| M_INT6| M_INT5| M_INT4| M_INT3| M_INT2| M_INT1);
    Cla1Regs.MIER.bit.INT2 = 1;     // Permite que interrupciones o cpu inicien la tarea 1
    Cla1Regs.MIER.bit.INT7 = 1;     // Permite que interrupciones o cpu inicien la tarea 1

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;

    //
    // Set the adca.1 as the trigger for task 7
    //
//    DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;
//    DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT


    //--- Select Task interrupt source                     /******** TRIGGER SOURCE FOR EACH TASK (unlisted numbers are reserved) ********/
        DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 0;    // 0=none       8=ADCBINT3  16=ADCDINT1  32=XINT4     42=EPWM7INT   70=TINT2     78=ECAP4INT   95=SD1INT     114=SPIRXINTC
        DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK2 = 0;    // 1=ADCAINT1   9=ADCBINT4  17=ADCDINT2  33=XINT5     43=EPWM8INT   71=MXEVTA    79=ECAP5INT   96=SD2INT
        DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK3 = 0;    // 2=ADCAINT2  10=ADCBEVT   18=ADCDINT3  36=EPWM1INT  44=EPWM9INT   72=MREVTA    80=ECAP6INT  107=UPP1INT
        DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK4 = 0;    // 3=ADCAINT3  11=ADCCINT1  19=ADCDINT4  37=EPWM2INT  45=EPWM10INT  73=MXEVTB    83=EQEP1INT  109=SPITXINTA
        DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK5 = 0;    // 4=ADCAINT4  12=ADCCINT2  20=ADCDEVT   38=EPWM3INT  46=EPWM11INT  74=MREVTB    84=EQEP2INT  110=SPIRXINTA
        DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK6 = 0;    // 5=ADCAEVT   13=ADCCINT3  29=XINT1     39=EPWM4INT  47=EPWM12INT  75=ECAP1INT  85=EQEP3INT  111=SPITXINTB
        DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK7 = 0;    // 6=ADCBINT1  14=ADCCINT4  30=XINT2     40=EPWM5INT  48=TINT0      76=ECAP2INT  87=HRCAP1INT 112=SPIRXINTB
        DmaClaSrcSelRegs.CLA1TASKSRCSEL2.bit.TASK8 = 0;    // 7=ADCBINT2  15=ADCCEVT   31=XINT3     41=EPWM6INT  69=TINT1      77=ECAP3INT  88=HRCAP2INT 113=SPITXINTC

        DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL1 = 0;     // Write a 1 to lock (cannot be cleared once set)
        DmaClaSrcSelRegs.CLA1TASKSRCSELLOCK.bit.CLA1TASKSRCSEL2 = 0;     // Write a 1 to lock (cannot be cleared once set)

    //
    // Enable all CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
    EDIS;
}
//
// EPWM_initEpwm - Initialize EPWM1 and EPWM2 settings
//
void EPWM_initEpwm(void)
{
//    EALLOW;
//    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Turn off the EPWM clock
//    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;          // Disable SOC on A group
//    EPwm1Regs.ETSEL.bit.SOCASEL    = 4;         // Select SOC on up-count
//    EPwm1Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event
//    EPwm1Regs.CMPA.bit.CMPA = PWM1_DUTY_CYCLE;  // Set compare A value to 2000
//                                                // counts
//    EPwm1Regs.TBPRD = PWM1_PERIOD;              // Set period to 4000 counts
//    EPwm1Regs.TBCTL.bit.CTRMODE = 3;            // freeze counter
//    EPwm1Regs.ETSEL.bit.SOCAEN = 1;             //enable SOCA
//    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
//
//    EPwm2Regs.ETSEL.bit.SOCAEN    = 0;          // Disable SOC on A group
//    EPwm2Regs.CMPA.bit.CMPA = PWM2_DUTY_CYCLE;  // Set compare A value to 10000
//                                                // counts
//    EPwm2Regs.TBPRD = PWM2_PERIOD;              // Set period to 20000 counts
//    EPwm2Regs.TBCTL.bit.CTRMODE = 3;            // freeze counter
//    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
//
//    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;          // Clear PWM1A on Zero
//    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;
//
//    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;          // Clear PWM2A on Zero
//    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;
//
//    EDIS;
    EALLOW;
    EPwm1Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm1Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm1Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm1Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm1Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm1Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm1Regs.TBPRD               = 500;  // Frecuencia de 50khz
//    EPwm1Regs.TBPRD               = 25000000;  // Frecuencia de 10hz
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
    EPwm1Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1

    EPwm1Regs.CMPA.bit.CMPA = 250;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
//
// Configuracion Deadband - 500ns de FED y RED
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
    EPwm1Regs.DBFED.bit.DBFED = 0x19; // 25 decimal

    // Configuración eWPM2
    EPwm2Regs.ETSEL.bit.SOCAEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm2Regs.ETSEL.bit.SOCASEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm2Regs.ETSEL.bit.SOCBEN    = 0;    // Deshabilita el inicio de la conversión del ADC
    EPwm2Regs.ETSEL.bit.SOCBSEL   = 1;    // Genera Evento cuando TBCTR = 0x00 (inicio conversión)
    EPwm2Regs.ETPS.bit.SOCAPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm2Regs.ETPS.bit.SOCBPRD    = 1;    // Genera el pulso en el 1st evento
    EPwm2Regs.TBPRD               = 500;  // Frecuencia de 50khz
    EPwm2Regs.TBCTL.bit.CTRMODE   = 3;    // freeze counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x1;  // Divide el reloj en 2
    EPwm2Regs.TBCTL.bit.CLKDIV    = 0x0;  // Divide el reloj en 1

    EPwm2Regs.CMPA.bit.CMPA = 250;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM1A on Zero
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;            // Set PWM1B on Zero
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
//
// Configuracion Deadband - 500ns de FED y RED
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED.bit.DBRED = 0x19; // 25 decimal
    EPwm2Regs.DBFED.bit.DBFED = 0x19; // 25 decimal
    EDIS;
}

//
// ADC_initAdcA - Initialize ADCA configurations and power it up
//
void ADC_initAdcA(void)
{
//    uint16_t i;

    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC

    // encender adcB
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //delay for > 1ms to allow ADC time to power up
    //
//    for(i = 0; i < 1000; i++)
//    {
//        asm("   RPT#255 || NOP");
//    }

    DELAY_US(1000);
//
//    //
//    //Select the channels to convert and end of conversion flag ADCA
//    //
//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is 100 SYSCLK cycles
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
}

void ADC_initAdcB(void)
{
//    uint16_t i;

    EALLOW;

    //
    //write configurations
    //
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC
    //
    //Set pulse positions to late
    //
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1; // Interrupción se genera al final de la conversión

    //
    //power up the ADC
    //
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1; // Enciende el circuito análogo del ADC



    DELAY_US(1000);
    EDIS;
}

void SetupADCEpwm(void)
{
    Uint16 acqps;
    acqps = 50; //1us

    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;//trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //fin de SOC7 genera INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag
    EDIS;
}

void SetupADCEpwm2(void)
{
    Uint16 acqps;
    acqps = 19; //1us -> (1e-6)/(5e-9)
    acqps = 200;
    EALLOW;
    SetupSOC(soc0, 0, acqps, 5); // SOC0 convierte ADC-A0 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc1, 1, acqps, 5); // SOC1 convierte ADC-A1 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc2, 2, acqps, 5); // SOC2 convierte ADC-A2 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc3, 3, acqps, 5); // SOC3 convierte ADC-A3 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc4, 4, acqps, 5); // SOC4 convierte ADC-A4 y hace trigger desde ePWM1(5) con una ventana de acqps
//    SetupSOC(soc5, 5, acqps, 5); // SOC5 convierte ADC-A5 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc6, 0, acqps, 5); // SOC6 convierte ADC-B0 y hace trigger desde ePWM1(5) con una ventana de acqps
    SetupSOC(soc7, 1, acqps, 5);//SOC7 convierte ADC-B1 y hace trigger desde ePWM1(5) con una ventana de acqps
//    channel = 0;
//    trigsel = 5;

//    AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
//    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
//    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
//    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC3 will convert pin A3
//    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC4 will convert pin A4
//    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
//    AdcaRegs.ADCSOC5CTL.bit.CHSEL = channel;  //SOC5 will convert pin A5
//    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window
//    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 4; //fin de SOC4 genera INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //Habilita INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //limpia el bit de INT1 flag
    EDIS;
}

void SetupSOC(Uint16 soc, Uint16 channel, Uint16 acqps, Uint16 trigsel)
{

    switch(soc)
        {
            case soc0:
            {
                AdcaRegs.ADCSOC0CTL.bit.CHSEL = channel;  //SOC0 will convert pin A0
                AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc1:
            {
                AdcaRegs.ADCSOC1CTL.bit.CHSEL = channel;  //SOC1 will convert pin A1
                AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc2:
            {
                AdcaRegs.ADCSOC2CTL.bit.CHSEL = channel;  //SOC2 will convert pin A2
                AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc3:
            {
                AdcaRegs.ADCSOC3CTL.bit.CHSEL = channel;  //SOC3 will convert pin A3
                AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc4:
            {
                AdcaRegs.ADCSOC4CTL.bit.CHSEL = channel;  //SOC4 will convert pin A4
                AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc5:
            {
                AdcaRegs.ADCSOC5CTL.bit.CHSEL = channel;  //SOC5 will convert pin A5
                AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;    //sample window
                AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc6:
            {
                AdcbRegs.ADCSOC6CTL.bit.CHSEL = channel;  //SOC5 will convert pin B0
                AdcbRegs.ADCSOC6CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC6CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
            case soc7:
            {
                AdcbRegs.ADCSOC7CTL.bit.CHSEL = channel;  //SOC7 will convert pin B1
                AdcbRegs.ADCSOC7CTL.bit.ACQPS = acqps;    //sample window
                AdcbRegs.ADCSOC7CTL.bit.TRIGSEL = trigsel;//trigger on ePWM1 SOCA/C
            break;
            }
        }
    }


//
// cla1Isr1 - CLA1 ISR 1
//
__interrupt void cla1Isr1 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr2 - CLA1 ISR 2
//
__interrupt void cla1Isr2 ()
{
//---------------variables-pll---------------//
    theta_l          = theta_g;
    theta_ant_l      = theta_ant_g;
    integrador_ant_l = integrador_ant_g;
    integrador_l     = integrador_g;
//-------------------------------------------//

//EPwm2Regs.CMPA.bit.CMPA = voltFilt;



    aaa ++;
    if (aaa >= 10)
        {
//            asm(" ESTOP0");
            aaa   = 0;
            vg_q1 = 0;
        }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // limpia el bit de la INT1 flag
    // Revisa si ocurrió overflow
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
//        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
    }
    PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);

}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr4 - CLA1 ISR 4
//
__interrupt void cla1Isr4 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr5 - CLA1 ISR 5
//
__interrupt void cla1Isr5 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr6 - CLA1 ISR 6
//
__interrupt void cla1Isr6 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr7 - CLA1 ISR 7
//
__interrupt void cla1Isr7 ()
{

//---------------variables-pll---------------//
    theta_l          = theta_g;
    theta_ant_l      = theta_ant_g;
    integrador_ant_l = integrador_ant_g;
    integrador_l     = integrador_g;
//-------------------------------------------//
//----------Controlador-de-voltaje-----------//
    x_ant_v_l = x_ant_v_g;
    x_v_l     = x_v_g;
//-------------------------------------------//
//---------Controlador-de-corriente-d--------//
    x_ant_id_l = x_ant_id_g;
    x_id_l     = x_id_g;
//-------------------------------------------//
//---------Controlador-de-corriente-q--------//
    x_ant_iq_l = x_ant_iq_g;
    x_iq_l     = x_iq_g;
//-------------------------------------------//


    //
    // Clear the ADC interrupt flag so the next SOC can occur
    // Clear the PIEACK bits so another interrupt can be taken
    //
//        EPwm2Regs.CMPA.bit.CMPA = voltFilt;




        aaa ++;
        if (aaa >= 10)
            {
                aaa     = 0;
                boton_2 = 1;
            }

        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // limpia el bit de la INT1 flag
        // Revisa si ocurrió overflow
        //
        if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // limpia INT1 overflow flag
//            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
        }
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);
}

//
// cla1Isr8 - CLA1 ISR 8
//
__interrupt void cla1Isr8 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 8
    //
    PieCtrlRegs.PIEACK.all = M_INT11;

    //
    // Uncomment to halt debugger here
    //
//    asm(" ESTOP0");
}

// End of File

