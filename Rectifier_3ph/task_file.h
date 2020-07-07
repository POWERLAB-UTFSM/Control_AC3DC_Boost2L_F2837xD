/*
 * task_file.h
 *
 *  Created on: 06-07-2020
 *      Author: Matias_L
 */
#ifndef TASK_FILE_H_
#define TASK_FILE_H_
//
// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Cla_defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//
#define FILTER_LEN    5
//
// Globals
//
//------------maquina-de-estados-------------//
//estados
#define idle            1
#define sync_pll        2
#define pre_charge      3
#define normal_mode     4
#define apagado         5
//cambio de estados
//#define boton_1         1
//#define boton_2         0
//-------------------------------------------//
//
//Task 1 (ASM) Variables
//
//------------Variables-de-prueba------------//
extern float voltFilt;
extern float Num,Den; //A/B user input at the C28x side
extern float Res;  //Final Result used in C28x code
extern float salida_prueba;
extern float a1;
extern float a2;
extern float a3;
extern float a4;
extern float a5;
extern float a6;
extern float a7;
//-------------------------------------------//

//-------------Variables-generales-----------//
#define pi          3.141592
#define f           50.0
#define L           0.00005 // (500e-6)
#define h_sample    0.00002 // (1/50k)

//-------------------------------------------//

//---------------variables-pll---------------//
#define kp_pll       8.7827
#define ki_pll       78956.83
#define lim_up_pll  6.283184

extern float theta_l;
extern float theta_g;
extern float theta_ant_l;
extern float theta_ant_g;
extern float integrador_ant_l;
extern float integrador_ant_g;
extern float integrador_l;
extern float integrador_g;

//-------------------------------------------//

//----------Controlador-de-voltaje-----------//
//extern Uint16 vdc_ref;
//extern float MAX_ACT_v;
//extern float MIN_ACT_v;
//extern float PIAWU_D1_v;
//extern float PIAWU_N1_v;
//extern float KNO_v;
//extern float x_ant_v;

#define kpv1         0.000019

#define vdc_ref      750.0
#define MAX_ACT_v    22.71 // 3*irms(7.57)
#define MIN_ACT_v   -22.71
#define PIAWU_D1_v   0.9825
#define PIAWU_N1_v  -901.0866
#define KNO_v        0.00001938

extern float x_v_l;
extern float x_v_g;
extern float x_ant_v_l;
extern float x_ant_v_g;
//-------------------------------------------//

//---------Controlador-de-corriente-d--------//
#define KN0_i       10.364817
#define MAX_ACT_i   217.78889
#define MIN_ACT_i  -217.78889
#define PIAWU_N1_i -0.014699
#define PIAWU_D1_i  0.847645

extern float x_id_l;
extern float x_id_g;

extern float x_ant_id_l;
extern float x_ant_id_g;
//-------------------------------------------//

//---------Controlador-de-corriente-q--------//
#define iq_ref      0

extern float x_ant_iq_l;
extern float x_ant_iq_g;

extern float x_iq_l;
extern float x_iq_g;
//-------------------------------------------//
//--------------dq-a-alpha_beta--------------//

//-------------------------------------------//
//--------------alpha_beta-a-abc-------------//

//-------------------------------------------//



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
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"

//#endif //end of _CLA_ADC_FIR32_SHARED_H_ definition
#endif /* TASK_FILE_H_ */

//
// End of file
//
