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
#define FILTER_LEN      7
#define ADC_BUF_LEN     70
#define ADC_BUF_PREV    48

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
#define fault_mode      6
extern float adc_read[7];
//cambio de estados
//#define boton_1         1
//#define boton_2         0
//-------------------------------------------//

//-------------Variables-fallas--------------//
#define vg_d_min     280
#define vg_q_max     10
#define vg_q_min    -10
#define vdc_max      850
#define vdc_charged  530
#define v_phase_max  5
#define v_phase_min -5
#define pot_max      5500

//-------------------------------------------//

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
extern float a8;
extern float a9;
extern float a10;
extern float vect[6];
//-------------------------------------------//

//-------------Variables-generales-----------//
#define pi          3.141592
#define f           50.0
#define L           0.00005 // (500e-6)
#define h_sample    0.00002 // (1/50k)
#define raiz_de_3   1.73205
#define uno_div_3   0.33334
#define dos_div_3   0.66667
//-------------------------------------------//

//---------------variables-pll---------------//
#define kp_pll       8.78568
#define ki_pll       78956.83
#define lim_up_pll   6.283184

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
extern float v_conv_a;
extern float v_conv_b;
extern float v_conv_c;
//-------------------------------------------//
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

#endif /* TASK_FILE_H_ */
//
// End of file
//
