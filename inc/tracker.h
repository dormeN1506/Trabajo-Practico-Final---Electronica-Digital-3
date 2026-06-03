#include "LPC17xx.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "LPC17xx_timer.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_dac.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

//define LDRs y adc


#define LDR_ARRIBA_IZQ_PORT     0
#define LDR_ARRIBA_IZQ_PIN      23    // Pin físico P0.23 -> Función AD0.0
#define LDR_ARRIBA_IZQ_CH       0     // Canal 0 del ADC

#define LDR_ARRIBA_DER_PORT     0
#define LDR_ARRIBA_DER_PIN      24    // Pin físico P0.24 -> Función AD0.1
#define LDR_ARRIBA_DER_CH       1     // Canal 1 del ADC

#define LDR_ABAJO_IZQ_PORT      0
#define LDR_ABAJO_IZQ_PIN       25    // Pin físico P0.25 -> Función AD0.2
#define LDR_ABAJO_IZQ_CH        2     // Canal 2 del ADC

#define LDR_ABAJO_DER_PORT      0
#define LDR_ABAJO_DER_PIN       3    // Pin físico P0.3 -> Función AD0.6
#define LDR_ABAJO_DER_CH        6     // Canal 6 del ADC

//Mascara para LDRs va para configurar el ADC en modo Burst con los 4 canales de LDRs
#define ADC_BURST_CHANNELS      ((1 << LDR_ARRIBA_IZQ_CH) | (1 << LDR_ARRIBA_DER_CH) | \
                                 (1 << LDR_ABAJO_IZQ_CH)  | (1 << LDR_ABAJO_DER_CH))

//SERVOS MOTORES
// --- CONTROL DEL SERVO HORIZONTAL (AZIMUT) ---
#define SERVO_HORIZ_PORT        0
#define SERVO_HORIZ_PIN         6     // Pin físico P0.6 -> Función MAT2.0 (Timer 2 Match 0)
#define SERVO_HORIZ_MAT_CH      0     // Canal de Match asignado

// --- CONTROL DEL SERVO VERTICAL (ELEVACIÓN) ---
#define SERVO_VERT_PORT         0
#define SERVO_VERT_PIN          7     // Pin físico P0.7 -> Función MAT2.1 (Timer 2 Match 1)
#define SERVO_VERT_MAT_CH       1     // Canal de Match asignado

// --- VALORES LÍMITE DE PASO DEL PWM (Ticks de reloj del Timer) ---
// Estos valores dependen de tu frecuencia de clock, pero sirven para tipificar los topes
#define SERVO_PWM_PERIODO        20000 // Período de 20ms para los Servos (Frecuencia: 50Hz)
#define SERVO_POS_MIN           1000  // Ancho de pulso para 0° (~1ms)
#define SERVO_POS_CENTRO        1500  // Ancho de pulso para 90° (~1.5ms) - Posición de Emergencia
#define SERVO_POS_MAX           2000  // Ancho de pulso para 180° (~2ms)


//DEFINES PARA UART

//declarar funciones
void trackerInit(void);










