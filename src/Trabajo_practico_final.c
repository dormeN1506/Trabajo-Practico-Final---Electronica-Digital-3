/*
 * Copyright 2022 NXP
 * NXP confidential.
 * This software is owned or controlled by NXP and may only be used strictly
 * in accordance with the applicable license terms.  By expressly accepting
 * such terms or by downloading, installing, activating and/or otherwise using
 * the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to
 * be bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software.
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "LPC17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_systick.h"

#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

//LDRs

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
#define LDR_ABAJO_DER_PIN       3     // Pin físico P0.3 -> Función AD0.6
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
#define SERVO_PWM_PERIODO       20000 // Período de 20ms para los Servos (Frecuencia: 50Hz)
#define SERVO_POS_MIN           1000  // Ancho de pulso para 0° (~1ms)
#define SERVO_POS_CENTRO        1500  // Ancho de pulso para 90° (~1.5ms) - Posición de Emergencia
#define SERVO_POS_MAX           2000  // Ancho de pulso para 180° (~2ms)

volatile uint32_t promedio_LDR_ARR_IZQ = 0;
volatile uint32_t promedio_LDR_ARR_DER = 0; //Variables para ir guardando los promedios de los valores tomados por cada
volatile uint32_t promedio_LDR_ABJ_IZQ = 0; //LDR usado
volatile uint32_t promedio_LDR_ABJ_DER = 0;
volatile int32_t ladoIzquierdo, ladoDerecho, ladoArriba, ladoAbajo; //Variables para sumar la luz toal que recibe cada lado
volatile uint16_t posHorizontal = SERVO_POS_CENTRO; //Los 2 servos empiezan en su posición central
volatile uint16_t posVertical = SERVO_POS_CENTRO;

volatile uint8_t convertir = 0; //Flag para empezar las conversiones. Empieza sin convertir
volatile uint8_t deadZone = 150; //Valor para considerar si hay que moverse o no
volatile uint8_t paso = 10; //Valor (en uSeg) que tengo que sumar (o restar) a los ciclos de trabajo de las ondas para mover los servos


//DEFINES PARA UART

void configPtos(void); //Función para configurar todos los puertos y pines que usamos
void configTimer(void); //Función para configurar el TIMER2 que genera las PWM de los servos
void TIMER2_IRQHandler(void); //Handler para manejar interrupciones por TIMER2
void configADC(void); //Función para habilitar el módulo ADC
void configSysTick(void); //Función para inicializar el SysTick para empezar las conversiones del ADC
void SysTick_Handler(void); //Handler para cambiar la flag de conversión
void promediarConversiones(void); //Función para tomar los valores de los LDR
void comparar(void); //Función para hacer la comparación entre los lados
void moverIzquierda(void); //Función para ajustar la posición del servo horizontal hacia la izquierda
void moverDerecha(void); //Función para ajustar la posición del servo horizontal hacia la derecha
void moverArriba(void); //Función para ajustar la posición del servo vertical hacia arriba
void moverAbajo(void); //Función para ajustar la posición del servo vertical hacia abajo

int main(void) {

	configPtos();
	configTimer();
	configADC();
	configSysTick();

	while(1){

		if(convertir){ //Si el flag para convertir está habilitado
			convertir = 0; //Deshabilito las conversiones hasta que vuelva a pasar el tiempo necesario
			promediarConversiones();

			ladoIzquierdo = promedio_LDR_ARR_IZQ + promedio_LDR_ABJ_IZQ;
			ladoDerecho = promedio_LDR_ARR_DER + promedio_LDR_ABJ_DER; //Sumo la luz total que recibe cada lado
			ladoArriba = promedio_LDR_ARR_IZQ + promedio_LDR_ARR_DER; //No hace falta promediar porque son valores totales
			ladoAbajo = promedio_LDR_ABJ_IZQ + promedio_LDR_ABJ_DER;

			comparar(); //Comparlo la luz que recibo de cada lado. El llamado y las actualizaciones tienen que ser acá
			//adentro para tener siempre los datos más recientes
		}
	}

    return 0 ;
}

void configPtos(void){

	PINSEL_CFG_Type cfgPto;

	cfgPto.Portnum = PINSEL_PORT_0; //Vamos a usar para práctciamente todas las conexiones el purto 0
	cfgPto.Funcnum = PINSEL_FUNC_1; //Función 1 (AD0.x) para los pines por donde se van a conectar los 4 LDR
	cfgPto.Pinmode = PINSEL_PINMODE_TRISTATE; //Sin resistencias internas para los pines
	cfgPto.OpenDrain = PINSEL_PINMODE_NORMAL; //No usamos open-drain para ningún pin
	for(int i=23;i<26;i++){
		cfgPto.Pinnum = i;
		PINSEL_ConfigPin(&cfgPto); //Desde P0.23 hasta P0.25 queda configurados como AD0.0 - AD0.2 con lo que definimos arriba
	}

	cfgPto.Funcnum = PINSEL_FUNC_2; //Función 2 para P0.3 (AD0.6)
	cfgPto.Pinnum = PINSEL_PIN_3; //Necesito cambiar a P0.3 para usar otro canal AD0.6 (P0.26 tiene salida del DAC)
	PINSEL_ConfigPin(&cfgPto); //Configuro AD0.3 en modo AD0.6 (todo lo demás definido queda igual)

	cfgPto.Pinnum = PINSEL_PIN_26; //Para P0.26. Función 2 para P0.26 (AOUT)
	PINSEL_ConfigPin(&cfgPto); //P0.26 queda configurado en modo AOUT para la salida al buzzer y transistor

	cfgPto.Funcnum = PINSEL_FUNC_3; //Función 3 para P0.6 y P0.7 en modo MAT2.0 y MAT2.1 respectivamente
	for(int i=6;i<8;i++){
		cfgPto.Pinnum = i;
		PINSEL_ConfigPin(&cfgPto); //P0.6 en modo MAT2.0 y P0.7 en modo MAT2.1
	}
}

void configTimer(void){

	TIM_TIMERCFG_Type cfgTimer;
	TIM_MATCHCFG_Type cfgMatch;

	cfgTimer.PrescaleOption = TIM_PRESCALE_USVAL; //Contamos en uSeg
	cfgTimer.PrescaleValue = 1; //Cada 1 uSeg se incrementa en uno el TC (mejor resolución)
	TIM_Init(LPC_TIM2,TIM_TIMER_MODE,&cfgTimer); //Inicializo el módulo TIMER2, PCLK = CCLK/4 (25 MHz) y configuro

	cfgMatch.MatchChannel = SERVO_HORIZ_MAT_CH; //Canal 0 (MR0) para controlar el ciclo de trabajo del servo horizontal
	cfgMatch.IntOnMatch = DISABLE; //Este canal no usa interrupciones, ni el del servo vertical
	cfgMatch.ResetOnMatch = DISABLE; //Este canal no reinicia el contador, el del servo vertical tampoco
	cfgMatch.StopOnMatch = DISABLE; //Ninguno de los canales usados para el contador en un evento
	cfgMatch.ExtMatchOutputType = TIM_EXTMATCH_LOW; //Un evento de match de este canal pone el pin MAT2.0 en nivel bajo
	//Lo mismo va a hacer el canal del servo vertical en cada evento de match pero en el pin MAT2.1 (con el EMR de cada canal)
	cfgMatch.MatchValue = SERVO_POS_CENTRO; //Empieza contando 1.5 mSeg (valor para dejar el servo en 90°). Se cambia desupués
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuramos el canal 0 con todo lo que definimos

	cfgMatch.MatchChannel = SERVO_VERT_MAT_CH; //Para el servo vertical usamos el canal 1 (MR1) del mismo timer
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuro ahora el canal 1 con lo que nos hace falta

	cfgMatch.MatchChannel = 2; //Para controlar el período de las ondas, usamos el canal 2 (MR2) del mismo timer
	cfgMatch.IntOnMatch = ENABLE; //Un evento en este canal si va a generar una interrupción
	cfgMatch.ResetOnMatch = ENABLE; //Y también va a reiniciar el contador
	cfgMatch.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; //El registro EMR de este canal no tiene que hacer nada
	cfgMatch.MatchValue = SERVO_PWM_PERIODO; //Este canal va a contar siempre hasta 20 mSeg (período para que funcionen
	//los servos). Este valor no va a cambiar nunca
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuro el canal 2 con todo lo que definimos

	NVIC_SetPriority(TIMER2_IRQn,0); //Esta va a ser la interrupción más prioritaria de todas (hace mover los servos)
	TIM_ClearIntPending(LPC_TIM2,TIM_MR2_INT); //Limpio la interrupción por evento de MATCH2 en el TIMER2
	NVIC_EnableIRQ(TIMER2_IRQn); //Habilito las interrupciones por eventos de MATCH en el TIMER2

	LPC_TIM2 -> EMR |= ((1 << 0) | (1 << 1)); //Fuerzo a que MAT2.0 y MAT2.1 queden en estado alto (al inicio) para que
	//se pongan en bajo con los eventos en los canales 0 y 1

	TIM_Cmd(LPC_TIM2,ENABLE); //Habilito el TIMER2 para que empiece a contar y mandar las ondas
}

void TIMER2_IRQHandler(void){

	if(TIM_GetIntStatus(LPC_TIM2,TIM_MR2_INT) == SET){ //Solamente entro acá por interrupciones del canal MR2, pero pregunto
		LPC_TIM2 -> EMR |= ((1 << 0) | (1 << 1)); //Fuerzo los pines MAT2.0 y MAT2.1 a un nivel alto para que se pongan
		//en nivel bajo en los próximos eventos de MATCH de MR0 y MR1
		TIM_ClearIntPending(LPC_TIM2,TIM_MR2_INT); //Limpio la bandera de interrupción por MR2
	}
}

void configADC(void){

	ADC_Init(LPC_ADC,200000); //Habilitamos el módulo, el CLKDIV se setea para tener una frecuencia de muestreo de 200 KHz
	ADC_BurstCmd(LPC_ADC,DISABLE); //No queremos que el ADC trabaje en modo burst
	//No puedo inicializar los canales todos a la vez si quiero empezar conversiones por software. Hay que hacerlo después
	//porque al habilitar lecturas por software, el módulo se vuelve monotarea y solo puede leer un canal a la vez
	//Tampoco quiero interrupciones por conversiones, así que no hace falta nada más
}

void configSysTick(void){

	SYSTICK_InternalInit(100); //Inicializo el SysTick para que use el clock interno y genere una interrupción cada 100 mSeg
	NVIC_SetPriority(SysTick_IRQn,1); //Seteo la prioridad en un poco menos importante que la del timer de los servos
	SYSTICK_Cmd(ENABLE); //Habilito el contador
	SYSTICK_IntCmd(ENABLE); //Habilito la interrupción por SysTick
}

void SysTick_Handler(void){ //Si entro acá, pasaron 100 mSeg y tengo que habilitar las conversiones

	convertir = 1; //Pongo la flag de conversión en uno
	//No hace falta limpiar banderas para el SysTick
}

void promediarConversiones(void){

	promedio_LDR_ARR_IZQ = 0; //Limpio todas las variables para que no queden con valores
	promedio_LDR_ARR_DER = 0; //basura antes de tomar los verdadores valores
	promedio_LDR_ABJ_IZQ = 0;
	promedio_LDR_ABJ_DER = 0;

	for(int i=0;i<16;i++){
		ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,ENABLE); //Habilito el canal 0 donde está el LDR de arriba a la izquierda
		ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
		while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_IZQ_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
		//vamos de este while
		promedio_LDR_ARR_IZQ += ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_IZQ_CH); //Voy sumando los valores que consigo
		ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

		ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,ENABLE); //Habilito el canal 1 donde está el LDR de arriba a la derecha
		ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
		while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_DER_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
		//vamos de este while
		promedio_LDR_ARR_DER += ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_DER_CH); //Voy sumando los valores que consigo
		ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

		ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,ENABLE); //Habilito el canal 2 donde está el LDR de abajo a la izquierda
		ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
		while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_IZQ_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
		//vamos de este while
		promedio_LDR_ABJ_IZQ += ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_IZQ_CH); //Voy sumando los valores que consigo
		ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

		ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,ENABLE); //Habilito el canal 6 donde está el LDR de abajo a la derecha
		ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
		while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_DER_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
		//vamos de este while
		promedio_LDR_ABJ_DER += ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_DER_CH); //Voy sumando los valores que consigo
		ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,DISABLE); //Deshabilito el canal para pasar al que sigue
	}

	promedio_LDR_ARR_IZQ = promedio_LDR_ARR_IZQ >> 4; //Pasado el bucle for de arriba, tomé 16 valores por cada canal.
	promedio_LDR_ARR_DER = promedio_LDR_ARR_DER >> 4; //Entonces tengo que dividir por 16 para promediar. Con la operación
	promedio_LDR_ABJ_IZQ = promedio_LDR_ABJ_IZQ >> 4; //de acá, hago esa divisón; pero esto toma 1 solo ciclo de reloj
	promedio_LDR_ABJ_DER = promedio_LDR_ABJ_DER >> 4; //a diferencia de la otra división (mucho más rápido)
}

void comparar(void){

	int32_t errorHorizontal = ladoIzquierdo - ladoDerecho; //Calculo los errores de ambos lados
	int32_t errorVertical = ladoArriba - ladoAbajo;

	if(errorHorizontal > deadZone){moverIzquierda();}
		//Si la diferencia es lo suficientemente grande, hay que moverse horizontalmente
		//Si además, la diferencia es muy positiva, nos tenemos que mover hacia la izquierda
	else if(errorHorizontal < -deadZone){moverDerecha();} //Lo mismo pero hacia la derecha ahora
	//Si el error está entre -150 y 150, estoy bastante centrado y no me muevo

	if(errorVertical > deadZone){moverArriba();} //Lo mismo de arriba pero para moverse verticalmente
	else if(errorVertical < -deadZone){moverAbajo();}
}

void moverIzquierda(void){

	posHorizontal -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posHorizontal <= SERVO_POS_MIN){
		posHorizontal = SERVO_POS_MIN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverDerecha(void){

	posHorizontal += paso; //Actualizo siempre el valor sumandole esos 10 uSeg
	if(posHorizontal >= SERVO_POS_MAX){
		posHorizontal = SERVO_POS_MAX; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverArriba(void){

	posVertical -= paso; //Actualizo siemore el valor restandole esos 10 uSeg
	if(posVertical <= SERVO_POS_MIN){
		posVertical = SERVO_POS_MIN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,posVertical); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverAbajo(void){

	posVertical += paso; //Actualizo siemore el valor sumandole esos 10 uSeg
	if(posVertical >= SERVO_POS_MAX){
		posVertical = SERVO_POS_MAX; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,posVertical); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}
