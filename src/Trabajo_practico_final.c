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

//SERVOMOTORES
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
#define SERVO_POS_MIN_PAN       1000  // Ancho de pulso para 0° (~1ms). Servo horizontal
#define SERVO_POS_MAX_PAN       2000  // Ancho de pulso para 180° (~2ms). Servo horizontal
#define SERVO_POS_MIN_TILT      1085  // Ancho de pulso para 15°. El rango de TILT no llega hasta los 180° limpios
#define SERVO_POS_MAX_TILT      1915  // Ancho de pulso para 90°
#define SERVO_POS_CENTRO        1500  // Ancho de pulso para 90° (~1.5ms) - Posición de Emergencia

// --- PARA EL MÓDULO GPDMA ---
#define N 64 //La LUT va a tener 64 muestras para transferir (64 valores hacen un seno bastante limpio)
// Tabla de 64 muestras para una onda senoidal en el DAC de 10 bits (con corrimiento de 6 bits)
const uint32_t ondaSenoidal[N] = {
    32768, 35968, 39104, 42240, 45248, 48192, 51008, 53632,
    55872, 58048, 59968, 61696, 63104, 64256, 65088, 65408,
    65472, 65408, 65088, 64256, 63104, 61696, 59968, 58048,
    55872, 53632, 51008, 48192, 45248, 42240, 39104, 35968,
    32768, 29568, 26432, 23296, 20288, 17344, 14528, 11904,
    9664,  7488,  5568,  3840,  2432,  1280,  448,   128,
    64,    128,   448,   1280,  2432,  3840,  5568,  7488,
    9664,  11904, 14528, 17344, 20288, 23296, 26432, 29568
};

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
void configDAC(void); //Función para configurar el módulo DAC
void configDMA(void); //Función para configurar el módulo DMA
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
	configDAC();
	configDMA();

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
	PINSEL_ConfigPin(&cfgPto); //Configuro AD0.3 en modo AD0.6 (lo demás definido queda igual)

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
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuramos el canal 0 con lo que definimos

	cfgMatch.MatchChannel = SERVO_VERT_MAT_CH; //Para el servo vertical usamos el canal 1 (MR1) del mismo timer
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuro ahora el canal 1 con lo que nos hace falta

	cfgMatch.MatchChannel = 2; //Para controlar el período de las ondas, usamos el canal 2 (MR2) del mismo timer
	cfgMatch.IntOnMatch = ENABLE; //Un evento en este canal si va a generar una interrupción
	cfgMatch.ResetOnMatch = ENABLE; //Y también va a reiniciar el contador
	cfgMatch.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; //El registro EMR de este canal no tiene que hacer nada
	cfgMatch.MatchValue = SERVO_PWM_PERIODO; //Este canal va a contar siempre hasta 20 mSeg (período para que funcionen
	//los servos). Este valor no va a cambiar nunca
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuro el canal 2 con lo que definimos

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
	uint8_t movimiento = 0; //Flag para saber si hago sonar el buzzer o no. Empiezo asumiendo que no hay que hacerlo sonar

	if(errorHorizontal > deadZone){moverIzquierda();movimiento = 1;}
		//Si la diferencia es lo suficientemente grande, hay que moverse horizontalmente
		//Si además, la diferencia es muy positiva, nos tenemos que mover hacia la izquierda
		//Si me tengo que mover, seteo la flag de movimiento en 1
	else if(errorHorizontal < -deadZone){moverDerecha();movimiento = 1;} //Lo mismo pero hacia la derecha ahora
	//Si el error está entre -150 y 150, estoy bastante centrado y no me muevo

	if(errorVertical > deadZone){moverArriba();movimiento = 1;} //Lo mismo de arriba pero para moverse verticalmente
	else if(errorVertical < -deadZone){moverAbajo();movimiento = 1;}

	if(movimiento){GPDMA_ChannelCmd(0,ENABLE);} //Si me tengo que mover, habilito el canal para transferir los datos y hacer sonar el buzzer
	else{GPDMA_ChannelCmd(0,DISABLE);} //Si no hay que moverse, deshabilito el canal y ya el buzzer no suena
}

void moverIzquierda(void){

	posHorizontal -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posHorizontal <= SERVO_POS_MIN_PAN){
		posHorizontal = SERVO_POS_MIN_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverDerecha(void){

	posHorizontal += paso; //Actualizo siempre el valor sumandole esos 10 uSeg
	if(posHorizontal >= SERVO_POS_MAX_PAN){
		posHorizontal = SERVO_POS_MAX_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverArriba(void){

	posVertical -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posVertical <= SERVO_POS_MIN_TILT){
		posVertical = SERVO_POS_MIN_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,posVertical); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void moverAbajo(void){

	posVertical += paso; //Actualizo siempre el valor sumandole esos 10 uSeg
	if(posVertical >= SERVO_POS_MAX_TILT){
		posVertical = SERVO_POS_MAX_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,posVertical); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}

void configDAC(void){

	DAC_CONVERTER_CFG_Type cfgDAC;

	cfgDAC.DBLBUF_ENA = ENABLE; //Habilito el doble buffer para que los datos pasen al DAC cuando el temporizador acabe
	cfgDAC.CNT_ENA = ENABLE; //Habilito el contador para que el DAC pueda generar las solicitudes DMA
	cfgDAC.DMA_ENA = ENABLE; //Habilito las solicitudes de DMA con el DAC

	DAC_Init(LPC_DAC); //Inicializo el módulo DAC con PCLK = 25 MHz y modo alto consumo y rapidez
	DAC_SetBias(LPC_DAC,DAC_MAX_CURRENT_350uA); //Voy a sacar la onda a 1 KHz. Como el modo bajo consumo soporta hasta
	//400 KHz, estoy sobradísimo. Y no consumo tanto (350 uA)
	DAC_ConfigDAConverterControl(LPC_DAC,&cfgDAC); //Configuro el control del DAC con los campos que determiné
	DAC_SetDMATimeOut(LPC_DAC,391); //Sabiendo que el DAC cuenta "ticks" a 25 MHz y quiero la onda con frecuencia 1 KHz,
	//quiero 1000 ondas por segundo (sabiendo que la onda está "partida" en 64 valores). Entonces -> 1000 ondas/segundo*64 valores = 64000 pedidos por segundo
	//Entonces tiene que contar X ticks para pedir un dato nuevo -> Ticks(PCLK)/Transferencias = 25MHz/64000 = 390.625 y redondeo
}

void configDMA(void){

	static GPDMA_LLI_Type LLI; //La transferencia va a ser cíclica. Se transfiere la onda cada vez que se necesite siempre
	GPDMA_Channel_CFG_Type cfgDMA;

	LLI.SrcAddr = (uint32_t) ondaSenoidal; //Los datos siempre vienen del arreglo (el nombre del arreglo ya es un puntero a su primera posición)
	LLI.DstAddr = (uint32_t) &(LPC_DAC -> DACR); //Los datos siempre van al DAC
	LLI.NextLLI = (uint32_t) &LLI; //Es una transferencia cíclica. Siempre se manda lo mismo
	LLI.Control = ((N << 0) | (2 << 18) | (2 << 21) | (1 << 26));
	LLI.Control &= ~((1 << 27) | (1 << 31)); //Transfiero siempre 64 muestras; el tamaño de los datos de origen y llegada
	//es siempre de 32 bits; habilito el incremento automático de dirección de origen; deshabilito el de dirección
	//de destino; y deshabilito las interrupciones por DMA (no las necesito)

	cfgDMA.ChannelNum = 0; //Vamos a usar el canal 0 del DMA para hacer las transferencias
	cfgDMA.TransferSize = N; //Transfiero siempre 64 muestras
	//cfgDMA.TransferWidth solamente se usa si la trasnferencia es M2M
	cfgDMA.SrcMemAddr = (uint32_t) ondaSenoidal; //El primer dato siempre viene desde la primera posición del arreglo
	cfgDMA.DstMemAddr = (uint32_t) &(LPC_DAC -> DACR); //Los datos van siempre hasta el DAC
	cfgDMA.TransferType = GPDMA_TRANSFERTYPE_M2P; //Las tranferencias siempre son de memoria a periférico
	cfgDMA.SrcConn = 0; //Nadie avisa que hay un dato listo para transferir
	cfgDMA.DstConn = GPDMA_CONN_DAC; //El DAC es el que "pide" que le manden un dato
	cfgDMA.DMALLI = (uint32_t) &LLI; //La referencia la primera vez que entro acá es la misma LLI que transfiero

	GPDMA_Init(); //Inicializo el módulo DMA
	GPDMA_Setup(&cfgDMA); //Configuro según los campos de la estructura
	LPC_GPDMACH0 -> DMACCControl &= ~(1 << 31); //Aseguro que las interrupciones por DMA se deshabiliten. No las necesito
	//No habilito el canal aún para hacer las transferencias de datos. Solamente lo voy a habilitar cuando los servos se
	//estén moviendo
}

//-----------------implementacion UART--------------------
// ── Limites de velocidad — 
#define PASO_MIN        5u
#define PASO_MAX        50u
// ── Buffer circular RX
#define UART_BUF_SIZE   64u
static volatile uint8_t rx_buffer[UART_BUF_SIZE];
static volatile uint8_t rx_head = 0;   // escribe 
static volatile uint8_t rx_tail = 0;   // lee 
volatile uint8_t modo_automatico = 1;   // 1=auto, 0=manual

//config uart1
void config_UART1(void)
{
   
	//configuramos pines TX1=P0.15 y RX1=P0.16 
    PINSEL_CFG_Type cfgPin;
    cfgPin.Portnum   = PINSEL_PORT_0;
    cfgPin.Funcnum   = PINSEL_FUNC_1;       // Función 1 = TXD1/RXD1
    cfgPin.Pinmode   = PINSEL_PINMODE_TRISTATE;
    cfgPin.OpenDrain = PINSEL_PINMODE_NORMAL;

    cfgPin.Pinnum = 15;                     // P0.15 = TXD1
    PINSEL_ConfigPin(&cfgPin);

    cfgPin.Pinnum = 16;                     // P0.16 = RXD1
    PINSEL_ConfigPin(&cfgPin);

    //parametros
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 9600;			//baudrate
    uartCfg.Databits  = UART_DATABIT_8;//8 bits de datos
    uartCfg.Parity    = UART_PARITY_NONE;//sin paridad
    uartCfg.Stopbits  = UART_STOPBIT_1;//1 bit de parada

  
    UART_Init(LPC_UART1, &uartCfg);

    //FIFO: interrupción por cada 1 byte recibido
    
    UART_FIFO_CFG_Type fifoCfg;
    fifoCfg.FIFO_Level      = UART_FIFO_TRGLEV0;
    fifoCfg.FIFO_ResetRxBuf = ENABLE;
    fifoCfg.FIFO_ResetTxBuf = ENABLE;
    fifoCfg.FIFO_DMAMode    = DISABLE;
    UART_FIFOConfig(LPC_UART1, &fifoCfg);

	//habilitamos interrupción por recepción de datos (RBR)
    UART_IntConfig(LPC_UART1, UART_INTCFG_RBR, ENABLE);

    //  NVIC con prioridad menor que Timer2 
    NVIC_SetPriority(UART1_IRQn, 2);
    NVIC_EnableIRQ(UART1_IRQn);

    //habilitamos la transmisión 
    UART_TxCmd(LPC_UART1, ENABLE);
}
//isr de uart1 
void UART1_IRQHandler(void)//guarda el byte recibido en el buffer
{
    // Identificar causa: 0x04=RDA (dato disponible), 0x0C=CTI (timeout FIFO)
    uint32_t causa = UART_GetIntId(LPC_UART1) & 0x0Eu;

    if (causa == 0x04u || causa == 0x0Cu)
    {
        uint8_t byte_rx   = UART_ReceiveByte(LPC_UART1);
        uint8_t next_head = (uint8_t)((rx_head + 1u) % UART_BUF_SIZE);

        if (next_head != rx_tail)       // solo guarda si hay espacio
        {
            rx_buffer[rx_head] = byte_rx;
            rx_head            = next_head;
        }
        // Si el buffer está lleno, el byte se descarta.
        
    }
}
//
static void UART1_EnviarString(const char *str)
{
    UART_Send(LPC_UART1,
              (uint8_t *)str,
              (uint32_t)strlen(str),
              BLOCKING);
}
//enviar telemetría por UART1
void UART1_EnviarTelemetria(void)
{
    char buf[72];
    snprintf(buf, sizeof(buf),
             "$AI:%lu,AD:%lu,BI:%lu,BD:%lu,PH:%u,PV:%u,PASO:%u\r\n",
             promedio_LDR_ARR_IZQ,
             promedio_LDR_ARR_DER,
             promedio_LDR_ABJ_IZQ,
             promedio_LDR_ABJ_DER,
             posHorizontal,
             posVertical,
             paso);

    UART_Send(LPC_UART1,
              (uint8_t *)buf,
              (uint32_t)strlen(buf),
              BLOCKING);
}
void revisar_comandos_uart(void)
{
    while (rx_head != rx_tail)
    {
        uint8_t cmd = rx_buffer[rx_tail];
        rx_tail     = (uint8_t)((rx_tail + 1u) % UART_BUF_SIZE);

        switch (cmd)
        {
            // Activa el seguidor automático
            case 'A': case 'a':
                modo_automatico = 1;
                UART1_EnviarString("ACK:AUTO\r\n");
                break;

            // Congela los servos en su posición actual
            case 'M': case 'm':
                modo_automatico = 0;
                UART1_EnviarString("ACK:MANUAL\r\n");
                break;

            // Envía telemetría inmediatamente sin esperar el contador
            case 'S': case 's':
                UART1_EnviarTelemetria();
                break;

            // Aumenta la velocidad de movimiento de los servos
            case '+':
            {
                if (paso < PASO_MAX) paso += 5u;
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            // Disminuye la velocidad de movimiento de los servos
            case '-':
            {
                if (paso > PASO_MIN) paso -= 5u;
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            // Lleva ambos servos al centro (90°) y activa modo manual
            // Actualiza posHorizontal/posVertical Y el Timer — ambos necesarios
            case 'R': case 'r':
                modo_automatico = 0;
                posHorizontal   = SERVO_POS_CENTRO;
                posVertical     = SERVO_POS_CENTRO;
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, posHorizontal);
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH,  posVertical);
                UART1_EnviarString("ACK:RESET\r\n");
                break;

            // Fin de línea del terminal — ignorar silenciosamente
            case '\r': case '\n':
                break;

            // Cualquier otro caracter no reconocido
            default:
                UART1_EnviarString("ERR:CMD\r\n");
                break;
        }
    }
}

/*asi seria el main
int main(void)
{
    
    configPtos();      
    configTimer();    
    configADC();       
    configSysTick();   
    config_UART1();     
    
    UART1_EnviarString("Seguidor Solar OK\r\n");


    static uint8_t tele_cnt = 0;

    while (1)
    {
        
        revisar_comandos_uart();

       
        if (convertir)
        {
            convertir = 0;

            
            promediarConversiones();

           
            ladoIzquierdo = promedio_LDR_ARR_IZQ + promedio_LDR_ABJ_IZQ;
            ladoDerecho   = promedio_LDR_ARR_DER + promedio_LDR_ABJ_DER;
            ladoArriba    = promedio_LDR_ARR_IZQ + promedio_LDR_ARR_DER;
            ladoAbajo     = promedio_LDR_ABJ_IZQ + promedio_LDR_ABJ_DER;

           
            if (modo_automatico)
            {
                comparar();
            }

            if (++tele_cnt >= 10u)
            {
                tele_cnt = 0;
                UART1_EnviarTelemetria();
            }
        }
    }

    return 0;
}


*/ 