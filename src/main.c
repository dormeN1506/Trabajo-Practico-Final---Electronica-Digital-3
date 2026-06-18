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
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_systick.h"

#include <cr_section_macros.h>
#include <stdio.h>
#include <math.h>

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
#define LDR_ABAJO_DER_PIN       2     // Pin físico P0.2 -> Función AD0.7
#define LDR_ABAJO_DER_CH        7     // Canal 7 del ADC

//SERVOMOTORES
// --- CONTROL DEL SERVO HORIZONTAL (PAN) ---
#define SERVO_HORIZ_PORT        0
#define SERVO_HORIZ_PIN         6     // Pin físico P0.6 -> Función MAT2.0 (Timer 2 Match 0)
#define SERVO_HORIZ_MAT_CH      0     // Canal de Match asignado

// --- CONTROL DEL SERVO VERTICAL (TILT) ---
#define SERVO_VERT_PORT         0
#define SERVO_VERT_PIN          7     // Pin físico P0.7 -> Función MAT2.1 (Timer 2 Match 1)
#define SERVO_VERT_MAT_CH       1     // Canal de Match asignado

// --- VALORES LÍMITE DE PASO DEL PWM (Ticks de reloj del Timer) ---

#define SERVO_PWM_PERIODO       20000 // Período de 20ms para los Servos (Frecuencia: 50Hz)
#define SERVO_POS_MIN_PAN       1000  // Ancho de pulso para 0° (~1ms). Servo horizontal
#define SERVO_POS_MAX_PAN       2000  // Ancho de pulso para 180° (~2ms). Servo horizontal
#define SERVO_POS_MIN_TILT      1000//1085  // Ancho de pulso para 15°. El rango de TILT no llega hasta los 180° limpios
#define SERVO_POS_MAX_TILT      2000//1915  // Ancho de pulso para 90°
#define SERVO_POS_CENTRO        1500  // Ancho de pulso para 90° (~1.5ms) - Posición de Emergencia
#define CANAL_MATCH_PERIODO     2     //El canal 2 del TIMER2 es el que usamos para controlar el período de la onda de los servos

// --- PARA EL MÓDULO UART ---
#define PIN_Tx                  0     //P0.0 es el Tx que vamos a usar
#define PIN_Rx                  1     //P0.1 es el Rx que vamos a usar

// --- PARA EL MÓDULO GPDMA ---

#define N 64 //La LUT va a tener 64 muestras para transferir (64 valores hacen un seno bastante limpio)
// Tabla de 64 muestras para una onda senoidal en el DAC de 10 bits (con corrimiento de 6 bits)
/*uint32_t ondaSenoidal[N] = {
		32768, 35968, 39104, 42240, 45248, 48192, 51008, 53632,
		55872, 58048, 59968, 61696, 63104, 64256, 65088, 65408,
		65472, 65408, 65088, 64256, 63104, 61696, 59968, 58048,
		55872, 53632, 51008, 48192, 45248, 42240, 39104, 35968,
		32768, 29568, 26432, 23296, 20288, 17344, 14528, 11904,
		9664,  7488,  5568,  3840,  2432,  1280,  448,   128,
		64,    128,   448,   1280,  2432,  3840,  5568,  7488,
		9664,  11904, 14528, 17344, 20288, 23296, 26432, 29568
};*/

uint32_t ondaSenoidal[N] = {
    512,  562,  611,  660,  707,  753,  797,  838,
    873,  907,  937,  964,  986, 1004, 1017, 1022,
   1023, 1022, 1017, 1004,  986,  964,  937,  907,
    873,  838,  797,  753,  707,  660,  611,  562,
    512,  462,  413,  364,  317,  271,  227,  186,
    151,  117,   87,   60,   38,   20,    7,    2,
      1,    2,    7,   20,   38,   60,   87,  117,
    151,  186,  227,  271,  317,  364,  413,  462
};

volatile uint32_t valor_LDR_ARR_IZQ = 0;
volatile uint32_t valor_LDR_ARR_DER = 0; //Variables para ir guardando los valores tomados por cada
volatile uint32_t valor_LDR_ABJ_IZQ = 0; //LDR usado
volatile uint32_t valor_LDR_ABJ_DER = 0;
volatile int32_t ladoIzquierdo, ladoDerecho, ladoArriba, ladoAbajo; //Variables para sumar la luz toal que recibe cada lado
volatile uint16_t posHorizontal = SERVO_POS_CENTRO; //Los 2 servos empiezan en su posición central
volatile uint16_t posVertical = SERVO_POS_CENTRO;

volatile uint8_t convertir = 0; //Flag para empezar las conversiones. Empieza sin convertir
volatile uint16_t deadZone = 100; //Valor para considerar si hay que moverse o no
volatile uint8_t paso = 30; //Valor (en uSeg) que tengo que sumar (o restar) a los ciclos de trabajo de las ondas para mover los servos
volatile uint8_t flag_nuevoComando = 0; //Flag para cuando se recibe un comando
volatile uint8_t comando; //Variable para guardar el comando recibido por la placa y saber qué acción tomar (son letras, así que 8 bits están bien)


void configPtos(void); //Función para configurar todos los puertos y pines que usamos
void configTimer(void); //Función para configurar el TIMER2 que genera las PWM de los servos
void TIMER2_IRQHandler(void); //Handler para manejar interrupciones por TIMER2
void configADC(void); //Función para habilitar el módulo ADC
void configSysTick(void); //Función para inicializar el SysTick para empezar las conversiones del ADC
void SysTick_Handler(void); //Handler para cambiar la flag de conversión
void configDAC(void); //Función para configurar el módulo DAC
void configDMA(void); //Función para configurar el módulo DMA
void configUART(void); //Función para configurar el módulo UART
void UART3_IRQHandler(void); //Handler para cuando se recibe desde la computadora un comando
void promediarConversiones(void); //Función para tomar los valores de los LDR
void comparar(void); //Función para hacer la comparación entre los lados
void moverIzquierda(void); //Función para ajustar la posición del servo horizontal hacia la izquierda
void moverDerecha(void); //Función para ajustar la posición del servo horizontal hacia la derecha
void moverArriba(void); //Función para ajustar la posición del servo vertical hacia arriba
void moverAbajo(void); //Función para ajustar la posición del servo vertical hacia abajo
/*void moverIzquierda(int32_t paso); //Función para ajustar la posición del servo horizontal hacia la izquierda
void moverDerecha(int32_t paso); //Función para ajustar la posición del servo horizontal hacia la derecha
void moverArriba(int32_t paso); //Función para ajustar la posición del servo vertical hacia arriba
void moverAbajo(int32_t paso);*/ //Función para ajustar la posición del servo vertical hacia abajo
void procesarComando(void); //Función para procesar el comando que llegó desde la computadora
/*
int main(void) {

    // 1. Inicializamos los pines y el Timer de los servos
    configPtos();
    configTimer();
    configDAC();
    	configDMA();
    	GPDMA_ChannelCmd(0,ENABLE);


    while(1){
        // --- POSICIÓN 1: MÍNIMO (0 grados) ---
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, SERVO_POS_MIN_PAN);
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH, SERVO_POS_MIN_TILT);

        // Esperamos un par de segundos para que el motor llegue físicamente
        for(volatile int i = 0; i < 10000000; i++);

        // --- POSICIÓN 2: CENTRO (90 grados) ---
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, SERVO_POS_CENTRO);
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH, SERVO_POS_CENTRO);

        // Esperamos de nuevo
        for(volatile int i = 0; i < 10000000; i++);

        // --- POSICIÓN 3: MÁXIMO ---
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, SERVO_POS_MAX_PAN);
        TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH, SERVO_POS_MAX_TILT);

        // Esperamos de nuevo
        for(volatile int i = 0; i < 10000000; i++);
    }

    return 0 ;
}
*/

// --- VARIABLES DE CALIBRACIÓN FIJA ---
//volatile int32_t offsetarribaizquiera = 956;
//volatile int32_t offsetarribaderecha = 1092;
//volatile int32_t offsetabajoizquierda = 1192;
//volatile int32_t offsetabajoderecha = 0;

int main(void) {

	configPtos();
	configTimer();
	configADC();
	configSysTick();
	configDAC();
	configDMA();
	configUART();

	// ----------------------------------------------------
	    // BYPASS DE PRUEBA: Generador de onda cuadrada manual
	    // ----------------------------------------------------
	    while(1){
	        DAC_UpdateValue(LPC_DAC, 1023); // Enciende a 3.3V
	        for(volatile int i = 0; i < 5000; i++); // Espera

	        DAC_UpdateValue(LPC_DAC, 0);    // Apaga a 0V
	        for(volatile int i = 0; i < 5000; i++); // Espera
	    }
	    // ----------------------------------------------------

	while(1){
		//DAC_UpdateValue(LPC_DAC, 1023);

		if(convertir){ //Si el flag para convertir está habilitado
			convertir = 0; //Deshabilito las conversiones hasta que vuelva a pasar el tiempo necesario
			promediarConversiones();

			//promedio_LDR_ABJ_IZQ += 97;
			 //   promedio_LDR_ABJ_DER -= 123;

			ladoIzquierdo = valor_LDR_ARR_IZQ + valor_LDR_ABJ_IZQ;
			ladoDerecho = valor_LDR_ARR_DER + valor_LDR_ABJ_DER;
			ladoArriba = valor_LDR_ARR_IZQ + valor_LDR_ARR_DER;
			ladoAbajo = valor_LDR_ABJ_IZQ + valor_LDR_ABJ_DER;

			/*
			// --- TEST 1: Solo TILT usando lado Izquierdo ---
            ladoArriba = valor_LDR_ARR_IZQ;
            ladoAbajo  = valor_LDR_ABJ_IZQ;

            // Anulamos el movimiento horizontal
            ladoIzquierdo = 0;
            ladoDerecho   = 0;
            */

			/*
			// --- TEST 2: Solo TILT usando lado Derecho ---
            ladoArriba = valor_LDR_ARR_DER;
            ladoAbajo  = valor_LDR_ABJ_DER;

            // Anulamos el movimiento horizontal
            ladoIzquierdo = 0;
            ladoDerecho   = 0;
			*/
/*

			// --- TEST 3: Solo PAN usando fila de Arriba ---
            ladoIzquierdo = valor_LDR_ARR_IZQ;
            ladoDerecho   = valor_LDR_ARR_DER;

            // Anulamos el movimiento vertical
            ladoArriba = 0;
            ladoAbajo  = 0;

/*
			// --- TEST 4: Solo PAN usando fila de Abajo ---
            ladoIzquierdo = valor_LDR_ABJ_IZQ;
            ladoDerecho   = valor_LDR_ABJ_DER;

            // Anulamos el movimiento vertical
            ladoArriba = 0;
            ladoAbajo  = 0;

*/

			comparar(); //Comparlo la luz que recibo de cada lado. El llamado y las actualizaciones tienen que ser acá
			//adentro para tener siempre los datos más recientes
		}

		if(flag_nuevoComando){ //Si llegó un comando desde la computadora
			flag_nuevoComando = 0; //Reinicio el flag para no volver a entrar
			procesarComando(); //Voy a procesar el comando que llegó
		}
	}

    return 0;
}
void configPtos(void){

	PINSEL_CFG_Type cfgPto;

	cfgPto.Portnum = PINSEL_PORT_0; //Vamos a usar para prácticamente todas las conexiones en el purto 0
	cfgPto.Funcnum = PINSEL_FUNC_1; //Función 1 (AD0.x) para los pines por donde se van a conectar los primeros 3 LDR
	cfgPto.Pinmode = PINSEL_PINMODE_TRISTATE; //Sin resistencias internas para los pines
	cfgPto.OpenDrain = PINSEL_PINMODE_NORMAL; //No usamos open-drain para ningún pin
	for(int i=LDR_ARRIBA_IZQ_PIN;i<=LDR_ABAJO_IZQ_PIN;i++){
		cfgPto.Pinnum = i;
		PINSEL_ConfigPin(&cfgPto); //Desde P0.23 hasta P0.25 queda configurados como AD0.0 - AD0.2 con lo que definimos arriba
	}

	cfgPto.Funcnum = PINSEL_FUNC_2; //Función 2 para P0.2 (AD0.2)
	cfgPto.Pinnum = LDR_ABAJO_DER_PIN; //Necesito cambiar a P0.2 para usar otro canal AD0.6 (P0.26 tiene salida del DAC)
	PINSEL_ConfigPin(&cfgPto); //Configuro AD0.2 en modo AD0.7 (lo demás definido queda igual)

	for(int i=PIN_Tx;i<2;i++){
		cfgPto.Pinnum = i; //P0.0 y P0.1 quedan con su función 2 (TXD3 y RXD3 respectivamente)
		PINSEL_ConfigPin(&cfgPto); //Configuro los pines según la estructura. Lo que no cambia queda con los valores anteriores
	}

	cfgPto.Pinnum = PINSEL_PIN_26; //Para P0.26. Función 2 para P0.26 (AOUT)
	PINSEL_ConfigPin(&cfgPto); //P0.26 queda configurado en modo AOUT para la salida al buzzer y transistor

	cfgPto.Funcnum = PINSEL_FUNC_3; //Función 3 para P0.6 y P0.7 en modo MAT2.0 y MAT2.1 respectivamente
	for(int i=SERVO_HORIZ_PIN;i<=SERVO_VERT_PIN;i++){
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
	cfgMatch.MatchValue = SERVO_POS_CENTRO; //Empieza contando 1.5 mSeg (valor para dejar el servo en 90°). Se cambia después
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuramos el canal 0 con lo que definimos

	cfgMatch.MatchChannel = SERVO_VERT_MAT_CH; //Para el servo vertical usamos el canal 1 (MR1) del mismo timer
	TIM_ConfigMatch(LPC_TIM2,&cfgMatch); //Configuro ahora el canal 1 con lo que nos hace falta

	cfgMatch.MatchChannel = CANAL_MATCH_PERIODO; //Para controlar el período de las ondas, usamos el canal 2 (MR2) del mismo timer
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
	//ADC_BurstCmd(LPC_ADC,DISABLE); //No queremos que el ADC trabaje en modo burst
	//No puedo inicializar los canales todos a la vez si quiero empezar conversiones por software. Hay que hacerlo después
	//porque al habilitar lecturas por software, el módulo se vuelve monotarea y solo puede leer un canal a la vez
	//Tampoco quiero interrupciones por conversiones, así que no hace falta nada más
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,ENABLE);
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,ENABLE);
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,ENABLE);
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,ENABLE);
	ADC_BurstCmd(LPC_ADC,ENABLE);

}

void configSysTick(void){

	SYSTICK_InternalInit(50); //Inicializo el SysTick para que use el clock interno y genere una interrupción cada 20 mSeg
	NVIC_SetPriority(SysTick_IRQn,1); //Seteo la prioridad en un poco menos importante que la del timer de los servos
	SYSTICK_Cmd(ENABLE); //Habilito el contador
	SYSTICK_IntCmd(ENABLE); //Habilito la interrupción por SysTick
}

void SysTick_Handler(void){ //Si entro acá, pasaron 100 mSeg y tengo que habilitar las conversiones
	convertir = 1; //Pongo la flag de conversión en uno
	//No hace falta limpiar banderas para el SysTick
}
/*
void promediarConversiones(void){

   // volatile uint32_t basura = 0; // Variable para tirar la lectura contaminada

	valor_LDR_ARR_IZQ = 0;
	valor_LDR_ARR_DER = 0;
	valor_LDR_ABJ_IZQ = 0;
	valor_LDR_ABJ_DER = 0;

    // ----------------------------------------------------
	// --- LDR ARRIBA IZQUIERDA (Canal 0) ---
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,ENABLE);

    // 1. LECTURA DE LIMPIEZA
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_IZQ_CH,ADC_DATA_DONE));
	basura = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_IZQ_CH); // Descartamos esto

    // 2. LECTURA REAL
    ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_IZQ_CH,ADC_DATA_DONE));
	valor_LDR_ARR_IZQ = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_IZQ_CH); // Guardamos la real
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,DISABLE);

    // ----------------------------------------------------
	// --- LDR ARRIBA DERECHA (Canal 1) ---
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,ENABLE);

    // 1. LECTURA DE LIMPIEZA (El capacitor se vacía del Canal 0)
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_DER_CH,ADC_DATA_DONE));
	basura = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_DER_CH);

    // 2. LECTURA REAL (Ahora sí tiene el voltaje del Canal 1)
    ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_DER_CH,ADC_DATA_DONE));
	valor_LDR_ARR_DER = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_DER_CH);
	ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,DISABLE);

    // ----------------------------------------------------
	// --- LDR ABAJO IZQUIERDA (Canal 2) ---
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,ENABLE);

    // 1. LECTURA DE LIMPIEZA
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_IZQ_CH,ADC_DATA_DONE));
	basura = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_IZQ_CH);

    // 2. LECTURA REAL
    ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_IZQ_CH,ADC_DATA_DONE));
	valor_LDR_ABJ_IZQ = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_IZQ_CH);
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,DISABLE);

    // ----------------------------------------------------
	// --- LDR ABAJO DERECHA (Canal 7) ---
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,ENABLE);

    // 1. LECTURA DE LIMPIEZA
	ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_DER_CH,ADC_DATA_DONE));
	basura = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_DER_CH);

    // 2. LECTURA REAL
    ADC_StartCmd(LPC_ADC,ADC_START_NOW);
	while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_DER_CH,ADC_DATA_DONE));
	valor_LDR_ABJ_DER = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_DER_CH);
	ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,DISABLE);
}*/

void promediarConversiones(void){

	valor_LDR_ARR_IZQ = 0; //Limpio todas las variables para que no queden con valores
	valor_LDR_ARR_DER = 0; //basura antes de tomar los verdadores valores
	valor_LDR_ABJ_IZQ = 0;
	valor_LDR_ABJ_DER = 0;


	//ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,ENABLE); //Habilito el canal 0 donde está el LDR de arriba a la izquierda
	//ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
	//while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_IZQ_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
	//vamos de este while
	valor_LDR_ARR_IZQ = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_IZQ_CH); //Tomo el valor muestreado
	//ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_IZQ_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

	//ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,ENABLE); //Habilito el canal 1 donde está el LDR de arriba a la derecha
	//ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
	//while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ARRIBA_DER_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
	//vamos de este while
	valor_LDR_ARR_DER = ADC_ChannelGetData(LPC_ADC,LDR_ARRIBA_DER_CH); //Tomo el valor muestreado
	//ADC_ChannelCmd(LPC_ADC,LDR_ARRIBA_DER_CH,DISABLE); //Deshabilito el canal para pasar al que sigue


	//ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,ENABLE); //Habilito el canal 2 donde está el LDR de abajo a la izquierda
	//ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
	//while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_IZQ_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
	//vamos de este while
	valor_LDR_ABJ_IZQ = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_IZQ_CH); //Tomo el valor muestreado
	//ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_IZQ_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

	//ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,ENABLE); //Habilito el canal 6 donde está el LDR de abajo a la derecha
	//ADC_StartCmd(LPC_ADC,ADC_START_NOW); //Iniciamos la conversión en el canal habilitado ahora
	//while(!ADC_ChannelGetStatus(LPC_ADC,LDR_ABAJO_DER_CH,ADC_DATA_DONE)); //Mientras la conversión no termine, no nos
	//vamos de este while
	valor_LDR_ABJ_DER = ADC_ChannelGetData(LPC_ADC,LDR_ABAJO_DER_CH); //Tomo el valor muestreado
	//ADC_ChannelCmd(LPC_ADC,LDR_ABAJO_DER_CH,DISABLE); //Deshabilito el canal para pasar al que sigue

}

void comparar(void){


	int32_t errorHorizontal = ladoIzquierdo - ladoDerecho; //Calculo los errores de ambos lados
	int32_t errorVertical  = ladoArriba - ladoAbajo;
	//uint8_t movimiento = 0; //Flag para saber si hago sonar el buzzer o no. Empiezo asumiendo que no hay que hacerlo sonar
	/*
	int32_t pasoDinamicoH = abs(errorHorizontal)/10; //Pasos proporcionales
	if(pasoDinamicoH > 80) pasoDinamicoH = 80;
	if(pasoDinamicoH < 15) pasoDinamicoH = 15;
	int32_t pasoDinamicoV = abs(errorVertical)/10;
	if(pasoDinamicoV > 80) pasoDinamicoV = 80;
	if(pasoDinamicoV < 15) pasoDinamicoV = 15;

	if(errorHorizontal > (int32_t)deadZone) moverIzquierda(pasoDinamicoH);
	if(errorHorizontal < -(int32_t)deadZone) moverDerecha(pasoDinamicoH);
	if(errorVertical > (int32_t)deadZone) moverArriba(pasoDinamicoV);
	if(errorVertical < -(int32_t)deadZone) moverAbajo(pasoDinamicoV);*/

	if(errorHorizontal > (int32_t)deadZone){moverIzquierda();}//movimiento = 1;}
		//Si la diferencia es lo suficientemente grande, hay que moverse horizontalmente
		//Si además, la diferencia es muy positiva, nos tenemos que mover hacia la izquierda
		//Si me tengo que mover, seteo la flag de movimiento en 1
	else if(errorHorizontal < -(int32_t)deadZone){moverDerecha();}//;movimiento = 1;} //Lo mismo pero hacia la derecha ahora
	//Si el error está entre -150 y 150, estoy bastante centrado y no me muevo

	if(errorVertical > (int32_t)deadZone){moverArriba();}//movimiento = 1;} //Lo mismo de arriba pero para moverse verticalmente
	else if(errorVertical < -(int32_t)deadZone){moverAbajo();}//movimiento = 1;}

	//if(movimiento) GPDMA_ChannelCmd(0,ENABLE); //Si me tengo que mover, habilito el canal para transferir los datos y hacer sonar el buzzer
	//else GPDMA_ChannelCmd(0,DISABLE); //Si no hay que moverse, deshabilito el canal y ya el buzzer no suena
}
//void moverIzquierda(int32_t pasoH){
void moverIzquierda(void){

	/*posHorizontal -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posHorizontal <= SERVO_POS_MIN_PAN){
		posHorizontal = SERVO_POS_MIN_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}*/
	//posHorizontal += pasoH;
	posHorizontal += paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posHorizontal >= SERVO_POS_MAX_PAN){
		posHorizontal = SERVO_POS_MAX_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}
//void moverDerecha(int32_t pasoH){
void moverDerecha(void){

	/*posHorizontal += paso; //Actualizo siempre el valor sumandole esos 10 uSeg
	if(posHorizontal >= SERVO_POS_MAX_PAN){
		posHorizontal = SERVO_POS_MAX_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}*/
	//pasoHorizontal -= pasoH;
	posHorizontal -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posHorizontal <= SERVO_POS_MIN_PAN){
		posHorizontal = SERVO_POS_MIN_PAN; //No dejo que el servo se mueva más de lo que permite el eje
	}
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,posHorizontal); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}
//void moverArriba(int32_t pasoV){
void moverArriba(void){
	//posVertical += pasoV;
	posVertical += paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posVertical >= SERVO_POS_MAX_TILT){
		posVertical = SERVO_POS_MAX_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}/*
	posVertical -= paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posVertical <= SERVO_POS_MIN_TILT){
		posVertical = SERVO_POS_MIN_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}*/
	TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,posVertical); //Actualizo el valor de match para que cambie el ciclo
	//de trabajo de la onda que controla este servomotor
}
//void moverAbajo(int32_t pasoV){
void moverAbajo(void){
	//posVertical -= pasoV;
	posVertical -= paso; //Actualizo siempre el valor sumandole esos 10 uSeg
	if(posVertical <= SERVO_POS_MIN_TILT){
		posVertical = SERVO_POS_MIN_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}
	/*
	posVertical += paso; //Actualizo siempre el valor restandole esos 10 uSeg
	if(posVertical >= SERVO_POS_MAX_TILT){
		posVertical = SERVO_POS_MAX_TILT; //No dejo que el servo se mueva más de lo que permite el eje
	}*/
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
	DAC_SetDMATimeOut(LPC_DAC,195); //Sabiendo que el DAC cuenta "ticks" a 25 MHz y quiero la onda con frecuencia 1 KHz,
	//quiero 1000 ondas por segundo (sabiendo que la onda está "partida" en 64 valores). Entonces -> 1000 ondas/segundo*64 valores = 64000 pedidos por segundo
	//Entonces tiene que contar X ticks para pedir un dato nuevo -> Ticks(PCLK)/Transferencias = 25MHz/64000 = 390.625 y redondeo
	DAC_ConfigDAConverterControl(LPC_DAC,&cfgDAC); //Configuro el control del DAC con los campos que determiné
}
/*
void configDMA(void){

	static GPDMA_LLI_Type LLI; //La transferencia va a ser cíclica. Se transfiere la onda cada vez que se necesite siempre
	GPDMA_Channel_CFG_Type cfgDMA;

	LLI.SrcAddr = (uint32_t) ondaSenoidal; //Los datos siempre vienen del arreglo (el nombre del arreglo ya es un puntero a su primera posición)
	LLI.DstAddr = (uint32_t) &(LPC_DAC -> DACR); //Los datos siempre van al DAC
	LLI.NextLLI = (uint32_t) &LLI; //Es una transferencia cíclica. Siempre se manda lo mismo
	LLI.Control = ((N << 0) | (1 << 18) | (1 << 21) | (1 << 25) | (1 << 26));
	LLI.Control &= ~((1 << 27) | (1 << 31)); //Transfiero siempre 64 muestras; el tamaño de los datos de origen y llegada
	//es siempre de 32 bits; habilito el incremento automático de dirección de origen; deshabilito el de dirección
	//de destino; y deshabilito las interrupciones por DMA (no las necesito)

	cfgDMA.ChannelNum = 0; //Vamos a usar el canal 0 del DMA para hacer las transferencias
	cfgDMA.TransferSize = N; //Transfiero siempre 64 muestras
	cfgDMA.TransferWidth=GPDMA_WIDTH_HALFWORD; //solamente se usa si la trasnferencia es M2M
	cfgDMA.SrcMemAddr = (uint32_t) ondaSenoidal; //El primer dato siempre viene desde la primera posición del arreglo
	cfgDMA.DstMemAddr = (uint32_t) &(LPC_DAC -> DACR); //Los datos van siempre hasta el DAC
	cfgDMA.TransferType = GPDMA_TRANSFERTYPE_M2P; //Las tranferencias siempre son de memoria a periférico
	cfgDMA.SrcConn = 0; //Nadie avisa que hay un dato listo para transferir
	cfgDMA.DstConn = GPDMA_CONN_DAC; //El DAC es el que "pide" que le manden un dato
	cfgDMA.DMALLI = (uint32_t) &LLI; //La referencia la primera vez que entro acá es la misma LLI que transfiero

	GPDMA_Init(); //Inicializo el módulo DMA
	GPDMA_Setup(&cfgDMA); //Configuro según los campos de la estructura
	//LPC_GPDMACH0 -> DMACCControl |= (1 << 25);
	//LPC_GPDMACH0 -> DMACCControl &= ~(1 << 31); //Aseguro que las interrupciones por DMA se deshabiliten. No las necesito
	LPC_GPDMACH0 -> DMACCControl = LLI.Control;
	//No habilito el canal aún para hacer las transferencias de datos. Solamente lo voy a habilitar cuando los servos se
	//estén moviendo
	GPDMA_ChannelCmd(0,ENABLE);
}*/
void configDMA(void) {

	GPDMA_Init();
    static GPDMA_LLI_Type LLI;
    GPDMA_Channel_CFG_Type cfgDMA;

    // 1. Configuración de la lista de enlace (LLI) para transferencia cíclica
    LLI.SrcAddr = (uint32_t) ondaSenoidal;
    LLI.DstAddr = (uint32_t) &(LPC_DAC->DACR);
    LLI.NextLLI = (uint32_t) &LLI; // Apunta a sí mismo para ser infinito

    // Control: N muestras, Ancho transferencia 16 bits (1 << 18),
    // Incrementar origen, no destino, sin interrupciones.
    LLI.Control = ((N << 0) | (2 << 18) | (2 << 21) | (1 << 26));
    LLI.Control &= ~(1 << 27);

    // 2. Configuración del canal DMA
    cfgDMA.ChannelNum = 0;
    cfgDMA.SrcMemAddr = (uint32_t) ondaSenoidal;
    cfgDMA.DstMemAddr = (uint32_t) &(LPC_DAC -> DACR);
    cfgDMA.TransferSize = N;
    cfgDMA.TransferWidth = 0; // 16 bits es lo correcto para el registro DACR
    cfgDMA.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memoria a Periférico
    cfgDMA.SrcConn = 0;
    cfgDMA.DstConn = GPDMA_CONN_DAC; // El DAC es quien pide los datos
    cfgDMA.DMALLI = (uint32_t) &LLI;

    GPDMA_Setup(&cfgDMA);

    // IMPORTANTE: Habilitamos el canal
    GPDMA_ChannelCmd(0, ENABLE);
}

void configUART(void){

	UART_CFG_Type cfgUART;
	UART_ConfigStructInit(&cfgUART); //Cargo la estructura con los valores por defecto (son los que vamos a usar)
	//9600 baudios, sin bit de paridad, 8 bits de transmisión y recepción, 1 solo bit de stop
	UART_Init(LPC_UART3,&cfgUART); //Inicializo el módulo UART3 con los valores que cargué en la estructura
	//Prende el módulo, configura el CCLK a 25 MHz (en este caso). Habilito así la recepción y la transmisión en la placa

	UART_FIFO_CFG_Type cfgFIFO;
	UART_FIFOConfigStructInit(&cfgFIFO); //Cargo la estructura con valores por defecto
	//Deshabilita el DMA, habilita los resets en las FIFO's cuando termina de transmitir o recibir y la FIFO se llena con
	//un solo byte antes de mandar o recibir
	UART_FIFOConfig(LPC_UART3,&cfgFIFO); //Configuro la función de la FIFO del UART2 según la estructura
	//Habilito los buffers para que no haya pérdidas de información

	UART_TxCmd(LPC_UART3,ENABLE); //Habilitamos el pin de transimisón por UART
	UART_IntConfig(LPC_UART3,UART_INTCFG_RBR,ENABLE); //Habilito las interrupciones solo cuando llegue un dato válido
	NVIC_SetPriority(UART3_IRQn,3); //Menos prioritaria que TIMERS y SysTick
	NVIC_EnableIRQ(UART3_IRQn); //Habilito interrupciones por UART2
}

void UART3_IRQHandler(void){

	uint32_t intSrc = UART_GetIntId(LPC_UART3); //Me fijo cuál es la identificación de la interrupción por la que entramos

	if((intSrc & UART_IIR_INTID_MASK) == UART_IIR_INTID_RDA){ //Filtro para asegurar que la interrupción fue por RDA (Recieve Data Avialable)
		comando = UART_ReceiveByte(LPC_UART3); //Guardo el comando recibido si la interrupción es por RDA
		flag_nuevoComando = 1; //Habilito la flag para procesar ese comando que llegó
	}
}

void procesarComando(void){

	char textoAEnviar[50]; //Para guardar el dato a mandar. 50 caracteres es suficiente para lo que se quiere mandar
	int cantidadBytes; //Para saber cuántos bytes se van a mandar

	switch(comando){ //Empiezo a ver qué comando llegó
	case 'P': //Comando de pausa (los servos se quedan donde están)
		TIM_Cmd(LPC_TIM2,DISABLE); //Deshabilito el timer para que los servos se queden quietos
		UART_Send(LPC_UART3,(uint8_t *)"MOTORES APAGADOS\r\n",18,BLOCKING); //Puede ser bloqueante porque se manda junto
		break;
	case 'R': //Comando para reanudar el movimiento de los servos
		TIM_Cmd(LPC_TIM2,ENABLE); //Habilito el movimiento otra vez
		UART_Send(LPC_UART3,(uint8_t *)"MOTORES ENCENDIDOS\r\n",20,BLOCKING); //Lo mismo pero para este comando
		break;
		/*
	case 'I': //Comando para recibir información de la posición de los motores
		cantidadBytes = sprintf(textoAEnviar,"Servos -> Pan: %d, Tilt: %d\r\n",posHorizontal,posVertical); //Armo el texto
		UART_Send(LPC_UART3,(uint8_t *)textoAEnviar,cantidadBytes,BLOCKING); //Mando el texto
		break;*/
	/*
	case 'I': // Comando de Información para Calibración
    // Enviamos los valores calculados de los 4 lados
    	cantidadBytes = sprintf(textoAEnviar, "ARR:%ld | ABJ:%ld | IZQ:%ld | DER:%ld\r\n",
                            ladoArriba, ladoAbajo, ladoIzquierdo, ladoDerecho);
    	UART_Send(LPC_UART3, (uint8_t *)textoAEnviar, cantidadBytes, BLOCKING);
    	break;
	*/

	case 'I': // Telemetría de valores individuales de los LDR
    // Formateamos el texto mostrando la matriz de sensores: Arriba (Izq/Der) y Abajo (Izq/Der)
    	cantidadBytes = sprintf(textoAEnviar,"INDIVIDUALES %d,%d,%d,%d\r\n", valor_LDR_ARR_IZQ, valor_LDR_ARR_DER,
                            valor_LDR_ABJ_IZQ, valor_LDR_ABJ_DER);

    	UART_Send(LPC_UART3, (uint8_t *)textoAEnviar, cantidadBytes, BLOCKING); 
    	break;

	case 'L': //Comando para mover los servos a su posición central
		TIM_UpdateMatchValue(LPC_TIM2,SERVO_HORIZ_MAT_CH,SERVO_POS_CENTRO); //Actualizo la posición de los 2 servos
		TIM_UpdateMatchValue(LPC_TIM2,SERVO_VERT_MAT_CH,SERVO_POS_CENTRO);
		UART_Send(LPC_UART3,(uint8_t *)"ACOMODANDO SERVOS...\r\n",22,BLOCKING);
		break;
	default: //Si llega un comando inválido
		UART_Send(LPC_UART3,(uint8_t *)"Comando Desconocido. Opciones: P, R, I, L\r\n",43,BLOCKING); //Lo mismo
		break;
	}
}
