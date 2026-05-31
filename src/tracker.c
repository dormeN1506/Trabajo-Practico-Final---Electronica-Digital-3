#include "tracker.h"

//implementar funciones

//ADC

void config_ADC(void) {
    ADC_Init( 200000); 
    ADC_PinConfig(ADC_CHANNEL_0);
    ADC_PinConfig(ADC_CHANNEL_1);
    ADC_PinConfig(ADC_CHANNEL_2);
    ADC_PinConfig(ADC_CHANNEL_6);
    ADC_BurstEnable();
    ADC_ChannelEnable(ADC_CHANNEL_0);
    ADC_ChannelEnable(ADC_CHANNEL_1);   
    ADC_ChannelEnable(ADC_CHANNEL_2);
    ADC_ChannelEnable(ADC_CHANNEL_6);
}
void Timer0_Init(void)
{   TIM_TIMERCFG_T timerCfg = { TIM_US, 1};
    TIM_InitTimer(LPC_TIM0, &timerCfg); 
    TIM_MATCHCFG_T matchCfg;
    matchCfg.channel = TIM_MATCH_0;
    matchCfg.intEn = ENABLE;
    matchCfg.stopEn = DISABLE;
    matchCfg.resetEn = ENABLE;
    matchCfg.extOpt = TIM_NOTHING;
    matchCfg.matchValue = 10; //10 us para el periodo de muestreo 
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);
    TIM_Enable(LPC_TIM0);
    NVIC_EnableIRQ(TIMER0_IRQn);
}

uint16_t matriz_datos_adc[4][16]; //matriz para almacenar las muestras de cada canal
typedef enum {
    CONVERTIR_ADC0 ,
    CONVERTIR_ADC1,
    CONVERTIR_ADC2,
    CONVERTIR_ADC6,
    PROCESAR_DATOS
}maquina_estado_adc_T  ;



maquina_estado_adc_T maquina_estado_adc = CONVERTIR_ADC0;
volatile uint8_t i = 0; //variable para recorrer las columnas de la matriz de datos
volatile uint32_t datos_adc[4]; //
void TIMER0_IRQHandler(void)
{

    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET) {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
        switch (maquina_estado_adc) {
            case CONVERTIR_ADC0:                      //convierto adc0
                ADC_StartCmd(ADC_START_NOW);
                maquina_estado_adc = CONVERTIR_ADC1;
                break;
            case CONVERTIR_ADC1:                     //guardo dato de adc0 y convierto adc1                           
                matriz_datos_adc[0][i] = ADC_ChannelGetData(ADC_CHANNEL_0);
                ADC_StartCmd(ADC_START_NOW);
                maquina_estado_adc = CONVERTIR_ADC2;
                break;
            case CONVERTIR_ADC2:                     //guardo dato de adc1 y convierto adc2              
                matriz_datos_adc[1][i] = ADC_ChannelGetData(ADC_CHANNEL_1);
                ADC_StartCmd(ADC_START_NOW);
                maquina_estado_adc = CONVERTIR_ADC6;
                break;
            case CONVERTIR_ADC6:                    //guardo dato de adc2 y convierto adc6  
                matriz_datos_adc[2][i] = ADC_ChannelGetData(ADC_CHANNEL_2);
                ADC_StartCmd(ADC_START_NOW);
                maquina_estado_adc = PROCESAR_DATOS;
                break;
            case PROCESAR_DATOS:                    //guardo dato de adc6 y proceso los datos       
                matriz_datos_adc[3][i] = ADC_ChannelGetData(ADC_CHANNEL_6);
                    i++;
                if (i >= 16) { //si ya tengo 16 muestras de cada canal, proceso los datos
                     i = 0; //reinicio el contador de columnas para la próxima tanda de conversiones
                     //procesar los datos
                        for (uint8_t canal = 0; canal < 4; canal++) {
                            uint32_t suma = 0;
                            for (uint8_t muestra = 0; muestra < 16; muestra++) {
                                suma += matriz_datos_adc[canal][muestra];
                            }
                            datos_adc[canal] = suma / 16; //guardo el valor promediado de cada canal
                        }
                }

                maquina_estado_adc = CONVERTIR_ADC0;
                break;
        }
    }
}



void DMA_IRQHandler(void) {
    
}