#include "tracker.h"

//implementar funciones

//ADC

//-----------BORRAR DESPUES DE MERGEAR LA OTRA RAMA----------------
volatile uint32_t promedio_LDR_ARR_IZQ = 0;
volatile uint32_t promedio_LDR_ARR_DER = 0; //Variables para ir guardando los promedios de los valores tomados por cada
volatile uint32_t promedio_LDR_ABJ_IZQ = 0; //LDR usado
volatile uint32_t promedio_LDR_ABJ_DER = 0;
//---------------------------------------------------------------------------

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
                        
                        promedio_LDR_ARR_IZQ = datos_adc[0]; //LDR Arriba Izquierdo
                        promedio_LDR_ARR_DER = datos_adc[1]; //LDR Arriba Derecho
                        promedio_LDR_ABJ_IZQ = datos_adc[2]; //LDR Abajo Izquierdo
                        promedio_LDR_ABJ_DER = datos_adc[3]; //LDR Abajo Derecho

                        //  agregar código PARA enviar telemetría.
                }       

                maquina_estado_adc = CONVERTIR_ADC0;
                break;
        }
    }
}




//-----------------implementacion uart--------------------

#define UART_BUF_SIZE 64

// Buffers circulares de software para RX
volatile uint8_t rx_buffer[UART_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// Variables de estado de tu proyecto
volatile uint8_t modo_automatico = 1;

void config_UART(void) {
    UART_CFG_T uartCfg;
    UART_FIFO_CFG_T FioCfg;

    // 1. Configurar Pines nativos usando la tabla estática del driver
    UART_PinConfig(0); // Configura TX0 en P0.2
    UART_PinConfig(1); // Configura RX0 en P0.3

    // 2. Parámetros de comunicación: 9600 baudios, 8 bits de datos, sin paridad, 1 bit de parada
    uartCfg.baudRate = 9600;
    uartCfg.dataBits = UART_DBITS_8;
    uartCfg.parity   = UART_PARITY_NONE;
    uartCfg.stopBits = UART_STOPBIT_1;
    
    // Inicializa el periférico y calcula divisores de clock automáticamente
    UART_Init(LPC_UART0, &uartCfg);

    // 3. Configurar FIFOs de hardware (Trig Level 0 = interrumpe por cada 1 byte recibido)
    FioCfg.level      = UART_FIFO_TRGLEV0;
    FioCfg.resetRxBuf = ENABLE;
    FioCfg.resetTxBuf = ENABLE;
    FioCfg.dmaMode    = DISABLE;
    UART_FIFOConfig(LPC_UART0, &FioCfg);

    // 4. Habilitar interrupción por Recepción de Datos (RBR) en el periférico
    UART_IntConfig(LPC_UART0, UART_INT_RBR, ENABLE);
    
    // 5. Habilitar la interrupción en el NVIC
    NVIC_EnableIRQ(UART0_IRQn);
    
    // 6. Habilitar la transmisión en la UART (Line Control)
    UART_TxEnable(LPC_UART0);
}

void UART0_IRQHandler(void) {
    // Leer el Identificador de Interrupción (IIR)
    uint32_t intsrc = UART_GetIntId(LPC_UART0);
    
    // Filtrar la causa de la interrupción. Buscamos UART_IID_RDA (Receive Data Available)
    if ((intsrc & 0x0E) == 0x04) { 
        
        uint8_t byte_recibido = UART_ReceiveByte(LPC_UART0); //
        
        // Insertar en el buffer circular de software
        uint16_t next_head = (rx_head + 1) % UART_BUF_SIZE;
        if (next_head != rx_tail) { 
            rx_buffer[rx_head] = byte_recibido;
            rx_head = next_head;
        }
    }
}

void UART_EnviarString(const char *str) {
    // UART_Send se encarga de recorrer el buffer e ir llenando la FIFO de hardware
    UART_Send(LPC_UART0, (const uint8_t *)str, strlen(str), BLOCKING);
}

void revisar_comandos_uart(void) {
    // Mientras haya bytes sin procesar en el buffer circular que llenó la ISR
    while (rx_head != rx_tail) {
        
        // Extraemos un solo carácter del buffer
        uint8_t comando = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % UART_BUF_SIZE; // Avanzamos la cola
        
        // Evaluamos directamente el carácter con un switch
        switch (comando) {
            
            case 'M': // Si llega la letra 'M' (Mayúscula)
            case 'm': // O si llega la letra 'm' (Minúscula)
                modo_automatico = 0; // Desactivar el seguidor solar automático
                UART_EnviarString("ACK: Modo Manual\r\n");
                break;
                
            case 'A':
            case 'a':
                modo_automatico = 1; // Activar el seguidor solar automático
                UART_EnviarString("ACK: Modo Auto\r\n");
                break;
                
            case 'S':
            case 's':
                UART_EnviarString("ACK: Enviando Estado...\r\n");
                break;

            // ─── FILTRO CRÍTICO PARA ENTER / FIN DE LÍNEA ───
            case '\r': // Retorno de carro (Carriage Return)
            case '\n': // Salto de línea (Line Feed)
                // Si llegan estos caracteres, hacemos un break directo. 
                // Los ignoramos en silencio para que no caigan en 'default'.
                break;

            default:
                // Si mandan cualquier otra letra que no reconoce el sistema
                UART_EnviarString("ERROR: Comando Desconocido\r\n");
                break;
        }
    }
}