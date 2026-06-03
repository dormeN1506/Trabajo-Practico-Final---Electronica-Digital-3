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

//conflicto con pin fisico adc6  y uart0
#define PASO_MIN   5u    // movimiento mínimo — muy lento
#define PASO_MAX   50u   // movimiento máximo — muy rápido
#define UART_BUF_SIZE 64

// Buffers circulares de software para RX
volatile uint8_t rx_buffer[UART_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// Variables de estado de tu proyecto
volatile uint8_t  modo_automatico = 1;   // 1=auto, 0=manual
volatile uint8_t  paso_max        = 5;   // velocidad servo [1..20]


//BORRAR DESPUES DE MERGEAR LA OTRA RAMA
volatile uint32_t ldr_raw[4];   // valores 12-bit de los LDRs
volatile int32_t  angulo_H;     // ángulo servo horizontal
volatile int32_t  angulo_V;     // ángulo servo vertical



void config_UART1(void)
{
    // ── 1. Configurar pines usando la tabla estática del driver ───
    //  La tabla PinCfg[] en UART_PinConfig() tiene índice fijo:
    //    índice 2 → {PORT_0, PIN_15, PINSEL_FUNC_01} → TX1
    //    índice 3 → {PORT_0, PIN_16, PINSEL_FUNC_01} → RX1
    UART_PinConfig(2);   // TX1 → P0.15
    UART_PinConfig(3);   // RX1 → P0.16

    // ── 2. Parámetros de comunicación ────────────────────────────
    UART_CFG_T uartCfg;
    uartCfg.baudRate = 9600;
    uartCfg.dataBits = UART_DBITS_8;
    uartCfg.parity   = UART_PARITY_NONE;
    uartCfg.stopBits = UART_STOPBIT_1;

    // UART_Init() internamente:
    //   - Habilita PCONP_PCUART1
    //   - Calcula y carga DLL/DLM/FDR con uart_set_divisors()
    //   - Configura LCR (8N1)
    //   - Limpia FIFOs, IER, ACR
    //   - Limpia registros específicos de UART1 (MCR, RS485CTRL, etc.)
    // El cast es necesario porque UART1 es LPC_UART1_TypeDef*
    // pero UART_Init acepta LPC_UART_TypeDef* — el driver lo maneja internamente
    UART_Init(LPC_UART1, &uartCfg);
    // ── 3. Configurar FIFO ────────────────────────────────────────
    //  TRGLEV0 = interrupción por cada 1 byte recibido
    //  Reset ambas FIFOs al iniciar
    UART_FIFO_CFG_T fifoCfg;
    fifoCfg.level      = UART_FIFO_TRGLEV0;
    fifoCfg.resetRxBuf = ENABLE;
    fifoCfg.resetTxBuf = ENABLE;
    fifoCfg.dmaMode    = DISABLE;
    UART_FIFOConfig(LPC_UART1, &fifoCfg);

    // ── 4. Habilitar interrupción RBR en el periférico ────────────
    //  UART_INT_RBR → IER bit 0 (Receive Buffer Register interrupt)
    UART_IntConfig(LPC_UART1, UART_INT_RBR, ENABLE);

    // ── 5. Habilitar interrupción en el NVIC ─────────────────────
    NVIC_EnableIRQ(UART1_IRQn);

    // ── 6. Habilitar transmisión (TER bit 0) ─────────────────────
    UART_TxEnable(LPC_UART1);
}

void UART1_IRQHandler(void)
{
    // UART_GetIntId() lee UARTx->IIR & 0x03CF
    // Bits [3:1] indican la causa:
    //   0x04 (010) = RDA  — Receive Data Available
    //   0x0C (110) = CTI  — Character Time-out (FIFO con datos sin leer)
    uint32_t iir   = UART_GetIntId(LPC_UART1);
    uint32_t causa = iir & 0x0Eu;

    if (causa == 0x04u || causa == 0x0Cu)
    {
        // UART_ReceiveByte() lee UARTx->RBR & 0xFF
        uint8_t byte_rx  = UART_ReceiveByte(LPC_UART1);
        uint8_t next_head = (uint8_t)((rx_head + 1u) % UART_BUF_SIZE);

        if (next_head != rx_tail)   // buffer no lleno
        {
            rx_buffer[rx_head] = byte_rx;
            rx_head         = next_head;
        }
        // Si el buffer está lleno, el byte se descarta silenciosamente.
    }
}

static void UART1_EnviarString(const char *str)
{
    UART_Send(LPC_UART1,
              (const uint8_t *)str,
              (uint32_t)strlen(str),
              BLOCKING);
}
void UART1_EnviarTelemetria(void)
{
    char buf[72];
    snprintf(buf, sizeof(buf),
             "$AI:%lu,AD:%lu,BI:%lu,BD:%lu,H:%ld,V:%ld,PASO:%u\r\n",
             (unsigned long)promedio_LDR_ARR_IZQ,
             (unsigned long)promedio_LDR_ARR_DER,
             (unsigned long)promedio_LDR_ABJ_IZQ,
             (unsigned long)promedio_LDR_ABJ_DER,
             (long)angulo_H,
             (long)angulo_V,
             (unsigned)paso);          //error por merge

    UART_Send(LPC_UART1,
              (const uint8_t *)buf,
              (uint32_t)strlen(buf),
              BLOCKING);
}

//  revisar_comandos_uart()
//  Llamar desde el main loop — nunca desde una ISR.
//  Vacía el buffer circular y ejecuta el comando correspondiente.

void revisar_comandos_uart(void)
{
    while (rx_head != rx_tail)
    {
        uint8_t cmd = rx_buffer[rx_tail];
        rx_tail     = (uint8_t)((rx_tail + 1u) % UART_BUF_SIZE);

        switch (cmd)
        {
            // ── Modo automático ──────────────────────────────────
            case 'A': case 'a':
                modo_automatico = 1;
                UART1_EnviarString("ACK:AUTO\r\n");
                break;

            // ── Modo manual ──────────────────────────────────────
            case 'M': case 'm':
                modo_automatico = 0;
                UART1_EnviarString("ACK:MANUAL\r\n");
                break;

            // ── Telemetría inmediata ─────────────────────────────
            case 'S': case 's':
                UART1_EnviarTelemetria();
                break;

            // ── Velocidad +5µs ───────────────────────────────────
            case '+':
            {
                if (paso < PASO_MAX) paso += 5;
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            // ── Velocidad -5µs ───────────────────────────────────
            case '-':
            {
                if (paso > PASO_MIN) paso -= 5;
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            // ── Reset servos al centro ───────────────────────────
            // En tu sistema la posición real está en posHorizontal
            // y posVertical (en µs), no en angulo_H/V.
            // Hay que actualizar esas variables Y el Timer.
            case 'R': case 'r':
                modo_automatico = 0;
                posHorizontal   = SERVO_POS_CENTRO;//error por merge
                posVertical     = SERVO_POS_CENTRO;//error por merge
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, posHorizontal);
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH,  posVertical);
                UART1_EnviarString("ACK:RESET\r\n");
                break;

            // ── Fin de línea del terminal — ignorar ──────────────
            case '\r': case '\n':
                break;

            // ── Comando desconocido ──────────────────────────────
            default:
                UART1_EnviarString("ERR:CMD\r\n");
                break;
        }
    }
}