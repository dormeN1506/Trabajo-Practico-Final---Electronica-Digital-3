/*
 * ============================================================
 *  SEGUIDOR SOLAR DUAL-EJE — LPC1769
 *  Versión 3 — Verificada contra Drivers_2026.pdf (David Trujillo)
 * ============================================================
 *
 *  MAPA DE PINES (sin conflictos)
 * ─────────────────────────────────────────────────────────
 *  P0.23  AD0.0  FUNC_01  → LDR Arriba-Izquierdo
 *  P0.24  AD0.1  FUNC_01  → LDR Arriba-Derecho
 *  P0.25  AD0.2  FUNC_01  → LDR Abajo-Izquierdo
 *  P0.3   AD0.6  FUNC_10  → LDR Abajo-Derecho
 *  P0.26  AOUT   FUNC_10  → DAC → Buzzer
 *  P0.6   MAT2.0 FUNC_11  → Servo Horizontal (Azimut)
 *  P0.7   MAT2.1 FUNC_11  → Servo Vertical   (Elevación)
 *  P0.15  TXD1   FUNC_01  → UART1 TX
 *  P0.16  RXD1   FUNC_01  → UART1 RX
 * ─────────────────────────────────────────────────────────
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* ============================================================
 *  SECCIÓN 1 — DEFINES
 * ============================================================ */

/* LDRs */
#define LDR_ARR_IZQ_CH     ADC_CHANNEL_0
#define LDR_ARR_DER_CH     ADC_CHANNEL_1
#define LDR_ABJ_IZQ_CH     ADC_CHANNEL_2
#define LDR_ABJ_DER_CH     ADC_CHANNEL_6

/* Servos */
#define SERVO_HORIZ_MAT_CH  TIM_MATCH_0
#define SERVO_VERT_MAT_CH   TIM_MATCH_1
#define SERVO_PWM_PERIODO   20000u
#define SERVO_POS_MIN       1000u
#define SERVO_POS_CENTRO    1500u
#define SERVO_POS_MAX       2000u

/* UART */
#define UART_BUF_SIZE   64u
#define PASO_MIN         5u
#define PASO_MAX        50u

/* Buzzer */
#define BUZZER_ON_VALUE   1023u
#define BUZZER_OFF_VALUE     0u

/* ADC */
#define ADC_MUESTRAS  16u

/* ============================================================
 *  SECCIÓN 2 — VARIABLES GLOBALES
 * ============================================================ */

volatile uint32_t promedio_LDR_ARR_IZQ = 0;
volatile uint32_t promedio_LDR_ARR_DER = 0;
volatile uint32_t promedio_LDR_ABJ_IZQ = 0;
volatile uint32_t promedio_LDR_ABJ_DER = 0;

volatile int32_t ladoIzquierdo = 0;
volatile int32_t ladoDerecho   = 0;
volatile int32_t ladoArriba    = 0;
volatile int32_t ladoAbajo     = 0;

volatile uint16_t posHorizontal = SERVO_POS_CENTRO;
volatile uint16_t posVertical   = SERVO_POS_CENTRO;

volatile uint8_t muestras_listas = 0;
volatile uint8_t modo_automatico = 1;
volatile uint8_t paso            = 10;
volatile uint8_t deadZone        = 150;

/* Buffer circular UART1 RX */
volatile uint8_t  rx_buffer[UART_BUF_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

/* Buffer DMA TX — alineado a 4 bytes por seguridad en accesos del DMA */
__attribute__((aligned(4))) static uint8_t dma_tx_buf[96];
volatile uint8_t dma_tx_busy = 0;

/* FSM ADC */
static uint16_t adc_raw[4][ADC_MUESTRAS];
static uint8_t  adc_col = 0;

typedef enum {
    ST_CONV_CH0 = 0,
    ST_CONV_CH1,
    ST_CONV_CH2,
    ST_CONV_CH6,
    ST_PROCESAR
} fsm_adc_t;

static fsm_adc_t fsm_adc = ST_CONV_CH0;

/* ============================================================
 *  SECCIÓN 3 — PROTOTIPOS
 * ============================================================ */
void configPins(void);
void configTimer2_PWM(void);
void configTimer0_ADC(void);
void configADC(void);
void configSysTick(void);
void configDAC(void);
void configDMA(void);
void config_UART1(void);

void comparar(void);
void moverIzquierda(void);
void moverDerecha(void);
void moverArriba(void);
void moverAbajo(void);
void buzzer_set(uint8_t on);

static void UART1_EnviarString(const char *str);
void UART1_EnviarTelemetria(void);
void revisar_comandos_uart(void);

/* ============================================================
 *  SECCIÓN 4 — CONFIGURACIÓN DE PINES
 * ============================================================
 *
 *  FUENTE: Drivers_2026.pdf pág. 3
 *  Tipo correcto : PINSEL_CFG_T  (NO PINSEL_CFG_Type)
 *  Campos        : .port (LPC_PORT), .pin (LPC_PIN),
 *                  .func (PINSEL_FUNC), .mode (PINSEL_MODE),
 *                  .openDrain (FunctionalState)
 *  Macros        : PORT_0, PIN_23, PINSEL_FUNC_01,
 *                  PINSEL_TRISTATE, DISABLE
 *
 *  El documento externo usaba PINSEL_CFG_Type con Portnum/Pinnum/
 *  Funcnum/Pinmode/OpenDrain — esos son los campos del driver NXP
 *  ORIGINAL, no del driver refactorizado del curso.
 */
void configPins(void)
{
    PINSEL_CFG_T cfg;
    cfg.openDrain = DISABLE;
    cfg.mode      = PINSEL_TRISTATE;
    cfg.port      = PORT_0;

    /* ── ADC: P0.23, P0.24, P0.25 → AD0.0, AD0.1, AD0.2 (FUNC_01) ── */
    cfg.func = PINSEL_FUNC_01;
    cfg.pin  = PIN_23; PINSEL_ConfigPin(&cfg);
    cfg.pin  = PIN_24; PINSEL_ConfigPin(&cfg);
    cfg.pin  = PIN_25; PINSEL_ConfigPin(&cfg);

    /* ── ADC: P0.3 → AD0.6 (FUNC_10) ── */
    cfg.func = PINSEL_FUNC_10;
    cfg.pin  = PIN_3;  PINSEL_ConfigPin(&cfg);

    /* ── DAC: P0.26 → AOUT (FUNC_10) ── */
    cfg.pin  = PIN_26; PINSEL_ConfigPin(&cfg);

    /* ── Servos: P0.6→MAT2.0, P0.7→MAT2.1 (FUNC_11) ── */
    cfg.func = PINSEL_FUNC_11;
    cfg.pin  = PIN_6;  PINSEL_ConfigPin(&cfg);
    cfg.pin  = PIN_7;  PINSEL_ConfigPin(&cfg);

    /* ── UART1: tabla del driver — índice 2=TXD1(P0.15), 3=RXD1(P0.16) ── */
    UART_PinConfig(2);
    UART_PinConfig(3);
}

/* ============================================================
 *  SECCIÓN 5 — TIMER2: PWM servos
 * ============================================================
 *
 *  FUENTE: Drivers_2026.pdf pág. 8-10
 *  Función de init : TIM_InitTimer()     (NO TIM_Init())
 *  Struct           : TIM_TIMERCFG_T con .prescaleOpt y .prescaleValue
 *  Prescaler enum   : TIM_US             (NO TIM_PRESCALE_USVAL)
 *  Enable           : TIM_Enable()       (NO TIM_Cmd())
 *  Match struct     : TIM_MATCHCFG_T con .channel, .intEn, .stopEn,
 *                     .resetEn, .extOpt, .matchValue
 *  extOpt enum      : TIM_LOW / TIM_NOTHING  (NO TIM_EXTMATCH_LOW)
 *
 *  El documento externo usaba TIM_Init()+TIM_PRESCALE_USVAL+
 *  MatchChannel+ExtMatchOutputType+TIM_Cmd() — esos son nombres del
 *  driver NXP original, no del driver del curso.
 */
void configTimer2_PWM(void)
{
    TIM_TIMERCFG_T timerCfg;
    timerCfg.prescaleOpt   = TIM_US;
    timerCfg.prescaleValue = 1;
    TIM_InitTimer(LPC_TIM2, &timerCfg);

    TIM_MATCHCFG_T matchCfg;

    /* MR0: ancho de pulso horizontal */
    matchCfg.channel    = TIM_MATCH_0;
    matchCfg.intEn      = DISABLE;
    matchCfg.stopEn     = DISABLE;
    matchCfg.resetEn    = DISABLE;
    matchCfg.extOpt     = TIM_LOW;
    matchCfg.matchValue = SERVO_POS_CENTRO;
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    /* MR1: ancho de pulso vertical */
    matchCfg.channel = TIM_MATCH_1;
    matchCfg.extOpt  = TIM_LOW;
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    /* MR2: período (20 ms) — resetea el contador */
    matchCfg.channel    = TIM_MATCH_2;
    matchCfg.intEn      = ENABLE;
    matchCfg.resetEn    = ENABLE;
    matchCfg.extOpt     = TIM_NOTHING;
    matchCfg.matchValue = SERVO_PWM_PERIODO;
    TIM_ConfigMatch(LPC_TIM2, &matchCfg);

    /* Fuerza pines a ALTO antes del primer ciclo */
    LPC_TIM2->EMR |= (1u << 0) | (1u << 1);

    NVIC_SetPriority(TIMER2_IRQn, 0);
    TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
    NVIC_EnableIRQ(TIMER2_IRQn);

    TIM_Enable(LPC_TIM2);
}

void TIMER2_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM2, TIM_MR2_INT) == SET)
    {
        LPC_TIM2->EMR |= (1u << 0) | (1u << 1);
        TIM_ClearIntPending(LPC_TIM2, TIM_MR2_INT);
    }
}

/* ============================================================
 *  SECCIÓN 6 — TIMER0: FSM muestreo ADC (tick = 10 µs)
 * ============================================================
 *
 *  La FSM lee el resultado del tick ANTERIOR y dispara la
 *  conversión del siguiente canal. Esto es NO bloqueante.
 *  El ADC a 200 kHz tarda ~5 µs: el resultado está listo
 *  cuando vuelve la ISR 10 µs después.
 *
 *  El documento externo proponía hacer
 *    while(!ADC_ChannelGetStatus(...))
 *  DENTRO de la ISR del timer — eso bloquea la ISR hasta
 *  ~5 µs y anula la ventaja de tener un timer periódico.
 *  Se mantiene el diseño no bloqueante original.
 */
void configTimer0_ADC(void)
{
    TIM_TIMERCFG_T timerCfg;
    timerCfg.prescaleOpt   = TIM_US;
    timerCfg.prescaleValue = 1;
    TIM_InitTimer(LPC_TIM0, &timerCfg);

    TIM_MATCHCFG_T matchCfg;
    matchCfg.channel    = TIM_MATCH_0;
    matchCfg.intEn      = ENABLE;
    matchCfg.stopEn     = DISABLE;
    matchCfg.resetEn    = ENABLE;
    matchCfg.extOpt     = TIM_NOTHING;
    matchCfg.matchValue = 10;
    TIM_ConfigMatch(LPC_TIM0, &matchCfg);

    NVIC_SetPriority(TIMER0_IRQn, 2);
    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Enable(LPC_TIM0);
}

void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT) == SET)
    {
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);

        switch (fsm_adc)
        {
            case ST_CONV_CH0:
                ADC_StartCmd(ADC_START_NOW);
                fsm_adc = ST_CONV_CH1;
                break;

            case ST_CONV_CH1:
                /* Lee CH0 (disparado en tick anterior) y dispara CH1 */
                adc_raw[0][adc_col] = ADC_ChannelGetData(LDR_ARR_IZQ_CH);
                ADC_StartCmd(ADC_START_NOW);
                fsm_adc = ST_CONV_CH2;
                break;

            case ST_CONV_CH2:
                adc_raw[1][adc_col] = ADC_ChannelGetData(LDR_ARR_DER_CH);
                ADC_StartCmd(ADC_START_NOW);
                fsm_adc = ST_CONV_CH6;
                break;

            case ST_CONV_CH6:
                adc_raw[2][adc_col] = ADC_ChannelGetData(LDR_ABJ_IZQ_CH);
                ADC_StartCmd(ADC_START_NOW);
                fsm_adc = ST_PROCESAR;
                break;

            case ST_PROCESAR:
                adc_raw[3][adc_col] = ADC_ChannelGetData(LDR_ABJ_DER_CH);
                adc_col++;

                if (adc_col >= ADC_MUESTRAS)
                {
                    adc_col = 0;
                    uint32_t s0=0, s1=0, s2=0, s3=0;
                    for (uint8_t m = 0; m < ADC_MUESTRAS; m++)
                    {
                        s0 += adc_raw[0][m];
                        s1 += adc_raw[1][m];
                        s2 += adc_raw[2][m];
                        s3 += adc_raw[3][m];
                    }
                    promedio_LDR_ARR_IZQ = s0 >> 4;
                    promedio_LDR_ARR_DER = s1 >> 4;
                    promedio_LDR_ABJ_IZQ = s2 >> 4;
                    promedio_LDR_ABJ_DER = s3 >> 4;
                    muestras_listas = 1;
                }

                fsm_adc = ST_CONV_CH0;
                break;
        }
    }
}

/* ============================================================
 *  SECCIÓN 7 — ADC
 * ============================================================
 *
 *  FUENTE: Drivers_2026.pdf pág. 11
 *  void ADC_Init(uint32_t rate)         — 1 parámetro, sin LPC_ADC
 *  void ADC_BurstDisable(void)          — sin parámetros
 *  void ADC_ChannelEnable(ADC_CHANNEL)  — 1 parámetro, sin LPC_ADC
 *
 *  El documento externo usaba ADC_Init(LPC_ADC, 200000),
 *  ADC_BurstCmd(LPC_ADC, DISABLE) y ADC_ChannelCmd(LPC_ADC, ch, ENABLE)
 *  — esas son las firmas del driver NXP original, no del curso.
 */
void configADC(void)
{
    ADC_Init(200000);

    ADC_PinConfig(LDR_ARR_IZQ_CH);
    ADC_PinConfig(LDR_ARR_DER_CH);
    ADC_PinConfig(LDR_ABJ_IZQ_CH);
    ADC_PinConfig(LDR_ABJ_DER_CH);

    ADC_BurstDisable();

    ADC_ChannelEnable(LDR_ARR_IZQ_CH);
    ADC_ChannelEnable(LDR_ARR_DER_CH);
    ADC_ChannelEnable(LDR_ABJ_IZQ_CH);
    ADC_ChannelEnable(LDR_ABJ_DER_CH);
}

/* ============================================================
 *  SECCIÓN 8 — SysTick (base de tiempo 100 ms)
 * ============================================================ */
void configSysTick(void)
{
    SYSTICK_InternalInit(100);
    NVIC_SetPriority(SysTick_IRQn, 3);
    SYSTICK_Cmd(ENABLE);
    SYSTICK_IntCmd(ENABLE);
}

void SysTick_Handler(void)
{
    /* Reservado. La telemetría se cuenta por tandas en el main loop. */
}

/* ============================================================
 *  SECCIÓN 9 — DAC + Buzzer
 * ============================================================
 *
 *  FUENTE: Drivers_2026.pdf pág. 13
 *  void DAC_Init(void)                  — sin parámetros
 *  void DAC_UpdateValue(uint32_t)       — 1 parámetro, sin LPC_DAC
 *
 *  El documento externo usaba DAC_Init(LPC_DAC) y
 *  DAC_UpdateValue(LPC_DAC, value) — firmas del driver NXP original.
 */
void configDAC(void)
{
    DAC_Init();
    DAC_UpdateValue(BUZZER_OFF_VALUE);
}

void buzzer_set(uint8_t on)
{
    DAC_UpdateValue(on ? BUZZER_ON_VALUE : BUZZER_OFF_VALUE);
}

/* ============================================================
 *  SECCIÓN 10 — DMA (GPDMA CH0 para UART1 TX)
 * ============================================================
 *
 *  FUENTE: Drivers_2026.pdf pág. 14-15
 *  Struct campos (minúsculas): channelNum, transferSize, type,
 *    srcMemAddr, dstMemAddr, srcConn, dstConn, src, dst, intTC,
 *    intErr, linkedList
 *  Enums correctos:
 *    GPDMA_M2P           (no GPDMA_TRANSFERTYPE_M2P)
 *    GPDMA_UART1_Tx      (no GPDMA_CONN_UART1_Tx)
 *    GPDMA_INTTC         (no GPDMA_STAT_INTTC)
 *    GPDMA_CLR_INTTC     (no GPDMA_STATCLR_INTTC)
 *    GPDMA_ChannelStart(GPDMA_CH_0) — con enum, no entero 0
 *
 *  El documento externo introdujo nombres inexistentes en el driver.
 */
void configDMA(void)
{
    GPDMA_Init();
    NVIC_SetPriority(DMA_IRQn, 2);
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_IRQHandler(void)
{
    if (GPDMA_IntGetStatus(GPDMA_INTTC, GPDMA_CH_0) == SET)
    {
        GPDMA_ClearIntPending(GPDMA_CLR_INTTC, GPDMA_CH_0);
        dma_tx_busy = 0;
    }
    if (GPDMA_IntGetStatus(GPDMA_INTERR, GPDMA_CH_0) == SET)
    {
        GPDMA_ClearIntPending(GPDMA_CLR_INTERR, GPDMA_CH_0);
        dma_tx_busy = 0;
    }
}

/* ============================================================
 *  SECCIÓN 11 — UART1
 * ============================================================
 *
 *  FUENTE: lpc17xx_uart.c (driver real del curso)
 *  Struct UART_CFG_T  con campos: baudRate, dataBits, stopBits
 *  Struct UART_FIFO_CFG_T con campos: level, resetTxBuf,
 *    resetRxBuf, dmaMode
 *  dataBits enum: UART_DBITS_8  (no UART_DATABIT_8)
 *  UART_TxEnable(LPC_UART_TypeDef*)  — 1 parámetro, sin ENABLE
 *
 *  El documento externo usaba UART_CFG_Type, UART_FIFO_CFG_Type,
 *  FIFO_Level, FIFO_ResetRxBuf, UART_DATABIT_8 y
 *  UART_TxEnable(LPC_UART1, ENABLE) — ninguno de esos existe en
 *  el driver del curso.
 */
void config_UART1(void)
{
    UART_PinConfig(2);   /* TXD1 → P0.15 (ya llamado en configPins, idempotente) */
    UART_PinConfig(3);   /* RXD1 → P0.16 */

    UART_CFG_T uartCfg;
    uartCfg.baudRate = 9600;
    uartCfg.dataBits = UART_DBITS_8;
    uartCfg.parity   = UART_PARITY_NONE;
    uartCfg.stopBits = UART_STOPBIT_1;
    UART_Init(LPC_UART1, &uartCfg);

    UART_FIFO_CFG_T fifoCfg;
    fifoCfg.level      = UART_FIFO_TRGLEV0;
    fifoCfg.resetTxBuf = ENABLE;
    fifoCfg.resetRxBuf = ENABLE;
    fifoCfg.dmaMode    = ENABLE;
    UART_FIFOConfig(LPC_UART1, &fifoCfg);

    UART_IntConfig(LPC_UART1, UART_INT_RBR, ENABLE);

    NVIC_SetPriority(UART1_IRQn, 3);
    NVIC_EnableIRQ(UART1_IRQn);

    UART_TxEnable(LPC_UART1);
}

void UART1_IRQHandler(void)
{
    uint32_t iir   = UART_GetIntId(LPC_UART1);
    uint32_t causa = iir & 0x0Eu;

    if (causa == 0x04u || causa == 0x0Cu)
    {
        uint8_t  byte_rx   = UART_ReceiveByte(LPC_UART1);
        uint16_t next_head = (uint16_t)((rx_head + 1u) % UART_BUF_SIZE);
        if (next_head != rx_tail)
        {
            rx_buffer[rx_head] = byte_rx;
            rx_head = next_head;
        }
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
    if (dma_tx_busy) return;

    uint32_t len = (uint32_t)snprintf(
        (char *)dma_tx_buf, sizeof(dma_tx_buf),
        "$AI:%lu,AD:%lu,BI:%lu,BD:%lu,PH:%u,PV:%u,PASO:%u\r\n",
        (unsigned long)promedio_LDR_ARR_IZQ,
        (unsigned long)promedio_LDR_ARR_DER,
        (unsigned long)promedio_LDR_ABJ_IZQ,
        (unsigned long)promedio_LDR_ABJ_DER,
        (unsigned)posHorizontal,
        (unsigned)posVertical,
        (unsigned)paso);

    if (len == 0 || len >= sizeof(dma_tx_buf)) return;

    GPDMA_Channel_CFG_T dmaCfg;
    dmaCfg.channelNum   = GPDMA_CH_0;
    dmaCfg.transferSize = len;
    dmaCfg.type         = GPDMA_M2P;
    dmaCfg.srcMemAddr   = (uint32_t)dma_tx_buf;
    dmaCfg.dstMemAddr   = 0;
    dmaCfg.srcConn      = 0;
    dmaCfg.dstConn      = GPDMA_UART1_Tx;
    dmaCfg.src.width    = GPDMA_BYTE;
    dmaCfg.src.burst    = GPDMA_BSIZE_1;
    dmaCfg.src.increment = ENABLE;
    dmaCfg.dst.width    = GPDMA_BYTE;
    dmaCfg.dst.burst    = GPDMA_BSIZE_1;
    dmaCfg.dst.increment = DISABLE;
    dmaCfg.intTC        = ENABLE;
    dmaCfg.intErr       = ENABLE;
    dmaCfg.linkedList   = 0;

    if (GPDMA_SetupChannel(&dmaCfg) == SUCCESS)
    {
        dma_tx_busy = 1;
        GPDMA_ChannelStart(GPDMA_CH_0);
    }
}

/* ============================================================
 *  SECCIÓN 12 — COMANDOS UART
 * ============================================================ */
void revisar_comandos_uart(void)
{
    while (rx_head != rx_tail)
    {
        uint8_t cmd = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % UART_BUF_SIZE);

        switch (cmd)
        {
            case 'A': case 'a':
                modo_automatico = 1;
                UART1_EnviarString("ACK:AUTO\r\n");
                break;

            case 'M': case 'm':
                modo_automatico = 0;
                UART1_EnviarString("ACK:MANUAL\r\n");
                break;

            case 'S': case 's':
                UART1_EnviarTelemetria();
                break;

            case '+':
            {
                if (paso < PASO_MAX) paso = (uint8_t)(paso + 5u);
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            case '-':
            {
                if (paso > PASO_MIN) paso = (uint8_t)(paso - 5u);
                char ack[20];
                snprintf(ack, sizeof(ack), "ACK:PASO:%u\r\n", paso);
                UART1_EnviarString(ack);
                break;
            }

            case 'R': case 'r':
                modo_automatico = 0;
                posHorizontal   = SERVO_POS_CENTRO;
                posVertical     = SERVO_POS_CENTRO;
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, posHorizontal);
                TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH,  posVertical);
                buzzer_set(0);
                UART1_EnviarString("ACK:RESET\r\n");
                break;

            case '\r': case '\n':
                break;

            default:
                UART1_EnviarString("ERR:CMD\r\n");
                break;
        }
    }
}

/* ============================================================
 *  SECCIÓN 13 — LÓGICA DE SEGUIMIENTO
 * ============================================================
 *
 *  SUAVIZADO DE SERVOS:
 *  El main loop llama comparar() solo cada 50 tandas de muestras.
 *  Cada tanda = 5 ticks × 10 µs × 16 muestras = 800 µs.
 *  50 × 800 µs = 40 ms entre ajustes de servo.
 *  Esto evita que el error pequeño mueva los servos a máxima
 *  velocidad en cada iteración del lazo.
 */
void comparar(void)
{
    int32_t errorH = ladoIzquierdo - ladoDerecho;
    int32_t errorV = ladoArriba    - ladoAbajo;
    uint8_t limite = 0;

    if      (errorH >  (int32_t)deadZone) moverIzquierda();
    else if (errorH < -(int32_t)deadZone) moverDerecha();

    if      (errorV >  (int32_t)deadZone) moverArriba();
    else if (errorV < -(int32_t)deadZone) moverAbajo();

    if (posHorizontal <= SERVO_POS_MIN || posHorizontal >= SERVO_POS_MAX ||
        posVertical   <= SERVO_POS_MIN || posVertical   >= SERVO_POS_MAX)
    {
        limite = 1;
    }
    buzzer_set(limite);
}

void moverIzquierda(void)
{
    posHorizontal = (posHorizontal > SERVO_POS_MIN + paso)
                    ? (uint16_t)(posHorizontal - paso)
                    : SERVO_POS_MIN;
    TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, posHorizontal);
}

void moverDerecha(void)
{
    posHorizontal = (posHorizontal < SERVO_POS_MAX - paso)
                    ? (uint16_t)(posHorizontal + paso)
                    : SERVO_POS_MAX;
    TIM_UpdateMatchValue(LPC_TIM2, SERVO_HORIZ_MAT_CH, posHorizontal);
}

void moverArriba(void)
{
    posVertical = (posVertical > SERVO_POS_MIN + paso)
                  ? (uint16_t)(posVertical - paso)
                  : SERVO_POS_MIN;
    TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH, posVertical);
}

void moverAbajo(void)
{
    posVertical = (posVertical < SERVO_POS_MAX - paso)
                  ? (uint16_t)(posVertical + paso)
                  : SERVO_POS_MAX;
    TIM_UpdateMatchValue(LPC_TIM2, SERVO_VERT_MAT_CH, posVertical);
}

/* ============================================================
 *  SECCIÓN 14 — MAIN
 * ============================================================ */
int main(void)
{
    configPins();
    configTimer2_PWM();
    configTimer0_ADC();
    configADC();
    configSysTick();
    configDAC();
    configDMA();
    config_UART1();

    UART1_EnviarString("Seguidor Solar OK\r\n");

    static uint32_t tele_cnt         = 0;
    static uint32_t filtro_servo_cnt = 0;

    while (1)
    {
        revisar_comandos_uart();

        if (muestras_listas)
        {
            muestras_listas = 0;

            ladoIzquierdo = (int32_t)(promedio_LDR_ARR_IZQ + promedio_LDR_ABJ_IZQ);
            ladoDerecho   = (int32_t)(promedio_LDR_ARR_DER + promedio_LDR_ABJ_DER);
            ladoArriba    = (int32_t)(promedio_LDR_ARR_IZQ + promedio_LDR_ARR_DER);
            ladoAbajo     = (int32_t)(promedio_LDR_ABJ_IZQ + promedio_LDR_ABJ_DER);

            /* Ajuste de servos cada 40 ms (~50 tandas) */
            if (modo_automatico && (++filtro_servo_cnt >= 50))
            {
                filtro_servo_cnt = 0;
                comparar();
            }

            /* Telemetría cada ~80 ms (~100 tandas) */
            if (++tele_cnt >= 100)
            {
                tele_cnt = 0;
                UART1_EnviarTelemetria();
            }
        }
    }

    return 0;
}
