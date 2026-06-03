
#include "tracker.c"



int main(void) {
    
    configPtos();
    configTimer();      // SysTick que activa flag 'convertir'
    configADC();
    configSysTick();
    configDAC();
    configDMA();
    config_UART1();
    UART1_EnviarString("Seguidor Solar OK\r\n");

    static uint32_t tele_cnt = 0;// Contador para telemetria periódica
    while (1) {
       revisar_comandos_uart();

        if (convertir)//da error porque no esta mergeada la otra rama
        {
            convertir = 0;

            // promediarConversiones() es bloqueante (~4ms)
            // La ISR de UART sigue funcionando durante ese tiempo
            // y guarda bytes en el buffer — no se pierde nada
            promediarConversiones();

           //error por merge
            ladoIzquierdo = promedio_LDR_ARR_IZQ + promedio_LDR_ABJ_IZQ;
            ladoDerecho   = promedio_LDR_ARR_DER + promedio_LDR_ABJ_DER;
            ladoArriba    = promedio_LDR_ARR_IZQ + promedio_LDR_ARR_DER;
            ladoAbajo     = promedio_LDR_ABJ_IZQ + promedio_LDR_ABJ_DER;

            // Solo mover servos si modo automático está activo
            // 'M' por UART congela los servos, 'A' los reactiva
            if (modo_automatico)
            {
                comparar();   
            }
            if (++tele_cnt >= 10)
            {
                tele_cnt = 0;
                UART1_EnviarTelemetria();
            }
        }

    }
    
    return 0;
}