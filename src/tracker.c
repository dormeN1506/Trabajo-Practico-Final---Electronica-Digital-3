#include "tracker.h"

//implementar funciones

//ADC

void config_ADC(void) {
    ADC_Init( 200000); 
    ADC_PinConfig(ADC_CHANNEL_0);
    ADC_PinConfig(ADC_CHANNEL_1);
    ADC_PinConfig(ADC_CHANNEL_2);
    ADC_PinConfig(ADC_CHANNEL_3);
    ADC_BurstEnable();
    ADC_StartCmd(ADC_START_CONTINUOUS);
    ADC_ChannelEnable(ADC_CHANNEL_0);
    ADC_ChannelEnable(ADC_CHANNEL_1);   
    ADC_ChannelEnable(ADC_CHANNEL_2);
    ADC_ChannelEnable(ADC_CHANNEL_3);
}
void config_DMA(void) {
    
    GPDMA_Init();
    GPDMA_Channel_CFG_T dmaCfgAdc0;
    

    lli1[0].srcAddr = (uint32_t)&LPC_ADC->ADDR0; 
    lli1[0].dstAddr = (uint32_t)&ldr0_raw_buffer_A[0];
    lli1[0].nextLLI = (uint32_t)&lli2[0];
    lli1[0].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
    //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 
    lli2[0].srcAddr = (uint32_t)&LPC_ADC->ADDR0; 
    lli2[0].dstAddr = (uint32_t)&ldr0_raw_buffer_B[0];
    lli2[0].nextLLI = (uint32_t)&lli1[0];
    lli2[0].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
     //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 


    
    
    //periferico a memoria
    dmaCfgAdc0.channelNum = GPDMA_CH_0;
    dmaCfgAdc0.transferSize = N_MUESTRAS; //16(valor de N_MUESTRAS) en total si transferimos de a 16bits
    dmaCfgAdc0.type = GPDMA_P2M;
    dmaCfgAdc0.srcMemAddr = (uint32_t)&LPC_ADC->ADDR0;//creo que no se usa  
    dmaCfgAdc0.dstMemAddr = (uint32_t)&ldr0_raw_buffer_B[0];//creo que se ignora esto pero por las dudas
    dmaCfgAdc0.srcConn = GPDMA_ADC;
    dmaCfgAdc0.dstConn = 0;
    dmaCfgAdc0.src.width = GPDMA_HALFWORD;
    dmaCfgAdc0.src.burst = GPDMA_BSIZE_1;
    dmaCfgAdc0.src.increment = DISABLE;
    dmaCfgAdc0.dst.width = GPDMA_HALFWORD;
    dmaCfgAdc0.dst.burst = GPDMA_BSIZE_1;
    dmaCfgAdc0.dst.increment = ENABLE;
    dmaCfgAdc0.intTC = ENABLE;
    dmaCfgAdc0.intErr = DISABLE;//VER SI ES NECESARIO
    dmaCfgAdc0.linkedList = &lli1[0]; //ver si queremos ubicar la lli en una direccion especifica
  

    GPDMA_SetupChannel(&dmaCfgAdc0);

    GPDMA_Channel_CFG_T dmaCfgAdc1;
    GPDMA_Channel_CFG_T dmaCfgAdc2;
    GPDMA_Channel_CFG_T dmaCfgAdc3;


    GPDMA_ChannelStart(GPDMA_CH_0);
    GPDMA_ChannelStart(GPDMA_CH_1);
    GPDMA_ChannelStart(GPDMA_CH_2);
    GPDMA_ChannelStart(GPDMA_CH_3);
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_IRQHandler(void) {
    //buffer A o B?
}