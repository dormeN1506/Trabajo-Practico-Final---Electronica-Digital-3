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
    ADC_StartCmd(ADC_START_CONTINUOUS);
    ADC_ChannelEnable(ADC_CHANNEL_0);
    ADC_ChannelEnable(ADC_CHANNEL_1);   
    ADC_ChannelEnable(ADC_CHANNEL_2);
    ADC_ChannelEnable(ADC_CHANNEL_6);
}
void config_DMA(void) {
    
    GPDMA_Init();

    //---------------- CONFIGURACION DEL DMA PARA EL ADC0 ----------------
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
    dmaCfgAdc0.intErr = DISABLE;//VER SI ES NECESARIO / capaz sirve para ver s el bus tira error
    dmaCfgAdc0.linkedList = &lli1[0]; //ver si queremos ubicar la lli en una direccion especifica
  

    GPDMA_SetupChannel(&dmaCfgAdc0);


    //---------------- CONFIGURACION DEL DMA PARA EL ADC1 ----------------
    GPDMA_Channel_CFG_T dmaCfgAdc1;
    

    lli1[1].srcAddr = (uint32_t)&LPC_ADC->ADDR1; 
    lli1[1].dstAddr = (uint32_t)&ldr1_raw_buffer_A[0];
    lli1[1].nextLLI = (uint32_t)&lli2[1];
    lli1[1].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
    //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 
    lli2[1].srcAddr = (uint32_t)&LPC_ADC->ADDR1; 
    lli2[1].dstAddr = (uint32_t)&ldr1_raw_buffer_B[0];
    lli2[1].nextLLI = (uint32_t)&lli1[1];
    lli2[1].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
     //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 

    
    //periferico a memoria
    dmaCfgAdc1.channelNum = GPDMA_CH_1;
    dmaCfgAdc1.transferSize = N_MUESTRAS; //16(valor de N_MUESTRAS) en total si transferimos de a 16bits
    dmaCfgAdc1.type = GPDMA_P2M;
    dmaCfgAdc1.srcMemAddr = (uint32_t)&LPC_ADC->ADDR1;//creo que no se usa  
    dmaCfgAdc1.dstMemAddr = (uint32_t)&ldr1_raw_buffer_B[0];//creo que se ignora esto pero por las dudas
    dmaCfgAdc1.srcConn = GPDMA_ADC;
    dmaCfgAdc1.dstConn = 0;
    dmaCfgAdc1.src.width = GPDMA_HALFWORD;
    dmaCfgAdc1.src.burst = GPDMA_BSIZE_1;
    dmaCfgAdc1.src.increment = DISABLE;
    dmaCfgAdc1.dst.width = GPDMA_HALFWORD;
    dmaCfgAdc1.dst.burst = GPDMA_BSIZE_1;
    dmaCfgAdc1.dst.increment = ENABLE;
    dmaCfgAdc1.intTC = ENABLE;
    dmaCfgAdc1.intErr = DISABLE;//VER SI ES NECESARIO
    dmaCfgAdc1.linkedList = &lli1[1]; //ver si queremos ubicar la lli en una direccion especifica
  
    GPDMA_SetupChannel(&dmaCfgAdc1);


    //---------------- CONFIGURACION DEL DMA PARA EL ADC2 ----------------
    GPDMA_Channel_CFG_T dmaCfgAdc2;
    

    lli1[2].srcAddr = (uint32_t)&LPC_ADC->ADDR2; 
    lli1[2].dstAddr = (uint32_t)&ldr2_raw_buffer_A[0];
    lli1[2].nextLLI = (uint32_t)&lli2[2];
    lli1[2].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
    //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 
    lli2[2].srcAddr = (uint32_t)&LPC_ADC->ADDR2; 
    lli2[2].dstAddr = (uint32_t)&ldr2_raw_buffer_B[0];
    lli2[2].nextLLI = (uint32_t)&lli1[2];
    lli2[2].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
     //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 

    
    //periferico a memoria
    dmaCfgAdc2.channelNum = GPDMA_CH_2;
    dmaCfgAdc2.transferSize = N_MUESTRAS; //16(valor de N_MUESTRAS) en total si transferimos de a 16bits
    dmaCfgAdc2.type = GPDMA_P2M;
    dmaCfgAdc2.srcMemAddr = (uint32_t)&LPC_ADC->ADDR2;//creo que no se usa  
    dmaCfgAdc2.dstMemAddr = (uint32_t)&ldr2_raw_buffer_B[0];//creo que se ignora esto pero por las dudas
    dmaCfgAdc2.srcConn = GPDMA_ADC;
    dmaCfgAdc2.dstConn = 0;
    dmaCfgAdc2.src.width = GPDMA_HALFWORD;
    dmaCfgAdc2.src.burst = GPDMA_BSIZE_1;
    dmaCfgAdc2.src.increment = DISABLE;
    dmaCfgAdc2.dst.width = GPDMA_HALFWORD;
    dmaCfgAdc2.dst.burst = GPDMA_BSIZE_1;
    dmaCfgAdc2.dst.increment = ENABLE;
    dmaCfgAdc2.intTC = ENABLE;
    dmaCfgAdc2.intErr = DISABLE;//VER SI ES NECESARIO
    dmaCfgAdc2.linkedList = &lli1[2]; //ver si queremos ubicar la lli en una direccion especifica

    GPDMA_SetupChannel(&dmaCfgAdc2);

    //---------------- CONFIGURACION DEL DMA PARA EL ADC6 ----------------
    GPDMA_Channel_CFG_T dmaCfgAdc6;
    

    lli1[3].srcAddr = (uint32_t)&LPC_ADC->ADDR6; 
    lli1[3].dstAddr = (uint32_t)&ldr3_raw_buffer_A[0];
    lli1[3].nextLLI = (uint32_t)&lli2[3];
    lli1[3].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
    //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 
    lli2[3].srcAddr = (uint32_t)&LPC_ADC->ADDR6; 
    lli2[3].dstAddr = (uint32_t)&ldr3_raw_buffer_B[0];
    lli2[3].nextLLI = (uint32_t)&lli1[3];
    lli2[3].control = (N_MUESTRAS)<<0 | (1 << 18) | (1 << 21) | (0 << 26)  | (1 << 27) | (1 << 31) ;
     //control :  transfer size: N_MUESTRAS (16), halfword src y dst, no incremento src, incremento dst, habilito interrupcion. 

    
    //periferico a memoria
    dmaCfgAdc6.channelNum = GPDMA_CH_3;
    dmaCfgAdc6.transferSize = N_MUESTRAS; //16(valor de N_MUESTRAS) en total si transferimos de a 16bits
    dmaCfgAdc6.type = GPDMA_P2M;
    dmaCfgAdc6.srcMemAddr = (uint32_t)&LPC_ADC->ADDR6;//creo que no se usa  
    dmaCfgAdc6.dstMemAddr = (uint32_t)&ldr3_raw_buffer_B[0];//creo que se ignora esto pero por las dudas
    dmaCfgAdc6.srcConn = GPDMA_ADC;
    dmaCfgAdc6.dstConn = 0;
    dmaCfgAdc6.src.width = GPDMA_HALFWORD;
    dmaCfgAdc6.src.burst = GPDMA_BSIZE_1;
    dmaCfgAdc6.src.increment = DISABLE;
    dmaCfgAdc6.dst.width = GPDMA_HALFWORD;
    dmaCfgAdc6.dst.burst = GPDMA_BSIZE_1;
    dmaCfgAdc6.dst.increment = ENABLE;
    dmaCfgAdc6.intTC = ENABLE;
    dmaCfgAdc6.intErr = DISABLE;//VER SI ES NECESARIO
    dmaCfgAdc6.linkedList = &lli1[3]; //ver si queremos ubicar la lli en una direccion especifica

    GPDMA_SetupChannel(&dmaCfgAdc6);




    GPDMA_ChannelStart(GPDMA_CH_0);
    GPDMA_ChannelStart(GPDMA_CH_1);
    GPDMA_ChannelStart(GPDMA_CH_2);
    GPDMA_ChannelStart(GPDMA_CH_3);
    NVIC_EnableIRQ(DMA_IRQn);
}

void DMA_IRQHandler(void) {
    //buffer A o B?
}