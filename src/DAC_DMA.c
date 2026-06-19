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

#include <cr_section_macros.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_gpdma.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

#define N 64

// Tabla de 64 muestras para una onda senoidal en el DAC de 10 bits (con corrimiento de 6 bits)
uint32_t ondaSenoidal[N] = {
		32768, 35968, 39104, 42240, 45248, 48192, 51008, 53632,
		55872, 58048, 59968, 61696, 63104, 64256, 65088, 65408,
		65472, 65408, 65088, 64256, 63104, 61696, 59968, 58048,
		55872, 53632, 51008, 48192, 45248, 42240, 39104, 35968,
		32768, 29568, 26432, 23296, 20288, 17344, 14528, 11904,
		9664,  7488,  5568,  3840,  2432,  1280,  448,   128,
		64,    128,   448,   1280,  2432,  3840,  5568,  7488,
		9664,  11904, 14528, 17344, 20288, 23296, 26432, 29568
};
/*
uint32_t ondaSenoidal[N] = {
    512,  562,  611,  660,  707,  753,  797,  838,
    873,  907,  937,  964,  986, 1004, 1017, 1022,
   1023, 1022, 1017, 1004,  986,  964,  937,  907,
    873,  838,  797,  753,  707,  660,  611,  562,
    512,  462,  413,  364,  317,  271,  227,  186,
    151,  117,   87,   60,   38,   20,    7,    2,
      1,    2,    7,   20,   38,   60,   87,  117,
    151,  186,  227,  271,  317,  364,  413,  462
};*/

void configPto(void);
void configDAC(void);
void configDMA(void);

int main(void) {

	configPto();
	configDAC();
	configDMA();

	while(1){
		__WFI();
	}

    return 0 ;
}

void configPto(void){

	PINSEL_CFG_Type cfgPto;

	cfgPto.Portnum = PINSEL_PORT_0;
	cfgPto.Pinnum = PINSEL_PIN_26;
	cfgPto.Funcnum = PINSEL_FUNC_2;
	cfgPto.Pinmode = PINSEL_PINMODE_TRISTATE;
	cfgPto.OpenDrain = PINSEL_PINMODE_NORMAL;

	PINSEL_ConfigPin(&cfgPto);
}

void configDAC(void){

	DAC_CONVERTER_CFG_Type cfgDAC;

	cfgDAC.DBLBUF_ENA = ENABLE;
	cfgDAC.CNT_ENA = ENABLE;
	cfgDAC.DMA_ENA = ENABLE;

	DAC_Init(LPC_DAC);
	DAC_SetDMATimeOut(LPC_DAC,391);
	DAC_ConfigDAConverterControl(LPC_DAC,&cfgDAC);
	//Sabiendo que el DAC cuenta "ticks" a 25 MHz y quiero la onda con frecuencia 1 KHz,
	//quiero 1000 ciclos por segundo (sabiendo que la onda está "partida" en 64 valores). Entonces -> 1000 ciclos/segundo*64 valores = 64000 pedidos por segundo
	//Entonces tiene que contar X ticks para pedir un dato nuevo -> Ticks(PCLK)/Transferencias = 25MHz/64000 = 390.625 y redondeo
}

void configDMA(void){

	GPDMA_Init();

	GPDMA_Channel_CFG_Type cfgDMA;
	static GPDMA_LLI_Type LLI;

	LLI.SrcAddr = (uint32_t) ondaSenoidal;
	LLI.DstAddr = (uint32_t) &(LPC_DAC -> DACR);
	LLI.NextLLI = (uint32_t) &LLI;
	LLI.Control = ((N << 0) | (2 << 18) | (2 << 21) | (1 << 26));
	LLI.Control &= ~(1 << 27);

	cfgDMA.ChannelNum = 7;
	cfgDMA.SrcMemAddr = (uint32_t) ondaSenoidal;
	cfgDMA.DstMemAddr = (uint32_t) &(LPC_DAC -> DACR);
	cfgDMA.TransferSize = N;
	cfgDMA.TransferType = GPDMA_TRANSFERTYPE_M2P;
	cfgDMA.SrcConn = 0;
	cfgDMA.DstConn = GPDMA_CONN_DAC;
	cfgDMA.DMALLI = (uint32_t) &LLI;

	GPDMA_Setup(&cfgDMA);
	GPDMA_ChannelCmd(7,ENABLE);
}
