/*
 * I2S_Test.c
 *
 *  Created on: Mar 27, 2012
 *      Author: rkd_1
 */
#include <stm32f4_discovery_audio_codec.h>
#include "I2S_Test.h"

#define BLOCK_SIZE			32

uint32_t 	pdmcnt 		= 0; // pdm count
uint32_t	pcmcnt		= 0; // pcm count

uint16_t 	pdm[PDM_MAX];
uint16_t 	pcm[PCM_MAX];

#if defined NO_FILTER
uint16_t        pcmout[FRAME_MAX];

#elif defined FIR_FILTER
float32_t	fb[FRAME_MAX];
float32_t	fb2[FRAME_MAX];
uint16_t        f_out[FRAME_MAX];

float32_t firCoeffs32[] = {
        -0.02043962792897770200,
        -0.02634946212041115000,
        -0.02868235622890297600,
        -0.02626513613324441300,
        -0.01837885534222827000,
        -0.00490181632825100170,
        0.01361752034440888300,
        0.03597286034812688400,
        0.06040118665800180800,
        0.08476347186193369800,
        0.10678382855970783000,
        0.12431662600373122000,
        0.13560602358565163000,
        0.13950246462193622000,
        0.13560602358565163000,
        0.12431662600373122000,
        0.10678382855970783000,
        0.08476347186193369800,
        0.06040118665800180800,
        0.03597286034812688400,
        0.01361752034440888300,
        -0.00490181632825100170,
        -0.01837885534222827000,
        -0.02626513613324441300,
        -0.02868235622890297600,
        -0.02634946212041115000,
        -0.02043962792897770200,
        -0.01239099118102916600
};

enum {
	NUM_TAPS = sizeof(firCoeffs32)/sizeof(firCoeffs32[0])
};

static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FRAME_MAX/BLOCK_SIZE;

arm_fir_instance_f32 S;
#endif


extern PDMFilter_InitStruct 	PDMFilter;

uint16_t 	I2S_Data;

/*
 *	I2S Port 설정
 *	--------------------------
 *	PortB Pin10 -> CLOCK IN
 *	PortC Pin3  -> PDM OUT
 *	--------------------------
 */
void I2S_Port_Setting(GPIO_InitTypeDef *GPIO_InitStructure)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);

	GPIO_InitStructure->GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure->GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure->GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure->GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure->GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOB, GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);

	GPIO_InitStructure->GPIO_Pin = GPIO_Pin_3;

	GPIO_Init(GPIOC, GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
}

/*
 *	SPIx_I2S 설정
 *	--------------------------
 *	MODE		:	MasterRx
 *	STANDARD	:	Phillips
 *	DATAFORMAT	:	16b
 *	AUDIOFREQ	:	32k	// input I2S frequency , output I2S frequency : 8k
 *	CPOL		:	High
 *	MCLKOUTPUT	:	Disable
 *	--------------------------
 */
void SPI_I2S_Setting(SPI_TypeDef *SPIx, I2S_InitTypeDef *I2S_InitStructure)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*GPIO Setting */
	I2S_Port_Setting(&GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	SPI_I2S_DeInit(SPIx);

	I2S_InitStructure->I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure->I2S_Standard = I2S_Standard_Phillips;
	I2S_InitStructure->I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure->I2S_AudioFreq = 32000;
	I2S_InitStructure->I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure->I2S_MCLKOutput = I2S_MCLKOutput_Enable;

	/* I2S INITAILIZATION */
	I2S_Init(SPIx, I2S_InitStructure);
}

/*
 *	I2S_NVIC 설정
 *	--------------------------
 *	Interrupt Request : SPI2
 *	--------------------------
 */
void SPIx_NVIC_Setting(NVIC_InitTypeDef *NVIC_InitStructure)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	NVIC_InitStructure->NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure->NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure->NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure->NVIC_IRQChannelCmd = ENABLE;

	/* NVIC INITAILIZATION */
	NVIC_Init(NVIC_InitStructure);
}

/*
 *	I2S 설정
 *	--------------------
 *	I2S에 필요한 기본 설정들 모음
 *	--------------------
 */
void I2S_Configure(SPI_TypeDef *SPIx, I2S_InitTypeDef *I2S_InitStructure, NVIC_InitTypeDef *NVIC_InitStructure)
{
	extern uint32_t	startflg;

	/* I2S SETTING */
	SPI_I2S_Setting(SPIx, I2S_InitStructure);

	/* NVIC SETTING */
	SPIx_NVIC_Setting(NVIC_InitStructure);

	I2S_Cmd(SPIx, ENABLE);
        
#if defined FIR_FILTER
	arm_fir_init_f32(&S, NUM_TAPS, &firCoeffs32[0], &firStateF32[0], blockSize);
#endif
	startflg = 0;
}

void SPI2_IRQHandler(void)
{
	uint16_t 	micgain	= 50;
	int i;
	extern uint32_t	startflg;

	if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)!= RESET)
	{
		I2S_Data = SPI_I2S_ReceiveData(SPI2);
		pdm[pdmcnt++] = HTONS(I2S_Data);

		if(pdmcnt > PDM_MAX - 1)
		{
			PDM_Filter_64_LSB((uint8_t *)pdm, (uint16_t *)pcm, micgain, (PDMFilter_InitStruct *)&PDMFilter);
#if defined NO_FILTER
			for(i = 0 ; i < PCM_MAX ; i++)
			{
				pcmout[pcmcnt++] = pcm[i];
			}

			if(pcmcnt > FRAME_MAX - 1)
			{
				STM_EVAL_LEDToggle(LED3);
				startflg = 1;
				pcmcnt = 0;
			}
                        
#elif defined FIR_FILTER                       
                        for( i = 0 ; i < PCM_MAX ; i++)
                        {
                            fb[pcmcnt++] = pcm[i];
                        }
                       
                        if(pcmcnt > FRAME_MAX - 1)
			{                                
                                for( i = 0 ; i < numBlocks ; i++)
                                {
                                    arm_fir_f32(&S, (float32_t *)fb + (i * blockSize) , (float32_t *)fb2 + (i * blockSize) , blockSize); // fir filter
                                } 
                             
                                for( i = 0 ; i < FRAME_MAX ; i++)
                                {
                                    f_out[i] = (uint16_t)fb2[i];
                                }
                                
				STM_EVAL_LEDToggle(LED3);
				startflg = 1;
				pcmcnt = 0;
			}
#endif
			pdmcnt = 0;
		}
	}
}



