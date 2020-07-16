/*********************************************************************************************************//**
 * @file    I2C/TouchKey/main.c
 * @version $Rev:: 2959         $
 * @date    $Date:: 2018-08-02 #$
 * @brief   Main program.
 *************************************************************************************************************
 * @attention
 *
 * Firmware Disclaimer Information
 *
 * 1. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, which is supplied by Holtek Semiconductor Inc., (hereinafter referred to as "HOLTEK") is the
 *    proprietary and confidential intellectual property of HOLTEK, and is protected by copyright law and
 *    other intellectual property laws.
 *
 * 2. The customer hereby acknowledges and agrees that the program technical documentation, including the
 *    code, is confidential information belonging to HOLTEK, and must not be disclosed to any third parties
 *    other than HOLTEK and the customer.
 *
 * 3. The program technical documentation, including the code, is provided "as is" and for customer reference
 *    only. After delivery by HOLTEK, the customer shall use the program technical documentation, including
 *    the code, at their own risk. HOLTEK disclaims any expressed, implied or statutory warranties, including
 *    the warranties of merchantability, satisfactory quality and fitness for a particular purpose.
 *
 * <h2><center>Copyright (C) Holtek Semiconductor Inc. All rights reserved</center></h2>
 ************************************************************************************************************/
// <<< Use Configuration Wizard in Context Menu >>>

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32.h"
#include "ht32_board.h"
#include "ht32_board_config.h"

/** @addtogroup HT32_Series_Peripheral_Examples HT32 Peripheral Examples
  * @{
  */

/** @addtogroup I2C_Examples I2C
  * @{
  */

/** @addtogroup TouchKey
  * @{
  */


/* Private types -------------------------------------------------------------------------------------------*/
typedef union {
	struct {
		unsigned long Key1     :1;
		unsigned long Key2     :1;
		unsigned long Key3     :1;
		unsigned long Key4     :1;
		unsigned long Key5     :1;
		unsigned long Key6     :1;
		unsigned long Key7     :1;
		unsigned long Key8     :1;
		unsigned long Key9     :1;
		unsigned long Key10    :1;
		unsigned long Key11    :1;
		unsigned long Key12    :1;
		unsigned long Key13    :1;
		unsigned long Key14    :1;
		unsigned long Key15    :1;
		unsigned long Key16    :1;
	} Bit;
	u16 Data;
} TouchKey_TypeDef;

typedef enum {
	zoom = 3,
	slide = 2,
	press = 1,
	none = 0
} TOUCHKEY_STATUS;


TM_TimeBaseInitTypeDef TimeBaseInit;
TOUCHKEY_STATUS status = none;
TouchKey_TypeDef Touch;

bool TK_CHECK = FALSE, TK_1SEC = FALSE;
u8 TK_L = 0, TK_R = 0;
u16 TK_COUNT = 0;


/* Private constants ---------------------------------------------------------------------------------------*/
#define I2C_TOUCHKEY_SPEED         (50000)          /*!< I2C speed                                          */
#define I2C_TOUCHKEY_DEV_ADDR      (0x50)           /*!< I2C device address                                 */
#define KEYSTATUS_CMD              (0x08)

/* Private function prototypes -----------------------------------------------------------------------------*/
void NVIC_Configuration(void);
void CKCU_Configuration(void);
void GPTM0_Configuration(void);
void GPTM0_IRQHandler(void);
void GPIO_Configuration(void);
void I2C_Configuration(void);
u32 Touchkey_ButtonRead(void);
void _I2C_Touchkey_AckPolling(void);
void get_TKLR(void);

/* Global functions ----------------------------------------------------------------------------------------*/
/*********************************************************************************************************//**
  * @brief  Main program.
  * @retval None
  ***********************************************************************************************************/
int main(void) {
	u32 uCounter;
	u8 slideValue = 0;
	u8 zoomValue = 0;

	NVIC_Configuration();               /* NVIC configuration                                                 */
	CKCU_Configuration();               /* System Related configuration                                       */
	GPTM0_Configuration();
	GPIO_Configuration();               /* GPIO Related configuration                                         */
	RETARGET_Configuration();           /* Retarget Related configuration                                     */

	I2C_Configuration();                /* I2C configuration                                                  */

	printf("\r\nTouchkey test start....\r\n");

	while (1) {
		if (!GPIO_ReadInBit(HT_GPIOC, GPIO_PIN_0)) {
			if (TK_1SEC == TRUE) TK_CHECK = TRUE;
			else TK_CHECK = FALSE;
			Touch.Data = Touchkey_ButtonRead();
			get_TKLR();
			printf("\r%d, PADS: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d, DATA=%04x, slide=%3d, zoom=%3d TK_CHECK = %2d, TK_1SEC = %2d",
				status,
				Touch.Bit.Key1,  Touch.Bit.Key2,  Touch.Bit.Key3,  Touch.Bit.Key4,
				Touch.Bit.Key5,  Touch.Bit.Key6,  Touch.Bit.Key7,  Touch.Bit.Key8,
				Touch.Bit.Key9,  Touch.Bit.Key10, Touch.Bit.Key11, Touch.Bit.Key12,
				Touch.Bit.Key13, Touch.Bit.Key14, Touch.Bit.Key15, Touch.Bit.Key16,
				Touch.Data,
				slideValue,
				zoomValue,
				TK_CHECK,
				TK_1SEC
			);
			uCounter = (HSE_VALUE >> 4);
			while (uCounter--);
		} else {
			TK_CHECK = FALSE;
			TK_1SEC = TRUE;
			TK_COUNT = 0;
			status = none;
		}
	}
}

/*********************************************************************************************************//**
  * @brief  Configure the NVIC vector table.
  * @retval None
  ***********************************************************************************************************/
u32 Touchkey_ButtonRead(void) {
	u32 uData;

	/* Touchkey addressread                                                                                   */
	_I2C_Touchkey_AckPolling();

	while (!I2C_CheckStatus(HT_I2C1, I2C_MASTER_TX_EMPTY));
	I2C_SendData(HT_I2C1, KEYSTATUS_CMD);

	/* Touchkey addressread                                                                                   */
	I2C_TargetAddressConfig(HT_I2C1, I2C_TOUCHKEY_DEV_ADDR, I2C_MASTER_READ);
	while (!I2C_CheckStatus(HT_I2C1, I2C_MASTER_RECEIVER_MODE));

	/* enable master receiver ACK                                                                             */
	I2C_AckCmd(HT_I2C1, ENABLE);

	/* sequential read                                                                                        */
	while (!I2C_CheckStatus(HT_I2C1, I2C_MASTER_RX_NOT_EMPTY));
	uData = I2C_ReceiveData(HT_I2C1);
	I2C_AckCmd(HT_I2C1, DISABLE);
	while (!I2C_CheckStatus(HT_I2C1, I2C_MASTER_RX_NOT_EMPTY));
	uData |= (I2C_ReceiveData(HT_I2C1) << 8);

	/* end of read                                                                                            */
	I2C_GenerateSTOP(HT_I2C1);

	return uData;
}

/*********************************************************************************************************//**
  * @brief  EEPROM acknowledge polling.
  * @retval None
  ***********************************************************************************************************/
void _I2C_Touchkey_AckPolling(void) {
	u32 reg;
	/* wait if bus busy                                                                                       */
	while (I2C_GetFlagStatus(HT_I2C1, I2C_FLAG_BUSBUSY));

	/* send slave address until ack reply                                                                     */
	while (1) {
		/* send slave address                                                                                   */
		I2C_TargetAddressConfig(HT_I2C1, I2C_TOUCHKEY_DEV_ADDR, I2C_MASTER_WRITE);

		while (1) {
			reg = HT_I2C1->SR;
			if (reg & I2C_FLAG_ADRS) return;
			if (reg & I2C_FLAG_RXNACK) {
				I2C_ClearFlag(HT_I2C1, I2C_FLAG_RXNACK);
				break;
			}
		}
	}
}

/*********************************************************************************************************//**
  * @brief  Configure the NVIC vector table.
  * @retval None
  ***********************************************************************************************************/
void NVIC_Configuration(void) {
	NVIC_SetVectorTable(NVIC_VECTTABLE_FLASH, 0x0);     /* Set the Vector Table base location at 0x00000000   */
	NVIC_EnableIRQ(GPTM0_IRQn);
}

/*********************************************************************************************************//**
  * @brief  Configure the system clocks.
  * @retval None
  ***********************************************************************************************************/
void CKCU_Configuration(void) {
#if 1
	CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
	CKCUClock.Bit.PA         = 1;
	CKCUClock.Bit.PB         = 1;
	CKCUClock.Bit.PC         = 1;
	CKCUClock.Bit.PD         = 1;
	CKCUClock.Bit.I2C1       = 1;
	CKCUClock.Bit.USART0     = 1;
	CKCUClock.Bit.USART1     = 1;
	CKCUClock.Bit.UART0      = 1;
	CKCUClock.Bit.UART1      = 1;
	CKCUClock.Bit.AFIO       = 1;
	CKCUClock.Bit.GPTM0      = 1;
	CKCUClock.Bit.GPTM1      = 1;
	CKCU_PeripClockConfig(CKCUClock, ENABLE);
#endif

#if (ENABLE_CKOUT == 1)
	CKOUTConfig();
#endif
}

#if (ENABLE_CKOUT == 1)
/*********************************************************************************************************//**
  * @brief  Configure the debug output clock.
  * @retval None
  ***********************************************************************************************************/
void CKOUTConfig(void)
{
  CKCU_CKOUTInitTypeDef CKOUTInit;

  AFIO_GPxConfig(GPIO_PA, AFIO_PIN_9, AFIO_MODE_15);
  CKOUTInit.CKOUTSRC = CKCU_CKOUTSRC_HCLK_DIV16;
  CKCU_CKOUTConfig(&CKOUTInit);
}
#endif

void GPTM0_Configuration(void) {
	TM_TimeBaseStructInit(&TimeBaseInit);                       // Init GPTM1 time-base
	TimeBaseInit.CounterMode = TM_CNT_MODE_UP;                  // up count mode
	TimeBaseInit.CounterReload = 36000;            				// interrupt in every 500us
	TimeBaseInit.Prescaler = 0;
	TimeBaseInit.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;          // reload immediately
	TM_TimeBaseInit(HT_GPTM0, &TimeBaseInit);                   // write the parameters into GPTM1
	TM_ClearFlag(HT_GPTM0, TM_FLAG_UEV);                        // Clear Update Event Interrupt flag
	TM_IntConfig(HT_GPTM0, TM_INT_UEV, ENABLE);                 // interrupt by GPTM update
	TM_Cmd(HT_GPTM0, ENABLE);                                   // enable the counter 1
}

/*********************************************************************************************************//**
  * @brief  Configure the GPIO ports.
  * @retval None
  ***********************************************************************************************************/
void GPIO_Configuration(void) {
	
	GPIO_DirectionConfig(HT_GPIOC, GPIO_PIN_0,  GPIO_DIR_IN);
	GPIO_InputConfig(HT_GPIOC, GPIO_PIN_0, ENABLE);
	
	#if (RETARGET_PORT == RETARGET_USART0)
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_2 | AFIO_PIN_3, AFIO_FUN_USART_UART);
	#endif

	#if (RETARGET_PORT == RETARGET_USART1)
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
	#endif

	#if (RETARGET_PORT == RETARGET_UART0)
	AFIO_GPxConfig(GPIO_PC, AFIO_PIN_4 | AFIO_PIN_5, AFIO_FUN_USART_UART);
	#endif

	#if (RETARGET_PORT == RETARGET_UART1)
	AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1 | AFIO_PIN_3, AFIO_FUN_USART_UART);
	#endif
}

/*********************************************************************************************************//**
  * @brief  Configure the I2C.
  * @retval None
  ***********************************************************************************************************/
void I2C_Configuration(void) {
	I2C_InitTypeDef I2C_InitStructure = { 0 };

	/* Configure I2C SCL pin, I2C SDA pin                                                                     */
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_0, AFIO_FUN_I2C);
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_1, AFIO_FUN_I2C);

	/* I2C configuration                                                                                      */
	I2C_InitStructure.I2C_GeneralCall = I2C_GENERALCALL_DISABLE;
	I2C_InitStructure.I2C_AddressingMode = I2C_ADDRESSING_7BIT;
	I2C_InitStructure.I2C_Acknowledge = I2C_ACK_DISABLE;
	I2C_InitStructure.I2C_OwnAddress = 0x00;
	I2C_InitStructure.I2C_Speed = I2C_TOUCHKEY_SPEED;
	I2C_InitStructure.I2C_SpeedOffset = 0;
	I2C_Init(HT_I2C1, &I2C_InitStructure);
	I2C_Cmd(HT_I2C1, ENABLE);
}

void get_TKLR(void) {
	int index, check = 0x0001;
	
	for (index = 0; index < 16; index++) {
		if ((Touch.Data & check) == check) {
			TK_L = index;
			break;
		} else {
			check <<= 1;
		}
	}
	check = 0x8000;
	for (index = 15; index >= 0; index--) {
		if ((Touch.Data & check) == check) {
			TK_R = index;
			break;
		} else {
			check >>= 1;
		}
	}
}

void GPTM0_IRQHandler(void) {
	TM_ClearFlag(HT_GPTM0, TM_FLAG_UEV);
	
	if (TK_CHECK) {
		if (TK_R - TK_L > 2) {
			status = zoom;
			TK_1SEC = FALSE;
		} else if (TK_R - TK_L <= 2) {
			status = slide;
		}
		
	}
	if (TK_1SEC && status != none) {
		if (TK_COUNT >= (2 * 1000)) {
			TK_1SEC = FALSE;
		} else {
			TK_COUNT += 1;
		}
	}
}

#if (HT32_LIB_DEBUG == 1)
/*********************************************************************************************************//**
  * @brief  Report both the error name of the source file and the source line number.
  * @param  filename: pointer to the source file name.
  * @param  uline: error line source number.
  * @retval None
  ***********************************************************************************************************/
void assert_error(u8* filename, u32 uline) {
	/*
	 This function is called by IP library that the invalid parameters has been passed to the library API.
	 Debug message can be added here.
	 Example: printf("Parameter Error: file %s on line %d\r\n", filename, uline);
	*/

	while (1) {
	}
}
#endif


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
