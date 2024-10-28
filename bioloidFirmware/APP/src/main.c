/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : Main program body
*******************************************************************************/
/*
Attribution: Part of this code is based on
https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/#dynamixel-sync-control
and
https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/#serial-communication
*/

/* Includes ------------------------------------------------------------------*/

//#include "includes.h"
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_GOAL_SPEED_L			32
#define P_GOAL_SPEED_H			33

#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_LIMIT_CW				6
#define P_LIMIT_CCW				8


#define DEFAULT_BAUDNUM		    1 // 1Mbps
#define CONTROL_PERIOD		    (1000) // msec (Large value is more slow)


#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB


#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11

#define USART_DXL			    0
#define USART_PC			    2

#define word                    u16
#define byte                    u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte                   gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte                   gbRxBufferWritePointer,gbRxBufferReadPointer;
volatile vu32                   gwTimingDelay,gw1msCounter;
u32                             Baudrate_DXL = 	1000000;
u32                             Baudrate_PC = 57600;
vu16                            CCR1_Val = 100; 		// 1ms
vu32                            capture = 0;



byte                            i;
byte 							motor_ids[6] = {1, 2, 3, 4, 5, 6}; // Motor IDs
byte                            CommStatus;


volatile byte 					gbPacketWritePointer;
volatile byte 					gbPacketReadPointer;
volatile byte 					gbpPacketDataBuffer[16+1+16];


word ShoulderPitches1[2] = {200,800}; // Limit for right Shoulder Pitch, Motor ID = 1
word ShoulderRolls3[2] = {250,500}; // Limit for right Shoulder Roll,  Motor ID : 3
word EllbowRolls5[2] = {200,500}; // Limit for right Ellbow Roll, Motor ID : 5

word ShoulderPitches2[2] = {200,800}; // Limit for left Shoulder Pitch, Motor ID : 2
word ShoulderRolls4[2] = {500,750}; // Limit for left Shoulder Roll, Motor ID : 4
word EllbowRolls6[2] = {500,800}; // Limit for left Ellbow Roll, Motor ID : 6

word                            bMoving1,bMoving2, bMoving3,bMoving4, bMoving5, bMoving6;
word							postion1,postion2,postion3,postion4,postion5,postion6;

byte ReceivedData[12]; //  Storage for the received bytes


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(u32);
void USART_Configuration(u8, u32);
void SysTick_Configuration(void);
void Timer_Configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void DisableUSART1(void);
void ClearBuffer256(void);
void CheckNewarrive(void);
void TxDByte_DXL(byte);
byte RxDByte_DXL(void);
byte RxDByte_PC(void);
void TxDByte_PC(byte);
void mDelay(u32);
void StartDiscount(s32);
byte CheckTimeOut(void);
void ControlMotor(byte id, byte position_low, byte position_high, byte var);
void MotorLimits(void);
int prufenAnfang(byte c[], byte wert1, byte wert2);
int prufenEnde(byte c[], byte wert15, byte wert16);
void MotorenBewegen(void);

int main(void)
{

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	SysTick_Configuration();
	Timer_Configuration();

	dxl_initialize( 0, 1 );
	USART_Configuration(1, 57600);

	dxl_write_word(1,P_LIMIT_CW,ShoulderPitches1[0]);
	dxl_write_word(1,P_LIMIT_CCW,ShoulderPitches1[1]);

	dxl_write_word(2,P_LIMIT_CW,ShoulderPitches2[0]);
	dxl_write_word(2,P_LIMIT_CCW,ShoulderPitches2[1]);

	dxl_write_word(3,P_LIMIT_CW,ShoulderRolls3[0]);
	dxl_write_word(3,P_LIMIT_CCW,ShoulderRolls3[1]);

	dxl_write_word(4,P_LIMIT_CW,ShoulderRolls4[0]);
	dxl_write_word(4,P_LIMIT_CCW,ShoulderRolls4[1]);

	dxl_write_word(5,P_LIMIT_CW,EllbowRolls5[0]);
	dxl_write_word(5,P_LIMIT_CCW,EllbowRolls5[1]);

	dxl_write_word(6,P_LIMIT_CW,EllbowRolls6[0]);
	dxl_write_word(6,P_LIMIT_CCW,EllbowRolls6[1]);

	while(1)
	{
	     for (i = 0; i < 12; i++) {
	            ReceivedData[i] = RxDByte_PC();
	        }
	     MotorenBewegen();
	}
	return 0;
}

// this main fuction is used including the motorLimits function
/*
int main(void)
{

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	SysTick_Configuration();
	Timer_Configuration();

	dxl_initialize( 0, 1 );
	USART_Configuration(1, 57600);

	dxl_write_word(1,P_LIMIT_CW,ShoulderPitches1[0]);
	dxl_write_word(1,P_LIMIT_CCW,ShoulderPitches1[1]);

	dxl_write_word(2,P_LIMIT_CW,ShoulderPitches2[0]);
	dxl_write_word(2,P_LIMIT_CCW,ShoulderPitches2[1]);

	dxl_write_word(3,P_LIMIT_CW,ShoulderRolls3[0]);
	dxl_write_word(3,P_LIMIT_CCW,ShoulderRolls3[1]);

	dxl_write_word(4,P_LIMIT_CW,ShoulderRolls4[0]);
	dxl_write_word(4,P_LIMIT_CCW,ShoulderRolls4[1]);

	dxl_write_word(5,P_LIMIT_CW,EllbowRolls5[0]);
	dxl_write_word(5,P_LIMIT_CCW,EllbowRolls5[1]);

	dxl_write_word(6,P_LIMIT_CW,EllbowRolls6[0]);
	dxl_write_word(6,P_LIMIT_CCW,EllbowRolls6[1]);

	while(1)
	{
	     for (i = 0; i < 12; i++) {
	            ReceivedData[i] = RxDByte_PC();
	        }

	     // Bei diesem Teil handelt es sich um eine Test phase der Motor Limits
    	if(!prufenAnfang(ReceivedData,0xff,0xff)&& !prufenEnde(ReceivedData,0xee,0xee)){

    		dxl_write_word( 1, P_GOAL_POSITION_L, 500 );
    		dxl_write_word( 2, P_GOAL_POSITION_L, 500 );
    		dxl_write_word( 3, P_GOAL_POSITION_L, 500 );
    		dxl_write_word( 4, P_GOAL_POSITION_L, 500 );
    		dxl_write_word( 5, P_GOAL_POSITION_L, 500 );
    		dxl_write_word( 6, P_GOAL_POSITION_L, 500 );

    	}
    	else {
    		MotorLimits();
    		MotorenBewegen();
    	}
	}

	return 0;
} */


void ControlMotor(byte id, byte position_low, byte position_high, byte var) {
	// Motor auf Bewegung setzen
	dxl_set_txpacket_parameter(2+3*var,id);
	dxl_set_txpacket_parameter(2+3*var+1, position_low);
	dxl_set_txpacket_parameter(2+3*var+2, position_high);
}

void MotorenBewegen(void){
	// Make syncwrite packet
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2);

      for(i = 0; i < 6; i++)
      {
		ControlMotor(motor_ids[i],ReceivedData[(i*2)],ReceivedData[(i*2+1)],i);
      }
      dxl_set_txpacket_length(22);
      dxl_txrx_packet();
}


// this function can be used to check start byte.(In case using the MotorLimits function)
int prufenAnfang(byte c[], byte wert1, byte wert2){

	return(c[0] == wert1 && c[1] == wert2);
}
// this function can be used to check stop byte.(In case using the MotorLimits function)
int prufenEnde(byte c[], byte wert15, byte wert16){

	return(c[14] == wert15 && c[15] == wert16);

}
// function to check and update motors limits
void MotorLimits(void){



	postion1 = (ReceivedData[2] << 8) | ReceivedData[3];
	postion2 = (ReceivedData[4] << 8) | ReceivedData[5];
	postion3 = (ReceivedData[6] << 8) | ReceivedData[7];
	postion4 = (ReceivedData[8] << 8) | ReceivedData[9];
	postion5 = (ReceivedData[10] << 8) | ReceivedData[11];
	postion6 = (ReceivedData[12] << 8) | ReceivedData[13];


	bMoving1 = dxl_read_word( 1, P_PRESENT_POSITION_L );
	bMoving2 = dxl_read_word( 2, P_PRESENT_POSITION_L );
	bMoving3 = dxl_read_word( 3, P_PRESENT_POSITION_L );
	bMoving4 = dxl_read_word( 4, P_PRESENT_POSITION_L );
	bMoving5 = dxl_read_word( 5, P_PRESENT_POSITION_L );
	bMoving6 = dxl_read_word( 6, P_PRESENT_POSITION_L );


	if((bMoving1 > 350 ) || (postion1 > 350)){
		dxl_write_word(5,P_LIMIT_CW,200);
		dxl_write_word(5,P_LIMIT_CCW,500);
	}

	else if ((bMoving1 < 350 && bMoving3 < 400) || (postion1 < 350 && postion3 < 400)){

		dxl_write_word(5,P_LIMIT_CW,490);
		dxl_write_word(5,P_LIMIT_CCW,499);

	}
	else if ((bMoving3 < 400 && bMoving5 > 300) || (postion3 < 400 && postion5 > 300)){

		dxl_write_word(1,P_LIMIT_CW,400);
		dxl_write_word(1,P_LIMIT_CCW,800);

	}

	/////////////////////////////////////////////////////

	if(bMoving2 <= 650){

		dxl_write_word(6,P_LIMIT_CW,500);
		dxl_write_word(6,P_LIMIT_CCW,800);

	}

	else if (bMoving2 > 650 && bMoving4 > 600){


		dxl_write_word(6,P_LIMIT_CW,501);
		dxl_write_word(6,P_LIMIT_CCW,510);

	}

	else if (bMoving4 <= 600){

		dxl_write_word(6,P_LIMIT_CW,500);
		dxl_write_word(6,P_LIMIT_CCW,800);

	}

	else if (bMoving2 < 400 && bMoving6 < 400){

		dxl_write_word(2,P_LIMIT_CW,200);
		dxl_write_word(2,P_LIMIT_CCW,600);

	}

}

// Configuration functions (modificated functions are marked with a comment)

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable USART1 and GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);

	/* Enable Timer2 and USART3 clocks */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART3 , ENABLE); // modification: RCC_APB1Periph_USART3 added

	PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
		// Set the Vector Table base location at 0x20000000
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
		// Set the Vector Table base location at 0x08003000
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
	#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// modification: USART3 NVIC_Configuration added
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_TXD | PIN_ENABLE_RXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

void USART1_Configuration(u32 baudrate)
{
	USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{

	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if( PORT == USART_DXL )
	{
		USART_DeInit(USART1);
		mDelay(10);
		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 Receive and Transmit interrupts */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
	}

	else if( PORT == USART_PC )
	{

		USART_DeInit(USART3);
		mDelay(10);
		/* Configure the USART3 */
		USART_Init(USART3, &USART_InitStructure);

		/* Enable USART3 Receive and Transmit interrupts */
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			// modification: USART3 Receive enabled
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);

		/* Enable the USART3 */
		USART_Cmd(USART3, ENABLE);
	}
}

void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void SysTick_Configuration(void)
{
	/* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	SysTick_SetReload(9000);

	/* Enable SysTick interrupt */
	SysTick_ITConfig(ENABLE);
}

// functions for receivung Data from USART3

// this function ist taken from https://emanual.robotis.com/docs/en/software/embedded_sdk/embedded_c_cm530/#serial-communication
void RxD1Interrupt(void)
{
	byte temp;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		temp = USART_ReceiveData(USART3);
		gbpPacketDataBuffer[gbPacketWritePointer] = temp;
		gbPacketWritePointer++;
		gbPacketWritePointer = gbPacketWritePointer & 0x1F;
	}
}

byte RxDByte_PC(void)
{
    byte temp;

	while(1)
	{
    	if(gbPacketReadPointer != gbPacketWritePointer) break;
	}

	temp = gbpPacketDataBuffer[gbPacketReadPointer];
	gbPacketReadPointer++;
	gbPacketReadPointer = gbPacketReadPointer & 0x1F;

	return temp;
}

// important function that are used in the .h Files
void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_Val);

		if(gw1msCounter > 0)
			gw1msCounter--;
	}
}

void __ISR_DELAY(void)
{
	if (gwTimingDelay != 0x00)
		gwTimingDelay--;
}

void mDelay(u32 nTime)
{
	/* Enable the SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Enable);

	gwTimingDelay = nTime;

	while(gwTimingDelay != 0);

	/* Disable SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Disable);
	/* Clear SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Clear);
}

void DisableUSART1(void)
{
	USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void)
{
	gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void)
{
	if(gbRxBufferReadPointer != gbRxBufferWritePointer)
		return 1;
	else
		return 0;
}

void TxDByte_DXL(byte bTxdData)
{
	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

	USART_SendData(USART1,bTxdData);
	while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}


byte RxDByte_DXL(void)
{
	byte bTemp;

	while(1)
	{
		if(gbRxBufferReadPointer != gbRxBufferWritePointer) break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
	gbRxBufferReadPointer++;

	return bTemp;
}


void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3,bTxdData);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}


void RxD0Interrupt(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	    gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}

void StartDiscount(s32 StartTime)
{
	gw1msCounter = StartTime;
}

u8 CheckTimeOut(void)
{
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	if(gw1msCounter == 0)
	      return 1;
	else
		return 0;
}


