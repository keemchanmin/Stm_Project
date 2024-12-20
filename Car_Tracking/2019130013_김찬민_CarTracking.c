/////////////////////////////////////////////////////////////
// PR1: Car Tracking system
// 제출자: 2019130013 김찬민
// 주요 내용
// -거리센서: ADC3 으로 2개의 가변저항 값 입력  by Scan Mode( DMA ) 
// - 모터 : TIM4_CH2 -> PWM ( LED 제어) 
// - 핸들 : TIM14_CH1 -> PWM (Buzzer 제어) 
// - 명령 : 1) UART 통신으로 'S' or 'M' 명령 수신 
//	   2) EXTI12 or EXTI14 를 통해 'S' or 'M' 명령
//
// 특이사항 :
// 	- 'S' 일 때 400ms 마다 0m 가 UART 로 송신 되도록 구현 
/////////////////////////////////////////////////////////////

#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"


void RunMenu(void); // 초기 LCD 초기화 함수
void _ADC_Init(void);	// ADC init
void DMAInit(void);	//DMA init
void TIMER1_OC_Init(void); //TIM1 init
void TIMER4_Init(void);		//TIM4 init
void TIMER14_PWM_Init(void);	//TIM14 init
void clear_rect(void);		//막대 초기화 함수
void _GPIO_Init(void);		//GPIO init
void _EXTI_Init(void);		// EXTI init
void USART1_Init(void);		//USART1 init
void USART_BRR_Configuration(uint32_t USART_BaudRate);	//USART1 BRR 설정 
void SerialSendChar(uint8_t c);	//1바이트 송신 함수
void SerialSendString(char *s); //문자열 송신 함수
void DelayMS(unsigned short wMS);// 딜레이 함수
void DelayUS(unsigned short wUS);// 딜레이 함수
void Move_Act(void);		// 'M' 상태일 때 동작 함수
void Stop_Act(void);		// 'S' 상태일 때 동작 함수 



uint16_t ADC_value[2];	// DMA 변수 
char str[20],str_duty[20]; // 더미 문자열 변수 
unsigned char fram_state='S'; // 명령 저장 변수
int distance2Car,distance2Ped; //선도차, 인도와의 거리 변수 
int st_flag=1;			// 'M' or 'S' 상태 flag 변수 


int main(void)
{
	LCD_Init();      // LCD 구동 함수
	DelayMS(10);      // LCD구동 딜레이
	
	TIMER1_OC_Init(); //TIM1 init
	_ADC_Init();		//ADC init
	DMAInit();		//DMA init
	TIMER4_Init();	//TIM4 init
	TIMER14_PWM_Init();	//TIM14 init
	USART1_Init();	//USART1 init
	_GPIO_Init(); 	//GPIO init
	_EXTI_Init();		//EXTI init
	
	Fram_Init();                    // FRAM 초기화 H/W 초기화
	Fram_Status_Config();           // FRAM 초기화 S/W 초기화  
	RunMenu();			  // 초기 LCD 디스플레이 초기화 
	DelayMS(50);      		  // 초기 LCD 디스플레이 딜레이 	
	fram_state = Fram_Read(1201);	  // Fram 1201주소 데이터 Read 
	
	if(fram_state=='S')		// 'S' 상태 일 때 
		Stop_Act(); 
	
	else if(fram_state=='M')	// 'M' 상태 일 때
		Move_Act();
	
	
	while(1)
	{
		
	}
}


void clear_rect(void)           // 막대 초기화 함수 
{
	LCD_SetBrushColor(RGB_WHITE);
	LCD_DrawFillRect(27,28,102,6);
	LCD_DrawFillRect(27,42,102,6);
}
void Move_Act(void)		// 'M' 상태 일 때 동작 함수 
{	
	Fram_Write(1201, 'M');	// 1201 번지 'M' Write
	LCD_SetTextColor(RGB_BLUE);
	st_flag=1;			// 명령 flag Set 
	LCD_DisplayChar(1,18,'M');	//'M' Display
	
}
void Stop_Act(void)		// 'S' 상태 일 때 동작 함수
{
	Fram_Write(1201, 'S');	// 1201 번지 'S' Write
	LCD_SetTextColor(RGB_BLUE);
	LCD_DisplayText(4,7,"00");
	st_flag=0;			// 명령 flag ReSet 
	LCD_DisplayChar(1,18,'S');	//'S' Display
	LCD_DisplayChar(4,18,'F');	//DIR='F'
	
	/***********TIM4 AND TIM14 CNT , CCR CLEAR**********/ 
	TIM4->CNT = 0; 		
	TIM14->CNT =0;
	TIM4->CCR2 = 0; 
	TIM14->CCR1 =0;
	/*************************************************/ 
	
	clear_rect();			// 막대 초기화 
	LCD_SetBrushColor(RGB_GREEN);
	LCD_DrawFillRect(27,42,1*33 +3,6);	// D2 거리 1일때의 막대 표시 
	LCD_DisplayText(2,17," 0");	
	LCD_DisplayText(3,17," 1");
}


void ADC_IRQHandler(void)
{
	ADC3->SR &= ~(1<<1);    // EOC flag clear
	if(st_flag)		  // 명령 상태 flag 가 set 일 때 
	{
		LCD_SetFont(&Gulim8);
		LCD_SetBackColor(RGB_WHITE);
		LCD_SetTextColor(RGB_BLUE);
		
		distance2Car= (int) ((3.3/1023)*ADC_value[0] *5 + 3); //"선도차와 거리" ADC 값 -> 전압값 -> 거리값  
		distance2Ped=(int)((3.3/1023)*ADC_value[1]);   	// "인도와의 거리" ADC 값 -> 전압값 
		
		/*****추종차 거리에 따른 속도(DR)*****/
		if(distance2Car%2 ==1)  // 거리가 홀수 일 때  		
			TIM4->CCR2 =(distance2Car/2)*5000 ;  
		else if(distance2Car%2 ==0) // 거리가 짝수 일 때  	
			TIM4->CCR2 =((distance2Car/2)-1)*5000 ;      
		
		sprintf(str_duty,"%2d",(TIM4->CCR2*100)/50000); // 듀티값 : (CCR * 100)/ ARR
		LCD_DisplayText(4,7,str_duty);
		/***********************/
		
		/*****인도와의 거리에 따른 DR*****/
		switch(distance2Ped)	
		{
		case 0 :
			LCD_DisplayChar(4,18,'R');	// 0 일때 'R'
			TIM14->CCR1 =24;
			break;
		case 1 :
			LCD_DisplayChar(4,18,'F');	// 1 일때 'F'
			TIM14->CCR1 =0;
			break   ;
		case 2 :
			LCD_DisplayChar(4,18,'L'); 	// 2 일때 'L'
			TIM14->CCR1 =72;
			break;
		case 3 :
			LCD_DisplayChar(4,18,'L');	// 2 일때 'L'
			TIM14->CCR1 =72;
			break;
		}
		/***********************/
		
		clear_rect();			// 막대기 초기화 
		
		LCD_SetBrushColor(RGB_RED);
		LCD_DrawFillRect(27,28,distance2Car*5 +7,6);	// 추종차와의 거리에 따른 막대 길이
		LCD_SetBrushColor(RGB_GREEN);
		LCD_DrawFillRect(27,42,distance2Ped*33 +3,6);	// 인도와의 거리에 따른 막대 길이
		sprintf(str,"%2d", distance2Car);     // 디스플레이하기 위해  double to string 
		LCD_DisplayText(2,17,str);
		sprintf(str,"%2dm ", distance2Car);     // 디스플레이하기 위해  double to string 
		SerialSendString(str);		    // 거리 USART1 송신 
		sprintf(str,"%2d",distance2Ped);     // 디스플레이하기 위해  double to string 
		LCD_DisplayText(3,17,str);
	}
	else
	{
		sprintf(str,"%s ", "0m");     // 디스플레이하기 위해  double to string 
		SerialSendString(str);	  // 거리 USART1 송신 
	}
	
}


void _EXTI_Init(void)    //EXTI11(PH11,SW3)
{
	RCC->AHB1ENR    |= (1<<7);    // 0x80, RCC_AHB1ENR GPIOH Enable
	RCC->APB2ENR    |= (1<<14);   // 0x4000, Enable System Configuration Controller Clock
	
	SYSCFG->EXTICR[3] |= 0x0707;   // EXTI12,EXTI14에 대한 소스 입력은 GPIOH로 설정   
	EXTI->IMR |= 0x00005000;
	EXTI->FTSR |= 0x00005000;      // 0x000800, Falling Trigger Enable  (EXTI11:PH11)
	NVIC->ISER[1] |=1<<(40-32);	 // Enable Interrupt EXTI15_10 
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & 0x1000) // EXTI12 
	{
		EXTI->PR |= 0x1000; 
		Move_Act();		//'M' 동작 수행
	}
	else if(EXTI->PR & 0x4000)	// EXTI14
	{
		EXTI->PR |= 0x4000; 
		Stop_Act();		//'S' 동작 수행
	}
}
void USART1_IRQHandler(void)   
{       
	if ( (USART1->SR & USART_SR_RXNE) ) // USART_SR_RXNE= 1? RX Buffer Full?
	{  
		LCD_SetTextColor(RGB_BLUE);
		char ch; 
		ch = (USART1->DR &0x01FF);   // 수신된 문자 저장
		
		if(ch=='M')		// 수신 문자가 'M' 일 때
		{
			Move_Act();	//'M' 동작 수행
		}
		
		else if(ch=='S')	// 수신 문자가 'S' 일 때
		{
			Stop_Act();	//'S' 동작 수행
		}   
	}  
}

void USART1_Init(void)
{
	// USART1 : TX(PA9)
	RCC->AHB1ENR   |= (1<<0);   // RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER   |= (2<<2*9);   // GPIOA PIN9 Output Alternate function mode               
	GPIOA->OSPEEDR   |= (3<<2*9);   // GPIOA PIN9 Output speed (100MHz Very High speed)
	GPIOA->AFR[1]   |= (7<<4);   // Connect GPIOA pin9 to AF7(USART1)
	
	// USART1 : RX(PA10)
	GPIOA->MODER    |= (2<<2*10);   // GPIOA PIN10 Output Alternate function mode
	GPIOA->OSPEEDR   |= (3<<2*10);   // GPIOA PIN10 Output speed (100MHz Very High speed
	GPIOA->AFR[1]   |= (7<<8);   // Connect GPIOA pin10 to AF7(USART1)
	
	RCC->APB2ENR   |= (1<<4);   // RCC_APB2ENR USART1 Enable
	
	USART_BRR_Configuration(19200); // USART Baud rate Configuration
	
	USART1->CR1   &= ~(1<<12);   // USART_WordLength 8 Data bit
	USART1->CR1   &= ~(1<<10);   // NO USART_Parity
	
	USART1->CR1   |= (1<<2);   // 0x0004, USART_Mode_RX Enable
	USART1->CR1   |= (1<<3);   // 0x0008, USART_Mode_Tx Enable
	
	USART1->CR2   &= ~(3<<12);   // 0b00, USART_StopBits_1
	USART1->CR3   = 0x0000;   // No HardwareFlowControl, No DMA
	
	USART1->CR1    |= (1<<5);   // 0x0020, RXNE interrupt Enable
	NVIC->ISER[1]   |= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37번)
	
	USART1->CR1    |= (1<<13);   //  0x2000, USART1 Enable
}

void SerialSendChar(uint8_t Ch) 
{
	// USART_SR_TXE(1<<7)=0?, TX Buffer NOT Empty? 
	// TX buffer Empty되지 않으면 계속 대기(송신 가능한 상태까지 대기)
	while((USART1->SR & USART_SR_TXE) == RESET); 
	USART1->DR = (Ch & 0x01FF);   // 전송 (최대 9bit 이므로 0x01FF과 masking)
}

void SerialSendString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialSendChar(*str);   // 포인터가 가르키는 곳의 데이터를 송신
		str++;          // 포인터 수치 증가
	}
}

// Baud rate 설정
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
	uint32_t tmpreg = 0x00;
	uint32_t APB2clock = 84000000;   //PCLK2_Frequency
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	
	// Find the integer part 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
		//  #define  USART_CR1_OVER8 ((uint16_t)0x8000) // USART Oversampling by 8 enable   
	{       // USART1->CR1.OVER8 = 1 (8 oversampling)
		// Computing 'Integer part' when the oversampling mode is 8 Samples 
		integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)  
	}
	else  // USART1->CR1.OVER8 = 0 (16 oversampling)
	{   // Computing 'Integer part' when the oversampling mode is 16 Samples 
		integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));  // 공식에 100을 곱한 곳임(소수점 두번째자리까지 유지하기 위함)    
	}                             // 100*(f_CK) / (8*2*Buadrate) = (25*f_CK)/(4*Buadrate)   
	tmpreg = (integerdivider / 100) << 4;
	
	// Find the fractional part 
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
	
	// Implement the fractional part in the register 
	if ((USART1->CR1 & USART_CR1_OVER8) != 0)   
	{   // 8 oversampling
		tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
	}
	else   // 16 oversampling
	{
		tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
	}
	
	// Write to USART BRR register
	USART1->BRR = (uint16_t)tmpreg;
}

void _ADC_Init(void)
{
	/* 1st Analog signal */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;     // RCC_AHB1ENR GPIOA Enable
	GPIOA->MODER |= 3<<(2*1);   // GPIOA PIN1(PA1)  Analog mode
	
	/* 2nd Analog signal */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;   // (1<<0) ENABLE GPIOB CLK 
	GPIOF->MODER |= (3<<2*3);    // GPIOF PIN3(PF3)  Analog mode
	
	/* ADC Common Init **********************************************************/
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;   // RCC_APB2ENR ADC3 Enable
	
	ADC->CCR &= ~(0x1F<<0);      // MULTI[4:0]: ADC_Mode_Independent
	ADC->CCR |= (1<<16);       // 0x00010000 ADCPRE:ADC_Prescaler_Div4 (ADC MAX Clock 36MHz, 84Mhz(APB2)/4 = 21MHz)
	
	
	/* ADC1 Init ****************************************************************/
	ADC3->CR1 |= (1<<5);           // Interrupt EN for EOC
	ADC3->CR1 |= (1<<24);   // RES[1:0]=0b00 : 12bit Resolution
	ADC3->CR1 |= 0x00000100;   // ADC_ScanCovMode Enable (SCAN=1)
	
	ADC3->CR2 &= ~(1<<1);   // ADC_ContinuousConvMode ENABLE (CONT=1)
	ADC3->CR2 |= (3<<28);   // EXTEN[1:0]=0b00: ADC_ExternalTrigConvEdge_None (외부트리거 사용안함)
	ADC3->CR2 |= (2<<24);   // EXTSEL : TIM1 CC3
	ADC3->CR2 &= ~(1<<11);   // ALIGN=0: ADC_DataAlign_Right
	ADC3->CR2 &= ~(1<<10);   // EOCS=0: The EOC bit is set at the end of each sequence of regular conversions
	
	ADC3->SQR1 |= (1<<20);   // ADC Regular channel sequece length = 2 conversion
	
	/* ADC_RegularChannelConfig *********************************************/
	ADC3->SMPR2 |= 0x07 << (3);           // ADC1_CH1 Sample TIme_480Cycles (3*Channel_1)
	ADC3->SQR3 |= 0x01 << (5*(1-1));   // ADC1_CH1 << (5 * (Rank - 1)),  Rank = 1 (1순위로 변환: 키트가변저항)
	
	ADC3->SMPR2 |= 0x07 << (27);   //ADC1_CH8 Sample Time_480Cycles (3*Channel_0)
	ADC3->SQR3 |= 0x09 << (5*(2-1));//ADC1_C//ADC1_CH0 << (5*(Rank-1)), Rank = 2 (2순위로 변환: 거리센서)
	
	/* Enable DMA request after last transfer (Single-ADC mode) */
	NVIC->ISER[0] |= (1<<18);   // Enable ADC global Interrupt
	
	ADC3->CR2 |=1<<9;               // DDS =1
	ADC3->CR2 |=1<<8;               // DMA EN
	ADC3->CR2 |= (1<<0);      // ADON=1: ADC ON
}

void DMAInit(void)
{
	// DMA2 Stream0 channel0 configuration *************************************
	RCC->AHB1ENR |= (1<<22);      //DMA2 clock enable
	DMA2_Stream0->CR |= (2<<25);   //DMA2 Stream0 channel 0 selected
	
	// ADC1->DR(Peripheral) ==> ADC_vlaue(Memory)
	DMA2_Stream0->PAR |= (uint32_t)&ADC3->DR;      //Peripheral address - ADC1->DR(Regular data) Address
	DMA2_Stream0->M0AR |= (uint32_t)&ADC_value; //Memory address - ADC_Value address 
	DMA2_Stream0->CR &= ~(3<<6);        //Data transfer direction : Peripheral-to-memory (P=>M)
	DMA2_Stream0->NDTR = 2;           //DMA_BufferSize = 2 (ADC_Value[2])
	
	DMA2_Stream0->CR &= ~(1<<9);    //Peripheral increment mode  - Peripheral address pointer is fixed
	DMA2_Stream0->CR |= (1<<10);   //Memory increment mode - Memory address pointer is incremented after each data transferd 
	DMA2_Stream0->CR |= (1<<11);   //Peripheral data size - halfword(16bit)
	DMA2_Stream0->CR |= (1<<13);   //Memory data size - halfword(16bit)   
	DMA2_Stream0->CR |= (1<<8);           //Circular mode enabled   
	DMA2_Stream0->CR |= (2<<16);   //Priority level - High
	
	DMA2_Stream0->FCR &= ~(1<<2);   //DMA_FIFO_direct mode enabled
	DMA2_Stream0->FCR |= (1<<0);   //DMA_FIFO Threshold_HalfFull , Not used in direct mode
	
	DMA2_Stream0->CR &= ~(3<<23);   //Memory burst transfer configuration - single transfer
	DMA2_Stream0->CR &= ~(3<<21);   //Peripheral burst transfer configuration - single transfer  
	DMA2_Stream0->CR |= (1<<0);   //DMA2_Stream0 enabled
}

void TIMER1_OC_Init(void) //TIM1_CH3 CC 
{
	
	// Timerbase Mode
	RCC->APB2ENR   |= 0x01;// RCC_APB1ENR TIMER1 Enable
	
	TIM1->PSC = 8400-1;   // Prescaler 168,000,000Hz/8400= 0.05ms
	TIM1->ARR = 8000-1;   //0.05ms *8000 =400ms
	
	TIM1->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
	TIM1->CR1 &= ~(1<<2);   // URS=0(Update Request Source  Selection):  By one of following events
	TIM1->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
	TIM1->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
	TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)
	
	TIM1->EGR |= (1<<0);    // UG: Update generation 
	
	TIM1->EGR |= (1<<3);    // UG: Update generation 
	TIM1->CCER |= (1<<8);   // CC3E: OC3 Active 
	TIM1->CCER &= ~(1<<9);  // CC3P: OCPolarity_Active High
	
	TIM1->CCR3 = 100;   // TIM1_Pulse
	
	TIM1->BDTR |= (1<<15);  // main output enable
	
	TIM1->CCMR2 &= ~(3<<8*0); // CC3S(CC1channel): Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	TIM1->CCMR2 |= (3<<4);   // OC3M: Output Compare 3 Mode : toggle
	
	TIM1->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
	TIM1->DIER |= (1<<3);   // CC3IE: Enable the Tim1 CC3 interrupt
	
	TIM1->CR1 |= (1<<0);   // CEN: Disable the Tim1 Counter 
	
}
void TIMER4_Init(void)
{
	RCC->AHB1ENR   |= RCC_AHB1ENR_GPIOBEN;   // 0x08, GPIOI Enable
	GPIOB->MODER    |= (2<<2*7);   //GPIOI PIN5 intput Alternate function mode                
	GPIOB->OSPEEDR    |= (2<<2*7);   //  GPIOI PIN5 Output speed (50MHz High speed)
	GPIOB->OTYPER   &= ~(1<<1*7);   // PB8 Output type push-pull (reset state)
	GPIOB->PUPDR   |= (1<<1*7);   // 0x00010000 PB8 Pull-up
	
	GPIOB->AFR[0]   |= (2<<28);   // Connect TIM8 pins(PI5) 
	
	
	RCC->APB1ENR |= 1<<2;   // RCC_APB1ENR TIMER4 Enable
	
	// Setting CR1 : 0x0000 
	TIM4->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
	TIM4->CR1 &= ~(1<<2);   // URS=0(Update Request Source  Selection):  By one of following events                      
	TIM4->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
	TIM4->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
	
	TIM4->CCER |= (1<<4);		// output enable
	TIM4->CCER &= ~(1<<5); 	//  active high
	
	TIM4->CCMR1 &= ~(3<<8); 	// CC2S : CC2 as Output
	TIM4->CCMR1 &= ~(1<<11); 	// preload disable
	TIM4->CCMR1 |= (6<<12);       // PWM Mode 1 
	
	
	TIM4->PSC = 8400-1;      // Prescaler 84,000,000Hz/8400 = 10,000 Hz 
	TIM4->ARR = 50000-1;      // Auto reload  0.1ms*50000 = 5s
	TIM4->CCR2 = 10;   
	
	TIM4->CR1 |= (1<<0);   // Enable the Tim4 Counter (clock enable)   
}

void TIMER14_PWM_Init(void)
{  
	
	RCC->AHB1ENR   |= (1<<5);   // GPIOF Enable
	RCC->APB1ENR    |= (1<<8);   // TIMER14 Enable 
	
	GPIOF->MODER    |= (2<<18);   //  Alternate function mode               
	GPIOF->OSPEEDR |= (3<<18);   //  speed (100MHz High speed)
	GPIOF->OTYPER   &= ~(1<<18);   // type push-pull (reset state)
	GPIOF->AFR[1]      |= (9<<4);    // AFR TIM14 to PF7
	
	TIM14->PSC   = 420-1;   // Prescaler 84,000,000Hz/420 = 0.000005s
	TIM14->ARR   = 80-1;   //  0.000005s*80 = 400us
	
	
	TIM14->CCER   |= (1<<0);   //  output enable
	TIM14->CCER   &= ~(1<<1);   //  output Polarity High         
	
	TIM14->CCR1   = 0;      // 초기 DR=0% 
	
	
	TIM14->CCMR1    &= ~(3<<0);    //  Output 
	TIM14->CCMR1    |= (1<<3);    // OC1PE=1: Output Compare 1 preload Enable
	TIM14->CCMR1   |= (6<<4);   // OC1M: PWM 1 mode
	TIM14->CCMR1   |= (1<<7);   // OC1CE: Output compare 1 Clear enable
	
	
	TIM14->CR1    &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
	TIM14->CR1    &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
	TIM14->CR1    &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)
	TIM14->CR1   |= (1<<7);   // ARPE: Auto-reload preload enable
	TIM14->CR1   |= (1<<0);   // CEN: Counter TIM14 enable
}

void RunMenu(void)		// 초기 메뉴 함수 
{
	LCD_Clear(RGB_WHITE);
	LCD_SetFont(&Gulim8);
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayText(0,0,"KCM 2019130013");
	LCD_DisplayText(1,0,"Tracking Car");
	LCD_DisplayText(2,0,"D1:");
	LCD_DisplayText(3,0,"D2:");
	LCD_DisplayText(4,0,"SP(DR):");
	LCD_DisplayChar(4,9,'%');
	LCD_DisplayText(4,10,"DIR(DR):");
	
}
void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
	RCC->AHB1ENR   |=  0x00000040;   // RCC_AHB1ENR : GPIOG(bit#6) Enable                     
	GPIOG->MODER    |=  0x00005555;   // GPIOG 0~7 : Output mode (0b01)                  
	GPIOG->OTYPER   &= ~0x00FF;   // GPIOG 0~7 : Push-pull  (GP8~15:reset state)   
	GPIOG->OSPEEDR    |=  0x00005555;   // GPIOG 0~7 : Output speed 25MHZ Medium speed 
	
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;   // RCC_AHB1ENR : GPIOH(bit#7) Enable                     
	GPIOH->MODER    &= ~0xFFFF0000;   // GPIOH 8~15 : Input mode (reset state)            
	GPIOH->PUPDR    &= ~0xFFFF0000;   // GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state  
	
	//NAVI.SW(PORT I) 설정
	RCC->AHB1ENR    |= 0x00000100;   // RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER    = 0x00000000;   // GPIOI PIN8~PIN15 Input mode (reset state)
	GPIOI->PUPDR    = 0x00000000;   // GPIOI PIN8~PIN15 Floating input (No Pull-up, pull-down) (reset state)
}   

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);            // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}