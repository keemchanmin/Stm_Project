/////////////////////////////////////////////////////////////
// HW2: STEP Motor 및 DC Motor 구동 드라이브신호 발생
// 제출자: 2019130013 김찬민
// 주요 내용 및 구현 방법
// -External Clock mode 를 통해 SW와  JoyStick 의 입력 횟수를 받아 PWM 와 OC 모드를 통해 펄스 발생 
/////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"

#define SW0_PUSH        0xFE00  //PH8
#define SW1_PUSH        0xFD00  //PH9
#define SW2_PUSH        0xFB00  //PH10
#define SW3_PUSH        0xF700  //PH11
#define SW4_PUSH        0xEF00  //PH12
#define SW5_PUSH        0xDF00  //PH13
#define SW6_PUSH        0xBF00  //PH14
#define SW7_PUSH        0x7F00  //PH15

// Joystick Define
#define NAVI_PUSH	0x03C0  //PI5 0000 0011 1100 0000 
#define NAVI_UP		0x03A0  //PI6 0000 0011 1010 0000 
#define NAVI_DOWN	0x0360  //PI7 0000 0011 0110 0000 
#define NAVI_RIGHT	0x02E0  //PI8 0000 0010 1110 0000 
#define NAVI_LEFT	0x01E0  //PI9 0000 0001 1110 0000 

void _GPIO_Init(void);
uint16_t KEY_Scan(void);
uint16_t JOY_Scan(void);	
void DisplayInitScreen(void);

void TIMER3_Init (void);
void TIMER4_Init (void);
void TIMER14_PWM_Init(void);
void TIMER5_COUNTER_Init(void);
void TIMER1_OC_Init(void);
void TIMER8_COUNTER_Init(void);


void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void BEEP(void);

uint16_t step_pos_prv; // 이전 위치값 
uint16_t INT_COUNT;    // OC모드에서 원하는 갯수만큼의 펄스를 발생시키기 위한 변수 


int main(void)
{
	_GPIO_Init();
	LCD_Init();		// LCD 구동 함수
	DelayMS(10);		// LCD구동 딜레이
	DisplayInitScreen();	// LCD 초기화면구동 함수

	/*******Tim 1,3,4,5,8,14 초기화*********/
	TIMER14_PWM_Init();	
        TIMER5_COUNTER_Init();	
	TIMER4_Init();
	TIMER1_OC_Init();
	TIMER8_COUNTER_Init();
	TIMER3_Init();
	/********************************/
	
	GPIOG->ODR &= ~0x00FF;	// 초기값: LED0~7 Off
    
	while(1)
	{

	}
}
void TIMER5_COUNTER_Init(void)
{  
	RCC->AHB1ENR	|= (1<<7);	// GPIOH Enable
	RCC->APB1ENR 	|= (1<<3);	// TIMER5 Enable 

	GPIOH->MODER 	|= (2<<22);	//  Alternate function mode 					
	GPIOH->PUPDR	&= ~(3<<22); 	// NO Pull-up
 
	GPIOH->AFR[1]	|= (2<<12);	// Connect TIM5 pins(PH11) 
 
	TIM5->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM5->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM5->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)
	TIM5->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM5->CR1 |=  (1<<7);	// ARPE=1(ARR Preload Enable)
	TIM5->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM5->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 :Edge-aligned mode(reset state)

	// PSC, ARR
	TIM5->PSC = 1-1;	// Prescaler=1 
	TIM5->ARR = 5-1;	// Auto reload  :  count값 범위: 0~4  
        
	// Update(Clear) the Counter
	TIM5->EGR |= (1<<0);    // UG=1, REG's Update (CNT clear) 

	// External Clock Mode 1
	TIM5->CCMR1 |= (1<<8); 	// CC2S(CC2 channel) : Input 
	TIM5->CCMR1 &= ~(15<<12); // IC2F: No Input Filter 
				
	// CCER(Capture/Compare Enable Register) : Enable "Channel 2" 
	TIM5->CCER &= ~(1<<4);	// CC2E=0: Capture Disable
	TIM5->CCER |= (1<<5);	// CC2P=01 : falling edge 
	TIM5->CCER &= ~(1<<7);	// CC2NP=0   	

	// SMCR(Slave Mode Control Reg.) : External Clock Enable
	TIM5->SMCR |= (6<<4);	// TS(Trigger Selection) :TI2FP2(Filtered Timer Input2 출력신호)
	TIM5->SMCR |= (7<<0);	// SMS(Slave Mode Selection)=0b111 : External Clock Mode 1

	TIM5->CR1 |= (1<<0);	// CEN: Enable the Tim5 Counter  	
	
}

void TIMER4_Init(void)
{
	RCC->APB1ENR |= 1<<2;	// RCC_APB1ENR TIMER4 Enable

	// Setting CR1 : 0x0000 
	TIM4->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
	TIM4->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events                      
	TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM4->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
                         

    // Deciding the Period
	TIM4->PSC = 8400-1;		// Prescaler 84,000,000Hz/8400 = 10,000 Hz (0.1ms)  (1~65536)
	TIM4->ARR = 1000-1;		// Auto reload  0.1ms*1000 = 100ms
	
   	// Clear the Counter
	TIM4->EGR |= (1<<0);	// UG(Update generation)=1 
                   

	// Setting an UI(UEV) Interrupt 
	NVIC->ISER[0] |= (1<<30); // Enable Timer4 global Interrupt
 	TIM4->DIER |= (1<<0);	// Enable the Tim4 Update interrupt

	TIM4->CR1 |= (1<<0);	// Enable the Tim4 Counter (clock enable)   
}

void TIM4_IRQHandler(void)  	
{	
	TIM4->SR &= ~(1<<0);	// Interrupt flag Clear

		if (step_pos_prv !=TIM5->CNT)  //이전 위치값과 다른지 비교
		{	
			TIM1->CCER |= (1<<4*(3-1));// CC3E Enable
			TIM1->CR1 |= (1<<0);	// TIM1 Enable
			step_pos_prv=TIM5->CNT;	 //이전 위치값 초기화
		}

	LCD_DisplayChar(3,10,TIM5->CNT+0x30);  //위치값 display 

}


void TIMER1_OC_Init(void) //TIM1_CH3 CC 
{
	// PE13: TIM1_CH3
	// PE13을 출력설정하고 Alternate function(TIM1_CH3)으로 사용 선언
	RCC->AHB1ENR   |= (1<<4);   // RCC_AHB1ENR GPIOE Enable
	GPIOE->MODER   |= (2<<2*13);   // GPIOE PIN13 Output Alternate function mode               
	GPIOE->OSPEEDR |= (3<<2*13);   // GPIOE PIN13 Output speed (100MHz High speed)
	GPIOE->OTYPER  = 0x00000000;   // GPIOE PIN13 Output type push-pull (reset state)
	GPIOE->PUPDR   |= (1<<2*13);   // GPIOE PIN13 Pull-up
	GPIOE->AFR[1]  |= (1<<4*(13-8)); // Connect TIM1 pins(PE13) to AF1(TIM1/2)

	// Timerbase Mode
	RCC->APB2ENR   |= 0x01;// RCC_APB1ENR TIMER1 Enable

	TIM1->PSC = 8400-1;   // Prescaler 168,000,000Hz/8400= 0.05ms
	TIM1->ARR = 20000-1;   //0.05ms *20000 =1초
	
	TIM1->CR1 &= ~(1<<4);   // DIR: Countermode = Upcounter (reset state)
	TIM1->CR1 &= ~(3<<8);   // CKD: Clock division = 1 (reset state)
	TIM1->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel): No(reset state)

	TIM1->EGR |= (1<<0);    // UG: Update generation 
    
	TIM1->CCER &= ~(1<<4*(3-1));   // CC3E: OC3 Active 
	TIM1->CCER |= (1<<(4*(3-1)+1));  // CC3P: OCPolarity_Active Low
	//TIM1->CCER &= ~(1<<(4*(3-1)+1));  // CC3P: OCPolarity_Active High

	TIM1->CCR3 = 100;   // TIM1_Pulse

	TIM1->BDTR |= (1<<15);  // main output enable
   
	TIM1->CCMR2 &= ~(3<<8*0); // CC3S(CC1channel): Output 
	TIM1->CCMR2 &= ~(1<<3); // OC3PE: Output Compare 3 preload disable
	TIM1->CCMR2 |= (3<<4);   // OC3M: Output Compare 3 Mode : toggle

	TIM1->CR1 &= ~(1<<7);   // ARPE: Auto reload preload disable
	TIM1->DIER |= (1<<3);   // CC3IE: Enable the Tim1 CC3 interrupt
   
	NVIC->ISER[0] |= (1<<27); // TIM1_CC
	TIM1->CR1 &= ~(1<<0);   // CEN: Disable the Tim1 Counter 
	
}

void TIM1_CC_IRQHandler(void)      //RESET: 0
{
	if ((TIM1->SR & 0x08) != RESET)	// CC3 interrupt flag 
	{
		TIM1->SR &= ~0x08;	// CC3 Interrupt Claer
			INT_COUNT++;		
			if(INT_COUNT>=(4*(TIM5->CNT)))	//CC 인터럽트 1번 -> 1펄스 발생 즉, 위치값 *2 만큼의 펄스 수 필요 -> 한번더 *2 
			{
				TIM1->CCER &= ~(1<<4*(3-1));// CC3E Disable 
				TIM1->CR1 &= ~(1<<0); // TIM1 Disable	
				INT_COUNT= 0;	      //INT_COUNT 초기화	
			}
	}
}

void TIMER8_COUNTER_Init(void)
{  

	RCC->AHB1ENR	|= (1<<8);	// 0x08, GPIOI Enable
	RCC->APB2ENR 	|= (1<<1);	// 0x04, TIMER8 Enable 

	GPIOI->MODER 	|= (2<<10);	//GPIOI PIN5 intput Alternate function mode 					
	GPIOI->OSPEEDR 	|= (2<<10);	//  GPIOI PIN5 Output speed (50MHz High speed)
	GPIOI->PUPDR	&= ~(3<<10); 	// GPIOI PIN5 NO Pull-up
  								
	GPIOI->AFR[0]	|= (3<<20);	// Connect TIM8 pins(PI5) 
  

	TIM8->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
	TIM8->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled)
	TIM8->CR1 &= ~(1<<2);	// URS=0(Update event source Selection)
	TIM8->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM8->CR1 |=  (1<<7);	// ARPE=1(ARR Preload Enable)
	TIM8->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM8->CR1 |= (3<<5); 	// CMS(Center-aligned mode Sel)

	// PSC, ARR
	TIM8->PSC = 1-1;	// Prescaler=1
	TIM8->ARR = 7;		// Auto reload  :  count값 범위: ‘0->1->…6->7->6->…1->0->1->2
	// Update(Clear) the Counter
	TIM8->EGR |= (1<<0);    // UG=1, REG's Update (CNT clear) 


	TIM8->CCMR1 |= (1<<0); 	// CC1S(CC1 channel) = '0b01' : Input 
	TIM8->CCMR1 &= ~(15<<4); // IC1F='0b0000: No Input Filter 
				
	TIM8->CCER &= ~(1<<0);	// CC1E=0: Capture Disable
	TIM8->CCER &= ~(1<<1);	// CC1P=0 
	TIM8->CCER &= ~(1<<3);	// CC1NP=0   
	

	TIM8->SMCR |= (5<<4);	// TS(Trigger Selection)=0b101 :TI1FP1(Filtered Timer Input 1 출력신호)
	TIM8->SMCR |= (7<<0);	// SMS(Slave Mode Selection)=0b111 : External Clock Mode 1
	TIM8->BDTR |= (1<<15);  // main output enable
	TIM8->CR1 |= (1<<0);	// CEN: Enable the Tim8 Counter  	
}


void TIMER3_Init(void)
{
	RCC->APB1ENR |= 0x02;	// RCC_APB1ENR TIMER3 Enable
 
	TIM3->CR1 &= ~(1<<4);  // DIR=0(Up counter)(reset state)
	TIM3->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events                         
	TIM3->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events                     
	TIM3->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
	TIM3->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
	TIM3->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
	TIM3->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
      
	
	TIM3->PSC = 16800-1;	// Prescaler 84,000,000Hz/16800 = 5,000 Hz (0.2ms)  (1~65536)
	TIM3->ARR = 100-1;		//  0.2ms*100 =20ms
	
	TIM3->EGR |= (1<<0);	// UG(Update generation)=1 
	NVIC->ISER[0] |= (1<<29); // Enable Timer3 global Interrupt
 	TIM3->DIER |= (1<<0);	// Enable the Tim3 Update interrupt
	TIM3->CR1 |= (1<<0);	// Enable the Tim3 Counter (clock enable)   
}

void TIM3_IRQHandler(void)  	// 20ms Interrupt
{
	TIM3->SR &= ~(1<<0);	// Interrupt flag Clear
	TIM14->CCR1 = 80-TIM8->CNT*8;	      // 명령값 0 ->DR=0% ,명령값 1 ->DR=10% ... PWM2 
	LCD_DisplayChar(5,8,TIM8->CNT+0x30);  // 명령값  Display
}


void TIMER14_PWM_Init(void)
{  

	RCC->AHB1ENR	|= (1<<5);	// GPIOF Enable
	RCC->APB1ENR 	|= (1<<8);	// TIMER14 Enable 
    						
	GPIOF->MODER 	|= (2<<18);	//  Alternate function mode					
	GPIOF->OSPEEDR 	|= (3<<18);	//  speed (100MHz High speed)
	GPIOF->OTYPER	&= ~(1<<18);	// type push-pull (reset state)
	GPIOF->AFR[1]	|= (9<<4); 	// AFR
    
	TIM14->PSC	= 420-1;	// Prescaler 84,000,000Hz/420 = 0.000005s
	TIM14->ARR	= 80-1;	//  0.000005s*80 = 400us
    

	TIM14->CCER	|= (1<<0);	// CC1E=1: OC1(TIM5_CH1) Active(Capture/Compare 1 output enable)
	TIM14->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1으로 반전없이 출력)         

	TIM14->CCR1	= 80;		// 초기 DR=0% by PWM2 Mode


	TIM14->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel)='0b00' : Output 
	TIM14->CCMR1 	|= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable
	TIM14->CCMR1	|= (7<<4);	// OC1M: PWM 2 mode
	TIM14->CCMR1	|= (1<<7);	// OC1CE: Output compare 1 Clear enable


	TIM14->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
	TIM14->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
	TIM14->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
	TIM14->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
	TIM14->CR1	|= (1<<0);	// CEN: Counter TIM14 enable
}

void _GPIO_Init(void)
{
	// LED (GPIO G) 설정
    	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
 	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
    
	// SW (GPIO H) 설정 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
	
	//Joy Stick SW(PORT I) 설정
	RCC->AHB1ENR 	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER 	&= ~0x000FFC00;	// GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR    &= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

void BEEP(void)			// Beep for 20 ms 
{ 	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(20);		// Delay 20 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}

void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);   // 1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}

void DisplayInitScreen(void)
{
    LCD_Clear(RGB_YELLOW);	// 화면 클리어
    LCD_SetFont(&Gulim8);	// 폰트 : 굴림 8
    LCD_SetBackColor(RGB_BLACK);
    LCD_SetTextColor(RGB_WHITE);
    LCD_SetBrushColor(RGB_BLACK);
    LCD_DrawFillRect(0,0,128,19);
   LCD_DisplayText(0,0,"Motor Contorl  ");  
    LCD_DisplayText(1,0,"2019130013 KCM  ");
    LCD_SetBackColor(RGB_YELLOW);
    LCD_SetTextColor(RGB_BLUE);
    LCD_DisplayText(2,0,"Step Motor");
    LCD_DisplayText(4,0,"DC Motor");
	  
    LCD_SetTextColor(RGB_BLACK);// 글자색 : Black
    LCD_DisplayText(3,1,"Position:");  // Title
    LCD_DisplayText(5,1,"Torque: ");  // Title

}

/* Joystick switch가 입력되었는지를 여부와 어떤 Joystick switch가 입력되었는지의 정보를 return하는 함수  */ 
uint8_t joy_flag = 0;
uint16_t JOY_Scan(void)	// input joy stick NAVI_* 
{ 
	uint16_t key;
	key = GPIOI->IDR & 0x03E0;	// any key pressed ?
	if(key == 0x03E0)		// if no key, check key off
	{  	if(joy_flag == 0)
			return key;
		else
		{	DelayMS(10);
			joy_flag = 0;
			return key;
		}
	}
  	else				// if key input, check continuous key
	{	if(joy_flag != 0)	// if continuous key, treat as no key input
			return 0x03E0;
		else			// if new key,delay for debounce
		{	joy_flag = 1;
			DelayMS(10);
 			return key;
		}
	}
}

uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{  	if(key_flag == 0)
			return key;
		else
		{	DelayMS(10);
			key_flag = 0;
			return key;
		}
	}
  	else				// if key input, check continuous key
	{	if(key_flag != 0)	// if continuous key, treat as no key input
			return 0xFF00;
		else			// if new key,delay for debounce
		{	key_flag = 1;
			DelayMS(10);
 			return key;
		}
	}
}
