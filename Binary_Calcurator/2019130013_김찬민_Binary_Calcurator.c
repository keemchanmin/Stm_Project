/////////////////////////////////////////////////////////////
// 과제명: TP2. 이진 연산기
// 과제개요:  이진 연산 과정을 SW를 통해 Operand A,B를 입력 받음
//		- 이때 10진수가 아닌 2진수로 Operand A,B 를 표현해  +,-,X,&,|,^ 계산
//		- 조이스틱 Up 입력시 연속 증가 모드 실행
//		- 연속 증가모드 일때만  SW6(EXTI14) : 연속 증가모드 탈출  동작하도록 구현 
// 사용한 하드웨어(기능): GPIO, SW , LED, BUZZER , EXTI ,FRAM, JOYSTICK
// 제출일: 2023. 6.07
// 제출자 클래스: 목요일반
// 학번: 2019130013
// 이름: 김찬민     
//////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void FirstDisplay(void);

void MINUS(void);			// 이진 빼기 함수 선언
void DisplayResult(uint8_t result);	// 결과화면 표시 함수 선언
void SearchIndex(void);		//FRAM 사용시 FRAM에 저장된 Operater 를 Read 하여 프로그램상에 다시 저장

uint8_t operA=0; 		// 8비트(2비트) 2진수 표현 Operand A 	// 우리는 2비트만 사용 
uint8_t operB=0;		// 8비트(2비트) 2진수 표현 Operand B
uint8_t Cresult=0;	//8비트(4비트) 2진수 표현 계산 결과		// 우리는 4비트만 사용
uint8_t Borrow=0;	// 뺄셈 연산시 자리 빌림 수
char operator[]={'+','-','X','&','|','^','+'};	//연산자 char 형 배열 
int operator_flag;		//연산자 지정 flag 변수
int exit_flag;		// 연속모드 탈출 flag 변수


int main()
{
	LCD_Init();
	DelayMS(10);
	_GPIO_Init();
	_EXTI_Init();
	
	Fram_Init();            	   	// FRAM 초기화 H/W 초기화
	Fram_Status_Config();  	  // FRAM 초기화 S/W 초기화
	
	Cresult  =Fram_Read(531)-'0';	//Fram 에 저장된 최종 결과값으로 초기화
	SearchIndex();				// Fram 에 저장된 최종 연산자로 초기화
	
	FirstDisplay();				//초기 디스플레이 함수 호출 
	GPIOG->ODR &= ~0x00FF;		// 모든 led off
	while(1)
	{
		LCD_SetFont(&Gulim8);
		LCD_SetBackColor(RGB_YELLOW);
		LCD_SetTextColor(RGB_BLACK);
		switch(KEY_Scan())
		{
		//Operand A
		case 0xFB00 :		//sw2 
			BEEP();
			operA^=2;		// 0000 과 0010 의 XOR 연산 
			LCD_DisplayChar(1,4,(operA>>1)+'0'); 	// 오른쪽 shift 1번 후 디스플레이
			break;
		case 0xF700 :		//sw3
			BEEP();
			operA^=1;		//0000과 0001의 XOR 연산
			LCD_DisplayChar(3,4,(operA&1)+'0'); 	//0001로 마스크를 씌우고 디스플레이 
			break;
		//Operand B	
		case 0xEF00 :		//sw4
			BEEP();
			operB^=2;		// 0000 과 0010 의 XOR 연산 
			LCD_DisplayChar(6,4,(operB>>1)+'0'); // 오른쪽 shift 1번 후 디스플레이
			break;
		case 0xDF00 :		//sw5
			BEEP();
			operB^=1;		//0000과 0001의 XOR 연산
			LCD_DisplayChar(8,4,(operB&1)+'0'); //0001로 마스크를 씌우고 디스플레이 
			break;
		}
	}
}

void SearchIndex(void)
{
	switch(Fram_Read(530))	// Fram 에서 읽어온 char형 데이터(연산자)에 해당하는 임의의 숫자로 
	{					// operator_flag 초기화 
	case '+' :
		operator_flag=0;
		break;
	case '-' :
		operator_flag=1;
		break;
	case 'X' :
		operator_flag=2;
		break;
	case '&' :
		operator_flag=3;
		break;
	case '|' :
		operator_flag=4;
		break;
	case '^' :
		operator_flag=5;
		break;
		
	}
}
void FirstDisplay(void)
{
	LCD_SetFont(&Gulim8);
	LCD_SetPenColor(GET_RGB(139,69,19));
	LCD_DrawRectangle(4,16,17,19); 		// A
	LCD_DrawRectangle(4,83,17,19); 		// B
	LCD_DrawRectangle(133,50,17,19); 	// C
	LCD_SetPenColor(RGB_BLACK);
	LCD_DrawRectangle(60,10,32,110); 	//가운데 사각형
	LCD_SetBrushColor(RGB_YELLOW);
	
	LCD_DrawFillRect(31,13,10,13);        // A MSB
	LCD_DrawRectangle(31,12,10,13);
	LCD_DrawFillRect(31,39,10,13); 	      // A LSB
	LCD_DrawRectangle(31,38,10,13);
	
	LCD_DrawFillRect(31,78,10,13);	      // B MSB
	LCD_DrawRectangle(31,77,10,13);
	LCD_DrawFillRect(31,104,10,13);     	// B LSB
	LCD_DrawRectangle(31,103,10,13);
	
	LCD_DrawFillRect(111,12,9,13);		// 부호 표시 + or -
	LCD_DrawRectangle(111,12,9,13);
	
	LCD_DrawFillRect(111,38,9,12);		// 연산결과 C(4 비트) 
	LCD_DrawRectangle(111,38,9,13);
	LCD_DrawFillRect(111,52,9,13);
	LCD_DrawRectangle(111,52,9,13);
	LCD_DrawFillRect(111,64,9,12);
	LCD_DrawRectangle(111,64,9,13);
	LCD_DrawFillRect(111,78,9,12);
	LCD_DrawRectangle(111,78,9,12);
	
	LCD_SetBackColor(RGB_YELLOW);
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayChar(1,4,operA+'0'); //Operand A bit
	LCD_DisplayChar(3,4,operA+'0'); //Operand A bit
	LCD_DisplayChar(6,4,operB+'0'); //Operand B bit
	LCD_DisplayChar(8,4,operB+'0'); //Operand B bit
	
	LCD_DisplayChar(1,14,'+'); 		//부호 디스플레이 
	DisplayResult(Cresult);		//Fram을 통해 초기화된 C 디스플레이 
	

	LCD_DrawFillRect(71,90,17,16);		// 연속 증가 모드 디스플레이 
	LCD_DrawRectangle(71,90,17,16);
	LCD_DisplayText(7,9,"+0");
	
	
	LCD_SetBrushColor(GET_RGB(255,0,170));	// 연산자 배경 사각형 
	LCD_DrawFillRect(69,50,15,18);
	LCD_DrawRectangle(69,50,15,18);
	
	LCD_SetFont(&Gulim10);
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayChar(1,1,'A');
	LCD_DisplayChar(5,1,'B');
	LCD_DisplayChar(3,17,'C');
	LCD_SetBackColor(GET_RGB(255,0,170));
	LCD_DisplayChar(3,9,operator[operator_flag]);	// SearchIndex()를 통해 초기화했던  
										//operator_flag로 지정된 연산자 배열에 해당하는 연산자 디스플레이
	
	LCD_SetPenColor(RGB_BLUE);		// 가운데 사각형과 잇는 선 디스플레이 
	LCD_DrawHorLine(41,19,20);
	LCD_DrawHorLine(41,45,20);
	LCD_DrawHorLine(41,84,20);
	LCD_DrawHorLine(41,110,20);
	LCD_DrawHorLine(92,17,19);
	LCD_DrawHorLine(92,48,19);
	LCD_DrawHorLine(92,60,19);
	LCD_DrawHorLine(92,72,19);
	LCD_DrawHorLine(92,84,19);
}

uint8_t key_flag = 0;   //키가 계속 눌려졌는지 확인하는 변수 
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
	uint16_t key;
	key = GPIOH->IDR & 0xFF00;	// any key pressed ?
	if(key == 0xFF00)		// if no key, check key off
	{	if(key_flag == 0)
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
/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer)) 초기 설정	*/
void _GPIO_Init(void)
{
	// LED (GPIO G) 설정 : Output mode
	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
	
	// SW (GPIO H) 설정 : Input mode 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
	
	// Buzzer (GPIO F) 설정 : Output mode 
	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	&=  ~0x000C0000; // MODER9의 2비트 확실히 지우기 위해 	
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
	GPIOF->OSPEEDR 	|= 0x00040000 ;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//JoyStick(GPIO I)
	RCC->AHB1ENR	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER	&= ~0x000FFC00;  // GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR	&= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

void DisplayResult(uint8_t result)		//연산결과 C를 디스플레이 하는 함수 
{
	LCD_DisplayChar(3,14, ((result&8)>>3)+'0'); //  1000 과 AND 하여 마스크를 씌우고 오른쪽 3번 shift 후 디스플레이
	LCD_DisplayChar(4,14, ((result&4)>>2)+'0'); //  0100 과 AND 하여 마스크를 씌우고 오른쪽 2번 shift 후 디스플레이
	LCD_DisplayChar(5,14,((result&2)>>1)+'0'); //  0010 과 AND 하여 마스크를 씌우고 오른쪽 1번 shift 후 디스플레이
	LCD_DisplayChar(6,14,(result&1)+'0'); //  0001 과 AND 하여 마스크를 씌우고 디스플레이
	LCD_DisplayChar(1,14,'+'); 		// 계산 결과 부호 표시 
							
}

void MINUS(void) 		
{	
	Cresult=operA-operB;			// 빼기 연산 수행 후 
	if( operA>=operB)			// operand A가operand B보다 크거나 같으면 계산결과 디스플레이 
		DisplayResult(Cresult);
	else
	{						// 계산 결과 음수 발생 시
		Cresult=~Cresult+1;		// 계산 결과에 2의 보수 취한 후 디스플레이 
		DisplayResult(Cresult);
		LCD_DisplayChar(1,14,'-');	// 음수 표시 디스플레이 
	}
	
}
void EXTI9_5_IRQHandler(void)	
{
	if(EXTI->PR & 0x0100)
	{
		EXTI->PR |= 0x0100;
		BEEP();
		LCD_SetFont(&Gulim10);
		LCD_SetBackColor(GET_RGB(255,0,170));
		operator_flag++;		// 초기 FRAM 사용 안할 시 연산자는 + 이기에  먼저 1을 더해줘 다음 연산자인 - 표시 
		switch(operator_flag)
		{
			case 1 :	
				LCD_DisplayChar(3,9,operator[operator_flag]);	// -
				break;
			case 2 :
				LCD_DisplayChar(3,9,operator[operator_flag]);	// X
				break;
			case 3 :
				LCD_DisplayChar(3,9,operator[operator_flag]);	//&
				break;
			case 4 :
				LCD_DisplayChar(3,9,operator[operator_flag]);	// |
				break;
			case 5:
				LCD_DisplayChar(3,9,operator[operator_flag]);	// ^
				break;
			case 6: 
				LCD_DisplayChar(3,9,operator[operator_flag]);	// +
				operator_flag=0;						// 다음 연산자인 - 를 표시하기 위해 다시 0으로 초기화  
				break;	
		}
		Fram_Write(530,operator[operator_flag]);				// 연산자 변경될 때 마다 FRAM 저장 
		LCD_SetFont(&Gulim8);
		LCD_SetBackColor(RGB_YELLOW);					//위에서 바꿔줬던 Gulim10을 다시 Gulim8로 초기화 및 배경색 노랑색으로 초기화 
		
	}
	else if(EXTI->PR &0x0200)
	{
		EXTI->PR |= 0x0200;
		BEEP();
		LCD_SetFont(&Gulim8);
		LCD_SetBackColor(RGB_YELLOW);
		switch(operator_flag)
		{
			case 1 : 
				MINUS();			// 뺄셈 연산 수행 함수 호출 
				break;
			case 2 :
				Cresult=operA*operB;	//곱셈 연산 수행	후 디스플레이 
				DisplayResult(Cresult);	
				break;
			case 3 :
				Cresult= operB&operA;	//AND 연산 수행후 디스플레이
				DisplayResult(Cresult);
				break;
			case 4 :
				Cresult= operB|operA;	//OR 연산 수행 후 디스플레이 
				DisplayResult(Cresult);
				break;
			case 5:
				Cresult= operB^operA;	//XOR 연산 수해 후 디스플레이 
				DisplayResult(Cresult);
				break;
			case 6: case 0 :
				Cresult=operB+operA;	// 덧셈 연산 수행 후 디스플레이 
				DisplayResult(Cresult);
				operator_flag=0;		//만약 operator_flag 가 6일 수 있기에 다시 0으로 초기화 (연산자 배열의 마지막 인덱스는 6)
				break;	
		}
		Fram_Write(530,operator[operator_flag]);	//연산자 및 견산 결과 FRAM 에 저장 
		Fram_Write(531,Cresult+'0');
	}
	else if(EXTI->PR & 0x0040)
	{
		EXTI->PR |= 0x0040;
		exit_flag=1;
		LCD_DisplayText(7,9,"+1");
		BEEP();
		GPIOG->ODR |= 0x0080;
		
		while(exit_flag)
		{
			Cresult ++;					//  0000 1111 이 상태에서 1을 증가시키면 0001 0000 이 되고 우리는 하위 4비트만 사용함으로
			DisplayResult (Cresult);			// 증감 연산자만을 이용해 구현 할 수 있다. 
			Fram_Write(531,Cresult+'0');		// 또한 uint_8t의 최대인 1111 1111 이 된후 다시 1을 더한다고 하더라도 오버플로우가 발생해 
			DelayMS(500);				// 다시 0000 0000 으로 된다. 
		}	
		LCD_DisplayText(7,9,"+0");			// 연속 증가 모드 탈출 핸들러가 종료되면 다시 EXTI6 핸들러로 돌아와 연속 증가 모드 종료  동작 수행
		DelayMS(300);					// 비프 3회 
		BEEP();
		DelayMS(300);
		BEEP();
		DelayMS(300);
		BEEP();
		GPIOG->ODR &= ~0x00FF;			// LED7 OFF 
		Fram_Write(531,Cresult+'0');			// 연속 증가 모드 최종 결과 FRAM 에 저장 
		exit_flag=0;						//exit_flag 다시 0으로 초기화 -> 연속증가모드 실행중 일 때만 EXTI14 동작 가능  
	}
}

void EXTI15_10_IRQHandler(void)	
{
	if(EXTI->PR & 0x4000)
	{
		EXTI ->PR |= 0x4000;
		if(exit_flag)
		{
			BEEP();
			DelayMS(1000);
			exit_flag=0;					//exit_flag 다시 0으로 초기화 -> 연속증가모드 실행중 일 때만 EXTI14 동작 가능
		}
	}
	
}
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOG Enable
	RCC->AHB1ENR		|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH  Input mode (reset state)	
	GPIOI->MODER		&= ~0x000FFC00; 
	
	SYSCFG->EXTICR[1] |= 0x0800;  //SW0 EXTI8
	SYSCFG->EXTICR[2] |= 0x0078;  //SW7 EXTI15
	SYSCFG->EXTICR[3] |= 0x0700;
	
	EXTI->FTSR |= 0x4340; //SW가 누르자 마자 동작되기 위해      
	EXTI->IMR   |= 0x4340 ; 		
	
	NVIC->ISER[0] |= (1<<23); //EXTI9_5 백터테이블23번
	NVIC->ISER[1] |= (1<<8); //EXTI15_10 백터테이블 40번
	
	
	NVIC->IP[23]= 0xF0;  		
	NVIC->IP[40]= 0xE0; 		// EXTI 14가  EXTI 6 보다 sw적 우선순위를 높이게 하기 위해 IP 레지트터로 우선순위 바꿔줌 
}
void BEEP(void)			/* beep for 30 ms */
{ 	
	GPIOF->ODR |= 0x0200;	// PF9 'H' Buzzer on
	DelayMS(3);			// Delay 30 ms
	GPIOF->ODR &= ~0x0200;	// PF9 'L' Buzzer off
}
void DelayMS(unsigned short wMS)
{
	register unsigned short i;
	for (i=0; i<wMS; i++)
		DelayUS(1000);	// 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{
	volatile int Dly = (int)wUS*17;
	for(; Dly; Dly--);
}