/////////////////////////////////////////////////////////////
// ������: TP2. ���� �����
// ��������:  ���� ���� ������ SW�� ���� Operand A,B�� �Է� ����
//		- �̶� 10������ �ƴ� 2������ Operand A,B �� ǥ����  +,-,X,&,|,^ ���
//		- ���̽�ƽ Up �Է½� ���� ���� ��� ����
//		- ���� ������� �϶���  SW6(EXTI14) : ���� ������� Ż��  �����ϵ��� ���� 
// ����� �ϵ����(���): GPIO, SW , LED, BUZZER , EXTI ,FRAM, JOYSTICK
// ������: 2023. 6.07
// ������ Ŭ����: ����Ϲ�
// �й�: 2019130013
// �̸�: ������     
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

void MINUS(void);			// ���� ���� �Լ� ����
void DisplayResult(uint8_t result);	// ���ȭ�� ǥ�� �Լ� ����
void SearchIndex(void);		//FRAM ���� FRAM�� ����� Operater �� Read �Ͽ� ���α׷��� �ٽ� ����

uint8_t operA=0; 		// 8��Ʈ(2��Ʈ) 2���� ǥ�� Operand A 	// �츮�� 2��Ʈ�� ��� 
uint8_t operB=0;		// 8��Ʈ(2��Ʈ) 2���� ǥ�� Operand B
uint8_t Cresult=0;	//8��Ʈ(4��Ʈ) 2���� ǥ�� ��� ���		// �츮�� 4��Ʈ�� ���
uint8_t Borrow=0;	// ���� ����� �ڸ� ���� ��
char operator[]={'+','-','X','&','|','^','+'};	//������ char �� �迭 
int operator_flag;		//������ ���� flag ����
int exit_flag;		// ���Ӹ�� Ż�� flag ����


int main()
{
	LCD_Init();
	DelayMS(10);
	_GPIO_Init();
	_EXTI_Init();
	
	Fram_Init();            	   	// FRAM �ʱ�ȭ H/W �ʱ�ȭ
	Fram_Status_Config();  	  // FRAM �ʱ�ȭ S/W �ʱ�ȭ
	
	Cresult  =Fram_Read(531)-'0';	//Fram �� ����� ���� ��������� �ʱ�ȭ
	SearchIndex();				// Fram �� ����� ���� �����ڷ� �ʱ�ȭ
	
	FirstDisplay();				//�ʱ� ���÷��� �Լ� ȣ�� 
	GPIOG->ODR &= ~0x00FF;		// ��� led off
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
			operA^=2;		// 0000 �� 0010 �� XOR ���� 
			LCD_DisplayChar(1,4,(operA>>1)+'0'); 	// ������ shift 1�� �� ���÷���
			break;
		case 0xF700 :		//sw3
			BEEP();
			operA^=1;		//0000�� 0001�� XOR ����
			LCD_DisplayChar(3,4,(operA&1)+'0'); 	//0001�� ����ũ�� ����� ���÷��� 
			break;
		//Operand B	
		case 0xEF00 :		//sw4
			BEEP();
			operB^=2;		// 0000 �� 0010 �� XOR ���� 
			LCD_DisplayChar(6,4,(operB>>1)+'0'); // ������ shift 1�� �� ���÷���
			break;
		case 0xDF00 :		//sw5
			BEEP();
			operB^=1;		//0000�� 0001�� XOR ����
			LCD_DisplayChar(8,4,(operB&1)+'0'); //0001�� ����ũ�� ����� ���÷��� 
			break;
		}
	}
}

void SearchIndex(void)
{
	switch(Fram_Read(530))	// Fram ���� �о�� char�� ������(������)�� �ش��ϴ� ������ ���ڷ� 
	{					// operator_flag �ʱ�ȭ 
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
	LCD_DrawRectangle(60,10,32,110); 	//��� �簢��
	LCD_SetBrushColor(RGB_YELLOW);
	
	LCD_DrawFillRect(31,13,10,13);        // A MSB
	LCD_DrawRectangle(31,12,10,13);
	LCD_DrawFillRect(31,39,10,13); 	      // A LSB
	LCD_DrawRectangle(31,38,10,13);
	
	LCD_DrawFillRect(31,78,10,13);	      // B MSB
	LCD_DrawRectangle(31,77,10,13);
	LCD_DrawFillRect(31,104,10,13);     	// B LSB
	LCD_DrawRectangle(31,103,10,13);
	
	LCD_DrawFillRect(111,12,9,13);		// ��ȣ ǥ�� + or -
	LCD_DrawRectangle(111,12,9,13);
	
	LCD_DrawFillRect(111,38,9,12);		// ������ C(4 ��Ʈ) 
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
	
	LCD_DisplayChar(1,14,'+'); 		//��ȣ ���÷��� 
	DisplayResult(Cresult);		//Fram�� ���� �ʱ�ȭ�� C ���÷��� 
	

	LCD_DrawFillRect(71,90,17,16);		// ���� ���� ��� ���÷��� 
	LCD_DrawRectangle(71,90,17,16);
	LCD_DisplayText(7,9,"+0");
	
	
	LCD_SetBrushColor(GET_RGB(255,0,170));	// ������ ��� �簢�� 
	LCD_DrawFillRect(69,50,15,18);
	LCD_DrawRectangle(69,50,15,18);
	
	LCD_SetFont(&Gulim10);
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayChar(1,1,'A');
	LCD_DisplayChar(5,1,'B');
	LCD_DisplayChar(3,17,'C');
	LCD_SetBackColor(GET_RGB(255,0,170));
	LCD_DisplayChar(3,9,operator[operator_flag]);	// SearchIndex()�� ���� �ʱ�ȭ�ߴ�  
										//operator_flag�� ������ ������ �迭�� �ش��ϴ� ������ ���÷���
	
	LCD_SetPenColor(RGB_BLUE);		// ��� �簢���� �մ� �� ���÷��� 
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

uint8_t key_flag = 0;   //Ű�� ��� ���������� Ȯ���ϴ� ���� 
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
/* GPIO (GPIOG(LED), GPIOH(Switch), GPIOF(Buzzer)) �ʱ� ����	*/
void _GPIO_Init(void)
{
	// LED (GPIO G) ���� : Output mode
	RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
	GPIOG->MODER 	|=  0x00005555;	// GPIOG 0~7 : Output mode (0b01)						
	GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
	GPIOG->OSPEEDR 	|=  0x00005555;	// GPIOG 0~7 : Output speed 25MHZ Medium speed 
	
	// SW (GPIO H) ���� : Input mode 
	RCC->AHB1ENR    |=  0x00000080;	// RCC_AHB1ENR : GPIOH(bit#7) Enable							
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH 8~15 : Input mode (reset state)				
	GPIOH->PUPDR 	&= ~0xFFFF0000;	// GPIOH 8~15 : Floating input (No Pull-up, pull-down) :reset state
	
	// Buzzer (GPIO F) ���� : Output mode 
	RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
	GPIOF->MODER 	&=  ~0x000C0000; // MODER9�� 2��Ʈ Ȯ���� ����� ���� 	
	GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
	GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
	GPIOF->OSPEEDR 	|= 0x00040000 ;	// GPIOF 9 : Output speed 25MHZ Medium speed 
	
	//JoyStick(GPIO I)
	RCC->AHB1ENR	|= 0x00000100;	// RCC_AHB1ENR GPIOI Enable
	GPIOI->MODER	&= ~0x000FFC00;  // GPIOI 5~9 : Input mode (reset state)
	GPIOI->PUPDR	&= ~0x000FFC00;	// GPIOI 5~9 : Floating input (No Pull-up, pull-down) (reset state)
}	

void DisplayResult(uint8_t result)		//������ C�� ���÷��� �ϴ� �Լ� 
{
	LCD_DisplayChar(3,14, ((result&8)>>3)+'0'); //  1000 �� AND �Ͽ� ����ũ�� ����� ������ 3�� shift �� ���÷���
	LCD_DisplayChar(4,14, ((result&4)>>2)+'0'); //  0100 �� AND �Ͽ� ����ũ�� ����� ������ 2�� shift �� ���÷���
	LCD_DisplayChar(5,14,((result&2)>>1)+'0'); //  0010 �� AND �Ͽ� ����ũ�� ����� ������ 1�� shift �� ���÷���
	LCD_DisplayChar(6,14,(result&1)+'0'); //  0001 �� AND �Ͽ� ����ũ�� ����� ���÷���
	LCD_DisplayChar(1,14,'+'); 		// ��� ��� ��ȣ ǥ�� 
							
}

void MINUS(void) 		
{	
	Cresult=operA-operB;			// ���� ���� ���� �� 
	if( operA>=operB)			// operand A��operand B���� ũ�ų� ������ ����� ���÷��� 
		DisplayResult(Cresult);
	else
	{						// ��� ��� ���� �߻� ��
		Cresult=~Cresult+1;		// ��� ����� 2�� ���� ���� �� ���÷��� 
		DisplayResult(Cresult);
		LCD_DisplayChar(1,14,'-');	// ���� ǥ�� ���÷��� 
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
		operator_flag++;		// �ʱ� FRAM ��� ���� �� �����ڴ� + �̱⿡  ���� 1�� ������ ���� �������� - ǥ�� 
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
				operator_flag=0;						// ���� �������� - �� ǥ���ϱ� ���� �ٽ� 0���� �ʱ�ȭ  
				break;	
		}
		Fram_Write(530,operator[operator_flag]);				// ������ ����� �� ���� FRAM ���� 
		LCD_SetFont(&Gulim8);
		LCD_SetBackColor(RGB_YELLOW);					//������ �ٲ���� Gulim10�� �ٽ� Gulim8�� �ʱ�ȭ �� ���� ��������� �ʱ�ȭ 
		
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
				MINUS();			// ���� ���� ���� �Լ� ȣ�� 
				break;
			case 2 :
				Cresult=operA*operB;	//���� ���� ����	�� ���÷��� 
				DisplayResult(Cresult);	
				break;
			case 3 :
				Cresult= operB&operA;	//AND ���� ������ ���÷���
				DisplayResult(Cresult);
				break;
			case 4 :
				Cresult= operB|operA;	//OR ���� ���� �� ���÷��� 
				DisplayResult(Cresult);
				break;
			case 5:
				Cresult= operB^operA;	//XOR ���� ���� �� ���÷��� 
				DisplayResult(Cresult);
				break;
			case 6: case 0 :
				Cresult=operB+operA;	// ���� ���� ���� �� ���÷��� 
				DisplayResult(Cresult);
				operator_flag=0;		//���� operator_flag �� 6�� �� �ֱ⿡ �ٽ� 0���� �ʱ�ȭ (������ �迭�� ������ �ε����� 6)
				break;	
		}
		Fram_Write(530,operator[operator_flag]);	//������ �� �߻� ��� FRAM �� ���� 
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
			Cresult ++;					//  0000 1111 �� ���¿��� 1�� ������Ű�� 0001 0000 �� �ǰ� �츮�� ���� 4��Ʈ�� ���������
			DisplayResult (Cresult);			// ���� �����ڸ��� �̿��� ���� �� �� �ִ�. 
			Fram_Write(531,Cresult+'0');		// ���� uint_8t�� �ִ��� 1111 1111 �� ���� �ٽ� 1�� ���Ѵٰ� �ϴ��� �����÷ο찡 �߻��� 
			DelayMS(500);				// �ٽ� 0000 0000 ���� �ȴ�. 
		}	
		LCD_DisplayText(7,9,"+0");			// ���� ���� ��� Ż�� �ڵ鷯�� ����Ǹ� �ٽ� EXTI6 �ڵ鷯�� ���ƿ� ���� ���� ��� ����  ���� ����
		DelayMS(300);					// ���� 3ȸ 
		BEEP();
		DelayMS(300);
		BEEP();
		DelayMS(300);
		BEEP();
		GPIOG->ODR &= ~0x00FF;			// LED7 OFF 
		Fram_Write(531,Cresult+'0');			// ���� ���� ��� ���� ��� FRAM �� ���� 
		exit_flag=0;						//exit_flag �ٽ� 0���� �ʱ�ȭ -> ����������� ������ �� ���� EXTI14 ���� ����  
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
			exit_flag=0;					//exit_flag �ٽ� 0���� �ʱ�ȭ -> ����������� ������ �� ���� EXTI14 ���� ����
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
	
	EXTI->FTSR |= 0x4340; //SW�� ������ ���� ���۵Ǳ� ����      
	EXTI->IMR   |= 0x4340 ; 		
	
	NVIC->ISER[0] |= (1<<23); //EXTI9_5 �������̺�23��
	NVIC->ISER[1] |= (1<<8); //EXTI15_10 �������̺� 40��
	
	
	NVIC->IP[23]= 0xF0;  		
	NVIC->IP[40]= 0xE0; 		// EXTI 14��  EXTI 6 ���� sw�� �켱������ ���̰� �ϱ� ���� IP ����Ʈ�ͷ� �켱���� �ٲ��� 
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