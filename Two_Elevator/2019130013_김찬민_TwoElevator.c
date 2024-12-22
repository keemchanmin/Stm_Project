/////////////////////////////////////////////////////////////
// ������: TP1. Two Elevators
// ��������:   -6�� �ǹ��� �δ��� ����������(L_Elevator, R_Elevator)�� ��ġ
//		-��������� ��ǥ�� ����ġ�� �����ϸ� ���������Ͱ� �̵�
//		-�̵��� ���� ũ���� ��ȭ�� ǥ��	
//		-  FL ��忡�� �����, ��ǥ���� ��� �ѹ����̶� ������ �ؾ� EX ��� �����ϵ��� ���� 
// ����� �ϵ����(���): GPIO, SW , LED, BUZZER , EXTI ,FRAM
// ������: 2023. 6.03
// ������ Ŭ����: ����Ϲ�
// �й�: 2019130013
// �̸�: ������     
//////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"
typedef struct structelevator    // ���������� ����ü ����
{
	int floor;   		  // ������������ ������ ����
	int YPos;  		  // ���������� �̵� ������ Y��ǥ ����
	int RectHeight;     // ���������� �̵� ������ ���� ���� ����
	char element; 	  // � ���������� ���� �Ǵ� �ϱ� ���� ����
	char* Name;        // GLCD�� � �������������� Display �� ���ڿ� ���� 
	UINT32 Color;      // ������������ ���� ����
}Elevator;

Elevator L_Elevator ={1,93,12,'L',"L-E",RGB_BLUE};     //���� ���������� ��ü ���� �� �ʱ�ȭ   // 93, 12,�� 1�� ���� ������������ Y��ǥ�� ���γ���
Elevator R_Elevator={1,93,12,'R',"R-E",RGB_GREEN};   //������ ���������� ��ü ���� �� �ʱ�ȭ  // ������ 1�� �ʱ�ȭ (�� �ʱ�ȭ�� ���� FRAM ��� ���� ���� �� �̿�)

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

void FirstDisplay(void);    					//���� GLCD�� Display�ϴ� �Լ�
void ExMode(void);        				 	// EXTI8 �߻��� ������� �����ϴ� �Լ�
void Fram_Elevator(Elevator* elevator);  		//  Flash �޸𸮷� ��Ʈ��� ���۽� 
									//FRAM �� ����� ������ ������ �� ������������ ������ �Է��ϴ� �Լ�
void UpFloor(Elevator* elevator, int Floor); 		// �������̰� �ö󰡴� ������ �����ϴ� �Լ� 
void DownFloor(Elevator* elevator, int Floor); 	// ���������Ͱ� �������� ������ �����ϴ� �Լ�
									// int Floor �Ű������� ���� ��������� ������, ��ǥ������ ������ ����
void Compare_Floor(Elevator* elevator);    // ������, �����, ��ǥ���� ���ϴ� �Լ�
void EL_Fram_Wirte(Elevator* elevator);	//�ش� ������������ ��ǥ���� Fram_Write �ϴ� �Լ�

int Start_Floor;       // ����� �Է� ����  
int Dst_Floor;         // ��ǥ��   �Է� �Ǽ� 
int HD_Flag;              // ������ �߿��� ������尡 ���� �� �� �ֵ��� �ϱ� ���� Flag ���� 
int EX_St_Flag;          // ������� ��ǥ���� ��� �����ؾ߸� �����尡 ���� �� �� �ֵ��� �ϱ� ���� ���� 
int EX_Dst_Flag;        // ���� ���� ��尡 ���� �Ǿ� LCD�� ����� < ��ǥ�� ������ �ִ��� �����, ��ǥ��(�Ѵ� ���)�� �׻� �Է��ؾ߸� ������ ���� ���� 

int main(void)
{
	LCD_Init();
	DelayMS(10);
	_GPIO_Init();
	_EXTI_Init();
	Fram_Init();            	   // FRAM �ʱ�ȭ H/W �ʱ�ȭ
	Fram_Status_Config();   // FRAM �ʱ�ȭ S/W �ʱ�ȭ
	
	GPIOG->ODR &= ~0x00FF;   //�ʱ� ��� LED OFF ��Ų ��
	GPIOG->ODR |= 0x0080;      // LED7 �� ON
	
	//Fram_Write(2023,'1');
	//Fram_Write(2024,'1');
	Fram_Elevator(&L_Elevator);   // ������ ������ ���� ������ ���������Ϳ� FRAM�� ������ �� ������ ���� �ϱ� ���� �Լ� ȣ��
	Fram_Elevator(&R_Elevator);
	FirstDisplay();			    // �ʱ� GLCD Display �Լ� ȣ�� (FRAM ������ ���� �ʱ� ȭ���� �޶����⿡ 
						   //  Fram_Elevator() �� FRAM ������ ���������� �������� �� ������ ���� Display�ϱ� ���� 
						   //  ���÷��� �Լ��� ���߿� ȣ�� )
	
	Start_Floor=L_Elevator.floor;		// FRAM ���� �ʱ� LCD�� ����� < ��ǥ�� �� FRAM 2023 �������� read �� ��, ���⼭ SW1, SW2���� ������ ǥ�õ� ������ +1�� �����ϱ� ������ 
	Dst_Floor=L_Elevator.floor;		// ��ǥ������ �켱 ���� ���������� ������ �Ѱ��� , ���� FRAM�� ������� �ʴ� ��쿡�� L_Elevator.floor�� 1�� �ʱ�ȭ �Ǿ��ֱ� ������ ���� ����
	
	while(1)
	{
		switch(KEY_Scan())
		{
		case 0xFD00 :   //SW1 : ����� ����
			GPIOG->ODR |=   0x0002;      // LED 1 ON
			GPIOG->ODR &= ~0x0004;   	// LED 2 OFF
			LCD_SetTextColor(RGB_RED);  
			BEEP();
			if(Start_Floor==6)          // ������� 6�̸� 0���� �ʱ�ȭ
				Start_Floor=0;
			Start_Floor++;		 // SW ���������� ����� +1 �� ����
			LCD_DisplayChar(6,8,Start_Floor+'0');	  // GLCD�� ����� ǥ��(int to char ���� int+'0')
			EX_St_Flag=1;  		 // ����� �����ߴٸ� EX_St_Flag = 1
			break;
			
		case 0xFB00 :    //SW2 : ����� ����
			GPIOG->ODR |=   0x0004;	     // LED 2 ON
			GPIOG->ODR &= ~0x0002;    // LED 1 OFF
			LCD_SetTextColor(RGB_RED);
			BEEP();
			if(Dst_Floor==6)           // ��ǥ���� 6�̸� 0���� �ʱ�ȭ
				Dst_Floor=0;        
			Dst_Floor++;                 // SW ���������� ��ǥ�� +1 �� ���� 
			LCD_DisplayChar(6,10,Dst_Floor+'0');   // GLCD�� ��ǥ�� ǥ��(int to char ���� int+'0')
			EX_Dst_Flag=1;          // ��ǥ�� �����ߴٸ� EX_Dst_Flag = 1
			break;
		}	
	}		
}
void EL_Fram_Wirte(Elevator* elevator)
{
	if(elevator->element=='L')						
		Fram_Write(2023,(elevator->floor)+'0');
	else                                                   	
		Fram_Write(2024,(elevator->floor)+'0');
}

void Fram_Elevator(Elevator* elevator)
{
	if(elevator->element=='L')     // ���� ���� ���������� ��� 
		elevator->floor=Fram_Read(2023)-'0';    // FRAM 2023������ ����� �� ������ ���� ���������� �������� ����
	
	else if(elevator->element=='R')    // ���� ������ ���������� ���
		elevator->floor=Fram_Read(2024)-'0';    // FRAM 2024������ ����� �� ������ ������ ���������� �������� ����
	
	elevator->YPos=92-(13*((elevator->floor)-1));   // �ʱ� ���������͸� DIsplay �ϱ����� �̵� ������ Y��ǥ,
	elevator->RectHeight=(13*((elevator->floor)));    // �̵������� ���� ���̱��̸� ���� ���� ���� �ʱ�ȭ 
}
void FirstDisplay(void)
{
	LCD_Clear(RGB_WHITE);     //�ʱ� LCDȭ�� �ʱ�ȭ 
	LCD_SetFont(&Gulim8);
	LCD_SetBackColor(RGB_WHITE);
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayText(0,2,"MC-Elevator(KCM)");
	LCD_SetTextColor(RGB_BLUE);
	LCD_DisplayChar(2,5,'6');
	LCD_DisplayChar(3,5,'5');
	LCD_DisplayChar(4,5,'4');
	LCD_DisplayChar(5,5,'3');
	LCD_DisplayChar(6,5,'2');
	LCD_DisplayChar(7,5,'1');
	LCD_DisplayText(4,8,"L-E");
	LCD_SetTextColor(RGB_RED);
	LCD_DisplayChar(2,13,'6');
	LCD_DisplayChar(3,13,'5');
	LCD_DisplayChar(4,13,'4');
	LCD_DisplayChar(5,13,'3');
	LCD_DisplayChar(6,13,'2');
	LCD_DisplayChar(7,13,'1');
	LCD_DisplayText(2,8,"FL");
	LCD_DisplayChar(5,9,'S');
	LCD_DisplayChar(6,8,L_Elevator.floor+'0');    // FRAM�� 2023���� �� ���� ������������ ���� ���� DIsplay   // FRAM ��� ���� ���� �� 1 Display
	LCD_DisplayChar(6,10,L_Elevator.floor+'0');  // FRAM�� 2023���� �� ���� ������������ ���� ���� DIsplay
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayChar(6,9,'>');
	LCD_SetBrushColor(RGB_BLUE);
	LCD_DrawFillRect(28,L_Elevator.YPos,8,L_Elevator.RectHeight);     
	LCD_SetBrushColor(RGB_GREEN);
	LCD_DrawFillRect(113,R_Elevator.YPos,8,R_Elevator.RectHeight);	
}

void ExMode(void)
{	
	if(abs(L_Elevator.floor-Start_Floor)<=abs(R_Elevator.floor-Start_Floor))        //���� ������������ �������� ������ ���������� ���� ������� ����� ���
	{	
		Compare_Floor(&L_Elevator);   // ���������� ���� �񱳸� �ϱ� ���� �Լ� ȣ��(�̶� ���ڴ� ���� ����������)
	}
	else if(abs(L_Elevator.floor-Start_Floor)>abs(R_Elevator.floor-Start_Floor))   //������ ������������ �������� ���� ���������� ���� ������� ����� ���
	{	
		Compare_Floor(&R_Elevator);   // ���������� ���� �񱳸� �ϱ� ���� �Լ� ȣ��(�̶� ���ڴ� ������ ����������)
	}
}

void Compare_Floor(Elevator* elevator)
{
	if(elevator->floor<Start_Floor)          		//���� ���� ����� ���� ���� ��� (������ -> ����� UP)
	{		
		if(Start_Floor==Dst_Floor)        		// ������� ��ǥ���� ���� ��� 
		{
			UpFloor(elevator,Start_Floor);  	 //���������ʹ� ������ -> ����� ���� ���� �� 1���� ����
			DelayMS(500);			     	// UpFloor()�Լ� ���� ���� DelayMS(500)�� ȣ���ϱ⿡ 1�ʸ� ���߱����� �ѹ� �� DelayMS(500) ȣ��
			EL_Fram_Wirte(elevator);
		}
		else					   	 // ������� ��ǥ���� �ٸ� ���	
		{
			UpFloor(elevator,Start_Floor);  // �켱 ���������� ����� ���� UP   
			
			if(Start_Floor<Dst_Floor)         // ���⼭ ���� ��ǥ���� ��������� ũ�ٸ� ����� -> ��ǥ�� UP
				UpFloor(elevator,Dst_Floor); 	
			else if(Start_Floor>Dst_Floor)  // ���⼭ ���� ��ǥ���� ��������� �۴ٸ� ����� -> ��ǥ�� DOWN
				DownFloor(elevator,Dst_Floor);
		}
	}
	else if(elevator->floor>Start_Floor)   //���� ���� ����� ���� ū ��� (������ -> ����� DOWN)
	{
		if(Start_Floor==Dst_Floor)        // ������� ��ǥ���� ���� ��� 
		{
			DownFloor(elevator,Start_Floor);    //���������ʹ� ������ -> ����� ���� ���� �� 1���� ����
			DelayMS(500);                             // DownFloor()�Լ� ���� ���� DelayMS(500)�� ȣ���ϱ⿡ 1�ʸ� ���߱����� �ѹ� �� DelayMS(500) ȣ��
			EL_Fram_Wirte(elevator);
		}
		else 				          // ������� ��ǥ���� �ٸ� ���	
		{
			DownFloor(elevator,Start_Floor);   //�켱 ���������� ����� ���� DOWN
			
			if(Start_Floor<Dst_Floor)  	// ���⼭ ���� ��ǥ���� ��������� ũ�ٸ� ����� -> ��ǥ�� UP
				UpFloor(elevator,Dst_Floor);
			
			else if(Start_Floor>Dst_Floor)	// ���⼭ ���� ��ǥ���� ��������� �۴ٸ� ����� -> ��ǥ�� DOWN
				DownFloor(elevator,Dst_Floor);
		}
	}	
	else if(elevator->floor==Start_Floor)   //���� ���� ������� ���� ��� 
	{
		DelayMS(1000);                      // 1�� �������� ���� 
		if(Start_Floor<Dst_Floor)           // ��ǥ���� ��������� ���� ��� ����� -> ��ǥ�� UP
			UpFloor(elevator,Dst_Floor);	
		
		else if(Start_Floor>Dst_Floor)     // ��ǥ���� ��������� ���� ��� ����� -> ��ǥ�� UP
			DownFloor(elevator,Dst_Floor);	
		else
		{
			DelayMS(1000);		      // ������ == ����� == ��ǥ�� �ΰ��
			EL_Fram_Wirte(elevator);		
		}
	}
}

void UpFloor(Elevator* elevator, int Floor)
{
	LCD_SetBrushColor(elevator->Color);      //�̵� ������ ������ ���ڷ� ���� ������������ ���� ���� ��� 
	LCD_SetTextColor(elevator->Color);	       //GLCD�� L-E Ȥ�� R-E ǥ���Ҷ� ���������� ���ڷ� ���� ������������ ���� ���� ��� 
	LCD_DisplayText(4,8,elevator->Name);     // ������������ Name ������ ���� Display
	LCD_SetTextColor(RGB_RED);			// ���� �̵� ����ǥ�ô� RED �̱� ������ RED�� �ؽ�Ʈ ���� �ʱ�ȭ 
	LCD_DisplayChar(5,9,'U');			// �ö󰡴� ��Ȳ�̱⿡ U ǥ��
	while(elevator->floor<Floor)			// ������������ �������� �Ű������� ���� �̵��ϰ��� �ϴ� ������ ���ѷ��� ����
	{				
		elevator->floor++;				//���������� ���� �� +1�� ���� 
		DelayMS(500);				// ������ 0.5��
		elevator->RectHeight+=13;		// �ö󰡴� ��Ȳ�̱⿡ �̵������� ���̴� ���� (13�� ����)
		elevator->YPos-=13;			// �ö󰡴� ��Ȳ�̱⿡ �̵������� Y��ǥ�� ���� (13�� ����)
		if(elevator->element =='L')		// ���� �������������� ������ �������������� Ȯ���� �� �ش� ��ǥ�� ���� �̵����� ���÷���
			LCD_DrawFillRect(28,elevator->YPos,8,elevator->RectHeight);
		else 
			LCD_DrawFillRect(113,elevator->YPos,8,elevator->RectHeight);
	}
	//�̵����밡 �� �̵� �� �� 
	if(Floor==Start_Floor)                           	//���� �Ű������� ���� ���ڰ� ���� ���̶�� 
	{
		DelayMS(500);				// ������ 0.5�� ��
		LCD_DisplayChar(5,9,'S');		// U-> S��  �ٲ�
		DelayMS(500);				// �ٽ� ������ 0.5�� (���������� ��������� �̵��ѵ� ���� �Լ��� ���ڷ� ��ǥ���� ������
								//  S�� �ٲ�� 0.5�� �ڿ� U�� �ٲ�� ������ ���⼭ ������) ���⼭ ������ 0.5�ʸ� �־��⿡ �Լ� ���ۺκп��� �ٷ� U�� ���÷��� ���� 						
	}
	else if(Floor==Dst_Floor)			// ���� �Ű������� ���� ���ڰ� ��ǥ ���̶��
	{
		DelayMS(500);				// ������ 0.5�� ��
		LCD_DisplayChar(5,9,'S');               // U-> S��  �ٲ� 
		EL_Fram_Wirte(elevator);
	}	
}

void DownFloor(Elevator* elevator, int Floor)
{
	LCD_SetBrushColor(elevator->Color);          //�̵� ������ ������ ���ڷ� ���� ������������ ���� ���� ��� 
	LCD_SetTextColor(elevator->Color);                   //GLCD�� L-E Ȥ�� R-E ǥ���Ҷ� ���������� ���ڷ� ���� ������������ ���� ���� ��� 
	LCD_DisplayText(4,8,elevator->Name);          // ������������ Name ������ ���� Display 
	LCD_SetTextColor(RGB_RED);                    	// ���� �̵� ����ǥ�ô� RED �̱� ������ RED�� �ؽ�Ʈ ���� �ʱ�ȭ  
	LCD_DisplayChar(5,9,'D');                          // �ö󰡴� ��Ȳ�̱⿡ D ǥ��
	while(elevator->floor>Floor)                      // ������������ �������� �Ű������� ���� �̵��ϰ��� �ϴ� ������ ���ѷ��� ����
	{	
		elevator->floor--;                           //���������� ���� ��  1�� ���� 
		DelayMS(500);                                 // ������ 0.5�� 
		elevator->RectHeight-=13;                // �������� ��Ȳ�̱⿡ �̵������� ���̴� ���� (13�� ����)
		elevator->YPos+=13;                       // �������� ��Ȳ�̱⿡ �̵������� Y��ǥ�� ���� (13�� ����)
		if(elevator->element =='L')                // ���� �������������� ������ �������������� Ȯ���� �� �ش� ��ǥ�� ���� �̵����� ���÷���
		{
			LCD_SetBrushColor(RGB_WHITE);		// �̵������� �������� ������ ǥ���ϱ� ���� ������ ���� ������� �̵����븦 �׸���
			LCD_DrawFillRect(28,20,8,70);		// �̵��� �̵����� ���÷��� 
			LCD_SetBrushColor(RGB_BLUE);		// ���� ���������� �̱⿡ �Ķ���
			LCD_DrawFillRect(28,elevator->YPos,8,elevator->RectHeight);
		}
		else
		{
			LCD_SetBrushColor(RGB_WHITE);
			LCD_DrawFillRect(113,20,8,70);	
			LCD_SetBrushColor(RGB_GREEN);             // ������ ���������� �̱⿡ �ʷϻ�
			LCD_DrawFillRect(113,elevator->YPos,8,elevator->RectHeight);	
		}		
	}
	if(Floor==Start_Floor)                                   //�̵����밡 �� �̵� �� �� 
	{                                                             //���� �Ű������� ���� ���ڰ� ���� ���̶�� 
		DelayMS(500);                                     // ������ 0.5�� �� 
		LCD_DisplayChar(5,9,'S');                       // D-> S��  �ٲ�  
		DelayMS(500);                                     // �ٽ� ������ 0.5�� 
	}                                                             
	else if(Floor==Dst_Floor)                              // ���� �Ű������� ���� ���ڰ� ��ǥ ���̶��
	{                                                             
		DelayMS(500);                                     // ������ 0.5�� �� 
		LCD_DisplayChar(5,9,'S');                      // D-> S��  �ٲ�  
		EL_Fram_Wirte(elevator);
	}
}

void EXTI9_5_IRQHandler(void)	
{
	if(EXTI->PR & 0x0100)
	{
		EXTI->PR |= 0x0100;
		EXTI->IMR  &= ~0x0100 ;            		// ä�͸� ������ ���� ���� EXTI8 �� �ٽ� mask ������
		if(EX_St_Flag&&EX_Dst_Flag)			// �����, ��ǥ�� �Ѵٸ� �Է��ؾ߸� ������ ���� ���� 
		{
			HD_Flag=1;					// �ߴܸ�尡 ������ �� �ִ� Flag ���� �ʱ�ȭ
			GPIOG->ODR&=~0x00FF;		// ��� LED OFF ��
			GPIOG->ODR|=0x0001;			// LED0 ON
			BEEP();	
			LCD_DisplayText(2,8,"EX");		// FL -> EX ���÷���	
			ExMode();					// ���� ��� �Լ� ȣ�� 
			LCD_DisplayText(2,8,"FL");		// ���������� �̵� �� ������ ���� ���ư��� ���� EX -> FL ���÷���
			GPIOG->ODR&=~0x0001;		// ��� LED OFF ��
			GPIOG->ODR|=0x0080;			// LED7 ON
			DelayMS(300);   			
			BEEP();            //���� 1ȸ 
			DelayMS(300);
			BEEP();            //���� 1ȸ 
			DelayMS(300);
			BEEP();  	      //���� 1ȸ 
			HD_Flag=0;				      // ����3ȸ �� LED7 �� �����⿡ ������ ���� -> �ߴܸ�� ���� ���ϰ� Flag ���� �ٽ� 0���� �ʱ�ȭ 
			EX_St_Flag=0;			      // �ٽ� �����, ��ǥ���� �Ѵ� �Է��� �� �����尡 �Ǳ� ���� Flag ���� 0���� �ʱ�ȭ
			EX_Dst_Flag=0;
		}
		EXTI->IMR  |= 0x8100 ; 		     // �ڵ鷯 �Լ��� ������  not mask 
	}
}
void EXTI15_10_IRQHandler(void)	
{
	if(EXTI->PR & 0x8000)
	{
		EXTI->PR |= 0x8000;
		EXTI->IMR  &= ~0x8000 ; 		  // ä�͸� ������ ���� ���� EXTI15 �� �ٽ� mask ������
		if(HD_Flag)					 // ���� �ߴܸ�� Flag�� 1�̶�� �ߴܸ�� ���� 
		{
			LCD_DisplayText(2,8,"HD");	// HD�� ���÷��� 
			GPIOG->ODR &=~0x00FF;	// ��� LED OFF
			GPIOG->ODR |= 0x0040;	// LED6 ON		
			for(int i=0;i<10;i++)
			{					//5�ʰ� 0.5�� �������� ������ ����� �� --> 0.5�ʸ� 10�� �ϸ� 5�ʰ� �Ǳ⿡ 0.5�� �����̿� �Բ� 10�� �ݺ�
				BEEP();
				DelayMS(500);
			}
			LCD_DisplayText(2,8,"EX");	//�ߴ� ��� ���� �� �ٽ� EX ���÷��� 
			GPIOG->ODR &=~0x0040;	//��� LED OFF
			GPIOG->ODR |= 0x0001;	// �ٽ� LED0 ON
		}
		EXTI->IMR  |= 0x8100 ; 		// �ڵ鷯 �Լ��� ������  not mask 
	}
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
}
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOG Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH  Input mode (reset state)	
	
	SYSCFG->EXTICR[2] |= 0x0007;  //SW0 EXTI8
	SYSCFG->EXTICR[3] |= 0x7000;  //SW7 EXTI15
	
	EXTI->FTSR |= 0x8100; //SW�� ������ ���� ���۵Ǳ� ����      
	EXTI->IMR  |= 0x8100 ; 		
	
	NVIC->ISER[0] |= (1<<23); //EXTI9_5 �������̺�23��
	NVIC->ISER[1] |= (1<<8); //EXTI15_10 �������̺� 40��
	
	//  EXTI8 ������ EXTI15�� �߻� �ؾ��Ѵ�. ������ IRQ no �� EXTI15�� ���� S/W �� priority ���� 0���� ���⿡  �߰��� Nested ���� ����
	NVIC->IP[23]= 0xF0;  	//  EXTI8 ������ EXTI15�� �߻��ϱ� ����(Nested) IP�������͸� ���� S/W ������  priority�� ���� 
	NVIC->IP[40]= 0xE0; 	// �̻��¿����� EXTI15_10�� EXTI9_5���� �켱������ ���� Nested �� 
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