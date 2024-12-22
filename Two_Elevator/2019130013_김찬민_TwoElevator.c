/////////////////////////////////////////////////////////////
// 과제명: TP1. Two Elevators
// 과제개요:   -6층 건물에 두대의 엘리베이터(L_Elevator, R_Elevator)를 설치
//		-출발층에서 목표층 스위치를 선택하면 엘리베이터가 이동
//		-이동은 막대 크기의 변화로 표시	
//		-  FL 모드에서 출발층, 목표층을 적어도 한번씩이라도 선택을 해야 EX 모드 동작하도록 구현 
// 사용한 하드웨어(기능): GPIO, SW , LED, BUZZER , EXTI ,FRAM
// 제출일: 2023. 6.03
// 제출자 클래스: 목요일반
// 학번: 2019130013
// 이름: 김찬민     
//////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "FRAM.h"
typedef struct structelevator    // 엘리베이터 구조체 선언
{
	int floor;   		  // 엘리베이터의 현재층 변수
	int YPos;  		  // 엘리베이터 이동 막대의 Y좌표 변수
	int RectHeight;     // 엘리베이터 이동 막대의 세로 높이 변수
	char element; 	  // 어떤 엘리베이터 인지 판단 하기 위한 변수
	char* Name;        // GLCD상 어떤 엘리베이터인지 Display 할 문자열 변수 
	UINT32 Color;      // 엘리베이터의 색상 변수
}Elevator;

Elevator L_Elevator ={1,93,12,'L',"L-E",RGB_BLUE};     //왼쪽 엘리베이터 객체 생성 및 초기화   // 93, 12,는 1층 기준 엘리베이터의 Y좌표및 세로높이
Elevator R_Elevator={1,93,12,'R',"R-E",RGB_GREEN};   //오른쪽 엘리베이터 객체 생성 및 초기화  // 층수는 1로 초기화 (이 초기화한 층은 FRAM 사용 하지 않을 시 이용)

void _GPIO_Init(void);
void _EXTI_Init(void);
uint16_t KEY_Scan(void);
void BEEP(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

void FirstDisplay(void);    					//최초 GLCD에 Display하는 함수
void ExMode(void);        				 	// EXTI8 발생시 실행모드로 동작하는 함수
void Fram_Elevator(Elevator* elevator);  		//  Flash 메모리로 부트모드 동작시 
									//FRAM 에 저장된 현재층 정보를 각 엘리베이터의 정보로 입력하는 함수
void UpFloor(Elevator* elevator, int Floor); 		// 엘리베이가 올라가는 동작을 수행하는 함수 
void DownFloor(Elevator* elevator, int Floor); 	// 엘리베이터가 내려가는 동작을 수행하는 함수
									// int Floor 매개변수로 내가 출발층으로 가는지, 목표층으로 가는지 결정
void Compare_Floor(Elevator* elevator);    // 현재층, 출발층, 목표층을 비교하는 함수
void EL_Fram_Wirte(Elevator* elevator);	//해당 엘리베이터의 목표층을 Fram_Write 하는 함수

int Start_Floor;       // 출발층 입력 변수  
int Dst_Floor;         // 목표층   입력 션수 
int HD_Flag;              // 실행모드 중에만 중지모드가 동작 될 수 있도록 하기 위한 Flag 변수 
int EX_St_Flag;          // 출발층과 목표층을 모두 선택해야만 실행모드가 수행 될 수 있도록 하기 위한 변수 
int EX_Dst_Flag;        // 만약 실행 모드가 수행 되어 LCD에 출발층 < 목표층 정보가 있더라도 출발층, 목표층(둘다 모두)을 항상 입력해야만 실행모드 수행 가능 

int main(void)
{
	LCD_Init();
	DelayMS(10);
	_GPIO_Init();
	_EXTI_Init();
	Fram_Init();            	   // FRAM 초기화 H/W 초기화
	Fram_Status_Config();   // FRAM 초기화 S/W 초기화
	
	GPIOG->ODR &= ~0x00FF;   //초기 모든 LED OFF 시킨 후
	GPIOG->ODR |= 0x0080;      // LED7 만 ON
	
	//Fram_Write(2023,'1');
	//Fram_Write(2024,'1');
	Fram_Elevator(&L_Elevator);   // 위에서 선언한 왼쪽 오른쪽 엘리베이터에 FRAM에 저장한 층 정보를 저장 하기 위한 함수 호출
	Fram_Elevator(&R_Elevator);
	FirstDisplay();			    // 초기 GLCD Display 함수 호출 (FRAM 정보에 따라 초기 화면이 달라지기에 
						   //  Fram_Elevator() 로 FRAM 정보를 엘리베이터 가져온후 각 정보에 따라 Display하기 위해 
						   //  디스플레이 함수를 나중에 호출 )
	
	Start_Floor=L_Elevator.floor;		// FRAM 사용시 초기 LCD의 출발층 < 목표층 은 FRAM 2023 번지에서 read 한 값, 여기서 SW1, SW2번을 누르면 표시된 값에서 +1씩 증가하기 때문에 
	Dst_Floor=L_Elevator.floor;		// 목표층에도 우선 왼쪽 엘리베이터 정보를 넘겨줌 , 또한 FRAM을 사용하지 않는 경우에도 L_Elevator.floor은 1로 초기화 되어있기 때문에 조건 부합
	
	while(1)
	{
		switch(KEY_Scan())
		{
		case 0xFD00 :   //SW1 : 출발층 설정
			GPIOG->ODR |=   0x0002;      // LED 1 ON
			GPIOG->ODR &= ~0x0004;   	// LED 2 OFF
			LCD_SetTextColor(RGB_RED);  
			BEEP();
			if(Start_Floor==6)          // 출발층이 6이면 0으로 초기화
				Start_Floor=0;
			Start_Floor++;		 // SW 누를때마다 출발층 +1 씩 증가
			LCD_DisplayChar(6,8,Start_Floor+'0');	  // GLCD에 출발층 표시(int to char 위해 int+'0')
			EX_St_Flag=1;  		 // 출발층 선택했다면 EX_St_Flag = 1
			break;
			
		case 0xFB00 :    //SW2 : 출발층 설정
			GPIOG->ODR |=   0x0004;	     // LED 2 ON
			GPIOG->ODR &= ~0x0002;    // LED 1 OFF
			LCD_SetTextColor(RGB_RED);
			BEEP();
			if(Dst_Floor==6)           // 목표층이 6이면 0으로 초기화
				Dst_Floor=0;        
			Dst_Floor++;                 // SW 누를때마다 목표층 +1 씩 증가 
			LCD_DisplayChar(6,10,Dst_Floor+'0');   // GLCD에 목표층 표시(int to char 위해 int+'0')
			EX_Dst_Flag=1;          // 목표층 선택했다면 EX_Dst_Flag = 1
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
	if(elevator->element=='L')     // 만약 왼쪽 엘리베이터 라면 
		elevator->floor=Fram_Read(2023)-'0';    // FRAM 2023번지에 저장된 층 정보를 왼쪽 엘리베이터 현재층에 대입
	
	else if(elevator->element=='R')    // 만약 오른쪽 엘리베이터 라면
		elevator->floor=Fram_Read(2024)-'0';    // FRAM 2024번지에 저장된 층 정보를 오른쪽 엘리베이터 현재층에 대입
	
	elevator->YPos=92-(13*((elevator->floor)-1));   // 초기 엘리베이터를 DIsplay 하기위한 이동 막대의 Y좌표,
	elevator->RectHeight=(13*((elevator->floor)));    // 이동막대의 세로 높이길이를 현재 층에 따라 초기화 
}
void FirstDisplay(void)
{
	LCD_Clear(RGB_WHITE);     //초기 LCD화면 초기화 
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
	LCD_DisplayChar(6,8,L_Elevator.floor+'0');    // FRAM의 2023번지 즉 왼쪽 엘리베이터의 현재 층을 DIsplay   // FRAM 사용 하지 않을 시 1 Display
	LCD_DisplayChar(6,10,L_Elevator.floor+'0');  // FRAM의 2023번지 즉 왼쪽 엘리베이터의 현재 층을 DIsplay
	LCD_SetTextColor(RGB_BLACK);
	LCD_DisplayChar(6,9,'>');
	LCD_SetBrushColor(RGB_BLUE);
	LCD_DrawFillRect(28,L_Elevator.YPos,8,L_Elevator.RectHeight);     
	LCD_SetBrushColor(RGB_GREEN);
	LCD_DrawFillRect(113,R_Elevator.YPos,8,R_Elevator.RectHeight);	
}

void ExMode(void)
{	
	if(abs(L_Elevator.floor-Start_Floor)<=abs(R_Elevator.floor-Start_Floor))        //왼쪽 엘리베이터의 현재층이 오른쪽 엘리베이터 보다 출발층과 가까운 경우
	{	
		Compare_Floor(&L_Elevator);   // 세부적으로 층별 비교를 하기 위한 함수 호출(이때 인자는 왼쪽 엘리베이터)
	}
	else if(abs(L_Elevator.floor-Start_Floor)>abs(R_Elevator.floor-Start_Floor))   //오른쪽 엘리베이터의 현재층이 왼쪽 엘리베이터 보다 출발층과 가까운 경우
	{	
		Compare_Floor(&R_Elevator);   // 세부적으로 층별 비교를 하기 위한 함수 호출(이때 인자는 오른쪽 엘리베이터)
	}
}

void Compare_Floor(Elevator* elevator)
{
	if(elevator->floor<Start_Floor)          		//현재 층이 출발층 보다 작은 경우 (현재층 -> 출발층 UP)
	{		
		if(Start_Floor==Dst_Floor)        		// 출발층과 목표층이 같은 경우 
		{
			UpFloor(elevator,Start_Floor);  	 //엘리베이터는 현재층 -> 출발층 까지 동작 후 1초후 종료
			DelayMS(500);			     	// UpFloor()함수 종료 직전 DelayMS(500)를 호출하기에 1초를 맞추기위해 한번 더 DelayMS(500) 호출
			EL_Fram_Wirte(elevator);
		}
		else					   	 // 출발층과 목표층이 다른 경우	
		{
			UpFloor(elevator,Start_Floor);  // 우선 현재층부터 출발층 까지 UP   
			
			if(Start_Floor<Dst_Floor)         // 여기서 만약 목표층이 출발층보다 크다면 출발층 -> 목표층 UP
				UpFloor(elevator,Dst_Floor); 	
			else if(Start_Floor>Dst_Floor)  // 여기서 만약 목표층이 출발층보다 작다면 출발층 -> 목표층 DOWN
				DownFloor(elevator,Dst_Floor);
		}
	}
	else if(elevator->floor>Start_Floor)   //현재 층이 출발층 보다 큰 경우 (현재층 -> 출발층 DOWN)
	{
		if(Start_Floor==Dst_Floor)        // 출발층과 목표층이 같은 경우 
		{
			DownFloor(elevator,Start_Floor);    //엘리베이터는 현재층 -> 출발층 까지 동작 후 1초후 종료
			DelayMS(500);                             // DownFloor()함수 종료 직전 DelayMS(500)를 호출하기에 1초를 맞추기위해 한번 더 DelayMS(500) 호출
			EL_Fram_Wirte(elevator);
		}
		else 				          // 출발층과 목표층이 다른 경우	
		{
			DownFloor(elevator,Start_Floor);   //우선 현재층부터 출발층 까지 DOWN
			
			if(Start_Floor<Dst_Floor)  	// 여기서 만약 목표층이 출발층보다 크다면 출발층 -> 목표층 UP
				UpFloor(elevator,Dst_Floor);
			
			else if(Start_Floor>Dst_Floor)	// 여기서 만약 목표층이 출발층보다 작다면 출발층 -> 목표층 DOWN
				DownFloor(elevator,Dst_Floor);
		}
	}	
	else if(elevator->floor==Start_Floor)   //현재 층과 출발층이 같은 경우 
	{
		DelayMS(1000);                      // 1초 딜레이후 실행 
		if(Start_Floor<Dst_Floor)           // 목표층이 출발층보다 높은 경우 출발층 -> 목표층 UP
			UpFloor(elevator,Dst_Floor);	
		
		else if(Start_Floor>Dst_Floor)     // 목표층이 출발층보다 높은 경우 출발층 -> 목표층 UP
			DownFloor(elevator,Dst_Floor);	
		else
		{
			DelayMS(1000);		      // 현재층 == 출발층 == 목표층 인경우
			EL_Fram_Wirte(elevator);		
		}
	}
}

void UpFloor(Elevator* elevator, int Floor)
{
	LCD_SetBrushColor(elevator->Color);      //이동 막대의 색상은 인자로 받은 엘리베이터의 색상 변수 사용 
	LCD_SetTextColor(elevator->Color);	       //GLCD에 L-E 혹은 R-E 표시할때 색상정보를 인자로 받은 엘리베이터의 색상 변수 사용 
	LCD_DisplayText(4,8,elevator->Name);     // 엘리베이터의 Name 변수를 통해 Display
	LCD_SetTextColor(RGB_RED);			// 현재 이동 상태표시는 RED 이기 때문에 RED로 텍스트 색상 초기화 
	LCD_DisplayChar(5,9,'U');			// 올라가는 상황이기에 U 표시
	while(elevator->floor<Floor)			// 엘리베이터의 현재층이 매개변수로 받은 이동하고자 하는 층까지 무한루프 실행
	{				
		elevator->floor++;				//엘리베이터 현재 층 +1씩 증가 
		DelayMS(500);				// 딜레이 0.5초
		elevator->RectHeight+=13;		// 올라가는 상황이기에 이동막대의 높이는 증가 (13씩 증가)
		elevator->YPos-=13;			// 올라가는 상황이기에 이동막대의 Y좌표는 감소 (13씩 감소)
		if(elevator->element =='L')		// 왼쪽 엘리베이터인지 오른쪽 엘리베이터인지 확인한 뒤 해당 좌표에 정보 이동막대 디스플레이
			LCD_DrawFillRect(28,elevator->YPos,8,elevator->RectHeight);
		else 
			LCD_DrawFillRect(113,elevator->YPos,8,elevator->RectHeight);
	}
	//이동막대가 다 이동 한 후 
	if(Floor==Start_Floor)                           	//만약 매개변수로 받은 인자가 현재 층이라면 
	{
		DelayMS(500);				// 딜레이 0.5초 후
		LCD_DisplayChar(5,9,'S');		// U-> S로  바뀜
		DelayMS(500);				// 다시 딜레이 0.5초 (현재층에서 출발층으로 이동한뒤 만약 함수의 인자로 목표층이 들어오면
								//  S로 바뀌고 0.5초 뒤에 U로 바뀌기 때문에 여기서 딜레이) 여기서 딜레이 0.5초를 주었기에 함수 시작부분에서 바로 U로 디스플레이 가능 						
	}
	else if(Floor==Dst_Floor)			// 만약 매개변수로 받은 인자가 목표 층이라면
	{
		DelayMS(500);				// 딜레이 0.5초 후
		LCD_DisplayChar(5,9,'S');               // U-> S로  바뀜 
		EL_Fram_Wirte(elevator);
	}	
}

void DownFloor(Elevator* elevator, int Floor)
{
	LCD_SetBrushColor(elevator->Color);          //이동 막대의 색상은 인자로 받은 엘리베이터의 색상 변수 사용 
	LCD_SetTextColor(elevator->Color);                   //GLCD에 L-E 혹은 R-E 표시할때 색상정보를 인자로 받은 엘리베이터의 색상 변수 사용 
	LCD_DisplayText(4,8,elevator->Name);          // 엘리베이터의 Name 변수를 통해 Display 
	LCD_SetTextColor(RGB_RED);                    	// 현재 이동 상태표시는 RED 이기 때문에 RED로 텍스트 색상 초기화  
	LCD_DisplayChar(5,9,'D');                          // 올라가는 상황이기에 D 표시
	while(elevator->floor>Floor)                      // 엘리베이터의 현재층이 매개변수로 받은 이동하고자 하는 층까지 무한루프 실행
	{	
		elevator->floor--;                           //엘리베이터 현재 층  1씩 감소 
		DelayMS(500);                                 // 딜레이 0.5초 
		elevator->RectHeight-=13;                // 내려가는 상황이기에 이동막대의 높이는 감소 (13씩 감소)
		elevator->YPos+=13;                       // 내려가는 상황이기에 이동막대의 Y좌표는 증가 (13씩 증가)
		if(elevator->element =='L')                // 왼쪽 엘리베이터인지 오른쪽 엘리베이터인지 확인한 뒤 해당 좌표에 정보 이동막대 디스플레이
		{
			LCD_SetBrushColor(RGB_WHITE);		// 이동막대의 내려가는 동작을 표현하기 위해 배경색과 같은 흰색으로 이동막대를 그린뒤
			LCD_DrawFillRect(28,20,8,70);		// 이동한 이동막대 디스플레이 
			LCD_SetBrushColor(RGB_BLUE);		// 왼쪽 엘리베이터 이기에 파란색
			LCD_DrawFillRect(28,elevator->YPos,8,elevator->RectHeight);
		}
		else
		{
			LCD_SetBrushColor(RGB_WHITE);
			LCD_DrawFillRect(113,20,8,70);	
			LCD_SetBrushColor(RGB_GREEN);             // 오른쪽 엘리베이터 이기에 초록색
			LCD_DrawFillRect(113,elevator->YPos,8,elevator->RectHeight);	
		}		
	}
	if(Floor==Start_Floor)                                   //이동막대가 다 이동 한 후 
	{                                                             //만약 매개변수로 받은 인자가 현재 층이라면 
		DelayMS(500);                                     // 딜레이 0.5초 후 
		LCD_DisplayChar(5,9,'S');                       // D-> S로  바뀜  
		DelayMS(500);                                     // 다시 딜레이 0.5초 
	}                                                             
	else if(Floor==Dst_Floor)                              // 만약 매개변수로 받은 인자가 목표 층이라면
	{                                                             
		DelayMS(500);                                     // 딜레이 0.5초 후 
		LCD_DisplayChar(5,9,'S');                      // D-> S로  바뀜  
		EL_Fram_Wirte(elevator);
	}
}

void EXTI9_5_IRQHandler(void)	
{
	if(EXTI->PR & 0x0100)
	{
		EXTI->PR |= 0x0100;
		EXTI->IMR  &= ~0x0100 ;            		// 채터링 현상을 막기 위해 EXTI8 만 다시 mask 씌워줌
		if(EX_St_Flag&&EX_Dst_Flag)			// 출발층, 목표층 둘다를 입력해야만 실행모드 실행 가능 
		{
			HD_Flag=1;					// 중단모드가 실행할 수 있는 Flag 변수 초기화
			GPIOG->ODR&=~0x00FF;		// 모든 LED OFF 후
			GPIOG->ODR|=0x0001;			// LED0 ON
			BEEP();	
			LCD_DisplayText(2,8,"EX");		// FL -> EX 디스플레이	
			ExMode();					// 실행 모드 함수 호출 
			LCD_DisplayText(2,8,"FL");		// 엘리베이터 이동 후 층선택 모드로 돌아가기 위해 EX -> FL 디스플레이
			GPIOG->ODR&=~0x0001;		// 모든 LED OFF 후
			GPIOG->ODR|=0x0080;			// LED7 ON
			DelayMS(300);   			
			BEEP();            //부저 1회 
			DelayMS(300);
			BEEP();            //부저 1회 
			DelayMS(300);
			BEEP();  	      //부저 1회 
			HD_Flag=0;				      // 부저3회 및 LED7 이 켜졌기에 실행모드 종료 -> 중단모드 실행 못하게 Flag 변수 다시 0으로 초기화 
			EX_St_Flag=0;			      // 다시 출발층, 목표층을 둘다 입력한 뒤 실행모드가 되기 위해 Flag 변수 0으로 초기화
			EX_Dst_Flag=0;
		}
		EXTI->IMR  |= 0x8100 ; 		     // 핸들러 함수가 끝나면  not mask 
	}
}
void EXTI15_10_IRQHandler(void)	
{
	if(EXTI->PR & 0x8000)
	{
		EXTI->PR |= 0x8000;
		EXTI->IMR  &= ~0x8000 ; 		  // 채터링 현상을 막기 위해 EXTI15 만 다시 mask 씌워줌
		if(HD_Flag)					 // 만약 중단모드 Flag가 1이라면 중단모드 실행 
		{
			LCD_DisplayText(2,8,"HD");	// HD로 디스플레이 
			GPIOG->ODR &=~0x00FF;	// 모든 LED OFF
			GPIOG->ODR |= 0x0040;	// LED6 ON		
			for(int i=0;i<10;i++)
			{					//5초간 0.5초 간격으로 부저가 울려야 함 --> 0.5초를 10번 하면 5초가 되기에 0.5초 딜레이와 함께 10번 반복
				BEEP();
				DelayMS(500);
			}
			LCD_DisplayText(2,8,"EX");	//중단 모드 종료 후 다시 EX 디스플레이 
			GPIOG->ODR &=~0x0040;	//모든 LED OFF
			GPIOG->ODR |= 0x0001;	// 다시 LED0 ON
		}
		EXTI->IMR  |= 0x8100 ; 		// 핸들러 함수가 끝나면  not mask 
	}
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
}
void _EXTI_Init(void)
{
	RCC->AHB1ENR 	|= 0x00000080;	// RCC_AHB1ENR GPIOG Enable
	RCC->APB2ENR 	|= 0x00004000;	// Enable System Configuration Controller Clock
	
	GPIOH->MODER 	&= ~0xFFFF0000;	// GPIOH  Input mode (reset state)	
	
	SYSCFG->EXTICR[2] |= 0x0007;  //SW0 EXTI8
	SYSCFG->EXTICR[3] |= 0x7000;  //SW7 EXTI15
	
	EXTI->FTSR |= 0x8100; //SW가 누르자 마자 동작되기 위해      
	EXTI->IMR  |= 0x8100 ; 		
	
	NVIC->ISER[0] |= (1<<23); //EXTI9_5 백터테이블23번
	NVIC->ISER[1] |= (1<<8); //EXTI15_10 백터테이블 40번
	
	//  EXTI8 실행중 EXTI15가 발생 해야한다. 하지만 IRQ no 는 EXTI15가 낮고 S/W 적 priority 또한 0으로 같기에  중간에 Nested 되지 못함
	NVIC->IP[23]= 0xF0;  	//  EXTI8 실행중 EXTI15가 발생하기 위해(Nested) IP레지스터를 통해 S/W 적으로  priority를 지정 
	NVIC->IP[40]= 0xE0; 	// 이상태에서는 EXTI15_10이 EXTI9_5보다 우선순위가 높아 Nested 됨 
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