//////////////////////////////////////////////////////////////////////////
// HW4. 가속도값(SPI)을 이용한 Ball game//  NSS pin:  PA8 (PA4(SPI1_CS) 대신에 사용)
//  제출자 : 2019130013   김찬민 
//  주요 내용 및 구현 내용 
//  - -키트를 기울임에 따라 가속도 값이 변함. 
// 가속도 크기와 방향에 따라공이 이동하도록 표시. 기울기 각도에 따라 이동속도를 변경
// - 추가적인 타이머 6를 추가해 ARR 값을 바꿔주면서 볼의 이동속도 제어 
//////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "GLCD.h"
#include "ACC.h"

void DisplayTitle(void);
void _GPIO_Init(void);
void SPI1_Init(void);
void TIMER10_Init(void);
void TIMER6_Init(void);
void Display_Process(int16 *pBuf);
void Display_Acc();
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);
void clear_rec(void);			// 볼 clear 함수 

const double RADIAN_TO_DEGREE =180/3.14159; // Raduab -> Degree
UINT8 bControl;						  // 가속도 값 적용 시작 flag 
UINT16 accx,accy,accz;					// 가속도 변수
double anglex,angley;					// x,y 각도 변수
uint8_t prv_x=52,prv_y=64,cur_x,cur_y;		// 이전 좌표 , 현재 좌표 


int minus_x,minus_y;	// 음수 falg 변수
int init_flag,chnage_pos_flag;	// 시작 flag 변수 
int abs_angle;			// 각도 절댓삾 
int tmp_cur_x,tmp_cur_y;		// 임시 저장 좌표 

int main(void)
{
  int16 buffer[3];
  
  LCD_Init();		// LCD 구동 함수
  DelayMS(10);		// LCD구동 딜레이
  DisplayTitle();		// LCD 초기화면구동 함수
  
  _GPIO_Init();		// LED, SW 초기화
  SPI1_Init();        	// SPI1 초기화
  
  ACC_Init();		// 가속도센서 초기화
  
  TIMER10_Init();		// 가속도센서 스캔 주기 생성
  TIMER6_Init();
  
  while(1)
  {
    if(bControl)
    {
      bControl = FALSE;     
      SPI1_Process(&buffer[0]);	
      Display_Process(&buffer[0]);
      init_flag=1;
      
      if(abs((int)anglex)/10 >=abs((int)angley)/10)		// x,y 축 각도중 각도 값이 큰 축에 따라 움직임 
        abs_angle=abs((int)anglex)/10 ;
      else
        abs_angle=abs((int)angley)/10 ;
      
      /************각도에 따른 이동속도 ( TIM6 -> ARR) *************/
      switch(abs_angle)						
      { 
      case 0 : 
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 9500-1;
          TIM6->CR1 |= (1<<0);
        }                                                       
      case 1 : 
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 7000-1;
          TIM6->CR1 |= (1<<0);
        }                                   
      case 2 : 
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 6000-1;
          TIM6->CR1 |= (1<<0);
        }
        break;
      case 3 :                                 
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 5000-1;
          TIM6->CR1 |= (1<<0);
        }
        break;
      case 4 :                                  
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 4000-1;
          TIM6->CR1 |= (1<<0);
        } 
        break;
      case 5 :                                  
        {
          
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 3000-1;
          TIM6->CR1 |= (1<<0);
        }
        break;
      case 6 :                                 
        {
          
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 2000-1;
          TIM6->CR1 |= (1<<0);
        }
        break;
      case 7 :                        
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 1000-1;
          TIM6->CR1 |= (1<<0);
        }
        break;
      case 8 :                            
        {
          TIM6->CR1 &= ~(1<<0);
          TIM6->CNT =0;
          TIM6->ARR = 500-1;
          TIM6->CR1 |= (1<<0);
        }
        break;    
      }
            /*************************************/
    }
  }
}

///////////////////////////////////////////////////////
// Master mode, Full-duplex, 8bit frame(MSB first), 
// fPCLK/32 Baud rate, Software slave management EN
void SPI1_Init(void)
{
  /*!< Clock Enable  *********************************************************/
  RCC->APB2ENR 	|= (1<<12);	// 0x1000, SPI1 Clock EN
  RCC->AHB1ENR 	|= (1<<0);	// 0x0001, GPIOA Clock EN		
  
  /*!< SPI1 pins configuration ************************************************/
  
  /*!< SPI1 NSS pin(PA8) configuration : GPIO 핀  */
  GPIOA->MODER 	|= (1<<(2*8));	// 0x00010000, PA8 Output mode
  GPIOA->OTYPER 	&= ~(1<<8); 	// 0x0100, push-pull(reset state)
  GPIOA->OSPEEDR 	|= (3<<(2*8));	// 0x00030000, PA8 Output speed (100MHZ) 
  GPIOA->PUPDR 	&= ~(3<<(2*8));	// 0x00030000, NO Pullup Pulldown(reset state)
  
  /*!< SPI1 SCK pin(PA5) configuration : SPI1_SCK */
  GPIOA->MODER 	|= (2<<(2*5)); 	// 0x00000800, PA5 Alternate function mode
  GPIOA->OTYPER 	&= ~(1<<5); 	// 0020, PA5 Output type push-pull (reset state)
  GPIOA->OSPEEDR 	|= (3<<(2*5));	// 0x00000C00, PA5 Output speed (100MHz)
  GPIOA->PUPDR 	|= (2<<(2*5)); 	// 0x00000800, PA5 Pull-down
  GPIOA->AFR[0] 	|= (5<<(4*5));	// 0x00500000, Connect PA5 to AF5(SPI1)
  
  /*!< SPI1 MOSI pin(PA7) configuration : SPI1_MOSI */    
  GPIOA->MODER 	|= (2<<(2*7));	// 0x00008000, PA7 Alternate function mode
  GPIOA->OTYPER	&= ~(1<<7);	// 0x0080, PA7 Output type push-pull (reset state)
  GPIOA->OSPEEDR 	|= (3<<(2*7));	// 0x0000C000, PA7 Output speed (100MHz)
  GPIOA->PUPDR 	|= (2<<(2*7)); 	// 0x00008000, PA7 Pull-down
  GPIOA->AFR[0] 	|= (5<<(4*7));	// 0x50000000, Connect PA7 to AF5(SPI1)
  
  /*!< SPI1 MISO pin(PA6) configuration : SPI1_MISO */
  GPIOA->MODER 	|= (2<<(2*6));	// 0x00002000, PA6 Alternate function mode
  GPIOA->OTYPER 	&= ~(1<<6);	// 0x0040, PA6 Output type push-pull (reset state)
  GPIOA->OSPEEDR 	|= (3<<(2*6));	// 0x00003000, PA6 Output speed (100MHz)
  GPIOA->PUPDR 	|= (2<<(2*6));	// 0x00002000, PA6 Pull-down
  GPIOA->AFR[0] 	|= (5<<(4*6));	// 0x05000000, Connect PA6 to AF5(SPI1)
  
  // Init SPI1 Registers 
  SPI1->CR1 |= (1<<2);	// MSTR(Master selection)=1, Master mode
  SPI1->CR1 &= ~(1<<15);	// SPI_Direction_2 Lines_FullDuplex
  SPI1->CR1 &= ~(1<<11);	// SPI_DataSize_8bit
  SPI1->CR1 |= (1<<9);  	// SSM(Software slave management)=1, 
  // NSS 핀 상태가 코딩에 의해 결정
  SPI1->CR1 |= (1<<8);	// SSI(Internal_slave_select)=1,
  // 현재 MCU가 Master이므로 NSS 상태는 'High' 
  SPI1->CR1 &= ~(1<<7);	// LSBFirst=0, MSB transmitted first    
  SPI1->CR1 |= (4<<3);	// BR(BaudRate)=0b100, fPCLK/32 (84MHz/32 = 2.625MHz)
  SPI1->CR1 |= (1<<1);	// CPOL(Clock polarity)=1, CK is 'High' when idle
  SPI1->CR1 |= (1<<0);	// CPHA(Clock phase)=1, 두 번째 edge 에서 데이터가 샘플링
  
  SPI1->CR1 |= (1<<6);	// SPE=1, SPI1 Enable 
}

void TIMER6_Init(void)
{	
  RCC->APB1ENR |= (1<<4);	// RCC_APB1ENR TIMER6 Enable
  TIM6->CR1 &= ~(1<<4);  // DIR=0(Up counter)	
  TIM6->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
  //  Counter Overflow/Underflow, 
  //  Setting the UG bit Set,
  //  Update Generation through the slave mode controller 
  // UDIS=1 : Only Update event Enabled by  Counter Overflow/Underflow,
  TIM6->CR1 &= ~(1<<2);	// URS=0(Update Request Source  Selection):  By one of following events
  //	Counter Overflow/Underflow, 
  // Setting the UG bit Set,
  //	Update Generation through the slave mode controller 
  // URS=1 : Only Update Interrupt generated  By  Counter Overflow/Underflow,
  TIM6->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM6->CR1 |= (1<<7);	// ARPE=1(ARR is buffered): ARR Preload Enalbe 
  TIM6->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
  TIM6->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts UP and DOWN alternatively
  
  // Deciding the Period
  TIM6->PSC = 840-1;      // 84Mhz / 840 = (0.00001s) 
  TIM6->ARR = 10000-1;   //주기 0.00001s * 10000 = 100ms
  // Clear the Counter
  TIM6->EGR |= (1<<0);	// UG(Update generation)=1 
  // Re-initialize the counter(CNT=0) & generates an update of registers   
  
  // Setting an UI(UEV) Interrupt 
  NVIC->ISER[1] |= (1<<(54-32)); // Enable Timer6 global Interrupt
  TIM6->DIER |= (1<<0);	// Enable the Tim2 Update interrupt
  TIM6->CR1 |= (1<<0);	// Enable the Tim2 Counter (clock enable)   
}


void TIM6_DAC_IRQHandler(void)
{
  TIM6->SR &= ~(1<<0);	//타이머 인터럽트 Clear 
  
  if(init_flag==1)
  { 
    
    int abs_anglex=abs((int)anglex);	// x축 y축 크기비교 위해 절댓값으로 변환 
    int abs_angley=abs((int)angley);
      
    if(abs_angley>5)           	       // y 축으로 각도가 5도 보다 클 때 
    {
      if(chnage_pos_flag==1)		// 외벽과 만났는지 확인하는 flag 변수 
      {
        
        if(angley<=0)      		// 각도 값 음수면 y축 증가 
          cur_y+=1;			
        else if(angley>0)
          cur_y-=1;			// 각도 값 양수면 y축 감가 
        
        tmp_cur_y=cur_y;		// 임시 좌표 변수에 현재 좌표 저장 
      }
      else
      {
        if(angley<=0) 
          tmp_cur_y+=1;
        else if(angley>0)
          tmp_cur_y-=1;
      }    
    }
    
    if(abs_anglex>5)               	// 외벽과 만났는지 확인하는 flag 변수 
    {                                  
      if(chnage_pos_flag==1)     
      {                                // 각도 값 음수면 x축 증가 
                                         
        if(anglex<0)                 
          cur_x+=1;                 // 각도 값 양수면 x축 감가 
        else if(anglex>=0)         
          cur_x-=1;                 // 임시 좌표 변수에 현재 좌표 저장 
        
        tmp_cur_x=cur_x;
      }
      else
      {
        if(anglex<=0) 
          tmp_cur_x+=1;
        else if(anglex>0)
          tmp_cur_x-=1;
      }
    }
    
    
    if((tmp_cur_x>2 &&tmp_cur_x<106) &&(tmp_cur_y>11&&tmp_cur_y<117) )     // 외벽과 만나지 않았을 때 
    {
      
      if((cur_x>=2 &&cur_x   <=106) &&(cur_y>=11&&cur_y<=117))
      {
        clear_rec();					// 볼 clear 
        LCD_SetPenColor(RGB_RED);
        LCD_DrawRectangle(cur_x,cur_y,7,7);    
        prv_x=cur_x;				// 이전 좌표에 현재 좌표 초기화 
        prv_y=cur_y;   
      }         
      chnage_pos_flag=1;       
    }
    
    else						// 외벽과 만났을 때 		
    {
      chnage_pos_flag=0;    
      tmp_cur_x=prv_x;
      tmp_cur_y=prv_y;           
    }   
  }
}

void TIMER10_Init(void)	// 가속도센서 측정 주기 생성: 250ms
{
  RCC->APB2ENR 	|= (1<<17);	// TIMER3 Clock Enable
  
  TIM10->PSC 	= 8400-1;	// Prescaler 168MHz/8400 = 0.00005s
  TIM10->ARR 	= 4000-1;	// Auto reload  0.00005s * 4000 = 200ms
  
  TIM10->CR1	&= ~(1<<4);	// Countermode = Upcounter (reset state)
  TIM10->CR1 	&= ~(3<<8);	// Clock division = 1 (reset state)
  TIM10->EGR 	|=(1<<0);	// Update Event generation    
  
  TIM10->DIER 	|= (1<<0);	// Enable Tim3 Update interrupt
  NVIC->ISER[0] 	|= (1 << 25);	// Enable Timer3 global Interrupt
  TIM10->CR1 	|= (1<<0);	// Enable Tim3 Counter    
}

void TIM1_UP_TIM10_IRQHandler (void)	// 250ms int
{
  TIM10->SR &= ~(1<<0);	//Interrupt flag Clear
  bControl = TRUE;	  		//  main 에 있는 flag set 
}

void Display_Process(int16 *pBuf)
{
  LCD_SetTextColor(RGB_RED);    
  char str[10];    
  UINT16 G_VALUE;
  // X 축 가속도 표시		
  if (pBuf[0] < 0)  //음수
  {
    G_VALUE = abs(pBuf[0]);
    LCD_DisplayChar(1,22,'-'); // g 부호 표시
    minus_x=1;
  }
  else				// 양수
  {
    G_VALUE = pBuf[0];
    LCD_DisplayChar(1,22,'+'); 
    minus_x=0; 
  }
  
  accx=G_VALUE;

  
  G_VALUE = 100 * G_VALUE / 0x4009; // 가속도 --> g 변환
  LCD_DisplayChar(1,23, G_VALUE/100 +0x30);
  LCD_DisplayChar(1,24,'.');
  LCD_DisplayChar(1,25, G_VALUE%100/10 +0x30);
  
  // Y 축 가속도 표시	
  if (pBuf[1] < 0)  //음수
  {
    G_VALUE = abs(pBuf[1]);
    
    LCD_DisplayChar(2,22,'-'); // g 부호 표시
    minus_y=1;
  }
  else				// 양수
  {
    G_VALUE = pBuf[1];
    LCD_DisplayChar(2,22,'+'); // g 부호 표시
    minus_y=0;
  }
  accy=G_VALUE;
  G_VALUE = 100 * G_VALUE / 0x4009; 
  LCD_DisplayChar(2,23, G_VALUE/100 +0x30);
  LCD_DisplayChar(2,24,'.');
  LCD_DisplayChar(2,25, G_VALUE%100/10 +0x30);
  
  
  // Z 축 가속도 표시	
  if (pBuf[2] < 0)  //음수
  {
    G_VALUE = abs(pBuf[2]);
    LCD_DisplayChar(3,22,'-'); // g 부호 표시
  }
  else				// 양수
  {
    G_VALUE = pBuf[2];
    LCD_DisplayChar(3,22,'+'); // g 부호 표시
  }
  accz=G_VALUE;
  G_VALUE = 100 * G_VALUE / 0x4009; 
  LCD_DisplayChar(3,23, G_VALUE/100 +0x30);
  LCD_DisplayChar(3,24,'.');
  LCD_DisplayChar(3,25, G_VALUE%100/10 +0x30);
  
  /******************** x축 , y축 가속도를 통해 기울기를 각도(Degree)로 환산 *************************/
  anglex= atan(accx/sqrt(pow(accy,2)+pow(accz,2)));        
  anglex*=RADIAN_TO_DEGREE;
  if(minus_x==1)
  {
    anglex*=-1;
    minus_x=0;
  }
  angley= atan(accy/sqrt(pow(accx,2)+pow(accz,2)));
  angley*=RADIAN_TO_DEGREE;
  
  if(minus_y==1)
  {
    angley*=-1;
    minus_y=0;
  }
  /***********************************************************************************************/
  
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
  
  // Buzzer (GPIO F) 설정 
  RCC->AHB1ENR	|=  0x00000020; // RCC_AHB1ENR : GPIOF(bit#5) Enable							
  GPIOF->MODER 	|=  0x00040000;	// GPIOF 9 : Output mode (0b01)						
  GPIOF->OTYPER 	&= ~0x0200;	// GPIOF 9 : Push-pull  	
  GPIOF->OSPEEDR 	|=  0x00040000;	// GPIOF 9 : Output speed 25MHZ Medium speed 
  
  GPIOG->ODR &= 0x00;	// LED0~7 Off 
}

void DelayMS(unsigned short wMS)
{
  register unsigned short i;
  
  for (i=0; i<wMS; i++)
    DelayUS(1000);		//1000us => 1ms
}

void DelayUS(unsigned short wUS)
{
  volatile int Dly = (int)wUS*17;
  for(; Dly; Dly--);
}

void DisplayTitle(void)
{
  LCD_Clear(RGB_WHITE);
  LCD_SetFont(&Gulim7);		//폰트 
  LCD_SetBackColor(RGB_WHITE);
  LCD_SetTextColor(RGB_BLACK);    //글자색
  LCD_DisplayText(0,0,"Ball game: KCM 2019130013");  // Title
  LCD_DisplayText(1,19,"Ax:");  // Title
  LCD_DisplayText(2,19,"Ay:");  // Title
  LCD_DisplayText(3,19,"Az:");  // Title
  
  
  LCD_SetPenColor(RGB_BLUE);
  LCD_DrawRectangle(1,10,113,116);        	//외벽 
  LCD_SetPenColor(RGB_RED);       
  
  cur_x=52;		// 초기 볼 좌표 
  cur_y=64;
  prv_x=cur_x;	// 이전 좌표 초기화 
  prv_x=cur_y;
  chnage_pos_flag=1;	// 외벽과 만나지 않았기 때문에 flag set 
}

void clear_rec(void)		// 볼 Clear 함수 
{
  LCD_SetBrushColor(RGB_WHITE);
  LCD_SetPenColor(RGB_WHITE);
  LCD_DrawFillRect(prv_x,prv_y,8,8);    
}
