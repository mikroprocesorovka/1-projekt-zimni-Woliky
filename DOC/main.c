/*
Dopravn�kov� p�s-neblokuj�c�m zp�sobem-realizace z lega
D�j rozd�len do stav� stavov�m automatem
1)stav: P�s se to�� (pomoc� serva kontinu�ln�ho) dokud nedojde ke zm�n� nap�t� na v�stupu optick� br�ny
2)stav: P�s se zastav�
3)stav: otev�e se br�na (180� servo) aby mohl b�t nalo�en materi�l
4)stav: �ek�me ne� se nalo�� materi�l
5)stav: Br�na se zav�r� a d�j se m��e opakovat znovu = p�s se to��...
*/
#include "spse_stm8.h"
#include "stm8s.h"
#include "milis.h"

void init_pwm(void);//pou��v�me pwm pro ��zen� serv, perioda 20ms, ���ka pulzu 1-2ms
void ADC_init(void);//adc p�evodn�k pou��v�me k p�eveden� nap�t� na v�stupu optick� br�ny

#define STAV_START 1 
#define STAV_STOP 2 
#define STAV_ZAVORA_OPEN 3
#define STAV_NAKLAD 4
#define STAV_ZAVORA_CLOSE 5

uint16_t x=0,y=0,z=0,a=1000,b=0;//pomocn� prom�n�
uint16_t last_time=0,last_time2=0,last_time3=0;prevod=0,open_time=0;//�asy pro neblokuj�c� zp�sob
uint8_t stav=STAV_START;//za��n�me ve stavu start

void main(void){
CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

GPIO_Init(GPIOD,GPIO_PIN_4,GPIO_MODE_OUT_PP_LOW_SLOW);//servo-pwm
GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_OUT_PP_LOW_SLOW);//servo-pwm

init_milis(); 
init_pwm(); 
ADC_init();

  while (1){
		switch(stav){//stavov� automat
			case STAV_START:
						TIM2_SetCompare1(1437);//rozje� p�s
						stav=STAV_STOP;//p�esko� na dla�� stav
						break;

			case STAV_STOP:
						if(prevod <=500 && x==0){//pokud se zm�n� nap�t� na v�stupu optick� br�ny
								x=1;
								last_time=milis();//za�ni odpo�et pro jednu sekundu
						}
						
						if(milis()-last_time >= 1000 && x==1){//chvilku po�kej ne� box dojede na ur�it� m�sto
								TIM2_SetCompare1(0);// zastav p�s
								x=2;
								open_time=milis();
								last_time=0;
								stav=STAV_ZAVORA_OPEN;    
						}
						break;
                
			case STAV_ZAVORA_OPEN:
						if(milis()-open_time >= 5 && z==0){//ka�d�ch 5ms oto� servem
							open_time=milis();
							a++;
							TIM2_SetCompare2(a);//pomalu ot��ej servo
							if(a==1900){//po�adovan� hodnota
								z=1;
								open_time=0;
							}
						}
						if(x==2 && z==1){
								last_time2=milis();
								x=3;
								stav=STAV_NAKLAD;
						}
						break;
						
			case STAV_NAKLAD://po�kej ne� se nalo��
						if(milis()-last_time2 >= 3000){
							stav=STAV_ZAVORA_CLOSE;
						}	
						break;
						
			case STAV_ZAVORA_CLOSE:
						if(milis()-open_time >= 5 && z==1){
								open_time=milis();
								a--;
								TIM2_SetCompare2(a);//zav�rej br�nu
								if(a==1000){
									z=0;
									b=1;
									last_time3=milis();//resetuj prom�n� aby jsme mohli jet od za��tku
									last_time2=0;
									last_time=0;
									open_time=0;
									stav=STAV_START;
								}
							}
						break;	
						
			}			
		
		prevod= ADC_get(ADC2_CHANNEL_2);//sleduje nap�t� na v�stupu optick� br�ny
		if(milis()-last_time3 >= 2000 && b==1){//chvilku po�k� ne� n�klad odjede aby n�m ho nezastavil 2x
				x=0;
				b=0;
				last_time3=0;
		}
  }
}





void init_pwm(void){
// nastav�me piny PD2,PD3,PD4 jako v�stupy (kan�ly TIM2, kan�l TIM_CH3 reampov�n z pinu PA3 na PD2)
TIM2_TimeBaseInit(TIM2_PRESCALER_16,19999);

TIM2_OC1Init( 							// inicializujeme kan�l 1 (TM2_CH1)
	TIM2_OCMODE_PWM1, 				// re�im PWM1
	TIM2_OUTPUTSTATE_ENABLE,	// V�stup povolen (TIMer ovl�d� pin)
	0,		// v�choz� hodnota ���ky pulzu je 1.5ms
	TIM2_OCPOLARITY_HIGH			// Z�t� rozsv�c�me hodnotou HIGH 
	);
	
TIM2_OC2Init(
	TIM2_OCMODE_PWM1,
	TIM2_OUTPUTSTATE_ENABLE,
	0,
	TIM2_OCPOLARITY_HIGH
	);
	
TIM2_OC1PreloadConfig(ENABLE);
TIM2_Cmd(ENABLE);
}


void ADC_init(void){
// na pinech/vstupech ADC_IN2 (PB2) a ADC_IN3 (PB3) vypneme vstupn� buffer
ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL2,DISABLE);
ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL3,DISABLE);
// nastav�me clock pro ADC (16MHz / 4 = 4MHz)
ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
// vol�me zarovn�n� v�sledku (typicky vpravo, jen vyjme�n� je v�hodn� vlevo)
ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
// nasatv�me multiplexer na n�kter� ze vstupn�ch kan�l�
ADC2_Select_Channel(ADC2_CHANNEL_2);
// rozb�hneme AD p�evodn�k
ADC2_Cmd(ENABLE);
// po�k�me ne� se AD p�evodn�k rozb�hne (~7us)
ADC2_Startup_Wait();
}

// pod t�mto koment��em nic nem��te 
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif