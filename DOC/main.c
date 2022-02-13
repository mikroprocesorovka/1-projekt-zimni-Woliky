/*
Dopravníkový pás-neblokujícím zpùsobem-realizace z lega
Dìj rozdìlen do stavù stavovým automatem
1)stav: Pás se toèí (pomocí serva kontinuálního) dokud nedojde ke zmìnì napìtí na výstupu optické brány
2)stav: Pás se zastaví
3)stav: otevøe se brána (180° servo) aby mohl být naložen materiál
4)stav: Èekáme než se naloží materiál
5)stav: Brána se zavírá a dìj se mùže opakovat znovu = pás se toèí...
*/
#include "spse_stm8.h"
#include "stm8s.h"
#include "milis.h"

void init_pwm(void);//používáme pwm pro øízení serv, perioda 20ms, šíøka pulzu 1-2ms
void ADC_init(void);//adc pøevodník používáme k pøevedení napìtí na výstupu optické brány

#define STAV_START 1 
#define STAV_STOP 2 
#define STAV_ZAVORA_OPEN 3
#define STAV_NAKLAD 4
#define STAV_ZAVORA_CLOSE 5

uint16_t x=0,y=0,z=0,a=1000,b=0;//pomocné promìné
uint16_t last_time=0,last_time2=0,last_time3=0;prevod=0,open_time=0;//èasy pro neblokující zpùsob
uint8_t stav=STAV_START;//zaèínáme ve stavu start

void main(void){
CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

GPIO_Init(GPIOD,GPIO_PIN_4,GPIO_MODE_OUT_PP_LOW_SLOW);//servo-pwm
GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_OUT_PP_LOW_SLOW);//servo-pwm

init_milis(); 
init_pwm(); 
ADC_init();

  while (1){
		switch(stav){//stavový automat
			case STAV_START:
						TIM2_SetCompare1(1437);//rozjeï pás
						stav=STAV_STOP;//pøeskoè na dlaší stav
						break;

			case STAV_STOP:
						if(prevod <=500 && x==0){//pokud se zmìní napìtí na výstupu optické brány
								x=1;
								last_time=milis();//zaèni odpoèet pro jednu sekundu
						}
						
						if(milis()-last_time >= 1000 && x==1){//chvilku poèkej než box dojede na urèité místo
								TIM2_SetCompare1(0);// zastav pás
								x=2;
								open_time=milis();
								last_time=0;
								stav=STAV_ZAVORA_OPEN;    
						}
						break;
                
			case STAV_ZAVORA_OPEN:
						if(milis()-open_time >= 5 && z==0){//každých 5ms otoè servem
							open_time=milis();
							a++;
							TIM2_SetCompare2(a);//pomalu otáèej servo
							if(a==1900){//požadovaná hodnota
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
						
			case STAV_NAKLAD://poèkej než se naloží
						if(milis()-last_time2 >= 3000){
							stav=STAV_ZAVORA_CLOSE;
						}	
						break;
						
			case STAV_ZAVORA_CLOSE:
						if(milis()-open_time >= 5 && z==1){
								open_time=milis();
								a--;
								TIM2_SetCompare2(a);//zavírej bránu
								if(a==1000){
									z=0;
									b=1;
									last_time3=milis();//resetuj promìné aby jsme mohli jet od zaèátku
									last_time2=0;
									last_time=0;
									open_time=0;
									stav=STAV_START;
								}
							}
						break;	
						
			}			
		
		prevod= ADC_get(ADC2_CHANNEL_2);//sleduje napìtí na výstupu optické brány
		if(milis()-last_time3 >= 2000 && b==1){//chvilku poèká než náklad odjede aby nám ho nezastavil 2x
				x=0;
				b=0;
				last_time3=0;
		}
  }
}





void init_pwm(void){
// nastavíme piny PD2,PD3,PD4 jako výstupy (kanály TIM2, kanál TIM_CH3 reampován z pinu PA3 na PD2)
TIM2_TimeBaseInit(TIM2_PRESCALER_16,19999);

TIM2_OC1Init( 							// inicializujeme kanál 1 (TM2_CH1)
	TIM2_OCMODE_PWM1, 				// režim PWM1
	TIM2_OUTPUTSTATE_ENABLE,	// Výstup povolen (TIMer ovládá pin)
	0,		// výchozí hodnota šíøky pulzu je 1.5ms
	TIM2_OCPOLARITY_HIGH			// Zátìž rozsvìcíme hodnotou HIGH 
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
// na pinech/vstupech ADC_IN2 (PB2) a ADC_IN3 (PB3) vypneme vstupní buffer
ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL2,DISABLE);
ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL3,DISABLE);
// nastavíme clock pro ADC (16MHz / 4 = 4MHz)
ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
// volíme zarovnání výsledku (typicky vpravo, jen vyjmeènì je výhodné vlevo)
ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
// nasatvíme multiplexer na nìkterý ze vstupních kanálù
ADC2_Select_Channel(ADC2_CHANNEL_2);
// rozbìhneme AD pøevodník
ADC2_Cmd(ENABLE);
// poèkáme než se AD pøevodník rozbìhne (~7us)
ADC2_Startup_Wait();
}

// pod tímto komentáøem nic nemìòte 
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