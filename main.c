#include "stm32f10x.h"
#include "arm_math.h"
#include "main.h"
#include <math.h>

void InitPWR(void);
void SetHSITo24(void);
void initEnco(void);
void initPWM(void);
void initSSD(void);
void initButtons(void);
void InitADC(void);
void initDisp(void);
void toDigit(uint16_t number);
void TIM6_DAC_IRQHandler(void);
void cathodeDis(uint8_t cathode);
void cathodeEn(uint8_t cathode);
void outDigit(uint8_t digit);
int TIM15_update(void);
void PWMout(uint16_t dutyCycle);
void startADC(void);
uint16_t convADCtoTemp(uint16_t data);
int TIM16_update(void);
void setTimeInd(int time);
int checkTime(void);
int ADCCompl(void);
int encoUpdate(void);
uint16_t filter(uint16_t data);
uint16_t rms(uint16_t data);

SSD_type SSD;
PWM_type PWM;
HIF_type HIF;	// Human InterFace �������� ��������� � ������
TEMP_type TEMP;

main()
{
	int n, k, j, i = 0;
	int16_t tmp;
	
	InitPWR();
	
	SetHSITo24();
	
	// ��������� ��� � ������������� ���������
	InitADC();
	// ��������� 3 ������� ��� ��������
	initEnco();
	// ��������� 2 ������� ��� ���
	initPWM();
	// ������������� ������ � ������� ��� ���. ���������
	initSSD();
	// ������ �� �������� � �������� �� ���������
	initButtons();
	// ������������� ���������� ������������
	initDisp();
	
	n = 0;
	
	TEMP.startADC();
	
	for(;;)
	{
		/*
		if (n == 30) { 
			SSD.toDigit(i++);
			n = 0;
		}
		else
			n++;
		*/
		/*
		tmp = (int16_t) TIM3->CNT;
		
		if (tmp < 0)
			TIM3->CNT = 0;
		else
			if (tmp > 100)
				TIM3->CNT = 100;
		
		
		SSD.toDigit(TIM3->CNT);
		*/
		//TIM2->CCR4 = TIM3->CNT;
		//PWM.out(TIM3->CNT);
		
		if(HIF.update()) {
		
			// ���, ������� ����� � "����" ���-����������
			if (TEMP.ADCCompl()) {
				TEMP.temp = TEMP.ADCtoTEMP(TEMP.adcSource);
				//TEMP.temp = TEMP.filter(TEMP.temp);
				TEMP.temp = TEMP.rms(TEMP.temp);
				TEMP.startADC();
			}
		
			if(HIF.encoUpDate()) {
				HIF.setTimer(20);			// ������ 2 �
				
				PWM.out(HIF.encoPos);
			}
			
			if (HIF.checkTimer()) {
				SSD.toDigit(TEMP.temp);
			}
			else {
				SSD.toDigit(HIF.encoPos);
			}
		}
		
		// ���. ���������. (������� ������ ���������)
		if(SSD.update()) {
			SSD.cathodeDis(SSD.numDig);
	
			if (SSD.numDig == 2)
				SSD.numDig = 0;
			else
				SSD.numDig++;
			
			SSD.outDigit(SSD.dig[SSD.numDig]);
			
			SSD.cathodeEn(SSD.numDig);
		}
	}
}


void SetHSITo24(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSI */    
  RCC->CR = ((uint32_t)RCC_CR_HSION);
  RCC->CFGR = (uint32_t)(RCC_CFGR_SW_HSI);
  /* Disable PLL */
  RCC->CR &= ~RCC_CR_PLLON;
  /* Enable HSE */    
  RCC->CR &= ~((uint32_t)RCC_CR_HSEON);
  
  /*setup calibrarion to default*/
  RCC->CR |= (uint32_t) (16 << 3);
	  
	/* HCLK = SYSCLK */
	RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
	  
	/* PCLK2 = HCLK */
	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
	
	/* PCLK1 = HCLK */
	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
	
	/*  PLL configuration:  = (HSI / 2) * 6 = 24 MHz */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLMULL6);    
	
	/* Enable PLL */
	RCC->CR |= RCC_CR_PLLON;
	
	/* Wait till PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{
	}
	
	/* Select PLL as system clock source */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
	
	/* Wait till PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
	{
	}
}

void InitPWR(void)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST;
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	
	PWR->CR = PWR_CR_PLS_2V9 | PWR_CR_PVDE;
}

void initEnco(void)
{
	// ������������� ������ �� ����
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// � �������� ��������� �� ��������� ��������, �� �� ������ ������
	GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_CNF7);
	GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;
	GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
	
	// ������� ������ 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// ��������� ��������, ���� �� ������ ��������
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	
	// ������ �� �������� (����� �� ���� ��� �� ��������)
	TIM3->SMCR |= TIM_SMCR_ETF;
	
	// ������� ���������� ������ �� ��������
	// ����� ����������� �������� �� ������� ������� ��������������� ����������
	TIM3->CCER |= TIM_CCER_CC1P;
	
	// ������������ ����� �� �������� �������
	TIM3->ARR = 0xFFFF;
	
	// ������� ������
	TIM3->CR1 |= TIM_CR1_CEN;
}

// TIM2 CH4
void initPWM(void)
{
	// ������������� ����� �� �����
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	GPIOA->CRL &= ~GPIO_CRL_CNF3;
	GPIOA->CRL |= GPIO_CRL_MODE3 | GPIO_CRL_CNF3_1;
	
	// ������� ������ 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// ���, ����� 2 (���������), ����������� ���.
	TIM2->CCMR2 |= TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE;
	
	// ���. 4 ����� ��������� �� �����
	TIM2->CCER |= TIM_CCER_CC4E;
	
	// ������������, �������� ������� ���
	TIM2->PSC = 240*10;
	
	// ������������ ����� �� �������� �������
	TIM2->ARR = 100;
	
	// ������� ������
	TIM2->CR1 |= TIM_CR1_CEN;
	
	PWM.out = PWMout;
}

void initSSD(void)
{
	// ������������� ���������
	SSD.toDigit = toDigit;
	SSD.cathodeDis = cathodeDis;
	SSD.cathodeEn = cathodeEn;
	SSD.outDigit = outDigit;
	SSD.digits = digits;
	SSD.update = TIM15_update;
	
	SSD.dig[0] = SSD.digits[DIG_OFF];
	SSD.dig[1] = SSD.digits[DIG_OFF];
	SSD.dig[2] = SSD.digits[DIG_OFF];
	
	// ������������� ������ �� �����
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// ������
	GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_CNF13 | GPIO_CRH_CNF14);
	GPIOB->CRH |= GPIO_CRH_MODE12 | GPIO_CRH_MODE13 | GPIO_CRH_MODE14;
	// ��������� �������
	GPIOB->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_CNF2);
	GPIOB->CRL |= GPIO_CRL_MODE1 | GPIO_CRL_MODE2;
	
	// ��������� ������� ��� ���. ���������
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	
	TIM15->PSC = 71;	// ����� �� 72
	TIM15->ARR = 999;	// ������� �� 1000, ���������� ������ 3 ��
	TIM15->CR1 |= TIM_CR1_CEN;
}

void initButtons(void)
{
	
}

void initDisp(void)
{
	HIF.update = TIM16_update;
	HIF.setTimer = setTimeInd;
	HIF.checkTimer = checkTime;
	HIF.encoUpDate = encoUpdate;
	
	// ��������� ������� ��� ���������� � ���������
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	
	TIM16->PSC = 2399;	// ����� �� 2400
	TIM16->ARR = 999;	// ������� �� 1000, ���������� ������ 100 ��
	TIM16->CR1 |= TIM_CR1_CEN;
}

void InitADC(void)
{
  	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;
	  
  	// ����������� ��� ��� �� 2 (�� ���������)
  	RCC->CFGR &= ~RCC_CFGR_ADCPRE;
	
  	// ���. ������������ ��� � ������
  	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
  
  	// ���������� ����
  	GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
	
	ADC1->SR = 0;
  	//ADC1->CR1 = ADC_CR1_SCAN;
	ADC1->CR1 = 0;
	// �������������� �� ������ �� ���������
  	ADC1->CR2 = ADC_CR2_EXTSEL;
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
  	//ADC1->SMPR1;
	// 8 9 ������, �� �������������� 239,5 ������
  	ADC1->SMPR2 |= ADC_SMPR2_SMP2_0 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_2;
	// ��������
  	ADC1->JOFR1 = 0;
 	ADC1->JOFR2 = 0;
  	ADC1->JOFR3 = 0;
  	ADC1->JOFR4 = 0;
	// ������ ������� (�� ������������)
  	ADC1->HTR = ADC_HTR_HT;
  	ADC1->LTR = 0;
	// ���������� ������������������ ��������������, ������ 1 ��������������
  	ADC1->SQR1 = 0;
  	ADC1->SQR2 = 0;
  	ADC1->SQR3 = 2;			// ������ �������������� - 2 �����
	
  	ADC1->JSQR = 0;
	
    /* Enable ADC1 */
  	ADC1->CR2 |= ADC_CR2_ADON;
	
	/* Enable ADC1 reset calibration register */
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while (ADC1->CR2 & ADC_CR2_RSTCAL)
		;
	
	/* Start ADC1 calibration */
	ADC1->CR2 |= ADC_CR2_CAL;
	while (ADC1->CR2 & ADC_CR2_CAL)
		;
		
	// ������������� ���������
	
	TEMP.startADC = startADC;
	TEMP.ADCCompl = ADCCompl;
	TEMP.ADCtoTEMP = convADCtoTemp;
	TEMP.filter = filter;
	TEMP.filter_data = 0;
	TEMP.rms = rms;
}

// �������������� �������������� � ����� �� ����������
void toDigit(uint16_t number)
{
	int tmp;
	
	if (number > 999) {
		//tmp = number / 1000;
		//number = tmp * 1000;
		number %= 1000;
	}
	
	tmp = number / 100;
	SSD.dig[2] = SSD.digits[tmp];
	number -= tmp * 100;
	
	tmp = number / 10;
	SSD.dig[1] = SSD.digits[tmp];
	number -= tmp * 10;
	
	SSD.dig[0] = SSD.digits[number];
}

// ���������� ������
void cathodeDis(uint8_t cathode)
{
	switch (cathode) {
		case 0 : 	GPIOB->BSRR |= GPIO_BSRR_BR14;
					break;
					
		case 1 : 	GPIOB->BSRR |= GPIO_BSRR_BR13;
					break;
					
		case 2 : 	GPIOB->BSRR |= GPIO_BSRR_BR12;
					break;
					
		default : 	break;
	}
}

// ��������� ������
void cathodeEn(uint8_t cathode)
{
	switch (cathode) {
		case 0 : 	GPIOB->BSRR |= GPIO_BSRR_BS14;
					break;
					
		case 1 : 	GPIOB->BSRR |= GPIO_BSRR_BS13;
					break;
					
		case 2 : 	GPIOB->BSRR |= GPIO_BSRR_BS12;
					break;
					
		default : 	break;
	}
}

// ����� ����� �� ������� (����� � �������)
void outDigit(uint8_t digit)
{
	int i;
	
	GPIOB->BSRR |= GPIO_BSRR_BR1;
	for (i = 0; i < 8; i++) {
		if (digit & 1)
			GPIOB->BSRR |= GPIO_BSRR_BS2;
		else
			GPIOB->BSRR |= GPIO_BSRR_BR2;
		
		GPIOB->BSRR |= GPIO_BSRR_BS1;
		digit >>= 1;
		GPIOB->BSRR |= GPIO_BSRR_BR1;
	}
}

int TIM15_update(void)
{
	if (TIM15->SR & TIM_SR_UIF) {
		TIM15->SR = 0;
		return 1;
	}
	else {
		return 0;
	}
}

void PWMout(uint16_t dutyCycle)
{
	TIM2->CCR4 = dutyCycle;
}

int TIM16_update(void)
{
	if (TIM16->SR & TIM_SR_UIF) {
		TIM16->SR = 0;
		
		if (HIF.timer > 0) {
			HIF.timer--;
		}
		
		return 1;
	}
	else {
		return 0;
	}
}

// ���������� ������ �� ���������, ����� � 0.1 �
void setTimeInd(int time)
{
	HIF.timer = time;
}

// ��������� ��������� �� �����, ����� ������� � ������� ����������� ����������
int checkTime(void)
{
	if (HIF.timer == 0) {
		return 1;
	}
	else {
		return 0;
	}
}

// ��������� ���������� �� ��������� ��������
int encoUpdate(void)
{
	int16_t	encoCount = 0;
	encoCount = (int16_t) TIM3->CNT;
	if (HIF.encoPos != encoCount) {
		if (encoCount > ENCO_MAX) {
			TIM3->CNT = ENCO_MAX;
		}
		else {
			if (encoCount < ENCO_MIN) {
				TIM3->CNT = ENCO_MIN;
			}
		}
		HIF.encoPos = TIM3->CNT;
		
		return 1;
	}
	else {
		return 0;
	}
}

void startADC(void)
{
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

int ADCCompl(void)
{
	if (ADC1->SR & ADC_SR_EOC) {
		TEMP.adcSource = ADC1->DR;
		ADC1->SR = 0;
		return 1;
	}
	else {
		return 0;
	}
}

uint16_t convADCtoTemp(uint16_t data)
{
	float temp;
	temp = (float) data;
	temp = temp / 5.281;		// koef K
	temp -= 80;					// koef B
	return (uint16_t) temp; 
}
 
uint16_t filter(uint16_t data)
{
	float tmp;
	tmp = (float) data;
	TEMP.filter_data = tmp*(1 - FILTER_KOEF) + TEMP.filter_data*FILTER_KOEF;
	return (uint16_t) TEMP.filter_data;
}

uint16_t rms(uint16_t data)
{
	float tmp = 0;
	int i;
	for (i = 0; i < RMS_SUM-1; i++) {
		TEMP.rms_data[i] = TEMP.rms_data[i+1];
	}
	TEMP.rms_data[RMS_SUM-1] = data;
	for (i = 0; i < RMS_SUM; i++) {
		tmp += TEMP.rms_data[i] * TEMP.rms_data[i];
	}
	tmp = tmp / RMS_SUM;
	return (uint16_t) sqrt(tmp);
}

