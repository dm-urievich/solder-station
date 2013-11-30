
#define DIG_OFF 11
#define ENCO_MAX 100	// максимум до которого считает энкодер
#define ENCO_MIN 0		// минимум от которого считает энкодер
#define FILTER_KOEF 0.5
#define RMS_SUM 10

// структура для управления семисигментным индикатором
typedef struct{
	uint8_t dig[3];
	uint8_t numDig;
	const uint8_t *digits;
	void (*toDigit)(uint16_t number);
	void (*cathodeDis)(uint8_t cathode);
	void (*cathodeEn)(uint8_t cathode);
	void (*outDigit)(uint8_t digit);
	int  (*update)(void);
}SSD_type;

typedef struct{
	void (*out)(uint16_t dutyCycle);
}PWM_type;

typedef struct{
	int			timer;
	
	int			encoBut;
	int			standBut;
	uint16_t	encoPos;
	int			(*checkTimer)(void);
	void		(*setTimer)(int);
	int 		(*update)(void);
	void		(*readButtons)(void);
	int 		(*encoUpDate)(void);
}HIF_type;			// Human InterFace

typedef struct{
	uint16_t	adcSource;
	uint16_t	temp;
	float		filter_data;
	uint16_t	rms_data[RMS_SUM];
	void		(*startADC)(void);
	int			(*ADCCompl)(void);
	uint16_t 	(*ADCtoTEMP)(uint16_t data);
	uint16_t 	(*filter)(uint16_t data);
	uint16_t 	(*rms)(uint16_t data);
}TEMP_type;



const uint8_t digits[] = {	
	0x5F,		// 0 : 0
	0x48,		// 1 : 1
	0x9B,		// 2 : 2
	0xDA,		// 3 : 3
	0xCC,		// 4 : 4
	0xD6,		// 5 : 5
	0xD7,		// 6 : 6
	0x4A,		// 7 : 7
	0xDF,		// 8 : 8
	0xDE,		// 9 : 9
	0x80,		// 10: -
	0x00		// 11:  
};
