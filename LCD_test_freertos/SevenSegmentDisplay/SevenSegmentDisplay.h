/*
 * SevenSegmentDisplay.h
 *
 *  Created on: Feb 9, 2024
 *      Author: joseram
 */

#ifndef SEVENSEGMENTDISPLAY_H_
#define SEVENSEGMENTDISPLAY_H_

#include "main.h"


typedef enum
{

sevenSegmentCommonCathode,
sevenSegmentCommonAnode

}sevenSegementMode_t;


typedef struct
{

	//GPIO PIN
	uint16_t 	A_pin;
	uint16_t	B_pin;
	uint16_t	C_pin;
	uint16_t	D_pin;
	uint16_t 	E_pin;
	uint16_t 	F_pin;
	uint16_t	G_pin;
	uint16_t	DP_pin;
	uint16_t	D1_pin;
	uint16_t	D2_pin;
	uint16_t	D3_pin;
	uint16_t	D4_pin;

	//GPIO PORTS
	GPIO_TypeDef *	A_Port;
	GPIO_TypeDef *	B_port;
	GPIO_TypeDef *	C_port;
	GPIO_TypeDef *	D_port;
	GPIO_TypeDef *	E_port;
	GPIO_TypeDef *	F_port;
	GPIO_TypeDef *	G_port;
	GPIO_TypeDef *	DP_port;
	GPIO_TypeDef *	D1_port;
	GPIO_TypeDef *	D2_port;
	GPIO_TypeDef *	D3_port;
	GPIO_TypeDef *	D4_port;

	//Refresh delay
	uint16_t refreshDelay;

	sevenSegementMode_t mode;

}sevenSegment_t;



/*Display constructor*/
sevenSegment_t SevenSegment_ctor(GPIO_TypeDef * A_port,GPIO_TypeDef * B_port,GPIO_TypeDef * C_port,
								GPIO_TypeDef * D_port,GPIO_TypeDef *E_port,GPIO_TypeDef * F_port, GPIO_TypeDef *G_port,
								GPIO_TypeDef *DP_port,GPIO_TypeDef *D1_port,GPIO_TypeDef * D2_port,
								uint16_t A_pin,uint16_t B_pin,uint16_t C_pin,uint16_t D_pin,uint16_t E_pin,
								uint16_t F_pin,uint16_t G_pin,uint16_t DP_pin,uint16_t D1_pin,uint16_t D2_pin,
								sevenSegementMode_t mode, uint8_t refresh);



void SevenSegmentDisplay_s1(sevenSegment_t const* mobj, uint8_t data);
void SevenSegmentDisplay_s2(sevenSegment_t const* mobj, uint8_t data);
void SevenSegmentDisplay_off(sevenSegment_t const* mobj);

#endif /* SEVENSEGMENTDISPLAY_H_ */
