/*
 * SevenSegmentDisplay.c
 *
 *  Created on: Feb 9, 2024
 *      Author: joseram
 */



/*Display constructor*/


#include "SevenSegmentDisplay.h"
#include "cmsis_os.h"

sevenSegment_t SevenSegment_ctor(GPIO_TypeDef * A_port,GPIO_TypeDef * B_port,GPIO_TypeDef * C_port,
								GPIO_TypeDef * D_port,GPIO_TypeDef *E_port,GPIO_TypeDef * F_port, GPIO_TypeDef *G_port,
								GPIO_TypeDef *DP_port,GPIO_TypeDef *D1_port,GPIO_TypeDef * D2_port,
								uint16_t A_pin,uint16_t B_pin,uint16_t C_pin,uint16_t D_pin,uint16_t E_pin,
								uint16_t F_pin,uint16_t G_pin,uint16_t DP_pin,uint16_t D1_pin,uint16_t D2_pin,
								sevenSegementMode_t mode, uint8_t refresh)
{


	sevenSegment_t display;

	display.A_Port =A_port;
	display.B_port =B_port;
	display.C_port= C_port;
	display.D_port= D_port;
	display.E_port= E_port;
	display.F_port= F_port;
	display.G_port= G_port;
	display.DP_port= DP_port;
	display.D1_port= D1_port;
	display.D2_port= D2_port;
	display.A_pin =A_pin;
	display.B_pin =B_pin;
	display.C_pin =C_pin;
	display.D_pin =D_pin;
	display.E_pin =E_pin;
	display.F_pin =F_pin;
	display.G_pin =G_pin;
	display.DP_pin =DP_pin;
	display.D1_pin =D1_pin;
	display.D2_pin =D2_pin;
	display.mode=mode;
	display.refreshDelay=refresh;

	return display;


}






void SevenSegmentDisplay_s1(sevenSegment_t const* mobj, uint8_t data)
{



		HAL_GPIO_WritePin(mobj->D1_port,mobj->D1_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D2_port,mobj->D2_pin, GPIO_PIN_RESET);

	switch (data)
	{
	case 0:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);

			break;
	case 2:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 3:

		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 4:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 5:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 6:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 7:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 8:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 9:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	default:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);

	break;
	}



}
void SevenSegmentDisplay_s2(sevenSegment_t const* mobj, uint8_t data)
{

		HAL_GPIO_WritePin(mobj->D1_port,mobj->D1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->D2_port,mobj->D2_pin, GPIO_PIN_SET);

	switch (data)
	{
	case 0:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);

			break;
	case 2:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 3:

		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 4:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 5:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 6:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 7:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 8:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	case 9:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
				break;
	default:
		HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);

	break;
	}


}


void SevenSegmentDisplay_off(sevenSegment_t const* mobj)
{

	HAL_GPIO_WritePin(mobj->A_Port,mobj->A_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->B_port,mobj->B_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->C_port,mobj->C_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->D_port,mobj->D_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->E_port,mobj->E_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->F_port,mobj->F_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->G_port,mobj->G_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->DP_port,mobj->DP_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mobj->D1_port,mobj->D1_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(mobj->D2_port,mobj->D2_pin, GPIO_PIN_RESET);

}
