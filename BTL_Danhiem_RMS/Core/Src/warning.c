/*
 * warning.c
 *
 *  Created on: Nov 13, 2024
 *      Author: DELL
 */

#include "warning.h"

void L1_Warning_On()
{
	HAL_GPIO_WritePin(L1_LED_GPIO_Port,L1_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L2_LED_GPIO_Port,L2_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L3_LED_GPIO_Port,L3_LED_Pin, GPIO_PIN_SET);
}

void L2_Warning_On ()
{
	HAL_GPIO_WritePin(L1_LED_GPIO_Port,L1_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L2_LED_GPIO_Port,L2_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L3_LED_GPIO_Port,L3_LED_Pin, GPIO_PIN_SET);
}

void L3_Warning_On ()
{
	HAL_GPIO_WritePin(L1_LED_GPIO_Port,L1_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L2_LED_GPIO_Port,L2_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L3_LED_GPIO_Port,L3_LED_Pin, GPIO_PIN_RESET);
}



void Warning ()
{
	  // khi không ấn nút( cảnh báo tự đông)
	  if(CO2_ppm<=1000 && CO_ppm <= 25 && Tvoc_ppb <=300)
	  {
		  L1_Warning_On();
	  }
	  else if((CO2_ppm>=2000) || (CO_ppm>=50) || (Tvoc_ppb>=1000))
	  {
		  L3_Warning_On();
	  }
	  else
	  {
		  L2_Warning_On();
	  }
}

