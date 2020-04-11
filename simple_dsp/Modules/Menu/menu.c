/*
 * menu.c
 *
 *  Created on: Apr 11, 2020
 *      Author: javi
 */


/* Includes */
#include "menu.h"
#include "../DSP/dsp.h"


/* Private variables */
const uint8_t menu_menuString[] = "Choose filter mode:\r\nP - Bypass\r\nL - Lowpass\r\nB - Bandpass\r\nH - Highpass\r\n";


/* Private function prototypes */
static void menu_InitUart(void);
static void menu_ShowMenu(void);


/* Public functions */

/**
 * Initulizes uart and show menu in serial port
 */
void menu_Init( void ) {
  menu_InitUart();
  menu_ShowMenu();
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void) {
  uint8_t c;
  if(LL_USART_IsActiveFlag_RXNE(USART2)) {
    c = LL_USART_ReceiveData8(USART2);
    switch(c){
    case 'P':
    case 'p':
      dsp_ChangeDSPMode(DSP_MODE_BYPASS);
      break;
    case 'L':
    case 'l':
      dsp_ChangeDSPMode(DSP_MODE_LOWPASS);
      break;
    case 'B':
    case 'b':
      dsp_ChangeDSPMode(DSP_MODE_BANDPASS);
      break;
    case 'H':
    case 'h':
      dsp_ChangeDSPMode(DSP_MODE_HIGHPASS);
      break;
    default:
      break;  // no action if not valid mode
    }
  }
}


/* Private functions */

static void menu_InitUart(void){
  LL_USART_EnableIT_RXNE(USART2);
}

/**
 * Shows menu in serial port
 */
static void menu_ShowMenu(void){
  uint32_t i;

  for(i=0;i<(sizeof(menu_menuString)/sizeof(uint8_t));i++){
    while (!LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_TransmitData8(USART2, menu_menuString[i]);
  }

}
