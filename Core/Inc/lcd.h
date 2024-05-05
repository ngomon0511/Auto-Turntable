#ifndef LCD_H_
#define LCD_H_

#include "stm32f1xx_hal.h"

/* cau hinh GPIO */
/* LCD PIN */
#define LCD_RS GPIO_PIN_9
#define LCD_RW GPIO_PIN_10
#define LCD_EN GPIO_PIN_11
#define LCD_D4 GPIO_PIN_12
#define LCD_D5 GPIO_PIN_13
#define LCD_D6 GPIO_PIN_14
#define LCD_D7 GPIO_PIN_15

/* LCD RCC */
#define LCD_RCC_RS_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_RW_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_EN_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_D4_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_D5_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_D6_EN __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RCC_D7_EN __HAL_RCC_GPIOB_CLK_ENABLE()

/* PORT */
#define LCD_RS_PORT GPIOB
#define LCD_RW_PORT GPIOB
#define LCD_EN_PORT GPIOB
#define LCD_D4_PORT GPIOB
#define LCD_D5_PORT GPIOB
#define LCD_D6_PORT GPIOB
#define LCD_D7_PORT GPIOB

void LCD_Init(void);
void LCD_PutC(uint8_t u8Data);
void LCD_GotoXY(uint8_t x, uint8_t y);
void LCD_Puts(char *s);

#endif
