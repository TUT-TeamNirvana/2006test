#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "main.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

// 请在main.h或你的GPIO配置文件中定义这些引脚
// 或者直接在这里修改为你实际的引脚定义
// 例如：
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_GYRO_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0

#ifndef CS1_ACCEL_GPIO_Port
#error "请定义BMI088的CS片选引脚！请在BMI088Middleware.h或main.h中定义CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin"
#endif


#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;

#elif defined(BMI088_USE_IIC)

#endif

#endif
