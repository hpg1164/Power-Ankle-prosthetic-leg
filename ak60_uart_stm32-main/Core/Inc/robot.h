#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_uart.h"

#ifdef __cplusplus
// #include "definations.hpp"

class Robot
{
public:
};

#endif

#ifdef __cplusplus
extern "C"
{
#endif
    void init_robot();
    void operate_robot();
#ifdef __cplusplus
}
#endif

#endif // ROBOT_H_
