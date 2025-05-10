#include "robot.h"
#include "bno08.hpp"
#include <stdio.h>
#include "usart.h"

Robot bot();
Bno08 imu(&huart3);
Bno08::ImuData data;
uint32_t last_tick = 0;

int _write(int file, char *data, int len)
{
    int i  = 0;
    for (; i < len; ++i)
    {
        ITM_SendChar(data[i]);
    }
    return i;
}

void init_robot()
{
    imu.init();
}
void operate_robot()
{
    uint32_t now = HAL_GetTick();

    if (now -last_tick < 10)
        return;

    last_tick = now;

    data = imu.get_data();
    printf("ypr: %f %f %f\n", data.yaw, data.pitch, data.roll);
}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
    if (huart->Instance == huart3.Instance)
    {
        imu.parse_imu_data();
    }
 }