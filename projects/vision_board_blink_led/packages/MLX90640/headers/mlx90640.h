/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-09-11     wakoj       the first version
 */
#ifndef SRC_MLX90640_H_
#define SRC_MLX90640_H_

#include <rtdevice.h>

struct mlx90640_point_data
{
    int16_t temp;
    uint16_t x;
    uint16_t y;
};

#define ARRAY_NUM(arr)          (sizeof(arr)/sizeof(arr[0]))

extern void tempSensorInit(int16_t *buff);
extern int readTempValues(int16_t *buff);
extern void generatePicture(int16_t *buff, int w, int h);
extern void readMaxTempFromBuff(int16_t *buff, struct mlx90640_point_data *obuff);
extern void readMinTempFromBuff(int16_t *buff, struct mlx90640_point_data *obuff);
extern void readMaxMinTempFromBuff(int16_t *buff, struct mlx90640_point_data *obuff);
extern void readTempFromBuff(int16_t *buff, struct mlx90640_point_data *obuff, uint16_t num);


extern void MLX90640_I2CInit(struct rt_i2c_bus_device *i2c_bus);

#endif /* SRC_MLX90640_H_ */
