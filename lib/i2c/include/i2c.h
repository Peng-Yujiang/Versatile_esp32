// defines the left and right keys on esp32 board
// Created by Yujiang Peng
// 
#ifndef _I2C_H_
#define _I2C_H_

/* i2c
1. 配置驱动程序
2. 安装驱动程序
3. a. 主机模式下通信
   b. 从机模式下通信
4. 中断处理
5. 用户自动逸配置
6. 错误处理
7. 删除驱动程序
*/
#include "driver/i2c.h"

static const char *TAG_I2C = "i2c";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO            GPIO_NUM_4  /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO            GPIO_NUM_5  /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM               0           /*!< I2C从机端口 */
#define I2C_SLAVE_TX_BUF_LEN        (2 * DATA_LENGTH)  /*!< I2C从机tx接收数据的缓存区大小 */
#define I2C_SLAVE_RX_BUF_LEN        (2 * DATA_LENGTH)  /*!< I2C从机rx接收数据的缓存区大小 */

#define I2C_MASTER_SCL_IO           GPIO_NUM_18 /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_19 /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              1           /*!< I2C主机端口 */
#define I2C_MASTER_FREQ_HZ          4000000      /*!< I2C主机时钟频率 */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define BH1750_SENSOR_ADDR  0x23   /*!< 从机地址BH1750 sensor */
#define BH1750_PWR_DOWN     0X00    //关闭模块
#define BH1750_PWR_ON       0X01    //打开模块等待测量命令
#define BH1750_RST          0x07    //重置数据寄存器在PowerOn模式下有效
#define BH1750_CON_H        0x10    //连续高分辨率模式，1 lx，120ms
#define BH1750_CON_H2       0x11    //连续高分辨率模式，0.5 lx，120ms
#define BH1750_CON_L        0x13    //连续低分辨率模式，4 lx，16ms
#define BH1750_ONE_H        0x20    //一次高分辨率模式，1 lx，120ms，测量后模块转到PowerDown模式
#define BH1750_ONE_H2       0x21    //一次高分辨率模式，0.5 lx，120ms，测量后模块转到PowerDown模式
#define BH1750_ONE_L        0x23    //一次高分辨率模式，4 lx，16ms，测量后模块转到PowerDown模式
#define BH1750_CMD_START    0x23   /*!< 操作模式 */
#define ESP_SLAVE_ADDR      0x28   /*!< ESP32从机地址, 可以设置任意7bit值 */

#define MPU6050_SENSOR_ADDR 0x6   /*!< 从机地址MPU6050 sensor */
#define MPU6050_PWR_MGMT_1  0x00    //正常启用
#define MPU6050_SMPLRT_DIV  0x07    //陀螺仪采用率 125Hz
#define MPU6050_CONFIG      0x06    //低通滤波器频率为 5Hz
#define MPU6050_GYRO_CONFIG 0x18    //陀螺仪不自检，输出满量程范围为 ± 2000 °/s
#define MPU6050_ACCEL_CONFIG 0x01   //加速度计不自检，输出的满量程范围为± 2g
#define MPU6050_CMD_START   0x68   /*!< 操作模式 */

#define MPU9250_SENSOR_ADDR         0x69   /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x75   /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B   /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT           7

#endif