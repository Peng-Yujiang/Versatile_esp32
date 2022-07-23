// defines the i2c bus on esp32 board
// Created by Yujiang Peng
#include "i2c.h"

/**
 * Brief:
 * This shows how to configure key gpios.
 *
 * GPIO status:
 * GPIO13: left input, pulled up, interrupt from rising edge and falling edge
 * GPIO15: right input, pulled up, interrupt from rising edge.
 */

SemaphoreHandle_t print_mux = NULL;

//1. 配置驱动程序-主机
//2. 安装驱动程序
/**
 * @brief i2c主机初始化，配置主机驱动程序
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {   //配置驱动程序
        .mode = I2C_MODE_MASTER,    //设置i2c工作模式，选择主机或从机模式
        .sda_io_num = I2C_MASTER_SDA_IO,    //设置通信管脚
        .sda_pullup_en = GPIO_PULLUP_ENABLE,//是否启用内部上拉电阻
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,//设置时钟速度，仅限主机模式
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);//使用端口号和i2c_config_t作为参数
    if (err != ESP_OK) {
        return err; 
    }
    //安装驱动程序
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
                            //端口号；主机或从机模式； ；用于分配中断的标志。
}

//1. 配置驱动程序-从机
//2. 安装驱动程序
/**
 * @brief i2c从机初始化
 */
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0, //是否应用10位寻址模式，仅限从机模式
        .slave.slave_addr = ESP_SLAVE_ADDR,//定义从机地址，仅限从机模式
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
        //端口号；主机或从机模式；在从机模式下发送和接收数据的缓存区大小；用于分配中断的标志。
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

//3. a. 主机模式下通信；主机读取数据
/**
 * @brief 读取esp-i2c从机数据
 *        需要填充从机缓存，以便主机可以读取数据。
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @note 无法用esp32c3读取从机数据，因为esp32c3只有一个i2c控制器
 */
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();//创建一个命令链接
    i2c_master_start(cmd); //启动位
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
                    //主机实际执行的操作信息储存在从机地址的最低有效位中
                    //指示从从机读取数据的命令链接
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);//主机读取(n-1)位数据
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);//主机读取最后一位数据
    i2c_master_stop(cmd);   //停止位
    //触发i2c控制器执行命令链接
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd); //释放命令链接使用的资源
    return ret;
}

//指示写入或读取数据
/**
 * @brief 写入esp-i2c从机
 *        主机写入数据到从机（两个均为esp32）,
 *        数据将被储存到从机缓存，
 *        可以将其从从机缓存中读取出来。
 * 
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 * @note 无法用esp32c3读取从机数据，因为esp32c3只有一个i2c控制器
 */
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
                    //主机实际执行的操作信息储存在从机地址的最低有效位中
                    //为了将数据写入从机，主机发送的命令链接应包含地址：从机地址|主机读取
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}