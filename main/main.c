#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "sdkconfig.h"

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BUF_SIZE (128)       // buffer size
#define TXD_PIN 1            // UART TX pin
#define RXD_PIN 3            // UART RX pin
#define UART_NUM UART_NUM_0  // UART port number
#define BAUD_RATE 115200     // Baud rate
#define M_PI 3.14159265358979323846

#define I2C_MASTER_SCL_IO GPIO_NUM_22  // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21  // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define REDIRECT_LOGS 1 // if redirect ESP log to another UART


esp_err_t ret = ESP_OK;
esp_err_t ret2 = ESP_OK;

uint16_t val0[6];

float task_delay_ms = 1000;


typedef struct {
    // Datos de temperatura y presión
    int calc_temp;
    int press_comp;

    // RMS 
    float rms_temp;
    float rms_press;

    // Ventanas de datos
    float *temperature_window;
    float *pressure_window;
} SensorData;

int window_size = 10;

SensorData sensor_data;



// -------------------- COM Serial --------------------- //

// Function for sending things to UART1
static int uart1_printf(const char *str, va_list ap) {
    char *buf;
    vasprintf(&buf, str, ap);
    uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    free(buf);
    return 0;
}

// Setup of UART connections 0 and 1, and try to redirect logs to UART1 if asked
static void uart_setup() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Redirect ESP log to UART1
    if (REDIRECT_LOGS) {
        esp_log_set_vprintf(uart1_printf);
    }
}

// Write message through UART_num with an \0 at the end
// int serial_write(const char *msg, int len){

//     char *send_with_end = (char *)malloc(sizeof(char) * (len + 1));
//     memcpy(send_with_end, msg, len);
//     send_with_end[len] = '\0';

//     int result = uart_write_bytes(UART_NUM, send_with_end, len+1);

//     free(send_with_end);

//     vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
//     return result;
// }

// Read UART_num for input with timeout of 1 sec
int serial_read(char *buffer, int size){
    int len = uart_read_bytes(UART_NUM, (uint8_t*)buffer, size, pdMS_TO_TICKS(1000));
    return len;
}

//------------------------------------------------------//

// -------------------- NVS ----------------------------//

void read_nvs_value() {
    esp_err_t err;

    // Iniciar la NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Si la partición NVS está dañada o se ha actualizado, hay que formatearla
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Abre el manejador de NVS para leer o escribir en la NVS
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error abriendo NVS!\n");
    } else {
        // Intenta leer el valor de "window_size" almacenado en la NVS
        int32_t value = 0;  // Temporal para el valor almacenado
        err = nvs_get_i32(nvs_handle, "window_size", &value);
        
        if (err == ESP_OK) {
            window_size = value;
            printf("Valor de 'window_size' leído de la NVS: %d\n", window_size);
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            printf("No se encontró el valor en la NVS, asignando valor por defecto: %d\n", window_size);
            // Guarda el valor por defecto en la NVS
            err = nvs_set_i32(nvs_handle, "window_size", window_size);
            if (err == ESP_OK) {
                err = nvs_commit(nvs_handle);  // Asegúrate de que se escriban los cambios
                if (err == ESP_OK) {
                    printf("Valor por defecto almacenado en la NVS\n");
                }
            }
        } else {
            printf("Error leyendo el valor de 'window_size' de la NVS!\n");
        }

        // Cerrar el manejador de NVS
        nvs_close(nvs_handle);
    }
}

void modify_nvs_value(int new_value) {
    esp_err_t err;

    // Iniciar la NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Si la partición NVS está dañada o se ha actualizado, hay que formatearla
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Abre el manejador de NVS para leer o escribir en la NVS
    nvs_handle_t nvs_handle;
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error abriendo NVS!\n");
    } else {
        // Modifica el valor de "window_size" en la NVS
        err = nvs_set_i32(nvs_handle, "window_size", new_value);
        if (err == ESP_OK) {
            // Guarda los cambios en la NVS
            err = nvs_commit(nvs_handle);
            if (err == ESP_OK) {
                printf("Valor de 'window_size' modificado a: %d\n", new_value);
            } else {
                printf("Error guardando los cambios en la NVS!\n");
            }
        } else {
            printf("Error modificando el valor de 'window_size' en la NVS!\n");
        }

        // Cierra el manejador de NVS
        nvs_close(nvs_handle);
    }
}


//------------------------------------------------------//
esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp);  // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("\nError en softreset: %s \n", esp_err_to_name(ret));
        return 1;
    } else {
        printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) {
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03);
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03;
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_check_forced_mode(void) {
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t tmp, tmp2, tmp3, tmp4, tmp5;

    ret = bme_i2c_read(I2C_NUM_0, &ctrl_hum, &tmp, 1);
    ret = bme_i2c_read(I2C_NUM_0, &gas_wait_0, &tmp2, 1);
    ret = bme_i2c_read(I2C_NUM_0, &res_heat_0, &tmp3, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_gas_1, &tmp4, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp5, 1);
    vTaskDelay(task_delay_ms / portTICK_PERIOD_MS);
    return (tmp == 0b001 && tmp2 == 0x59 && tmp3 == 0x00 && tmp4 == 0b100000 && tmp5 == 0b01010101);
}

void bme_temp_celsius(uint32_t temp_adc, uint32_t press_adc) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion de la temperatura
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;
    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par_t[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par_t, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par_t + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par_t + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par_t + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par_t + 4, 1);

    par_t1 = (par_t[1] << 8) | par_t[0];
    par_t2 = (par_t[3] << 8) | par_t[2];
    par_t3 = par_t[4];

    // Se obtienen los parametros de calibracion de la presion
    uint8_t addr_par_p1_lsb = 0x8E, addr_par_p1_msb = 0x8F;
    uint8_t addr_par_p2_lsb = 0x90, addr_par_p2_msb = 0x91;
    uint8_t addr_par_p3_lsb = 0x92;
    uint8_t addr_par_p4_lsb = 0x94, addr_par_p4_msb = 0x95;
    uint8_t addr_par_p5_lsb = 0x96, addr_par_p5_msb = 0x97;
    uint8_t addr_par_p6_lsb = 0x99;
    uint8_t addr_par_p7_lsb = 0x98;
    uint8_t addr_par_p8_lsb = 0x9C, addr_par_p8_msb = 0x9D;
    uint8_t addr_par_p9_lsb = 0x9E, addr_par_p9_msb = 0x9F;
    uint8_t addr_par_p10_lsb = 0xA0;

    uint32_t par_p1, par_p2, par_p3, par_p4, par_p5;
    uint32_t par_p6, par_p7, par_p8, par_p9, par_p10;

    uint8_t par_p[16];

    bme_i2c_read(I2C_NUM_0, &addr_par_p1_lsb, par_p, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_msb, par_p + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_lsb, par_p + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_msb, par_p + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p3_lsb, par_p + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_lsb, par_p + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_msb, par_p + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_lsb, par_p + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_msb, par_p + 8, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p6_lsb, par_p + 9, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p7_lsb, par_p + 10, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_lsb, par_p + 11, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_msb, par_p + 12, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_lsb, par_p + 13, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_msb, par_p + 14, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p10_lsb, par_p + 15, 1);

    par_p1 = (par_p[1] << 8) | par_p[0];
    par_p2 = (par_p[3] << 8) | par_p[2];
    par_p3 = par_p[4];
    par_p4 = (par_p[6] << 8) | par_p[5];
    par_p5 = (par_p[8] << 8) | par_p[7];
    par_p6 = par_p[9];
    par_p7 = par_p[10];
    par_p8 = (par_p[12] << 8) | par_p[11];
    par_p9 = (par_p[14] << 8) | par_p[13];
    par_p10 = par_p[15];


    // Calculo de la temperatura
    int64_t var1_t;
    int64_t var2_t;
    int64_t var3_t;
    int t_fine;
    int calc_temp;

    var1_t = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2_t = (var1_t * (int32_t)par_t2) >> 11;
    var3_t = ((var1_t >> 1) * (var1_t >> 1)) >> 12;
    var3_t = ((var3_t) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2_t + var3_t);
    calc_temp = (((t_fine * 5) + 128) >> 8);
    
    // Calculo de la presion
    uint32_t var1_p, var2_p, var3_p;
    int calc_press;

    var1_p = ((int32_t)t_fine >> 1) - 64000;  
    var2_p = ((((var1_p >> 2) * (var1_p >> 2)) >> 11) * (int32_t)par_p6) >> 2;  
    var2_p = var2_p + ((var1_p * (int32_t)par_p5) << 1);   
    var2_p = (var2_p >> 2) + ((int32_t)par_p4 << 16);  
    var1_p = (((((var1_p >> 2) * (var1_p >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) + (((int32_t)par_p2 * var1_p) >> 1); 
    
    var1_p = var1_p >> 18;  
    var1_p = ((32768 + var1_p) * (int32_t)par_p1) >> 15;  
    calc_press = 1048576 - press_adc;  
    calc_press = (uint32_t)((calc_press - (var2_p >> 12)) * ((uint32_t)3125));  
    if (calc_press >= (1 << 30)) {
        calc_press = ((calc_press / (uint32_t)var1_p) << 1);
    }  
    else {
        calc_press = ((calc_press << 1) / (uint32_t)var1_p);  
    }
    var1_p = ((int32_t)par_p9 * (int32_t)(((calc_press >> 3) * (calc_press >> 3)) >> 13)) >> 12;  
    var2_p = ((int32_t)(calc_press >> 2) * (int32_t)par_p8) >> 13;  
    var3_p = ((int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8) * (int32_t)(calc_press >> 8) * (int32_t)par_p10) >> 17;  
    calc_press = (int32_t)(calc_press) + ((var1_p + var2_p + var3_p + ((int32_t)par_p7 << 7)) >> 4);    

    sensor_data.calc_temp = calc_temp;
    sensor_data.press_comp = calc_press;
}

void bme_get_mode(void) {
    uint8_t reg_mode = 0x74;
    uint8_t tmp;

    ret = bme_i2c_read(I2C_NUM_0, &reg_mode, &tmp, 1);

    tmp = tmp & 0x3;

    printf("Valor de BME MODE: %2X \n\n", tmp);
}

void bme_read_data() {
    // Datasheet[23:41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    uint8_t tmp;
    uint8_t tmp2;

    // Se obtienen los datos de temperatura
    uint8_t forced_temp_addr[] = {0x22, 0x23, 0x24};
    uint8_t forced_press_addr[] = {0x1F, 0x20, 0x21};
    for (int i = 0; i < window_size; i++) {
        uint32_t temp_adc = 0;
        uint32_t press_adc = 0;
        bme_forced_mode();
        // Datasheet[41]
        // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41

        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
        temp_adc = temp_adc | tmp << 12;
        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
        temp_adc = temp_adc | tmp << 4;
        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[2], &tmp, 1);
        temp_adc = temp_adc | (tmp & 0xf0) >> 4;

        bme_i2c_read(I2C_NUM_0, &forced_press_addr[0], &tmp2, 1);
        press_adc = press_adc | tmp2 << 12;
        bme_i2c_read(I2C_NUM_0, &forced_press_addr[1], &tmp2, 1);
        press_adc = press_adc | tmp2 << 4;
        bme_i2c_read(I2C_NUM_0, &forced_press_addr[2], &tmp2, 1);
        press_adc = press_adc | (tmp2 & 0xf0) >> 4;

        bme_temp_celsius(temp_adc, press_adc);
        uint32_t temp = sensor_data.calc_temp;
        uint32_t press = sensor_data.press_comp;

        sensor_data.temperature_window[i] = (float)temp / 100;
        sensor_data.pressure_window[i] = (float)press / 100;
    }
}


// ------------ App ------------ //
int create_window_data(){
    sensor_data.temperature_window = (float *)malloc(sizeof(float) * window_size);
    sensor_data.pressure_window = (float *)malloc(sizeof(float) * window_size);
    if (sensor_data.temperature_window == NULL || sensor_data.pressure_window == NULL){
        return 1;
    }
    return 0;
}

int change_window_size(int new_size){
    window_size = new_size;
    modify_nvs_value(new_size);
    sensor_data.temperature_window = (float *)realloc(sensor_data.temperature_window, sizeof(float) * new_size);
    sensor_data.pressure_window = (float *)realloc(sensor_data.pressure_window, sizeof(float) * new_size);
    if (sensor_data.temperature_window == NULL || sensor_data.pressure_window == NULL){
        return 1;
    }
    return 0;
}

void calculate_rms(void) {
    float sum_t = 0;
    float sum_p = 0;
    for (int i = 0; i < window_size; i++){
        sum_t += pow(sensor_data.temperature_window[i], 2);
        sum_p += pow(sensor_data.pressure_window[i], 2);
    }
    float rms_t = sqrt(sum_t / window_size);
    float rms_p = sqrt(sum_p / window_size);

    sensor_data.rms_temp = rms_t;
    sensor_data.rms_press = rms_p;
}

int close_connection(void) {
    free(sensor_data.temperature_window);
    free(sensor_data.pressure_window);
    return 0;
}

int initilize_esp_bme(void) {
    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    bme_softreset();
    bme_get_mode();
    bme_forced_mode();
    uart_setup();
    read_nvs_value();
    return 0;
}

void send_window_data(void){
    bme_read_data();
    calculate_rms();

    char dataResponse1[6];
    while (1)
    {
        int rLen = serial_read(dataResponse1, 6);
        if (rLen > 0)
        {
            if (strcmp(dataResponse1, "BEGIN") == 0)
            {
                break;
            }
        }
    }

    // Data sending, can be stopped receiving an END between sendings
    char dataResponse2[4];
    while (1)
    {
        float data[4];
        
        for (int i = 0; i < window_size; i++) {
            data[0] = sensor_data.temperature_window[i];
            data[1] = sensor_data.pressure_window[i];
            data[2] = sensor_data.rms_temp;
            data[3] = sensor_data.rms_press;

            const char* dataToSend = (const char*)data;


            int len = sizeof(float)*4;

            uart_write_bytes(UART_NUM, dataToSend, len);

            int rLen = serial_read(dataResponse2, 4);
            if (rLen > 0) {

                if (strcmp(dataResponse2, "END") == 0) {
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(300));  // Delay for 1 second
        }
    }
}

void wait_menu(void){
    char dataResponse[2];
    while(1){
        int rLen = serial_read(dataResponse, 2);
        if (rLen > 0){
            if (strcmp(dataResponse, "1") == 0){
                send_window_data();
            } 
            else if (strcmp(dataResponse, "2") == 0){
                char new_window_size[4];
                int rLen = serial_read(new_window_size, 4);
                if (rLen > 0){
                    int new_size = atoi(new_window_size);
                    change_window_size(new_size);
                }
            }
            else if (strcmp(dataResponse, "3") == 0){
                close_connection();
                break;
            }
        }
    }
}

void handshake(void){
    // OK!
    // revisa si recibe el ok del otro lado
    printf("Handshake\n");
    while(1){
        char buffer[6];
        int rlen = serial_read(buffer, 6);

        if(rlen > 0){

            if (strcmp(buffer, "Okay") == 0){
                uart_write_bytes(UART_NUM_0, "Okay\n", 6);
                break;
            }
        }  
    }
}

void app_main(void) {
    initilize_esp_bme();
    create_window_data();
    change_window_size(10);
    uart_setup(); // Uart setup
    // Waiting for an BEGIN to initialize data sending
    handshake();
    wait_menu();
}
