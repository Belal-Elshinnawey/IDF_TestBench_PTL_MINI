/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "led_strip.h"

#define Activate_ir 16
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO (16) // Define the output GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT// Set duty resolution to 13 bits
#define LEDC_DUTY (16)                // Set duty to 50%. ((2 ** 13) - 1) * 25% = 2048
#define LEDC_FREQUENCY (20500)           // Frequency in Hertz.

#define TIMES              512
#define GET_UNIT(x)        ((x>>3) & 0x1)

#define ADC_RESULT_BYTE     2
#define ADC_CONV_LIMIT_EN   1                       //For ESP32, this should always be set to 1
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1  //ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1


#define I2C_MASTER_SCL_IO 25        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 26        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 100
#define ACK_CHECK_EN true         /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS false       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0               /*!< I2C ack value */
#define NACK_VAL 0x1              /*!< I2C nack value */ 
static uint8_t SEG34_ADDR = 0x48; /*!< Slave address of the 3RD and 4TH 7 Segments 24*/
static uint8_t SEG12_ADDR = 0x40; /*!< Slave address of the 5th and 6TH 7 Segments 20 */


#define led_control GPIO_NUM_14
#define RMT_TX_CHANNEL RMT_CHANNEL_0
static led_strip_t *strip;
#define write_to_output 2
#define write_to_config 6
/* Odd Byte Digit 
   ////A(3)////
   //        //
   F(2)      B(4)
   //        //
   ////G(1)////
   //        //
   E(8)      C(6)
   //        //
   ////D(7)////   DP(5)
   0:b01110111
   1:b00010100
   2:b10110011
   3:b10110110
   4:b11010100
   5:b11100110
   6:b11100111
   7:b00110100
   8:b11110111
   9:b11110110
*/

uint8_t oddbyte[10] = {
   119,
   20,
   179,
   182,
   212,
   230,
   231,
   52,
   247,
   246
};

/* Even Byte Digit 
   ////A(7)////
   //        //
   F(6)      B(8)
   //        //
   ////G(5)////
   //        //
   E(4)      C(2)
   //        //
   ////D(3)////   DP(1)
   0:b01110111
   1:b01000001
   2:b00111011
   3:b01101011
   4:b01001101
   5:b01101110
   6:b01111110
   7:b01000011
   8:b01111111
   9:b01101111
*/
uint8_t evenbyte[10]= {
   119,
   65,
   59,
   107,
   77,
   110,
   126,
   67,
   127,
   111,
};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 80000 Hz

fixed point precision: 16 bits

* 0 Hz - 19000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 20000 Hz - 21000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 22000 Hz - 40000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 47
static int filter_taps[FILTER_TAP_NUM] = {
  151,
  80,
  -163,
  10,
  136,
  33,
  -218,
  11,
  245,
  -16,
  -310,
  56,
  357,
  -91,
  -415,
  144,
  466,
  -203,
  -516,
  275,
  559,
  -354,
  -595,
  442,
  620,
  -537,
  -634,
  636,
  635,
  -738,
  -622,
  839,
  594,
  -937,
  -551,
  1030,
  494,
  -1113,
  -425,
  1186,
  343,
  -1245,
  -252,
  1288,
  154,
  -1315,
  -52,
  1324,
};






static uint16_t adc1_chan_mask = BIT(7);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[6] = {ADC1_CHANNEL_0,ADC1_CHANNEL_3,ADC1_CHANNEL_6,ADC1_CHANNEL_7,ADC1_CHANNEL_4,ADC1_CHANNEL_5};

static const char *TAG = "ADC DMA";

static void i2c_master_init(void *args)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
    };
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 1);


    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SEG12_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, write_to_config, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 255, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 255, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SEG34_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, write_to_config, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 255, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 255, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SEG12_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, write_to_output, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SEG34_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, write_to_output, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
}

void display_number(int number, int position) {
  if (number > 99) {number = 99;}
  else if (number < 0) {number = 0;}


  int digit1 = number % 10;
  number = (int)(number/10);
  int digit2 = number %  10;
  
  if (position >= 1) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, SEG12_ADDR, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, write_to_config, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, oddbyte[digit1], ACK_CHECK_EN);
        i2c_master_write_byte(cmd, evenbyte[digit2], ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);
  }
  else if (position <= 0) {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, SEG34_ADDR, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, write_to_config, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, oddbyte[digit1], ACK_CHECK_EN);
        i2c_master_write_byte(cmd, evenbyte[digit2], ACK_CHECK_EN);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd); 
  }  
}

void display_color( int number, int position){

   if (number > 99) {number = 99;}
   else if (number < 0) {number = 0;}

   if (position >= 1) {
     if (number >= 80){
        strip->set_pixel(strip,9, 255, 0, 0);
        strip->set_pixel(strip,8, 255, 90, 0);
        strip->set_pixel(strip,7, 0, 90, 0);
        strip->set_pixel(strip,6, 0, 85, 128);
        strip->set_pixel(strip,5, 0, 0, 90);
     } else if (number >= 60){
        strip->set_pixel(strip,9, 0, 0, 0);
        strip->set_pixel(strip,8, 255, 90, 0);
        strip->set_pixel(strip,7, 0, 90, 0);
        strip->set_pixel(strip,6, 0, 85, 128);
        strip->set_pixel(strip,5, 0, 0, 90);
     } else if (number >= 40){
        strip->set_pixel(strip,9, 0, 0, 0);
        strip->set_pixel(strip,8, 0, 0, 0);
        strip->set_pixel(strip,7, 0, 90, 0);
        strip->set_pixel(strip,6, 0, 85, 128);
        strip->set_pixel(strip,5, 0, 0, 90);
     } else if (number >= 20){
        strip->set_pixel(strip,9, 0, 0, 0);
        strip->set_pixel(strip,8, 0, 0, 0);
        strip->set_pixel(strip,7, 0, 0, 0);
        strip->set_pixel(strip,6, 0, 85, 128);
        strip->set_pixel(strip,5, 0, 0, 90);
     } else if (number >= 5){
        strip->set_pixel(strip,9, 0, 0, 0);
        strip->set_pixel(strip,8, 0, 0, 0);
        strip->set_pixel(strip,7, 0, 0, 0);
        strip->set_pixel(strip,6, 0, 0, 0);
        strip->set_pixel(strip,5, 0, 0, 90);
     }else {
        strip->set_pixel(strip,9, 0, 0, 0);
        strip->set_pixel(strip,8, 0, 0, 0);
        strip->set_pixel(strip,7, 0, 0, 0);
        strip->set_pixel(strip,6, 0, 0, 0);
        strip->set_pixel(strip,5, 0, 0, 0);
     }
   } else {
      if (number >= 80){
        strip->set_pixel(strip,0, 255, 0, 0);
        strip->set_pixel(strip,1, 255, 90, 0);
        strip->set_pixel(strip,2, 0, 90, 0);
        strip->set_pixel(strip,3, 0, 85, 128);
        strip->set_pixel(strip,4, 0, 0, 90);
     } else if (number >= 60){
        strip->set_pixel(strip,0, 0, 0, 0);
        strip->set_pixel(strip,1, 255, 90, 0);
        strip->set_pixel(strip,2, 0, 90, 0);
        strip->set_pixel(strip,3, 0, 85, 128);
        strip->set_pixel(strip,4, 0, 0, 90);
     } else if (number >= 40){
        strip->set_pixel(strip,0, 0, 0, 0);
        strip->set_pixel(strip,1, 0, 0, 0);
        strip->set_pixel(strip,2, 0, 90, 0);
        strip->set_pixel(strip,3, 0, 85, 128);
        strip->set_pixel(strip,4, 0, 0, 90);
     } else if (number >= 20){
        strip->set_pixel(strip,0, 0, 0, 0);
        strip->set_pixel(strip,1, 0, 0, 0);
        strip->set_pixel(strip,2, 0, 0, 0);
        strip->set_pixel(strip,3, 0, 85, 128);
        strip->set_pixel(strip,4, 0, 0, 90);
     } else if (number >= 5){
        strip->set_pixel(strip,0, 0, 0, 0);
        strip->set_pixel(strip,1, 0, 0, 0);
        strip->set_pixel(strip,2, 0, 0, 0);
        strip->set_pixel(strip,3, 0, 0, 0);
        strip->set_pixel(strip,4, 0, 0, 90);
     }else {
        strip->set_pixel(strip,0, 0, 0, 0);
        strip->set_pixel(strip,1, 0, 0, 0);
        strip->set_pixel(strip,2, 0, 0, 0);
        strip->set_pixel(strip,3, 0, 0, 0);
        strip->set_pixel(strip,4, 0, 0, 0);
     }
   }
    strip->refresh(&*strip, 100);
}

static void rgb_led_init(void *args)
{   
    rmt_config_t rmt_config_pointer_top = RMT_DEFAULT_CONFIG_TX(led_control, RMT_TX_CHANNEL);
    rmt_config_pointer_top.clk_div = 2;
    rmt_config(&rmt_config_pointer_top);
    rmt_driver_install(rmt_config_pointer_top.channel, 0, 0);
    static led_strip_config_t strip_config_top = LED_STRIP_DEFAULT_CONFIG(10, (led_strip_dev_t)RMT_TX_CHANNEL);
    strip = led_strip_new_rmt_ws2812(&strip_config_top);
    for (int i = 0; i < 10; i++)
    {
        strip->set_pixel(strip, i, 0, 0, 0);
    }
    strip->refresh(&*strip, 100);
}

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = TIMES*4,
        .conv_num_each_intr = TIMES,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = ADC_CONV_LIMIT_EN,
        .conv_limit_num = TIMES - 5,
        .sample_freq_hz = 80 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));
}



typedef struct{
    int32_t buff[FILTER_TAP_NUM]; //for circular buffer array
    uint8_t buffIndex; //Tracking the index of the circular buffer
    int32_t out; //the output value of the circular buffer
} FIRFilter;

static FIRFilter fir0;
static FIRFilter fir3;
static FIRFilter fir4;
static FIRFilter fir5;
static FIRFilter fir6;
static FIRFilter fir7;


//function to initialise the circular buffer value
void FIRFilter_init(FIRFilter *fir){ //use pointer to FIRFilter variable so that we do not need to copy the memory value (more efficient)
    //clear the buffer of the filter
    for(int i=0;i<FILTER_TAP_NUM;i++){
        fir->buff[i]=0;
    }

    //Reset the buffer index
    fir->buffIndex=0;

    //clear filter output
    fir->out=0; 
}

//function to calculate (process) the filter output
int32_t FIRFilter_calc(FIRFilter *fir, int32_t inputVal){

    int32_t out=0;

    /*Implementing CIRCULAR BUFER*/
    //Store the latest sample=inputVal into the circular buffer
    fir->buff[fir->buffIndex]=inputVal;

    //Increase the buffer index. retrun to zero if it reach the end of the index (circular buffer)
    fir->buffIndex++;
    uint8_t sumIndex=fir->buffIndex;
    uint8_t sumIndex_inv = fir->buffIndex+1;
    if (sumIndex_inv > FILTER_TAP_NUM-1) {
        sumIndex_inv = 0;
    }
    if(fir->buffIndex==FILTER_TAP_NUM){
        fir->buffIndex=0;
    }

    //Compute the filtered sample with convolution
    fir->out=0;
    int32_t acc  = 0;
    for(int i=0;i<FILTER_TAP_NUM;i++){
        //decrese sum index and warp it if necessary
        if (sumIndex_inv > FILTER_TAP_NUM-1) {
            sumIndex_inv = 0;
        }
        if(sumIndex>0){
            sumIndex--;
        }
        else{
            sumIndex=FILTER_TAP_NUM-1;
        }
        acc = acc +(((fir->buff[sumIndex]+fir->buff[sumIndex_inv])>>1)*filter_taps[i]);
        sumIndex_inv++;
        //The convolution process: Multyply the impulse response with the SHIFTED input sample and add it to the output
    }
    fir->out= acc<<1;
    //return the filtered data
    return out;

}



void app_main(void)
{
    i2c_master_init((void*)NULL);
    rgb_led_init((void*)NULL);
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    //Declaring the filter struct variable
    FIRFilter_init(&fir0);
    FIRFilter_init(&fir3);
    FIRFilter_init(&fir4);
    FIRFilter_init(&fir5);
    FIRFilter_init(&fir6);
    FIRFilter_init(&fir7);
    int32_t out0=0;
    int32_t out3=0;
    int32_t out4=0;
    int32_t out5=0;
    int32_t out6=0;
    int32_t out7=0;
    int32_t outOld0=0;
    int32_t outRateOld0=0;


    int32_t avg1 =0;
    int32_t avg2 = 0;    

    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    adc_digi_start();
    
    while(1) {
        ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
            if (ret == ESP_ERR_INVALID_STATE) {
                /**
                 * @note 1
                 * Issue:
                 * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
                 * fast for the task to handle. In this condition, some conversion results lost.
                 *
                 * Reason:
                 * When this error occurs, you will usually see the task watchdog timeout issue also.
                 * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
                 * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
                 * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
                 *
                 * Solution:
                 * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
                 */
            }

            for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
                adc_digi_output_data_t *p = (void*)&result[i];                
                switch (p->type1.channel)
                {
                case 0:
                    FIRFilter_calc(&fir0, (int32_t)p->type1.data);
                    out0 = abs(fir0.out);
                    //out0 = (int32_t)p->type1.data;
                    break;
                case 3:
                    FIRFilter_calc(&fir3, (int32_t)p->type1.data);
                    out3 = abs(fir3.out);
                    //out3 = (int32_t)p->type1.data;
                    break;
                
                case 4:
                    FIRFilter_calc(&fir4, (int32_t)p->type1.data);
                    out4 = abs(fir4.out);
                    //out4 = (int32_t)p->type1.data;
                    break;
                case 5:
                    FIRFilter_calc(&fir5, (int32_t)p->type1.data);
                    out5 = abs(fir5.out);
                    //out6 = (int32_t)p->type1.data;
                    break;
                case 6:
                    FIRFilter_calc(&fir6, (int32_t)p->type1.data);
                    out6 = abs(fir6.out);
                    //out5 = (int32_t)p->type1.data;
                    break;
                case 7:
                    FIRFilter_calc(&fir7, (int32_t)p->type1.data);
                    out7 = abs(fir7.out);
                    //out7 = (int32_t)p->type1.data;
                    break;
                default:
                    break;
                }
                //ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %d", 1, p->type1.channel, p->type1.data);
            }
            //avg1 = (out0 + out3 + out6)/3;
            //avg2 = (out5 + out4 + out7)/3;
            
            //printf("lowerlimit:0,Channel0:%d,Channel3:%d,Channel4:%d,Channel5:%d,Channel6:%d,Channel7:%d,upperlimit:4500000\n",out0, out3, out6, out5, out4, out7);
            printf("lowerlimit:0,Channel0:%d,Channel0Rate:%d,Channel0Rate2:%d,upperlimit:4500000\n",out3,( out3 - outOld0)<<4, ( out3 - outOld0 -outRateOld0)<<4);
            outRateOld0 = out3 - outOld0;
            outOld0 = out3;
            //printf("AVG1: %d, AVG2: %d\n",avg1, avg2);
            //avg1 = 99 * avg1 / (36+12);
            //avg2 = 99 * avg2 / (36+12);
            //display_color( avg1, 0);
            //display_color( avg2, 1);
            //display_number(avg1,  1);
            //display_number(avg2, 0);
            //See `note 1`
            vTaskDelay(1);
        } else if (ret == ESP_ERR_TIMEOUT) {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
            vTaskDelay(1000);
        }

    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
}

