/* NEC remote infrared RMT example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"

static const char* NEC_TAG = "NEC";


#define RMT_TX_CARRIER_EN    1   /*!< Enable carrier for IR transmitter test with IR led */
#define RMT_CARRIER_FREQ          38750
#define RMT_CARRIER_DUTY          45
#define RMT_TX_CHANNEL    RMT_CHANNEL_1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM   GPIO_NUM_26    /*!< GPIO number for transmitter signal */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    3200                         /*!< TECO protocol header: pulses for 3.2ms */
#define NEC_HEADER_LOW_US     1600                         /*!< TECO protocol header: low for 1.6ms*/
#define NEC_BIT_ONE_HIGH_US    400                         /*!< TECO protocol data bit 1: positive 400us */
#define NEC_BIT_ONE_LOW_US    1200                         /*!< TECO protocol data bit 1: negative 1200us */
#define NEC_BIT_ZERO_HIGH_US   400                         /*!< TECO protocol data bit 0: positive 400us */
#define NEC_BIT_ZERO_LOW_US    400                         /*!< TECO protocol data bit 0: negative 400us */
#define NEC_BIT_END              0                         /*!< NEC protocol end: */
#define NEC_BIT_MARGIN          20                         /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   74  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  1    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define GPIO_INPUT        GPIO_NUM_4  /*Input that activates the IR tx */
#define GPIO_LED          GPIO_NUM_27 /*LED output pin 27 */

/*
* @brief Build register value of waveform for NEC one data bit
*/
static inline void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}

/*
* @brief Generate NEC header value: active 9ms + negative 4.5ms
*/
static void nec_fill_item_header(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_HEADER_HIGH_US, NEC_HEADER_LOW_US);
}

/*
* @brief Generate NEC data bit 1: positive 0.56ms + negative 1.69ms
*/
static void nec_fill_item_bit_one(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_ONE_HIGH_US, NEC_BIT_ONE_LOW_US);
}

/*
* @brief Generate NEC data bit 0: positive 0.56ms + negative 0.56ms
*/
static void nec_fill_item_bit_zero(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_ZERO_HIGH_US, NEC_BIT_ZERO_LOW_US);
}

/*
* @brief Generate NEC end signal: positive 0.56ms
*/
static void nec_fill_item_end(rmt_item32_t* item)
{
    nec_fill_item_level(item, NEC_BIT_END, 0x7fff);
}


/*
* @brief Build NEC 32bit waveform.
*/
static int nec_build_items(int channel, rmt_item32_t* item)
{
    
    //1st byte 00110000
    nec_fill_item_header(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);

    //2nd byte 11111111
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);

    //3rd byte 01011111
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);

    //4th byte 00111111
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);

    //5th byte 00011111
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);

    //6th byte 00111010
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);

    //7th byte 00011001
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);

    //8th byte 00100000
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);

    //9th byte 10000000
    nec_fill_item_bit_one(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);
    nec_fill_item_bit_zero(item++);

    nec_fill_item_bit_zero(item++);
    /*
    i++;
    for(j = 0; j < 16; j++) {
        if(cmd_data & 0x1) {
            nec_fill_item_bit_one(item);
        } else {
            nec_fill_item_bit_zero(item);
        }
        item++;
        i++;
        cmd_data >>= 1;
    }
    */
    nec_fill_item_end(item);
    
}

/*
* @brief RMT transmitter initialization
*/
static void nec_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = RMT_CARRIER_DUTY;
    rmt_tx.tx_config.carrier_freq_hz = RMT_CARRIER_FREQ;
    rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}



/**
* @brief RMT transmitter demo, this task will periodically send NEC data. (100 * 32 bits each time.)
*
*/
static void rmt_example_nec_tx_task() //void *pvParameters
{
    vTaskDelay(10);
    nec_tx_init();
    esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    int nec_tx_num = RMT_TX_DATA_NUM;
    
    ESP_LOGI(NEC_TAG, "RMT TX DATA");
    size_t size = (sizeof(rmt_item32_t) * NEC_DATA_ITEM_NUM * nec_tx_num);
    //each item represent a cycle of waveform.
    rmt_item32_t* item = (rmt_item32_t*) malloc(size);
    int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
    memset((void*) item, 0, size);
    int i, offset = 0;

    nec_build_items(RMT_TX_CHANNEL, item);
    
    //To send data according to the waveform items.
    //Serial.print("rmt_write_items");
    rmt_write_items(RMT_TX_CHANNEL, item, item_num, true);
    //Wait until sending is done.
    rmt_wait_tx_done(RMT_TX_CHANNEL);
    //before we free the data, make sure sending is already done.
    free(item);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  
    //vTaskDelete(NULL);
    return;
}

void setup()
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1<<GPIO_INPUT);  //bit mask of the pin
    io_conf.mode = GPIO_MODE_INPUT;  //set as input mode  
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  //enable pull-up mode
    gpio_config(&io_conf);

    pinMode(GPIO_LED, OUTPUT); 
}

void loop()
{
    
    //Send command when IO4 is low (R30)
    if (!digitalRead(GPIO_INPUT)) {
      digitalWrite(GPIO_LED, HIGH);
      rmt_example_nec_tx_task();
      digitalWrite(GPIO_LED, LOW);
    }
      
    
}
