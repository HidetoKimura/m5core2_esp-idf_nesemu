// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "spi_lcd.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"

#include <esp_log.h>

//#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MISO GPIO_NUM_38
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5
#define PIN_NUM_DC   GPIO_NUM_15
//#define PIN_NUM_CS   GPIO_NUM_14
//#define PIN_NUM_DC   GPIO_NUM_27
//#define PIN_NUM_RST  GPIO_NUM_33
//#define PIN_NUM_BCKL GPIO_NUM_32

//#define SPI_NUM  0x3
#define SPI_NUM  0x2

#define LCD_TYPE_ILI 0
#define LCD_TYPE_ST 1

//----------
#define LCD_SEL_CMD()   gpio_set_level(GPIO_NUM_27, 0) // Low to send command 
#define LCD_SEL_DATA()  gpio_set_level(GPIO_NUM_27, 1) // High to send data
#define LCD_RST_SET()   gpio_set_level(GPIO_NUM_33, 1)
#define LCD_RST_CLR()   gpio_set_level(GPIO_NUM_33, 0)
#define LCD_BKG_ON()    gpio_set_level(GPIO_NUM_32, 1) // Backlight ON
#define LCD_BKG_OFF()   gpio_set_level(GPIO_NUM_32, 0) //Backlight OFF
//----------
/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

spi_device_handle_t spi;

#if 1
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
		{0xCF, {0x00, 0x83, 0X30}, 3}, 						// Power control B register
		{0xED, {0x64, 0x03, 0X12, 0X81}, 4},				// Power on sequence register 
		{0xE8, {0x85, 0x01, 0x79}, 3},						// Driver timing control A
		{0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},			// Power control A register
		{0xF7, {0x20}, 1},									// Pump ratio control register
		{0xEA, {0x00, 0x00}, 2},							// Driver timing control B
		{0xC0, {0x26}, 1},         							// Power Control 1 register
		{0xC1, {0x11}, 1},          						// Power Control 2 register 
		{0xC5, {0x35, 0x3E}, 2},    						// VCOM Control 1 register
		{0xC7, {0xBE}, 1},          						// VCOM Control 1 register
		{0x36, {0x28}, 1},        							// Memory Access Control register
		{0x3A, {0x55}, 1},									// Pixel Format register 
		{0xB1, {0x00, 0x1B}, 2},							// Frame Rate Control (In Normal Mode)
		{0xF2, {0x08}, 1},									// 3 Gamma enable register
		{0x26, {0x01}, 1},									// Gamma register
		{0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15}, // Positive Gamma Correction register
		{0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15}, // Negative Gamma Correction register 
		{0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},				// Colomn address register 
		{0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},				// Page address register
		{0x2C, {0}, 0},										// GRAM register
		{0xB7, {0x07}, 1},									// Entry Mode Set
		{0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},				// Display Function Control register
		{0x11, {0}, 0x80},									// Sleep out register
		{0x29, {0}, 0x80},									// Display on register
		#if 1
		{0x2A, {0x00, 0x78, 0x00, 0x87}, 4},     // Colomn address register
		{0x2B, {0x00, 0xA0, 0x00, 0xAF}, 4},     // Page address registe 
		#endif
		{0, {0}, 0xff},
	};
#else
DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[]={
    {0xEF, {0x03, 0x80, 0x02}, 3},              // Unnknown
    {0xCF, {0x00, 0XC1, 0X30}, 3},              // Power control B register 
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},        // Power on sequence register
    {0xE8, {0x85, 0x00, 0x78}, 3},              // Driver timing control A 
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},  // Power control A register
    {0xF7, {0x20}, 1},                          // Pump ratio control register
    {0xEA, {0x00, 0x00}, 2},                    // Driver timing control B
    {0xC0, {0x23}, 1},                          // Power Control 1 register: VRH [5:0]: Set the GVDD level,
    {0xC1, {0x10}, 1},                          // Power Control 2 register
    {0xC5, {0x3e, 0x28}, 2},                    // VCOM Control 1 register
    {0xC7, {0x86}, 1},                          // VCOM Control 2 register
    // {0x36, {0x48}, 1},
    {0x36, {0x08}, 1},                          // Memory Access Control register:D7=MY,D6=MX,D5=MV,D4=ML,D3=BGR,D2=MH
    {0x3A, {0x55}, 1},                          // Pixel Format register
    // {0xB1, {0x00, 0x18}, 2},
    {0xB1, {0x00, 0x1B}, 2},                    // Frame Rate Control (In Normal Mode)
    {0xB6, {0x08, 0x82, 0x27}, 3},              // Display Function Control register
    {0xF2, {0x00}, 1},                          // 3 Gamma enable register
    {0x26, {0x01}, 1},                          // Gamma register
    {0xE0, {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00}, 15},   // Positive Gamma Correction register
    {0XE1, {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F}, 15},   // Negative Gamma Correction register 

    {0x2A, {0x00, 0x67, 0x00, 0x77}, 4},     // Colomn address register
    {0x2B, {0x00, 0x8f, 0x00, 0x9f}, 4},     // Page address registe 
    // {0x2C, {0}, 0},                          // GRAM register
    // {0xB7, {0x07}, 1},                       // Entry Mode Set
    {0x11, {0}, 0x80},                          // Sleep out register
    {0x29, {0}, 0x80},                          // Display on register
    {0, {0}, 0xff},                             // NOP
};
#endif

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}
//------
uint8_t test_bmp[16][16 * 2];
//Initialize the display
void lcd_init(spi_device_handle_t spi) 
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;
//    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_INPUT);
//    vTaskDelay(1 / portTICK_RATE_MS);
//    bool lcd_version = gpio_get_level(PIN_NUM_RST);

    //Initialize non-SPI GPIOs
    gpio_pad_select_gpio(PIN_NUM_DC);
	gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);


//    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
//    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    
    // gpio_set_direction(GPIO_NUM_25, GPIO_MODE_INPUT);

#if 0
    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
#endif

    //detect LCD type
    // uint32_t lcd_id = lcd_get_id(spi);
    uint32_t lcd_id = 0;
    int lcd_detected_type = 0;
    int lcd_type;

    printf("LCD ID: %08X\n", lcd_id);
    lcd_detected_type = LCD_TYPE_ILI;
    printf("ILI9341 detected...\n");   

    lcd_type = lcd_detected_type; 
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes!=0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
//    if(lcd_version) lcd_cmd(spi, 0x21);
    ///Enable backlight
    // gpio_set_level(PIN_NUM_BCKL, 1);

	uint8_t data[] = {0x08, 0x88, 0x28, 0xE8};
    lcd_cmd(spi, 0x36);
    lcd_data(spi, (void *) &data[0], 1);

    lcd_cmd(spi, 0x21);

	for(int y = 0; y < 16; y++) {
		for(int x = 0; x < 16; x++) {
			test_bmp[y][x * 2] 		= 0x00; // BBBB BGGG 
			test_bmp[y][x * 2 + 1] 	= 0x1F; // GGGR RRRR
		}
	} 

    lcd_cmd(spi, 0x2C);
    lcd_data(spi, test_bmp, 512);

    ESP_LOGI("nes", "lcd_init() done");

}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

void ili9341_spi_init()
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        // .clock_speed_hz=10*1000*1000,               //Clock out at 10 MHz
        .clock_speed_hz=40*1000*1000,               //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY, 
    };
    //Initialize the SPI bus
//    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    ret=spi_bus_initialize(SPI2_HOST, &buscfg, 1);
    // assert(ret==ESP_OK);

    //Attach the LCD to the SPI bus
 //   ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    // assert(ret==ESP_OK);

    //Initialize the LCD
   lcd_init(spi);
}
//------


//.............LCD API END----------
void lcd_setBrightness(int duty) {

    #define LEDC_HS_TIMER          LEDC_TIMER_0
    #define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
    #define LEDC_HS_CH0_GPIO       (32)
    #define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0    
    #define LEDC_TEST_DUTY         (10)

    ledc_timer_config_t ledc_timer = {
        .bit_num = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 5000,              // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,   // timer mode
        .timer_num = LEDC_HS_TIMER    // timer index
    };

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = 
    {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    // ledc_fade_func_install(0);

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

#define U16x2toU32(m,l) ((((uint32_t)(l>>8|(l&0xFF)<<8))<<16)|(m>>8|(m&0xFF)<<8))

extern uint16_t myPalette[];

extern void ili9341_init(void);
extern void ili9341_flush2(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[]);

extern uint16_t nes_xs = 0;
extern uint16_t nes_ys = 0; 
extern uint16_t nes_width = 0;
extern uint16_t nes_height = 0;
extern void* nes_data = NULL;


void ili9341_write_frame(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t * data[])
{
#if 0
    nes_xs = xs;
    nes_ys = ys;
    nes_width = width;
    nes_height = height;
    nes_data = (void*)data;

//   ESP_LOGI("nes", "ili9341_write_frame() %p ", nes_data);

//    ili9341_flush2(xs, ys, width, height, data);

#else
    int x, y;
    int i;
    uint16_t x1, y1;
    uint32_t xv, yv, dc;
    uint32_t temp[16];
    dc = (1 << PIN_NUM_DC);
    
    for (y=0; y<height; y++) {
        //start line
        x1 = xs+(width-1);
        y1 = ys+y+(height-1);
        xv = U16x2toU32(xs,x1);
        yv = U16x2toU32((ys+y),y1);
        
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2A); //Colomn address register
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), xv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2B); // Page address register
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 31, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), yv);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        GPIO.out_w1tc = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
        WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), 0x2C); // GRAM register
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
        
        x = 0;
        GPIO.out_w1ts = dc;
        SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 511, SPI_USR_MOSI_DBITLEN_S);
        while (x<width) {
            for (i=0; i<16; i++) {
                if(data == NULL){
                    temp[i] = 0;
                    x += 2;
                    continue;
                }
                x1 = myPalette[(unsigned char)(data[y][x])]; x++;
                y1 = myPalette[(unsigned char)(data[y][x])]; x++;
                temp[i] = U16x2toU32(x1,y1);
            }
            while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
            for (i=0; i<16; i++) {
                WRITE_PERI_REG((SPI_W0_REG(SPI_NUM) + (i << 2)), temp[i]);
            }
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
        }
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
#endif
}

void ili9341_nes_init()
{
#if 1
    ESP_LOGI("nes", "ili9341_nes_init()");
    ili9341_spi_init();
//    lcd_setBrightness(800);
#endif
}