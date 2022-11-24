// mosquitto_sub -h "Server_name" -t "topic_name" -d -q 2
// mosquitto_pub -h "Server_name" -t "topic_name" -m "message" -d -q 2

// mosquitto_sub -h test.mosquitto.org -t my_t -d -q 2
// mosquitto_pub -h test.mosquitto.org -t my_t -m message -d -q 2

#include <stdio.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <esp_event.h>
#include <esp_system.h>
#include "driver/uart.h"
#include "cJSON.h"
#include "mqtt_client.h"

// My Files
#include "TM_MFRC522.h"
#include "Nextion.h"
#include "rfid.h"
#include "mqtt.h"
#include "Indicator.h"
#include <driver/gpio.h>




// RFID
uint8_t Prev_ID[5]={0x00 , 0x00 , 0x00 , 0x00 , 0x00};


const rc522_start_args_t start_args =
	{
    .miso_io  = 19,
    .mosi_io  = 23,
    .sck_io   = 18,
    .sda_io   = 5,
	};

//
char idBuffer[30];
uint8_t CardID[5];


void app_main(void)
{
	//Indicators
	gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(BUZZER, GPIO_PULLDOWN_ONLY );

	gpio_set_direction(BLUE, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(BLUE, GPIO_PULLDOWN_ONLY );

	gpio_set_direction(GREEN, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(GREEN, GPIO_PULLDOWN_ONLY );

	gpio_set_direction(RED, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(RED, GPIO_PULLDOWN_ONLY );

	gpio_set_direction(POWER, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(POWER, GPIO_PULLDOWN_ONLY );



	// Initializations
	nvs_flash_init();
	uart_init();
	rc522_start(start_args);
	wifi_connection();
	mqtt_app_start();

    gpio_set_level(POWER, 1);  //Power Led ON

    while(true)
	{
		if ((TM_MFRC522_Check(CardID) == MI_OK))
			{
				sprintf(idBuffer,"0x%x0x%x0x%x0x%x0x%x", CardID[0] , CardID[1] , CardID[2] , CardID[3] , CardID[4]);
								if(Prev_ID_check(CardID ,Prev_ID ) == 0)
									{
										esp_mqtt_client_publish(client, PUB_ATTENDANCE_TOPIC, idBuffer , 0, 2, 0);
										vTaskDelay(2000/ portTICK_PERIOD_MS);
										for(uint8_t j = 0 ; j < 5 ; j++)
											{
												Prev_ID[j] = CardID[j];
											}
									}
								else if(Prev_ID_check(Prev_ID , CardID) == 1)
									{
										Display_Nextion("t4", "Remove your Card");
										vTaskDelay(2000/ portTICK_PERIOD_MS);
										Display_Nextion("t4", "Place your Card");
									}

			}
		else
			{
				uint32_t j = 0 ;  										//variable to count ticks
				while(j <= 3000000000)                                  //loop to reset RFID as it may burn if not reset
					{
						no_card_state();
							for(uint8_t i = 0 ; i < 5 ; i++ )
								{
									Prev_ID[i] = 0x00;
								}
							if(j == 3000000000)
							{
								rc522_start(start_args);
							}
							else if(TM_MFRC522_Check(CardID) == MI_OK)
								{
									break;
								}
							else
								{
									j++;
								}
					}
			}
	}

}





