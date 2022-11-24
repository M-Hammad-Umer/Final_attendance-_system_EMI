/*
 * Indicator.c
 *
 *  Created on: 12-Sep-2022
 *      Author: Muhammad Asghar
 */

#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt.h"
#include "Nextion.h"
#include "Indicator.h"

void all_good()
{
	gpio_set_level(GREEN, 1);  					//ACK Led ON
	gpio_set_level(BUZZER, 0);
	vTaskDelay(150/ portTICK_PERIOD_MS);
	gpio_set_level(BUZZER, 1);
	vTaskDelay(100/ portTICK_PERIOD_MS);
	gpio_set_level(BUZZER, 0);
	vTaskDelay(100/ portTICK_PERIOD_MS);
	gpio_set_level(BUZZER, 1);
	vTaskDelay(650/ portTICK_PERIOD_MS);
	gpio_set_level(GREEN, 0);
}



void all_bad()
{

	gpio_set_level(RED, 1);  					//ACK Led ON
	gpio_set_level(BUZZER, 0);
	vTaskDelay(350/ portTICK_PERIOD_MS);
//	gpio_set_level(RED, 0);
}


void no_card_state()
{
	gpio_set_level(GREEN, 0);  					//ACK Led ON
	gpio_set_level(RED, 0);
	gpio_set_level(BLUE, 0);
	gpio_set_level(BUZZER, 1);
}




uint8_t Prev_ID_check(uint8_t *Current_ID , uint8_t* Prev_ID)
{
	uint8_t count = 0;
	for(uint8_t i = 0 ; i < 5 ; i++)
	{
		if( Current_ID[i] == Prev_ID[i])
		{
			count++;
		}
	}
	if(count == 5)
	{
		return 1;
	}
	return 0;
}




void seperateString(char*c1 , char* c2 , char* main_string)
{
    char string1[30];
    char string2[5];

    int i = 0;
    while(main_string[i] != '\0')
    {
        if(main_string[i] == 44){
            memcpy(string1, &main_string[0], i);
            string1[i] = '\0';
            break;
        }
        i++;
    }


    memcpy(string2, &main_string[i+1], strlen(main_string));
    string2[strlen(main_string) - i] = '\0';
    // flag checking
    for(uint8_t i= 0 ; i <= strlen(string2) ; i++)
    {
    	if(string2[i] == 'I' || string2[i] == 'O')
    	{
    		all_good();
    	}
    	else if(string1[i] == 'e' && i == 2 && string1[i+4] == 'o')
    	{
    		all_bad();
    	}
    }

    Display_Nextion(c1 , string1);
    Display_Nextion(c2 , string2);

}




void publish_card_ID( uint8_t* CardID)
{
	char buffer[10];

		for(uint8_t i = 0 ; i < 5 ; i++)
		{
			itoa(CardID[i] , &buffer[i+(i*1)] , 16);
		}

		esp_mqtt_client_publish(client, PUB_CARD_ID_TOPIC , buffer , 0, 2, 0);


//	Display_Nextion("t4", buffer);
//	vTaskDelay(3000/ portTICK_PERIOD_MS);
}



void indication_mqtt_connected()
{
	Display_Nextion("t3", "Connected");
	Display_Nextion("t4", "Place your card");
	Display_Nextion("t5", "");

}


void indication_mqtt_disconnected()
{
    Display_Nextion("t3", "Disonnected");
    Display_Nextion("t4", "Server not responding");
    Display_Nextion("t5", "");
    gpio_set_level(BLUE, 1);
	gpio_set_level(BUZZER, 1);  // Buzzer Active LOW

}
void indication_mqtt_subscribed()
{
    Display_Nextion("t3", "Subscribed");
    Display_Nextion("t4", "Place your card");
    Display_Nextion("t5", "");
}
void indication_mqtt_unsubscribed()
{
	Display_Nextion("t3", "UnSubscribed");
}
void indication_mqtt_published()
{
	gpio_set_level(BLUE, 0);
	Display_Nextion("t3", "Published");
	vTaskDelay(1000/ portTICK_PERIOD_MS);
	Display_Nextion("t3", "Connected");
	Display_Nextion("t4", "Place your card");
	Display_Nextion("t5", "");
	vTaskDelay(200/ portTICK_PERIOD_MS);
}
void indication_mqtt_data()
{

	}
void indication_mqtt_error()
{
    gpio_set_level(BLUE, 1);
	gpio_set_level(BUZZER, 1);  // Buzzer Active LOW

}



