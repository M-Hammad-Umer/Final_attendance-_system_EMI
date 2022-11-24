/*
 * mqtt.h
 *
 *  Created on: 07-Sep-2022
 *      Author: Muhammad Asghar
 */

#ifndef MAIN_MQTT_H_
#define MAIN_MQTT_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"


#include "esp_log.h"
#include "mqtt_client.h"
#include "Nextion.h"





#define PUBLISH 1
#define SUBSCRIBE 0
// WiFi
#define SSID "EMI StormFiber"
#define PASSWORD "wlanad7f97"

// MQTT
#define BROKER      "mqtt://192.168.1.200:4200"  //"mqtt://test.mosquitto.org"

// Tattendenceopics
#define PUB_ATTENDANCE_TOPIC "publish-user-Attendance"
#define SUB_TIME_TOPIC "publish-time"
#define PUB_CARD_ID_TOPIC "id"
#define SERVER_FEEDBACK "publish-user"


// Credentials
#define USERNAME "user"
#define BROKER_PASSWORD "1234"
#define PORT 4200

void log_error_if_nonzero(const char *message, int error_code);
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */

esp_mqtt_client_handle_t client;

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start();
void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_connection();
void Task1_RFID_and_Publishing();
void Task2_Subscription();









#endif /* MAIN_MQTT_H_ */
