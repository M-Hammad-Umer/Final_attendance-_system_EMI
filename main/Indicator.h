/*
 * Indicator.h
 *
 *  Created on: 12-Sep-2022
 *      Author: Muhammad Asghar
 */

#ifndef MAIN_INDICATOR_H_
#define MAIN_INDICATOR_H_



#define RED 25
#define BLUE 26
#define GREEN 27
#define BUZZER 33
#define POWER 32

//Indicators
void all_good();
void all_bad();
void no_card_state();
void seperateString(char* , char* , char*);
void publish_card_ID(uint8_t*);
uint8_t Prev_ID_check(uint8_t *Current_ID , uint8_t* Prev_ID);

// indications at mqtt events
void indication_mqtt_connected();
void indication_mqtt_disconnected();
void indication_mqtt_subscribed();
void indication_mqtt_unsubscribed();
void indication_mqtt_published();
void indication_mqtt_data();
void indication_mqtt_error();








#endif /* MAIN_INDICATOR_H_ */
