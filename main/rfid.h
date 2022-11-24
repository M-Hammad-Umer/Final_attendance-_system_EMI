/*
 * rdid.h
 *
 *  Created on: 26-Aug-2022
 *      Author: Muhammad Asghar
 */

#ifndef MAIN_RFID_H_
#define MAIN_RFID_H_

#pragma once
#include "driver/spi_master.h"

#define RC522_DEFAULT_MISO                 (25)
#define RC522_DEFAULT_MOSI                 (23)
#define RC522_DEFAULT_SCK                  (19)
#define RC522_DEFAULT_SDA                  (22)
#define RC522_DEFAULT_SPI_HOST             (VSPI_HOST)
#define RC522_DEFAULT_SCAN_INTERVAL_MS     (125)
#define RC522_DEFAULT_TACK_STACK_SIZE      (4 * 1024)
#define RC522_DEFAULT_TACK_STACK_PRIORITY  (4)

typedef void(*rc522_tag_callback_t)(uint8_t*);





typedef struct {
    int miso_io;                    /*<! MFRC522 MISO gpio (Default: 25) */
    int mosi_io;                    /*<! MFRC522 MOSI gpio (Default: 23) */
    int sck_io;                     /*<! MFRC522 SCK gpio  (Default: 19) */
    int sda_io;                     /*<! MFRC522 SDA gpio  (Default: 22) */
    spi_host_device_t spi_host_id;  /*<! Default VSPI_HOST (SPI3) */
    rc522_tag_callback_t callback;  /*<! Scanned tags handler */
    uint16_t scan_interval_ms;      /*<! How fast will ESP32 scan for nearby tags, in miliseconds. Default: 125ms */
    size_t task_stack_size;         /*<! Stack size of rc522 task (Default: 4 * 1024) */
    uint8_t task_priority;          /*<! Priority of rc522 task (Default: 4) */
} rc522_config_t;

typedef rc522_config_t rc522_start_args_t;


/**
 * @brief Initialize RC522 module.
 *        To start scanning tags - call rc522_resume or rc522_start2 function.
 * @param config Configuration
 * @return ESP_OK on success
 */
esp_err_t rc522_init(rc522_config_t* config);

esp_err_t rc522_start(rc522_start_args_t start_args);




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


















#endif /* MAIN_RFID_H_ */
