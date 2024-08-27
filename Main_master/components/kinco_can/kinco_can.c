#include "kinco_can.h"

void initTwai (const uint8_t tx, const uint8_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        //printf("Driver installed\n");
    } else {
        //printf("Failed to install driver\n");
        return;
    }
    if (twai_start() == ESP_OK) {
        //printf("Driver started\n");
    } else {
        //printf("Failed to start driver\n");
        return;
    }
}

void setModesOfOperation (const uint8_t id, const int8_t msg) { //6060 
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x60, 0x60, 0x00, d1, 0x00, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    } 
}

void setTargetVelocity (const uint8_t id, const int32_t msg) { //60FF 
    if (msg>3000 || msg<-3000) return;
    //int32_t rpm = msg*2730.6666667*-1; //TODO why?
    int32_t rpm = msg*2730.6666667; //TODO check
    uint8_t d1 = (rpm & 0x000000FF) >> 0; 
    uint8_t d2 = (rpm & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpm & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpm & 0xFF000000) >> 24; 
    twai_message_t message = {.ss = 1, .identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0xFF, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setProfileAcceleration (const uint8_t id, const uint32_t msg) { //6083
    uint32_t rpsps = msg*163.93442623; //TODO why?
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x83, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setProfileDeceleration (const uint8_t id, const uint32_t msg) { //6084
    uint32_t rpsps = msg*163.93442623; //TODO why?
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x84, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setTargetPosition (const uint8_t id, const int32_t msg) { //607A 
    int32_t inc = msg; //TODO check?
    uint8_t d1 = (inc & 0x000000FF) >> 0; 
    uint8_t d2 = (inc & 0x0000FF00) >> 8; 
    uint8_t d3 = (inc & 0x00FF0000) >> 16; 
    uint8_t d4 = (inc & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x7A, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setProfileSpeed (const uint8_t id, const uint32_t msg) { //6081
    uint32_t rpm = msg*2730.6666667; //TODO
    uint8_t d1 = (rpm & 0x000000FF) >> 0; 
    uint8_t d2 = (rpm & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpm & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpm & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x81, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setControlWord (const uint8_t id, const uint16_t msg) { //6040 
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x40, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

uint16_t getStatusWord (const uint8_t id) { //6041
    twai_message_t message_transmit = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00} };
    if (twai_transmit(&message_transmit, pdMS_TO_TICKS(1000)) == ESP_OK) {
        twai_message_t message_receive;
        if (twai_receive(&message_receive, pdMS_TO_TICKS(10000)) == ESP_OK) {
            uint8_t d1 = message_receive.data[4];
            uint8_t d2 = message_receive.data[5];
            return (d2*256 + d1);
        }
        else return 0;
    }
    else return 0;
}

void setDin2Function (const uint8_t id, const uint16_t msg) { //2010:04
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x04, d1, d2, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

void setDinSimulate (const uint8_t id, const uint16_t msg) { //2010:02
    uint8_t d1 = (msg & 0x000000FF) >> 0; 
    uint8_t d2 = (msg & 0x0000FF00) >> 8; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x10, 0x20, 0x02, d1, d2, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}