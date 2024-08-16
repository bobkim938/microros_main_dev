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
    // setModesOfOperation(1, 3); 
    // setModesOfOperation(2, 3); 
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
    uint32_t rpsps = msg*1; //TODO
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
    uint32_t rpsps = msg*1; //TODO
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
    //int32_t inc = msg*2730.6666667; //TODO check?
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
    int32_t inc = msg; 
    uint8_t d1 = (inc & 0x000000FF) >> 0; 
    uint8_t d2 = (inc & 0x0000FF00) >> 8; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2B, 0x40, 0x60, 0x00, d1, d2, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    }
    else {
    }
}

/*
    //Wait for message to be received
//  twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
        printf("Message received\n");
    } else {
        printf("Failed to receive message\n");
        return;
    }
    //Process received message
    if (message.extd) {
        printf("Message is in Extended Format\n");
    } else {
        printf("Message is in Standard Format\n");
    }
    printf("ID is %ld\n", message.identifier);
    if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }
*/

/*
    //Stop the TWAI driver
    if (twai_stop() == ESP_OK) {
        printf("Driver stopped\n");
    } else {
        printf("Failed to stop driver\n");
        return;
    }
    //Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        printf("Driver uninstalled\n");
    } else {
        printf("Failed to uninstall driver\n");
        return;
    }
*/


    // twai_handle_t twai_bus_1;
    // twai_handle_t twai_bus_2;
/* This is a futile attempt to install a second TWAI bus
void initTwai (const uint8_t tx1, const uint8_t rx1, const uint8_t tx2, const uint8_t rx2) {
    twai_handle_t twai_bus_0;
    twai_handle_t twai_bus_1;
    twai_general_config_t g_config_1 = TWAI_GENERAL_CONFIG_DEFAULT(tx1, rx1, TWAI_MODE_NORMAL);
    // twai_general_config_t g_config_1 = {
    //     .controller_id = 0,
    //     .mode = TWAI_MODE_NORMAL,
    //     .tx_io = tx1,
    //     .rx_io = rx1,
    // };
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
	g_config_1.controller_id = 0;
    if (twai_driver_install_v2(&g_config_1, &t_config, &f_config, &twai_bus_0) == ESP_OK) {
        printf("Driver 1 installed\n");
    } else {
        printf("Failed to install driver 1\n");
        return;
    }
    if (twai_start_v2(twai_bus_0) == ESP_OK) {
        printf("Driver 1 started\n");
    } else {
        printf("Failed to start driver 1\n");
        return;
    }
    twai_general_config_t g_config_2 = TWAI_GENERAL_CONFIG_DEFAULT(tx2, rx2, TWAI_MODE_NORMAL);
    // twai_general_config_t g_config_2 = {
    //     .controller_id = 1,
    //     .mode = TWAI_MODE_NORMAL,
    //     .tx_io = tx2,
    //     .rx_io = rx2,
    // };
	g_config_2.controller_id = 1;
    // g_config.tx_io = tx2;
    // g_config.rx_io = rx2;
    if (twai_driver_install_v2(&g_config_2, &t_config, &f_config, &twai_bus_1) == ESP_OK) {
        printf("Driver 2 installed\n");
    } else {
        printf("Failed to install driver 2\n");
        return;
    }
    if (twai_start_v2(twai_bus_1) == ESP_OK) {
        printf("Driver 2 started\n");
    } else {
        printf("Failed to start driver 2\n");
        return;
    }
}
*/
/*
void setTargetVelocity (const uint8_t id, const int32_t msg) { //60FF 
    int32_t rpm = msg*2730.6666667*-1; //TODO why?
    uint8_t d1 = (rpm & 0x000000FF) >> 0; 
    uint8_t d2 = (rpm & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpm & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpm & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0xFF, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit_v2(twai_bus_1, &message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}

void setModesOfOperation (const uint8_t id, const int8_t msg) { //6060 
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x60, 0x60, 0x00, d1, 0x00, 0x00, 0x00} };
    if (twai_transmit_v2(twai_bus_1, &message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    } 
}

void setProfileAcceleration (const uint8_t id, const uint32_t msg) { //6083
    uint32_t rpsps = msg*1; //TODO
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x83, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit_v2(twai_bus_1, &message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}

void setProfileDeceleration (const uint8_t id, const uint32_t msg) { //6084
    uint32_t rpsps = msg*1; //TODO
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x84, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit_v2(twai_bus_1, &message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}

void sendArbitraryCAN (const uint8_t id) { 
    twai_message_t message = {.identifier = id, .data_length_code = 8, .data = {0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01} };
    if (twai_transmit_v2(twai_bus_2, &message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}
*/