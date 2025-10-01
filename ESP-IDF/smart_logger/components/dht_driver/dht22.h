#ifndef DHT22_H
#define DHT22_H

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t dht22_init(gpio_num_t gpio);
void dht_task(void *pvParameters);

#endif