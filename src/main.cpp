#include <Arduino.h> 
#include <atomic>
#include <esp_now.h>
#include <stripLED.hpp>
#include <WiFi.h>

uint8_t front_mac_addr[] = {0xDC, 0x54, 0x75, 0xF1, 0xE2, 0x38}; 
uint8_t rear_mac_addr[] = {0xDC, 0x54, 0x75, 0xF1, 0xE2, 0x20}; 

LED::led_state_t DEFAULT_STARTUP_STATE = LED::led_state_t::white;

typedef struct led_data_t
{
  uint8_t led_state; 
  uint8_t led_brightness;
} led_data_t;

esp_now_peer_info_t front_info;
esp_now_peer_info_t rear_info; 

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.printf("Last Packet Send Status: %s \n", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void send_state()
{
  led_data_t state;
  state.led_brightness = LED::getBrightness();
  state.led_state = LED::getState();

  esp_now_send(front_mac_addr, (uint8_t *) &state, sizeof(state));
  vTaskDelay(pdMS_TO_TICKS(1));
  esp_now_send(rear_mac_addr, (uint8_t *) &state, sizeof(state));
}

void setup() 
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW \n");
    return;
  }

  esp_now_register_send_cb(on_data_sent);

  memcpy(front_info.peer_addr, &front_info, sizeof(front_info));
  front_info.channel = 2;
  front_info.encrypt = 0;
  esp_now_add_peer(&rear_mac_addr);

  memcpy(rear_mac_addr.peer_addr, &rear_mac_addr, sizeof(rear_mac_addr));
  rear_mac_addr.channel = 2;
  rear_mac_addr.encrypt = 0;
  esp_now_add_peer(&rear_mac_addr);
}

void loop() 
{
  vTaskSuspend(NULL);
}
