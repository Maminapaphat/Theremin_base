#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "driver/i2s.h"
#include <math.h>

// Pins for MAX98357A
#define I2S_BCLK 27
#define I2S_LRC  26
#define I2S_DOUT 25

#define N_4G 0x100000000ULL

volatile float freq[2], vol[2];

// I2S Configuration
const int sampleRate = 44100;
const int WAVE_SIZE = 4096;
const float octaveRange = 4.0f;
const int bufferSize = 256;
const float baseFreq = 440.0f;
const float filterAlpha = 0.08f;
const float maxVol = 25000.0f;  // leave some head room for bass boost
const float maxBoost = 32767.0f/maxVol;
uint32_t counter;
int16_t sineTable[WAVE_SIZE];

void IRAM_ATTR OnDataRcvd(const esp_now_recv_info_t *recv_info,  const uint8_t *data, int len) {
  float *tsi = (float *)data;
  static float targetFreq, targetVol, pitch, roll, filteredFreq[2] = {baseFreq, baseFreq}, filteredVol[2];
  if(len != 10 || data[9] != 0x7d){
    Serial.println('?');
    return;
  }
  int inx = data[8]&1;
  if(data[8] & 2){
      float angle = tsi[0];
      if(angle < -90.0f)  angle = -90.0f;
      if(angle >  90.0f)  angle =  90.0f;
      float octaveOffset = angle / (data[8] & 8 ? 45.0f : 30.0f);
      freq[inx] = baseFreq * exp2f(octaveOffset);
  }
  if(data[8] & 4){
    float angle = tsi[1];
    if(angle < -90.0f)  angle = -90.0f;
    if(angle >  90.0f)  angle =  90.0f;
    vol[inx] =  angle > 0.0f ? angle/90.0f : 0.0f;
  }     
//  return;

  if(counter++ % 1000)  return;
  Serial.printf("%1d freq %5.1f %5.1f vol %5.3f, %5.3f ",data[8]&15, freq[0], freq[1], vol[0], vol[1]);
  float temp_celsius = temperatureRead(); 
  Serial.print("Internal Chip Temp: ");
  Serial.print(temp_celsius);
  Serial.println(" °C");
}

void audioLoop(void *pvParameters) {
  static float boost[2], attenuate[2];
  static int16_t samples[bufferSize * 2]; // Stereo buffer
  static uint32_t step[2], acc[2];
  while(1){
    boost[0] = boost[1] = attenuate[0] = attenuate[1] = 1.0f;
    for(int k=0; k<2; k++){
      if(freq[k] < 400) boost[k] = 1.0f + (400.0f - freq[k]) * 0.009f;
      if(freq[k] > 1000 && freq[k] < 5000)  attenuate[k] = 1.0f - (freq[k]-1000.0f)/1000.0f * 0.3f;
      step[k] = freq[k] * N_4G / sampleRate;
    }
    for (int i = 0; i < bufferSize; i++) {
      for(int k=0; k<2; k++){
        float x = vol[k] * boost[k] * attenuate[k];
//        x = x - x*x*x*0.3333333f;   // soft clip
        x = constrain(x, 0.0f, maxBoost);
        samples[i*2+k] = sineTable[acc[k]>>20] * x;
        acc[k] += step[k];
      }
    }
    // 4. Write to I2S Buffer
    size_t bytes_written;
    i2s_write(I2S_NUM_0, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK)
    Serial.printf("MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  else Serial.println("Failed to read MAC address");

  if (esp_now_init() != ESP_OK)   Serial.println("Error initializing ESP-NOW");
  else    Serial.println("esp_now_init success");

  if (esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRcvd)) != ESP_OK)  Serial.println("Error registering callback");
  else Serial.println("esp_now_register_recv_cb success");

  freq[0] = freq[1] = vol[0] = vol[1] = 0;

  i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = bufferSize,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);

    float x = 2.0 * M_PI / (double)WAVE_SIZE;
    for(int i=0; i<WAVE_SIZE; i++){
      sineTable[i] = 25000.0f * sin(x*i);
//      if((i&15) == 0)  Serial.printf("\n%4d ", i);
//      Serial.printf("%7d", sineTable[i]);
    }
    Serial.println();

    TaskHandle_t I2STaskHandle = NULL;
    // Priority 20 is very high (standard tasks are usually 1-5)
    xTaskCreatePinnedToCore(
        audioLoop,          // Function to run
        "I2S_Feeder",      // Name
        10000,             // Stack size (in bytes/words depending on framework)
        NULL,              // Parameter to pass
        20,                // Priority (High!)
        &I2STaskHandle,    // Task handle
        1                  // Core ID (1 is usually best for apps)
    );
}

void loop(){}
