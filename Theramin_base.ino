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

volatile float freq[2], vol[2]; // these are the values received from the theremin remote

// I2S Configuration
const int sampleRate = 44100;   // standard sample rate for CD quality sound, 44.1kHz
const int WAVE_SIZE = 4096;     // size of sine table
const float octaveRange = 4.0f; // default number of octaves in theremin range
const int bufferSize = 256;     // size of I2S buffer
const float baseFreq = 440.0f;  // center frequency, if remote is level
const float filterAlpha = 0.08f;// for filtering values - avoid abrupt changes
const float maxVol = 25000.0f;  // leave some head room for bass boost
const float maxBoost = 32767.0f/maxVol;
uint32_t counter;               // for debug printing
int16_t sineTable[WAVE_SIZE];   // table of precomputed sine values. the sin function is slow


// this gets called when a new ESP_NOW message is received
void IRAM_ATTR OnDataRcvd(const esp_now_recv_info_t *recv_info,  const uint8_t *data, int len) {
  float *tsi = (float *)data;       // the first two values are floats, pitch and roll
  static float targetFreq, targetVol, pitch, roll, filteredFreq[2] = {baseFreq, baseFreq}, filteredVol[2];
  if(len != 10 || data[9] != 0x7d){ // is this a valid message?
    Serial.println('?');            // it is NOT valid
    return;                         // don't do anything
  }
  int inx = data[8]&1;              // data[8] contains the strapping options, this is the L/R selector
  if(data[8] & 2){                  // this is the frequency selector option
      float angle = tsi[0];         // get the accelerometer pitch
      if(angle < -90.0f)  angle = -90.0f; // constrain to valid range
      if(angle >  90.0f)  angle =  90.0f;
      float octaveOffset = angle / (data[8] & 8 ? 45.0f : 30.0f); // which octave? data[8] & 8 is the 6 octave range option
      freq[inx] = baseFreq * exp2f(octaveOffset);   // frequency corresponding to angle - logarithnic scale
  }
  if(data[8] & 4){                  // this is the volume selector
    float angle = tsi[1];           // get the accelerometer roll value
    if(angle < -90.0f)  angle = -90.0f;   // constrain the angle to a valid range
    if(angle >  90.0f)  angle =  90.0f;
    vol[inx] =  angle > 0.0f ? angle/90.0f : 0.0f;  // volume corresponding to roll angle
  }     
//  return;

// print out some stuffto help debugging
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
  while(1){   // go forever
    boost[0] = boost[1] = attenuate[0] = attenuate[1] = 1.0f;
    for(int k=0; k<2; k++){
      // low notes sound very quiet, boost the volume for low frequencies
      if(freq[k] < 400) boost[k] = 1.0f + (400.0f - freq[k]) * 0.009f;
      // this range sounds excessively loud because our ears are most sensitive to the frequencies, attenuate the volume
      if(freq[k] > 1000 && freq[k] < 5000)  attenuate[k] = 1.0f - (freq[k]-1000.0f)/1000.0f * 0.3f;
      step[k] = freq[k] * N_4G / sampleRate;        // calculate the step size as a function of frequency
    }
    for (int i = 0; i < bufferSize; i++) {          // for each pair (L,R) words  in the samples buffer
      for(int k=0; k<2; k++){                       // fill the left and right sound values
        float x = vol[k] * boost[k] * attenuate[k]; // apply the boost and attenuate values to the volume factor
//        x = x - x*x*x*0.3333333f;   // soft clip
        x = constrain(x, 0.0f, maxBoost);           // constrain
        samples[i*2+k] = sineTable[acc[k]>>20] * x; // fill the sample buffer with the sine value of the step * the loudness
        acc[k] += step[k];                          // next step in the sine table
      }
    }
    // 4. Write to I2S Buffer
    size_t bytes_written;                           // send the filled message buffer to the MAX98357 I2S amps
    i2s_write(I2S_NUM_0, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);                                   // initialize the Serial port for debug messages
  WiFi.mode(WIFI_STA);                                    // turn on Wifi
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);         // select channel 1 - must be the same as the base channel
  esp_wifi_set_promiscuous(false);

  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac); // get our MAC address
  if (ret == ESP_OK)                                      // get_mac successful?
    Serial.printf("MAC %02x:%02x:%02x:%02x:%02x:%02x\n",  // yes, print our MAC address
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  else Serial.println("Failed to read MAC address");      // failed to read MAC address

// initialize ESP_NOW messaging
  if (esp_now_init() != ESP_OK)   Serial.println("Error initializing ESP-NOW");
  else    Serial.println("esp_now_init success");

// register callback function to handle received messages
  if (esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRcvd)) != ESP_OK)  Serial.println("Error registering callback");
  else Serial.println("esp_now_register_recv_cb success");

  freq[0] = freq[1] = vol[0] = vol[1] = 0;

// initialize I2S communications - i don't understand most of this - It came from Gemini and seems to work
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

  // configure I2S pins
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);    // Start I2S
    i2s_set_pin(I2S_NUM_0, &pin_config);                    // tell I2S what pins are connected


// fill the sine table with sines from 0 to 2*pi in 4096 increments
    float x = 2.0 * M_PI / (double)WAVE_SIZE;
    for(int i=0; i<WAVE_SIZE; i++){
      sineTable[i] = 25000.0f * sin(x*i);
//      if((i&15) == 0)  Serial.printf("\n%4d ", i);
//      Serial.printf("%7d", sineTable[i]);
    }
    Serial.println();

// run the I2S app in a high priority thread locked to core 1
    TaskHandle_t I2STaskHandle = NULL;
    // Priority 20 is very high (standard tasks are usually 1-5)
    xTaskCreatePinnedToCore(
        audioLoop,          // Function to run
        "I2S_Feeder",       // Name
        10000,              // Stack size (in bytes/words depending on framework)
        NULL,               // Parameter to pass
        20,                 // Priority (High!)
        &I2STaskHandle,     // Task handle
        1                   // Core ID (1 is usually best for apps)
    );
}

void loop(){}               // not used
