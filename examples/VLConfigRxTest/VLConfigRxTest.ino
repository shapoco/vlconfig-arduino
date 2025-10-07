#define VLCFG_IMPLEMENTATION
#define VLCFG_DEBUG
#include "VLConfig.h"

#define VLCFG_SENSOR_ADC_CH (0)
#define VLCFG_INTERVAL_MS (10)

char ssidBuff[32 + 1];
char passBuff[32 + 1];
char ipBuff[6];
char netMaskBuff[6];
char gatewayBuff[6];
VlcfgEntry configEntries[] = {
  { "s", ssidBuff, VlcfgType::TEXT_STR, sizeof(ssidBuff), 0 },
  { "p", passBuff, VlcfgType::TEXT_STR, sizeof(passBuff), 0 },
  { "i", ipBuff, VlcfgType::BYTE_STR, sizeof(ipBuff), 0 },
  { "n", netMaskBuff, VlcfgType::BYTE_STR, sizeof(netMaskBuff), 0 },
  { "g", gatewayBuff, VlcfgType::BYTE_STR, sizeof(gatewayBuff), 0 },
};
#define NUM_CONFIG_ENTRIES (sizeof(configEntries) / sizeof(configEntries[0]))

VlcfgReceiver receiver(256);

uint64_t nextSampleTimeMs = 0;
bool finished = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello.");
  receiver.init(configEntries, NUM_CONFIG_ENTRIES);
  Serial.println("VLCFG Initialized.");
  nextSampleTimeMs = millis() + VLCFG_INTERVAL_MS;
}

void loop() {
  uint64_t nowMs = millis();
  if (!finished) {
    if (nowMs >= nextSampleTimeMs) {
      VlcfgRxState rx_state;
      receiver.update(analogRead(VLCFG_SENSOR_ADC_CH), &rx_state);
      if (rx_state == VlcfgRxState::COMPLETED || rx_state == VlcfgRxState::ERROR) {
        finished = true;
      }
      nextSampleTimeMs += VLCFG_INTERVAL_MS;
    }
    delayMicroseconds(100);
  } else {
    delay(1000);
  }
}
