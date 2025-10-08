#define VLCFG_IMPLEMENTATION
#define VLCFG_DEBUG
#include "VLConfig.h"

#define VLCFG_SENSOR_ADC_CH (A0)
#define VLCFG_INTERVAL_MS (10)

#define LED_PIN (2)

const char *KEY_TEXT = "t";
const char *KEY_PASS = "p";
const char *KEY_NUMBER = "n";
const char *KEY_IP_ADDR = "i";
const char *KEY_LED_ON = "l";
char textBuff[32 + 1];
char passBuff[32 + 1];
int32_t numberBuff;
uint8_t ipBuff[6];
uint8_t ledOnBuff;
VlcfgEntry configEntries[] = {
  { KEY_TEXT, textBuff, VlcfgType::TEXT_STR, sizeof(textBuff), 0 },
  { KEY_PASS, passBuff, VlcfgType::TEXT_STR, sizeof(passBuff), 0 },
  { KEY_NUMBER, &numberBuff, VlcfgType::INT, sizeof(numberBuff), 0 },
  { KEY_IP_ADDR, ipBuff, VlcfgType::BYTE_STR, sizeof(ipBuff), 0 },
  { KEY_LED_ON, &ledOnBuff, VlcfgType::BOOLEAN, sizeof(ledOnBuff), 0 },
  { nullptr, nullptr, vlcfg::ValueType::NONE, 0, 0 },  // terminator
};

VlcfgReceiver receiver(256);

uint64_t nextSampleTimeMs = 0;
bool finished = false;

void onReceived();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello.");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  receiver.init(configEntries);
  Serial.println("VLCFG Initialized.");
  nextSampleTimeMs = millis() + VLCFG_INTERVAL_MS;
}

void loop() {
  uint64_t nowMs = millis();
  if (!finished) {
    if (nowMs >= nextSampleTimeMs) {
      VlcfgRxState rx_state;
      receiver.update(analogRead(VLCFG_SENSOR_ADC_CH), &rx_state);
      if (rx_state == VlcfgRxState::COMPLETED) {
        Serial.println("VLCFG RX Success.");
        onReceived();
        finished = true;
      } else if (rx_state == VlcfgRxState::ERROR) {
        Serial.println("VLCFG RX Failed.");
        finished = true;
      }
      nextSampleTimeMs += VLCFG_INTERVAL_MS;
    }
    delayMicroseconds(100);
  } else {
    delay(1000);
  }
}

void onReceived() {
  VlcfgEntry *e;

  e = receiver.entry_from_key(KEY_TEXT);
  Serial.print("Text: ");
  if (e->was_received()) {
    Serial.print("'");
    Serial.print(textBuff);
    Serial.println("'");
  } else {
    Serial.println("(none)");
  }

  e = receiver.entry_from_key(KEY_PASS);
  Serial.print("Pass: ");
  if (e->was_received()) {
    Serial.print("'");
    Serial.print(passBuff);
    Serial.println("'");
  } else {
    Serial.println("(none)");
  }

  e = receiver.entry_from_key(KEY_NUMBER);
  Serial.print("Number: ");
  if (e->was_received()) {
    Serial.println(numberBuff);
  } else {
    Serial.println("(none)");
  }

  e = receiver.entry_from_key(KEY_IP_ADDR);
  Serial.print("IP Addr: ");
  if (e->was_received()) {
    Serial.print(ipBuff[0]);
    Serial.print(".");
    Serial.print(ipBuff[1]);
    Serial.print(".");
    Serial.print(ipBuff[2]);
    Serial.print(".");
    Serial.print(ipBuff[3]);
    Serial.println();
  } else {
    Serial.println("(none)");
  }

  e = receiver.entry_from_key(KEY_LED_ON);
  Serial.print("LED On: ");
  if (e->was_received()) {
    bool ledOn = (ledOnBuff != 0);
    Serial.println(ledOn ? "true" : "false");
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  } else {
    Serial.println("(none)");
  }
}
