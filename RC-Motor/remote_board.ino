#include <SPI.h>
#include <RF24.h>

// Pin mapping matches the remote-control circuit and the vehicle receiver.
const uint8_t SPEED_POT_PIN = A0;
const uint8_t STEERING_POT_PIN = A1;
const uint8_t NRF_CE_PIN = 7;
const uint8_t NRF_CSN_PIN = 8;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte RADIO_ADDRESS[6] = "CAR01";
const uint8_t RADIO_CHANNEL = 76;
const unsigned long SEND_INTERVAL_MS = 20;

// Must match the receiver's ControlPacket exactly.
struct ControlPacket {
  uint16_t speed;
  uint16_t steering;
};

bool radioReady = false;
unsigned long lastSendMs = 0;
unsigned long lastInitAttemptMs = 0;
unsigned long lastDiagnosticMs = 0;

bool initializeRadio() {
  if (!radio.begin()) return false;
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.setPayloadSize(sizeof(ControlPacket));
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("[Sender] Start; Serial 115200"));
  radioReady = initializeRadio();
  lastInitAttemptMs = millis();
  Serial.print(F("[Sender] nRF24 init: "));
  Serial.println(radioReady ? F("OK") : F("ERROR; retry after 1s"));
}

void loop() {
  unsigned long now = millis();

  // Retry initialization if the radio was not ready at startup.
  if (!radioReady) {
    if (now - lastInitAttemptMs >= 1000) {
      lastInitAttemptMs = now;
      radioReady = initializeRadio();
    }
    return;
  }

  if (now - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  // Send raw 10-bit ADC readings (no scaling or steering/speed processing).
  ControlPacket packet;
  packet.speed = (uint16_t)analogRead(SPEED_POT_PIN);
  packet.steering = (uint16_t)analogRead(STEERING_POT_PIN);
  bool sent = radio.write(&packet, sizeof(packet));
  
  // Log at a readable rate
  if (now - lastDiagnosticMs >= 500) {
    lastDiagnosticMs = now;
    Serial.print(F("[Sender] speedADC=")); Serial.print(packet.speed);
    Serial.print(F(" steeringADC=")); Serial.print(packet.steering);
    Serial.print(F(" radio=")); Serial.print(radioReady ? F("OK") : F("ERROR"));
    Serial.print(F(" TX=")); Serial.println(sent ? F("ACK") : F("no ACK"));
  }
}
