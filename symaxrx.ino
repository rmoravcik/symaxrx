#include <SPI.h>

#include "ServoTimer1.h"
#include "symax_protocol.h"

#define SERIAL_DEBUG false

#if SERIAL_DEBUG
uint8_t lastState = 255;
#endif

#define LED_STATUS true

#if LED_STATUS
#define LED_PIN 1
unsigned long statusLedChangeTime = 0;
byte statusLedState = HIGH;
#endif

ServoTimer1 left;
ServoTimer1 right;

nrf24l01p wireless; 
symaxProtocol protocol;

void setup() {
#if SERIAL_DEBUG
  Serial.begin(115200);
#endif

  left.attach(9);
  right.attach(10);

  // SS pin must be set as output to set SPI to master !
  pinMode(SS, OUTPUT);

#if LED_STATUS
  // Set status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
#endif

  // Set CE pin to 5 and CS pin to 4
  wireless.setPins(5,4);
  
  // Set power (PWRLOW,PWRMEDIUM,PWRHIGH,PWRMAX)
  wireless.setPwr(PWRLOW);

  protocol.init(&wireless);

#if SERIAL_DEBUG
  Serial.println("Start...");
#endif
}

rx_values_t rxValues;

void loop()
{
  int left_value, left_mapped, right_value, right_mapped;
  uint8_t state = protocol.run(&rxValues);

#if LED_STATUS
  unsigned long currentMillis = millis();

  if (state == BOUND)
  {
    digitalWrite(LED_PIN, LOW);
    statusLedState = LOW;
  }
  else
  {
    // Blink the LED  
    if (currentMillis - statusLedChangeTime > 200)
    {
      if (statusLedState == LOW)
      {
        digitalWrite(LED_PIN, HIGH);
        statusLedState = HIGH;
      }
      else 
      {
        digitalWrite(LED_PIN, LOW);
        statusLedState = LOW;
      }
      statusLedChangeTime = millis();
    }
  }
#endif

#if SERIAL_DEBUG
  uint8_t currentState = protocol.getState();
  if (currentState != lastState)
  {
    switch(currentState)
    {
      case BOUND:
        Serial.println(F("Bound"));
        break;
      case NO_BIND:
        Serial.println(F("Not bound"));
      case WAIT_FIRST_SYNCHRO:
        Serial.println(F("Waiting first synchronization"));
        break;
    }
    lastState = currentState;
  }
#endif

  switch (state)
  {
    case NOT_BOUND:
      left.write(0);
      right.write(0);
      break;

    case BIND_IN_PROGRESS:
      break;

    case BOUND_NEW_VALUES:
//#if SERIAL_DEBUG
//      Serial.print(rxValues.throttle);
//      Serial.print("\t"); Serial.print(rxValues.yaw);
//      Serial.print("\t"); Serial.print(rxValues.pitch);
//      Serial.print("\t"); Serial.print(rxValues.roll);
//      Serial.print("\t"); Serial.print(rxValues.trim_yaw);
//      Serial.print("\t"); Serial.print(rxValues.trim_pitch);
//      Serial.print("\t"); Serial.print(rxValues.trim_roll);
//      Serial.print("\t"); Serial.print(rxValues.video);
//      Serial.print("\t"); Serial.print(rxValues.picture);
//      Serial.print("\t"); Serial.print(rxValues.highspeed);
//      Serial.print("\t"); Serial.println(rxValues.flip);
//#endif

      if (rxValues.roll > 0) // left
      {
        left_value = map(rxValues.roll, 0, 127, rxValues.throttle, 0);
        right_value = rxValues.throttle;
      }
      else if (rxValues.roll < 0) // right
      {
        left_value = rxValues.throttle;
        right_value = map(rxValues.roll, 0, -127, rxValues.throttle, 0);
      }
      else // forward
      {
        left_value = rxValues.throttle;
        right_value = rxValues.throttle;
      }

      if (left_value > 230)
        left_mapped = 4.8 * left_value - 1044;
      else
        left_mapped = 0.195 * left_value + 15;

      if (right_value > 230)
        right_mapped = 4.8 * right_value - 1044;
      else
        right_mapped = 0.195 * right_value + 15;

#if SERIAL_DEBUG
      Serial.print(left_value);
      Serial.print("\t => "); Serial.print(left_mapped);
      Serial.print("\t\t"); Serial.print(right_value);
      Serial.print("\t => "); Serial.println(right_mapped);
#endif

      left.write(left_mapped);
      right.write(right_mapped);
      break;

    case BOUND_NO_VALUES:
      break;

    default:
      break;
  }
}
