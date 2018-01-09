#include <SPI.h>

#include "ServoTimer1.h"
#include "symax_protocol.h"

ServoTimer1 left;
ServoTimer1 right;

nrf24l01p wireless; 
symaxProtocol protocol;

unsigned long time = 0;

void setup() {
  Serial.begin(115200);

  left.attach(9);
  right.attach(10);

  // SS pin must be set as output to set SPI to master !
  pinMode(SS, OUTPUT);

  // Set CE pin to 5 and CS pin to 4
  wireless.setPins(5,4);
  
  // Set power (PWRLOW,PWRMEDIUM,PWRHIGH,PWRMAX)
  wireless.setPwr(PWRLOW);
  
  protocol.init(&wireless);
  
  time = micros();
  Serial.println("Start");
}

rx_values_t rxValues;

unsigned long newTime;

void loop() 
{
  time = micros();
  uint8_t value = protocol.run(&rxValues); 
  newTime = micros();
  int left_value, right_value;
   
  switch (value)
  {
    case NOT_BOUND:
      Serial.println("Not bound");
      break;
      
    case BIND_IN_PROGRESS:
      Serial.println("Bind in progress");
      break;
    
    case BOUND_NEW_VALUES:
      Serial.print(newTime - time);
      Serial.print(" :\t");Serial.print(rxValues.throttle);
      Serial.print("\t"); Serial.print(rxValues.yaw);
      Serial.print("\t"); Serial.print(rxValues.pitch);
      Serial.print("\t"); Serial.print(rxValues.roll);
      Serial.print("\t"); Serial.print(rxValues.trim_yaw);
      Serial.print("\t"); Serial.print(rxValues.trim_pitch);
      Serial.print("\t"); Serial.print(rxValues.trim_roll);
      Serial.print("\t"); Serial.print(rxValues.video);
      Serial.print("\t"); Serial.print(rxValues.picture);
      Serial.print("\t"); Serial.print(rxValues.highspeed);
      Serial.print("\t"); Serial.println(rxValues.flip);

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

      left.write(map(left_value, 0, 255, 0, 180));
      right.write(map(right_value, 0, 255, 0, 180));
      break;

    case BOUND_NO_VALUES:
      break;

    default:
      break;
  }
}
