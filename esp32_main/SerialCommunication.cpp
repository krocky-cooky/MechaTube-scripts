#include "SerialCommunication.hpp"

SerialCommunication::SerialCommunication()
  : power(false), motorControl(false), mode(Mode::SpdCtrl), trq(0.0f), spd(0.0f)
{
}

void SerialCommunication::loop()
{
  char buf[SERIAL_BUFSIZE] = "";
  int i = 0;
  int retval = 0;

  while (Serial.available()) {
    char c = Serial.read();         // read a recieved character
    buf[i] = c;                     // add the character to the buffer
    i++;                            // move the index to the next address
    if (i >= SERIAL_BUFSIZE) i = 0; // if the index exceeds the length of buffer, reset the index to zero

    if (c == '\n') { // 改行でコマンドの終了を検知する
      i = 0;
      decodeCommand(buf);
    }
  }
}

char SerialCommunication::decodeCommand(const char *buf)
{
  char key;
  float value;
  sscanf(buf, "%c%f", &key, &value); // scan the command

  switch (key) { // copy the commanded value
    case 'p':
      power = (value > 0.5) ? true : false; // float value contains small error so it is not exactly equal to 0 or 1 integer
      if (!power && motorControl) {
        motorControl = false; // if the power command is false but the motorControl is still on, turns the motorControl OFF
      }
      trq = 0.0;
      spd = 0.0;
      break;
    case 'm':
      motorControl = (value > 0.5) ? true : false;
      if (motorControl && !power) {
        power = true; // if the motorControl command is true but motor power is off, turns the power ON
      }
      trq = 0.0;
      spd = 0.0;
      break;
    case 't':
      mode = Mode::TrqCtrl;
      power = true;
      motorControl = true;
      trq = value;
      spd = 0.0;
      break;
    case 'e':
      // increaseOfToraueForEccentricMotion = value;
      break;
    case 's':
      mode = Mode::SpdCtrl;
      power = true;
      motorControl = true;
      spd = value;
      trq = 0.0;
      break;
    case 'i':
      // maxSpeedWhileConcentricMotion = value;
      break;
    case 'a':
      // increaseOfToraueWhenPeak = value;
      break;
    case 'b':
      // rotationAngleFromInitialPositionWhenPeak = value;
      break;
    case 'c':
      // rangeOfTorqueChange = value;
      break;
    default:
      return 0;
  }
  return key;
}
