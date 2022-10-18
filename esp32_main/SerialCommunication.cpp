#include "SerialCommunication.hpp"

SerialCommunication::SerialCommunication()
{
}

bool SerialCommunication::check()
{
  int i = 0;
  int retval = 0;

  memset(buf, 0, SERIAL_BUFSIZE); // fill buffer zero

  while (Serial.available()) {
    char c = Serial.read();         // read a recieved character
    buf[i] = c;                     // add the character to the buffer
    i++;                            // move the index to the next address
    if (i >= SERIAL_BUFSIZE) i = 0; // if the index exceeds the length of buffer, reset the index to zero

    if (c == '\n' || c == '}') { // 改行でコマンドの終了を検知する
      i = 0;
      return true;
    }
  }
  return false;
}

String SerialCommunication::receive()
{
  if (buf[0] != 0) {
    return String(buf);
  } else {
    return String("");
  }
}
