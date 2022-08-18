#include <Arduino.h>
#include "sbus.h"
#include "crsf.h"
#include "_config.h"

#if defined(FUTABA)
#define STICK_END_MIN 368
#define STICK_END_MAX 1680
#define SWITCH_END_MIN 144
#define SWITCH_END_MAX 1904
#elif defined(FRSKY)
#define STICK_END_MIN 172
#define STICK_END_MAX 1810
#define SWITCH_END_MIN 172
#define SWITCH_END_MAX 1810
#elif defined(TURNIGY_EVOLUTION)
#define STICK_END_MIN 193
#define STICK_END_MAX 1791
#define SWITCH_END_MIN 193
#define SWITCH_END_MAX 1791
#else
#define STICK_END_MIN 368
#define STICK_END_MAX 1680
#define SWITCH_END_MIN 144
#define SWITCH_END_MAX 1904
#endif

bfs::SbusRx sbus(&Serial);

HardwareSerial crsf(1);

uint8_t crsfPacket[CRSF_PACKET_SIZE];
int rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

void setup()
{
  // Serial.begin(115200);
// begin the SBUS communication
#if defined(ESP32)
  sbus.Begin(13, 13, true);
#else
  sbus.Begin();
#endif
  rcChannels[4] = CRSF_CHANNEL_MIN;
  for (int i = 12; i < 16; i++)
  {
    rcChannels[i] = CRSF_CHANNEL_MID;
  }
  delay(1000);
#if defined(ESP32)
  crsf.begin(SERIAL_BAUDRATE, SERIAL_8N1, 3, 1, true);
#else
  crsf.begin(SERIAL_BAUDRATE, SERIAL_8N1);
#endif
}

void loop()
{
  uint32_t currentMicros = micros();
  if (currentMicros > crsfTime)
  {
    if (sbus.Read())
    {
      if (sbus.failsafe())
      {
// Serial.println("S.Bus fail safe.");
#if defined(AETR) && not defined(AETR_TO_TAER)
        rcChannels[0] = CRSF_CHANNEL_MID;
        rcChannels[1] = CRSF_CHANNEL_MID;
        rcChannels[2] = CRSF_CHANNEL_MIN;
        rcChannels[3] = CRSF_CHANNEL_MID;
#else
        rcChannels[0] = CRSF_CHANNEL_MIN;
        rcChannels[1] = CRSF_CHANNEL_MID;
        rcChannels[2] = CRSF_CHANNEL_MID;
        rcChannels[3] = CRSF_CHANNEL_MID;
#endif
        rcChannels[4] = CRSF_CHANNEL_MIN;
      }
      else
      {
#if defined(AETR) && defined(AETR_TO_TAER)
        rcChannels[0] = map(sbus.ch().at(2), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[1] = map(sbus.ch().at(0), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[2] = map(sbus.ch().at(1), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[3] = map(sbus.ch().at(3), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
#else
        rcChannels[0] = map(sbus.ch().at(0), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[1] = map(sbus.ch().at(1), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[2] = map(sbus.ch().at(2), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[3] = map(sbus.ch().at(3), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
#endif
        for (int i = 4; i < 12; i++)
        {
          rcChannels[i] = map(sbus.ch().at(i), SWITCH_END_MIN, SWITCH_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        }
      }
      crsfPreparePacket(crsfPacket, rcChannels);
      crsf.write(crsfPacket, CRSF_PACKET_SIZE);
    }
    crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
  }
}