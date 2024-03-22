#include <Arduino.h>
#include "sbus.h"
#include "crsf.h"
#include "_config.h"

#if defined(END_POINT_CUSTOM) && (not defined(STICK_END_MIN) || not defined(STICK_END_MAX) || not defined(SWITCH_END_MIN) || not defined(SWITCH_END_MAX))
#error "END_POINT_CUSTOM defined but end point is not defined!"
#elif defined(END_POINT_CUSTOM)
// Ignore RC brand.
#elif defined(TX_FUTABA)
#define STICK_END_MIN 368
#define STICK_END_MAX 1680
#define SWITCH_END_MIN 144
#define SWITCH_END_MAX 1904
#elif defined(TX_FRSKY)
#define STICK_END_MIN 172
#define STICK_END_MAX 1810
#define SWITCH_END_MIN 172
#define SWITCH_END_MAX 1810
#elif defined(TX_FLYSKY)
#define STICK_END_MIN 193
#define STICK_END_MAX 1791
#define SWITCH_END_MIN 193
#define SWITCH_END_MAX 1791
#else
#error "Unsupported RC brand and END_POINT_CUSTOM is not defined."
#endif

#define AUX1 4
enum input_order
{
#if defined(INPUT_AETR)
    INPUT_AILERON,
    INPUT_ELEVATOR,
    INPUT_THROTTLE,
    INPUT_RUDDER
#elif defined(INPUT_TAER)
    INPUT_THROTTLE,
    INPUT_AILERON,
    INPUT_ELEVATOR,
    INPUT_RUDDER
#else
#error "Unsupported input channel order."
#endif
};

enum output_order
{
#if defined(OUTPUT_AETR)
    OUTPUT_AILERON,
    OUTPUT_ELEVATOR,
    OUTPUT_THROTTLE,
    OUTPUT_RUDDER
#elif defined(OUTPUT_TAER)
    OUTPUT_THROTTLE,
    OUTPUT_AILERON,
    OUTPUT_ELEVATOR,
    OUTPUT_RUDDER
#else
#error "Unsupported output channel order."
#endif
};

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
  rcChannels[AUX1] = CRSF_CHANNEL_MIN;
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
        rcChannels[OUTPUT_THROTTLE] = CRSF_CHANNEL_MIN; // Throttle low
        rcChannels[OUTPUT_AILERON] = CRSF_CHANNEL_MID;  //
        rcChannels[OUTPUT_ELEVATOR] = CRSF_CHANNEL_MID; //
        rcChannels[OUTPUT_RUDDER] = CRSF_CHANNEL_MID;   //
        rcChannels[AUX1] = CRSF_CHANNEL_MIN;     // ARM low
      }
      else
      {
        rcChannels[OUTPUT_AILERON] = map(sbus.ch().at(INPUT_AILERON), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[OUTPUT_ELEVATOR] = map(sbus.ch().at(INPUT_ELEVATOR), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[OUTPUT_THROTTLE] = map(sbus.ch().at(INPUT_THROTTLE), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        rcChannels[OUTPUT_RUDDER] = map(sbus.ch().at(INPUT_RUDDER), STICK_END_MIN, STICK_END_MAX, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
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