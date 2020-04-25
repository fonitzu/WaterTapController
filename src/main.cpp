#include <Arduino.h>


// Enable debug prints to serial monitor
#define MY_DEBUG


// Enable and select radio type attached
//#define MY_RADIO_RF24
#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

// Smart sleep delay, to allow the controller send any buffered messages
// Use a small value to save battery.
#define MY_SMART_SLEEP_WAIT_DURATION_MS	50

#include <MySensors.h>

// End of MySensors configuration
// -----------------------------------------------------------------------------


void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}