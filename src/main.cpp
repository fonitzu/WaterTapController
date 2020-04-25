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

// On-board LED Green turned on when the MCU is active.
// Used for debugging only, to avoid power consumption.
#define ACTIVE_LED	PIN_LED1
// On-board LED Red, turned on when the water tap is open.
// Used for debugging only, to avoid power consumption.
#define STATE_LED	PIN_LED2_R

#undef LED_ON
#undef LED_OFF

// Digital output level for turning a LED off.
#define LED_OFF		1
// Digital output level for turning a LED off.
// Turn LEDs on only in debug mode
#ifdef MY_DEBUG
	#define LED_ON		0
#else
	#define LED_ON		LED_OFF
#endif

// Turn water tap's motor on.
#define MOTOR_ON	1
// Turn water tap's motor off.
#define MOTOR_OFF	0

// GPIO used to turn the water tap on.
#define VALVE_OPEN_PIN	13
// GPIO used to turn the water tap off.
#define VALVE_CLOSE_PIN	15

// Child IDs for MySensors library.
typedef enum
{
	// (0) MySensors child ID for the water tap switch.
	WATER_TAP_CHILD_ID = 0,
	// (1) MySensors child ID for the temperature sensor.
	TEMPERATURE_CHILD_ID,
} MySensorsChildId;

#define SKETCH_NAME		"Water Tap"
#define SKETCH_VERSION	"0.1"

// Shortly blink with the blue LED to signal "initialization done".
// This LED will always blink after reset, no matter if MY_DEBUG is
// defined or not.
// It shall occur only when new batteries are inserted.
void blink( void )
{
	pinMode( PIN_LED2_B, OUTPUT );
	for( uint8_t i = 0; i < 3; ++ i )
	{
		digitalWrite( PIN_LED2_B, 0 );
		delay( 100 );
		digitalWrite( PIN_LED2_B, LED_OFF );
		delay( 300 );
	}
}


// Function called by MySensors before setup().
// MySensors will try to initialize the transport layer
// between before() and setup(). 
// If this operation fails, setup() may not be reached.
// Performing the setup and closing the water tap in before()
// ensures that no water will leak (worst case scenario).
void before( void )
{
	pinMode( VALVE_OPEN_PIN, OUTPUT );
	digitalWrite( VALVE_OPEN_PIN, MOTOR_OFF );
	
	pinMode( VALVE_CLOSE_PIN, OUTPUT );
	digitalWrite( VALVE_CLOSE_PIN, MOTOR_OFF );
	
	pinMode( ACTIVE_LED, OUTPUT );
	digitalWrite( ACTIVE_LED, LED_ON );
	
	pinMode( STATE_LED, OUTPUT );
	digitalWrite( STATE_LED, LED_OFF );

	blink();
}


// Present child nodes to the controller.
// Required by MySensors library.
void presentation( void )
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo( SKETCH_NAME, SKETCH_VERSION );

	present( WATER_TAP_CHILD_ID, S_BINARY );
	present( TEMPERATURE_CHILD_ID, S_TEMP );
}

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}