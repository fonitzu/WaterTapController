#include <Arduino.h>


// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_DEBUG_VERBOSE_NRF5_ESB

// Enable and select radio type attached
//#define MY_RADIO_RF24
#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// nRF52840 can send with +8 dBm
#define MY_NRF5_ESB_PA_LEVEL	(8)

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
	// (2) MySensors child ID for the multimeter.
	// The multimeter is used to report the battery voltage to controller.
	MULTIMETER_CHILD_ID,
} MySensorsChildId;

// Corresponding enum for V_STATUS ( 0 = Off, 1 = On )
typedef enum WaterTapState_t
{
	// Water tap state "OFF".
	Off,
	// Water tap state "ON".
	On
} WaterTapState;

// Water tap's state (On/Off).
WaterTapState CurrentState;

// Valve motor state.
typedef enum MotorState_t
{
	// The motor is off
	MotorOff,
	// The motor opens the valve.
	MotorOpen,
	// The motor closes the valve.
	MotorClose,
} MotorState;

// The time needed to turn the valve on or off (milliseconds).
#define ON_OFF_VALVE_TRIP_TIME		1000

#define SKETCH_NAME		"Water Tap"
#define SKETCH_VERSION	"0.2"


// Maximum time we wait for the controller to send sprinkler's state (milliseconds).
#define CONTROLLER_WAIT_TIME	3000

// Wake-up interval (milliseconds)
#define WAKE_UP_INTERVAL	(10 * 1000/*second*/)

// Multimeter message reporting the battery voltage.
MyMessage voltageMsg( MULTIMETER_CHILD_ID, V_VOLTAGE );

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
	present( MULTIMETER_CHILD_ID, S_MULTIMETER );
}


void setup()
{
	// put your setup code here, to run once:
}

// Return the supply voltage level in millivolt.
uint32_t supplyVoltage( void )
{
	// NRF52:
	// Sampling time 3uS@700uA
	int32_t sample;
	NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
	NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
	NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD << SAADC_CH_PSELP_PSELP_Pos;
	NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos) |
	                          (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
	                          (SAADC_CH_CONFIG_TACQ_3us << SAADC_CH_CONFIG_TACQ_Pos) |
	                          (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
	                          (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
	                          (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
	                          (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos);
	NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
	NRF_SAADC->RESULT.MAXCNT = 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)&sample;

	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->TASKS_START = 1;
	while (!NRF_SAADC->EVENTS_STARTED);
	NRF_SAADC->EVENTS_STARTED = 0;

	NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->TASKS_SAMPLE = 1;
	while (!NRF_SAADC->EVENTS_END);
	NRF_SAADC->EVENTS_END = 0;

	NRF_SAADC->EVENTS_STOPPED = 0;
	NRF_SAADC->TASKS_STOP = 1;
	while (!NRF_SAADC->EVENTS_STOPPED);
	NRF_SAADC->EVENTS_STOPPED = 1;

	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);

	return (sample*3600)/4096;
}

// Report the battery level to controller.
// The battery level is sent as real battery voltage.
void batteryLevel( void )
{
	voltageMsg.set( supplyVoltage() / 1000.0, 3);
	send( voltageMsg );
}

// Main application loop:
// - Request the water tap on/off state from controller with timeout,
// - Verify and update the battery level,
// - Sleep
void loop()
{
	digitalWrite( ACTIVE_LED, LED_ON );
	request( WATER_TAP_CHILD_ID, V_STATUS );
	wait( CONTROLLER_WAIT_TIME, C_REQ, V_STATUS );
	batteryLevel();
	digitalWrite( ACTIVE_LED, LED_OFF );
	smartSleep( WAKE_UP_INTERVAL ); 
}

// Turns the motor off or on (open or close the valve)
void motorControl( MotorState state )
{
	switch( state )
	{
		case MotorOpen:
		{
			digitalWrite( VALVE_CLOSE_PIN, MOTOR_OFF );
			digitalWrite( VALVE_OPEN_PIN, MOTOR_ON );
			return;
		}
		case MotorClose:
		{
			digitalWrite( VALVE_OPEN_PIN, MOTOR_OFF );
			digitalWrite( VALVE_CLOSE_PIN, MOTOR_ON );
			return;
		}
		
		// Off is a safe state.
		// If both VALVE_CLOSE_PIN and VALVE_OPEN_PIN are active,
		// a short circuit will occur between VDD and GND.

		// case Off:
		default:
		{
			digitalWrite( VALVE_CLOSE_PIN, MOTOR_OFF );
			digitalWrite( VALVE_OPEN_PIN, MOTOR_OFF );
			return;
		}
	}
}

// Wait for motor operation (open or close the valve).
void waitForMotor( void )
{
	wait( ON_OFF_VALVE_TRIP_TIME );
}

// Turn the water on.
void OpenWaterTap( void )
{
	CurrentState = On;
	motorControl( MotorOpen );
	waitForMotor();
	motorControl( MotorOff );
	digitalWrite( STATE_LED, LED_ON );
}

// Turn the water off.
void CloseWaterTap( void )
{
	// safety: avoid both pins high because this will short the battery
	motorControl( MotorClose );
	waitForMotor();
	motorControl( MotorOff );
	CurrentState = WaterTapState::Off;
	digitalWrite( STATE_LED, LED_OFF );
}


// Process a Status message from controller.
void receiveStatus( const MyMessage &message )
{
	if( message.getSensor() != WATER_TAP_CHILD_ID )
	{
		Serial.print( "Received V_STATUS message for unknown child " );
		Serial.println( message.getSensor() );
		return;
	}

	// Change only when a new value was received
	if( message.getInt() == CurrentState )
	{
		return;
	}
	
	if( message.getInt() == Off )
	{
		CloseWaterTap();
		return;
	}

	if( message.getInt() == On )
	{
		OpenWaterTap();
		return;
	}

	// Unknown value
	Serial.print( "Unknown sprinkler state received (0=OFF, 1=ON): " );
	Serial.println( message.getInt() );
}

// Process a message received from the controller.
void receive( const MyMessage &message )
{
	switch( message.getType() )
	{
		case V_STATUS:	{ receiveStatus( message ); return; }
		default:
		{
			Serial.println( "Unknown message received:" );
			Serial.println( message.getType() );
		}
	}
}

