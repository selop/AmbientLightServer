#include "LEDController.h"
#include "Logging.h"
#include "HardwareConfig.h"

#include <boost/timer.hpp>

LEDController* LEDController::Instance = 0;

// wiringPi only compiles on the RASPBERRY_PI
#ifdef RASPBERRY_PI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

LEDController::LEDController() : m_isSetup(true), m_prevColorBuffer(0)
{
	// TODO: remove this log, APA102 depends on SPI and has fixed pins
	LOG_INFO("LEDController: setting up with ClockPin1=" << GPIO_CLOCK_PIN1 << " DataPin1=" << GPIO_DATA_PIN1
					<< " ClockPin2=" << GPIO_CLOCK_PIN2 << " DataPin2=" << GPIO_DATA_PIN2);

	Instance = this;

	// Prev color buffer for interpolation
	m_prevColorBuffer = new Color[TOTAL_NUM_LEDS];
	for (int i=0; i<TOTAL_NUM_LEDS; i++)
		m_prevColorBuffer[i] = Color(0, 0, 0);

	#ifdef RASPBERRY_PI
	wiringPiSetup();
	if (wiringPiSPISetup(0, 6000000) < 0)
	{
		LOG_ERROR("LEDController: Failed to setup wiringPi");
		m_isSetup = false;
		return;
	}

	//pinMode(GPIO_CLOCK_PIN1, OUTPUT);
	//pinMode(GPIO_CLOCK_PIN2, OUTPUT);

	//pinMode(GPIO_DATA_PIN1, OUTPUT);
	//pinMode(GPIO_DATA_PIN2, OUTPUT);
	#endif
}

void LEDController::UpdateLeds(Color* colorBuffer, float deltaTime, float fadeTimeMS)
{
	if (!m_isSetup)
		return;

	if (colorBuffer == 0)
		return;

	// This will update the led strands of 25 separately
	// I could have just attached the two strands together making it
	//  one big strand of 50, which uses just on data and one clock pin.
	// But unfortunately I was getting a lot of noise at the end of the 
	//  strand, so I split it up

	// Linear interpolation term for smoothness!
	float deltaTimeMS = deltaTime * 1000.0f;
	if (fadeTimeMS < deltaTimeMS)
		fadeTimeMS = deltaTimeMS;
	float lerpTerm = deltaTimeMS/fadeTimeMS;

	// Send opening bits for APA102
	uint8_t buf[1];
	for(int i = 0; i < 4; i++) {
		buf[0] = 0x00;
		wiringPiSPIDataRW(0, buf, 1);
	}

	uint8_t led_frame[4];
	// Update all LEDs
	for (int i=0; i<TOTAL_NUM_LEDS; i++)
	{
		Color color = lerpColor(m_prevColorBuffer[i], colorBuffer[i], lerpTerm);
		
		led_frame[0] = 0b11100000 | (0b00011111 & 15);
		led_frame[1] = color.ByteB();
		led_frame[2] = color.ByteG();
		led_frame[3] = color.ByteR();
		wiringPiSPIDataRW(0, led_frame, 4);
		
		//ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, color.ByteR());
		//ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, color.ByteG());
		//ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, color.ByteB());

		m_prevColorBuffer[i] = color;
	}

	// Send closing bits for APA102
	for(int i = 0; i < 4; i++) {
		buf[0] = 0xFF;
		wiringPiSPIDataRW(0, buf, 1);
	}

	/* Update second strand of 25
	for (int i=NUM_LEDS_PER_STRAND; i<TOTAL_NUM_LEDS; ++i)
	{
		Color color = lerpColor(m_prevColorBuffer[i], colorBuffer[i], lerpTerm);

		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, color.ByteR());
		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, color.ByteG());
		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, color.ByteB());

		m_prevColorBuffer[i] = color;
	}
	
	#ifdef RASPBERRY_PI
	digitalWrite(GPIO_CLOCK_PIN1, 0);
	digitalWrite(GPIO_CLOCK_PIN2, 0);
	delay(1);
	#endif
	*/
}

void LEDController::UpdateLedsFixed(Color fixedColor, float deltaTime)
{
	// Update first strand of 25
	for (int i=0; i<NUM_LEDS_PER_STRAND; ++i)
	{
		ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, fixedColor.ByteR());
		ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, fixedColor.ByteG());
		ShiftOut8Bits(GPIO_CLOCK_PIN1, GPIO_DATA_PIN1, fixedColor.ByteB());
	}

	// Update second strand of 25
	for (int i=NUM_LEDS_PER_STRAND; i<TOTAL_NUM_LEDS; ++i)
	{
		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, fixedColor.ByteR());
		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, fixedColor.ByteG());
		ShiftOut8Bits(GPIO_CLOCK_PIN2, GPIO_DATA_PIN2, fixedColor.ByteB());
	}
	
	#ifdef RASPBERRY_PI
	digitalWrite(GPIO_CLOCK_PIN1, 0);
	digitalWrite(GPIO_CLOCK_PIN2, 0);
	delay(1);
	#endif
}

void LEDController::ShiftOut8Bits(int clockPin, int dataPin, uint8_t c)
{
	#ifdef RASPBERRY_PI
	for (int bit = 0; bit < 8; bit++)
	{
		bool val = (c >> (7 - bit)) & 1;

		digitalWrite(clockPin, 0);
		digitalWrite(dataPin, val);
		digitalWrite(clockPin, 1);
	}
	#endif
}
