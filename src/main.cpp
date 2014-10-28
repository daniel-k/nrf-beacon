#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_config.hpp>
#include <cmath>

#include "hardware.hpp"

using namespace xpcc::stm32;

using namespace xpcc::nrf24;

constexpr int id_module_1 = 0x0028003C;
constexpr int id_module_2 = 0x001D003C;


/* Setup logger over uart */

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::INFO

xpcc::IODeviceWrapper< Hardware::Uart > loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);



typedef xpcc::Nrf24Phy<Hardware::Spi, Hardware::SpiCsn> nrf24phy;
typedef xpcc::Nrf24Config<nrf24phy> nrf24config;

MAIN_FUNCTION
{
    defaultSystemClock::enable();

    Hardware::initialize();


    XPCC_LOG_INFO << "Hello from nrf-beacon" << xpcc::endl;

    nrf24phy::initialize();

    nrf24config::setMode(Mode::Rx);
    nrf24config::setSpeed(Speed::MBps2);
    nrf24config::setChannel(45);
    nrf24config::setCrc(Crc::Crc2Byte);

	xpcc::delayMilliseconds(5);

	uint16_t x_idleOffset = 0;
	uint16_t y_idleOffset = 0;

	uint8_t i;	// iterator
	for(i = 0; i < 15; i++)
	{
		xpcc::delayMilliseconds(5);

		x_idleOffset += Hardware::getAnalogStickRawX();
        y_idleOffset += Hardware::getAnalogStickRawY();
	}
	x_idleOffset = x_idleOffset / 15;
	y_idleOffset = y_idleOffset / 15;

	uint16_t x_max, y_max, x_min, y_min;
	x_max = Hardware::getAnalogStickRawX();
	x_min = Hardware::getAnalogStickRawX();

	y_max = Hardware::getAnalogStickRawY();
	y_min = Hardware::getAnalogStickRawY();

    while (1)
    {
        if(Hardware::getUniqueId() == id_module_1)
        {
            Hardware::LedGreen::toggle();
        } else
        {
            Hardware::LedWhite::toggle();
        }

        xpcc::delayMilliseconds(200);

        // clear screen
        XPCC_LOG_INFO.printf("\033[2J");

        uint16_t raw_x, raw_y, x, y;
		int x_normal, y_normal;

        raw_x = Hardware::getAnalogStickRawX();
        raw_y = Hardware::getAnalogStickRawY();

		if(raw_x > x_max)
		{
			x_max = raw_x;
		}
		if(raw_y > y_max)
		{
			y_max = raw_y;
		}
		if(raw_x < x_min)
		{
			x_min = raw_x;
		}
		if(raw_y < y_min)
		{
			y_min = raw_y;
		}

        x = raw_x >> 8;
        y = raw_y >> 8;
	
		x_normal = raw_x - x_idleOffset;
		y_normal = raw_y - y_idleOffset;

#define X_VISUALIZATION_OFFSET 2
#define Y_VISUALIZATION_OFFSET 8

        XPCC_LOG_INFO.printf("Raw X: %4d Y: %4d", raw_x, raw_y);

        // go to next line
        XPCC_LOG_INFO.printf("\033[2;0H");
        XPCC_LOG_INFO.printf("    X: %4d Y: %4d", x, y);

        XPCC_LOG_INFO.printf("\033[3;0H");
        XPCC_LOG_INFO.printf("    X_normal: %4d Y_normal: %4d", x_normal, y_normal);

		// get angle if reasonable normalized values
		if(x_normal > 25 || y_normal > 25 || x_normal < -25 || y_normal < -25)
		{
			double value = std::atan2((double)-x_normal, (double)-y_normal);
			int beforeComma = static_cast<int>(value);
			int afterComma = std::abs(static_cast<int>((value - beforeComma) * 1000));

		    XPCC_LOG_INFO.printf("\033[4;0H");
			XPCC_LOG_INFO.printf("    phi: %d.%03ld", beforeComma, afterComma );

			XPCC_LOG_INFO.printf("\033[%d;%dH", ((int8_t)(cos(value) * -8) + Y_VISUALIZATION_OFFSET + 8),
				((int8_t)(sin(value) * -8) + X_VISUALIZATION_OFFSET + 8));
			XPCC_LOG_INFO.printf("Y");
		}
		else
		{
		    XPCC_LOG_INFO.printf("\033[4;0H");
		    XPCC_LOG_INFO.printf("    phi: NaN");
			XPCC_LOG_INFO.printf("\033[%d;%dH", 8 + Y_VISUALIZATION_OFFSET, 8 + X_VISUALIZATION_OFFSET);
		    XPCC_LOG_INFO.printf("Y");
		}

        XPCC_LOG_INFO.printf("\033[5;0H");
        XPCC_LOG_INFO.printf("    x_max: %4d x_min: %4d", x_max, x_min);
        XPCC_LOG_INFO.printf("\033[6;0H");
        XPCC_LOG_INFO.printf("    y_max: %4d y_min: %4d", y_max, y_min);

		// print frame for better relative visualization
		XPCC_LOG_INFO.printf("\033[%d;0H", Y_VISUALIZATION_OFFSET - 1);
		XPCC_LOG_INFO.printf("OOOOOOOOOOOOOOOO0OO");
		uint8_t i = 0;	// iterator
		for( i = Y_VISUALIZATION_OFFSET; i < 18 + Y_VISUALIZATION_OFFSET; i++ )
		{
			XPCC_LOG_INFO.printf("\033[%d;0H", i);
			XPCC_LOG_INFO.printf("O");
			XPCC_LOG_INFO.printf("\033[%d;19H", i);
			XPCC_LOG_INFO.printf("O");
		}
		XPCC_LOG_INFO.printf("\033[%d;0H", 17 + Y_VISUALIZATION_OFFSET);
		XPCC_LOG_INFO.printf("OOOOOOOOOOOOOO0OO0O");		

        // print X to visualize stick position
        XPCC_LOG_INFO.printf("\033[%d;%dH", y + Y_VISUALIZATION_OFFSET, x + X_VISUALIZATION_OFFSET);
        XPCC_LOG_INFO.printf("X");

        // move cursor back to 0, 0
        XPCC_LOG_INFO.printf("\033[0;0H");
    }

    return 0;
}


