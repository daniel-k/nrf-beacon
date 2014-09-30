#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_config.hpp>

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

        raw_x = Hardware::getAnalogStickRawX();
        raw_y = Hardware::getAnalogStickRawY();

        x = raw_x / 200 + 3;
        y = raw_y / 200 + 3;


        XPCC_LOG_INFO.printf("Raw X: %4d Y: %4d", raw_x, raw_y);

        // go to next line
        XPCC_LOG_INFO.printf("\033[2;0H");
        XPCC_LOG_INFO.printf("    X: %4d Y: %4d", x, y);

        // print X to visualize stick position
        XPCC_LOG_INFO.printf("\033[%d;%dH", y, x);
        XPCC_LOG_INFO.printf("X");

        // move cursor back to 0,0
        XPCC_LOG_INFO.printf("\033[0;0H");
    }

    return 0;
}
