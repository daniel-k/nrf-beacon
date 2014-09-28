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

    uint8_t rf_ch;
    uint64_t addr;



    while (1)
    {
        if(Hardware::getUniqueId() == id_module_1)
        {
            Hardware::LedGreen::toggle();
        } else
        {
            Hardware::LedWhite::toggle();
        }

        xpcc::delayMilliseconds(500);

        nrf24phy::setRxAddress(0, 0xdeadb33f05);
        addr = nrf24phy::getRxAddress(0);
        XPCC_LOG_INFO.printf("Setting RX_P0 address to:  0xDEADB33F05\n");
        XPCC_LOG_INFO.printf("Reading RX_P0 address:     0x%x%x\n", static_cast<uint32_t>((addr >> 32) & 0xffffffff), static_cast<uint32_t>(addr & 0xffffffff));

        nrf24phy::setTxAddress(0xabcdef55ff);
        addr = nrf24phy::getTxAddress();
        XPCC_LOG_INFO.printf("Setting TX address to:     0xABCDEF55FF\n");
        XPCC_LOG_INFO.printf("Reading TX address:        0x%x%x\n", static_cast<uint32_t>((addr >> 32) & 0xffffffff), static_cast<uint32_t>(addr & 0xffffffff));

        rf_ch = nrf24phy::readRegister(xpcc::nrf24::Register::RF_CH);
        XPCC_LOG_INFO.printf("Expected output for RF_CH: 0x2\n");
        XPCC_LOG_INFO.printf("Reading RF_CH:             0x%x\n\n", rf_ch);


        if(Hardware::isVoltageLow())
        {
            XPCC_LOG_INFO.printf("Battery voltage is low!\n");
        }

//      XPCC_LOG_INFO.printf("Unique id: 0x ");
//      for(int i = 2; i >= 0; i--)
//      {
//          XPCC_LOG_INFO.printf("%08x ", unique_id_base[i]);
//          xpcc::delayMilliseconds(1);
//      }
//      XPCC_LOG_INFO.printf("\n");

    }

    return 0;
}
