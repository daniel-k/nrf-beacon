#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>

#include "hardware.hpp"

using namespace xpcc::stm32;

#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::INFO

xpcc::IODeviceWrapper< Hardware::Uart > loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);


typedef xpcc::Nrf24Phy<Hardware::Spi, Hardware::SpiCsn> nrf24phy;


MAIN_FUNCTION
{
	defaultSystemClock::enable();

	Hardware::initialize();


	XPCC_LOG_INFO << "Hello from nrf-beacon" << xpcc::endl;

	nrf24phy::initialize();

	uint8_t rf_ch;
	uint64_t addr;

	while (1)
	{
		Hardware::LedGreen::toggle();
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

		Hardware::LedWhite::toggle();
		xpcc::delayMilliseconds(500);

	}

	return 0;
}
