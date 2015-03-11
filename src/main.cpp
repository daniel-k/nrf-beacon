#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_config.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_data.hpp>

#include "hardware.hpp"

using namespace xpcc::stm32;

constexpr int id_module_1 = 0x002e0029;
constexpr int id_module_2 = 0x0028003c;		// module 2 is broken
constexpr int id_module_3 = 0x001d003c;

constexpr uint64_t base_addr = 0xdeadbeef00;

constexpr uint8_t addr_module_1 = 0x11;
constexpr uint8_t addr_module_2 = 0x22;
constexpr uint8_t addr_module_3 = 0x33;

/* Setup logger over uart */

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::INFO

xpcc::IODeviceWrapper<Hardware::Uart, xpcc::IOBuffer::BlockIfFull> loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);

typedef xpcc::Nrf24Phy<Hardware::Spi, Hardware::SpiCsn, Hardware::Ce> nrf24phy;
typedef xpcc::Nrf24Config<nrf24phy> nrf24config;
typedef xpcc::Nrf24Data<nrf24phy> nrf24data;

constexpr int payload_length_phy = 6;
constexpr int payload_length_data = payload_length_phy -
                                    sizeof(nrf24data::Header);


MAIN_FUNCTION
{
	defaultSystemClock::enable();
	xpcc::cortex::SysTickTimer::enable();

	XPCC_LOG_INFO << "[log-start] nrf-beacon" << xpcc::endl;

	nrf24data::Address addr_own, addr_other;

	uint32_t id = Hardware::getUniqueId();
	switch(id)
	{
	case id_module_1:
		addr_own = addr_module_1;
		addr_other = addr_module_3;
		break;

	case id_module_3:
		addr_own = addr_module_3;
		addr_other = addr_module_1;
		break;

	case id_module_2:
	default:
		XPCC_LOG_INFO << "Module Id unknown, exiting" << xpcc::endl;
		return 0;
		break;
	}



	Hardware::initialize();
	nrf24phy::initialize(payload_length_phy);
	nrf24data::initialize(base_addr, addr_own);

	// Set channel as this is not set by data layer
	nrf24config::setChannel(10);

	nrf24config::setAutoRetransmitCount(nrf24config::AutoRetransmitCount::Retransmit5);
	nrf24config::setAutoRetransmitDelay(nrf24config::AutoRetransmitDelay::us750);

	nrf24data::Packet packet;
	uint32_t* data = reinterpret_cast<uint32_t*>(packet.payload.data);

	*data = 0;

	xpcc::PeriodicTimer sendTimer(2000);
	xpcc::Timeout answerTimeout(500);

	answerTimeout.stop();

	while (1)
	{
		if (answerTimeout.execute() || sendTimer.execute())
		{
			*data += 1;

			if(nrf24data::sendPacket(packet))
			{
				XPCC_LOG_INFO << "Packet queued for sending" << xpcc::endl;
			} else {
				XPCC_LOG_ERROR << "Packet NOT sent" << xpcc::endl;
				XPCC_LOG_DEBUG.printf("Status: 0x%02x\n", nrf24phy::readStatus());
				XPCC_LOG_DEBUG.printf("FifoStatus: 0x%02x\n", nrf24phy::readFifoStatus());
			}
			answerTimeout.stop();
			sendTimer.restart(2000);
			Hardware::LedWhite::toggle();
		}

		// wait for feedback
		if (nrf24data::isPacketProcessed())
		{
			switch (nrf24data::getSendingFeedback()) {
			case nrf24data::SendingState::FinishedAck:
				XPCC_LOG_INFO << "ACK" << xpcc::endl;
				Hardware::LedGreen::toggle();
				break;
			case nrf24data::SendingState::FinishedNack:
				XPCC_LOG_INFO << "NACK" << xpcc::endl;
				break;
			case nrf24data::SendingState::DontKnow:
				XPCC_LOG_INFO << "Don't know" << xpcc::endl;
				break;
			case nrf24data::SendingState::Failed:
				XPCC_LOG_INFO << "Failed" << xpcc::endl;
				break;
			default:
				XPCC_LOG_INFO << "unknown error" << xpcc::endl;
				break;
			}
		}

		if (nrf24data::getPacket(packet))
		{
			XPCC_LOG_INFO.printf("Received packet from 0x%02x\n", packet.src);
			XPCC_LOG_INFO.printf("Data: %02x %02x %02x %02x\n",
					packet.payload.data[3],
					packet.payload.data[2],
					packet.payload.data[1],
					packet.payload.data[0]);

			answerTimeout.restart(500);
		}


		nrf24data::update();
	}
}
