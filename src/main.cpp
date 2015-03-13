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

	Hardware::initialize();

	XPCC_LOG_INFO << "[log-start] nrf-beacon" << xpcc::endl;

	nrf24data::Address addr_own, addr_other;

	uint32_t id = Hardware::getUniqueId();
	XPCC_LOG_INFO.printf("HardwareId = 0x%08x\n", id);

	switch(id)
	{
	case id_module_1:
		addr_own   = addr_module_3;
		addr_other = addr_module_1;
		break;

	case id_module_3:
		addr_own   = addr_module_1;
		addr_other = addr_module_3;
		break;

	case id_module_2:
		// This module is dead.

	default:
		XPCC_LOG_INFO << "Module Id unknown, exiting" << xpcc::endl;
		return 0;
		break;
	}

	XPCC_LOG_INFO.printf("Own   address = 0x%02x\n", addr_own);
	XPCC_LOG_INFO.printf("Other address = 0x%02x\n", addr_other);

	nrf24phy::initialize(payload_length_phy);
	nrf24data::initialize(base_addr, addr_own);

	// Hop channel
	uint8_t currentChannel = 20;

	// Set channel as this is not set by data layer
	nrf24config::setChannel(currentChannel);

	nrf24config::setAutoRetransmitCount(nrf24config::AutoRetransmitCount::Retransmit10);
	nrf24config::setAutoRetransmitDelay(nrf24config::AutoRetransmitDelay::us750);

	nrf24data::Packet packet;
	uint32_t* data = reinterpret_cast<uint32_t*>(packet.payload.data);

	*data = currentChannel;

	packet.dest = addr_other;

	xpcc::PeriodicTimer sendTimer(50);
	xpcc::PeriodicTimer printTimer(1000);

	uint32_t packetsSentOk = 0;
	uint32_t packetsSentFail = 0;
	uint32_t packetsAcked = 0;
	uint32_t packetsNacked = 0;


	while (1)
	{
		if (id == id_module_1)
		{
			if (sendTimer.execute())
			{
				if(nrf24data::sendPacket(packet))
				{
					// XPCC_LOG_INFO << "Packet queued for sending" << xpcc::endl;
					++packetsSentOk;
				} else {
					XPCC_LOG_ERROR << "Packet NOT sent" << xpcc::endl;
					XPCC_LOG_DEBUG.printf("Status: 0x%02x\n", nrf24phy::readStatus());
					XPCC_LOG_DEBUG.printf("FifoStatus: 0x%02x\n", nrf24phy::readFifoStatus());
					++packetsSentFail;
				}
				Hardware::LedWhite::toggle();
			}

			// wait for feedback
			if (nrf24data::isPacketProcessed())
			{
				switch (nrf24data::getSendingFeedback()) {
				case nrf24data::SendingState::FinishedAck:
					XPCC_LOG_DEBUG << "ACK" << xpcc::endl;
					Hardware::LedGreen::toggle();
					++packetsAcked;
					break;
				case nrf24data::SendingState::FinishedNack:
					XPCC_LOG_DEBUG << "NACK" << xpcc::endl;
					++packetsNacked;
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

			if (printTimer.execute()) 
			{
				XPCC_LOG_INFO.printf("---------------- Channel %3d -----------------------------\n", currentChannel);
				// XPCC_LOG_INFO.printf("SentOk   %4d\n", packetsSentOk);
				// XPCC_LOG_INFO.printf("SentFail %4d\n", packetsSentFail);
				// XPCC_LOG_INFO.printf("Ack      %4d\n", packetsAcked);
				// XPCC_LOG_INFO.printf("Nack     %4d\n", packetsNacked);
				XPCC_LOG_INFO.printf("RatioA%% %4d\n", int( (double(packetsAcked)  / (double(packetsSentOk)) ) * 100.0 )  );
				// XPCC_LOG_INFO.printf("RatioN%% %4d\n", int( (double(packetsNacked) / (double(packetsSentOk)) ) * 100.0 )  );
				packetsSentOk = 0;
				packetsSentFail = 0;
			 	packetsAcked = 0;
			 	packetsNacked = 0;

 				++currentChannel;
 				if (currentChannel > 80) { currentChannel = 0; }

 				*data = currentChannel;

 				// Request channel change

 				xpcc::Timeout timeout(10000);

				XPCC_LOG_INFO.printf("Request channel change to %d \n", currentChannel);
 				while(true)
 				{	
 					bool ret = nrf24data::sendPacket(packet);
 					// XPCC_LOG_INFO.printf("Request channel change returned %d\n", ret);

 					while (!nrf24data::isPacketProcessed()) { 
 						nrf24data::update();
 					};

 					// XPCC_LOG_INFO.printf("Channel change packet aired to %d\n", currentChannel);

					if ( (nrf24data::getSendingFeedback() == nrf24data::SendingState::FinishedAck) or (timeout.execute()) ) {
						// XPCC_LOG_INFO.printf("Channel change acked to %d\n", currentChannel);
						printTimer.restart(1000);
						break;
					}

					xpcc::delayMilliseconds(5);
 				} // while
 				nrf24config::setChannel(currentChannel);
 				xpcc::delayMilliseconds(5);
			}

		}

		if (id == id_module_3)
		{

			if (nrf24data::getPacket(packet))
			{
				Hardware::LedGreen::toggle();
				XPCC_LOG_INFO.printf("Received packet from 0x%02x\n", packet.src);
				XPCC_LOG_INFO.printf("Data: %02x %02x %02x %02x\n",
						packet.payload.data[3],
						packet.payload.data[2],
						packet.payload.data[1],
						packet.payload.data[0]);
				if ((*data != currentChannel) and (packet.src == addr_other)) {
					currentChannel = *data;
					XPCC_LOG_INFO.printf("Changing channel to %d\n", currentChannel);
					nrf24config::setChannel(currentChannel);
					Hardware::LedWhite::toggle();
				}
			}
		}

		nrf24data::update();
	}
}
