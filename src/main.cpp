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

	constexpr int channelMin = 40;
	constexpr int channelMax = 80;

	// Hop channel
	uint8_t currentChannel = channelMin;

	// Set channel as this is not set by data layer
	nrf24config::setChannel(currentChannel);

	nrf24config::setAutoRetransmitCount(nrf24config::AutoRetransmitCount::Retransmit10);
	nrf24config::setAutoRetransmitDelay(nrf24config::AutoRetransmitDelay::us750);
	nrf24config::setSpeed(nrf24config::Speed::MBps1);
	nrf24config::setCrc(nrf24config::Crc::Crc2Byte);

	nrf24data::Packet packet;
	uint32_t* data = reinterpret_cast<uint32_t*>(packet.payload.data);

	*data = currentChannel;

	packet.dest = addr_other;

	xpcc::PeriodicTimer sendTimer(20);
	xpcc::PeriodicTimer printTimer(1000);

	constexpr int noPacketTimeoutInterval = 200;
	xpcc::Timeout noPacketTimeout;

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
					++packetsSentOk;
				} else {
					XPCC_LOG_DEBUG << "Packet NOT sent" << xpcc::endl;
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
					noPacketTimeout.stop();
					Hardware::LedGreen::toggle();
					++packetsAcked;
					break;
				case nrf24data::SendingState::FinishedNack:
					XPCC_LOG_DEBUG << "NACK" << xpcc::endl;
					++packetsNacked;
					break;
				default:
					XPCC_LOG_DEBUG << "Neither ACK nor NACK" << xpcc::endl;
					break;
				}
			}

			if (printTimer.execute())
			{
				XPCC_LOG_DEBUG.printf("---------------- Channel %3d (0x%02x)-----------------------------\n", currentChannel, currentChannel);
				XPCC_LOG_DEBUG.printf("SentOk   %4d\n", packetsSentOk);
				XPCC_LOG_DEBUG.printf("SentFail %4d\n", packetsSentFail);
				XPCC_LOG_DEBUG.printf("Ack      %4d\n", packetsAcked);
				XPCC_LOG_DEBUG.printf("Nack     %4d\n", packetsNacked);
				XPCC_LOG_DEBUG.printf("RatioA%% %4d\n", int( (double(packetsAcked)  / (double(packetsSentOk)) ) * 100.0l )  );
				XPCC_LOG_DEBUG.printf("RatioN%% %4d\n", int( (double(packetsNacked) / (double(packetsSentOk)) ) * 100.0l )  );

				// Reset stats
				packetsSentOk = 0;
				packetsSentFail = 0;
			 	packetsAcked = 0;
			 	packetsNacked = 0;

			 	// increment channel
 				++currentChannel;
 				if (currentChannel > channelMax) { currentChannel = channelMin; }
 				*data = currentChannel;


 				// Request channel change
				XPCC_LOG_DEBUG.printf("Request channel change to %d \n", currentChannel);
 				while(true)
 				{

 					if(!nrf24data::sendPacket(packet)) {
 						XPCC_LOG_DEBUG.printf("Request channel change not possible\n");
 						continue;
 					} else {
 						XPCC_LOG_DEBUG << "Channel change requested successfully" << xpcc::endl;
 					}

 					XPCC_LOG_DEBUG << "Wait for feedback" << xpcc::endl;
 					do {
 						nrf24data::update();
 					}
 					while (!nrf24data::isPacketProcessed());

 					XPCC_LOG_DEBUG << "Got feedback" << xpcc::endl;

					if ( (nrf24data::getSendingFeedback() == nrf24data::SendingState::FinishedAck)) {
						XPCC_LOG_DEBUG << "Channel change acknowledged" << xpcc::endl;
//						nrf24phy::dumpRegisters();
						printTimer.restart();

						break;
					} else {
						XPCC_LOG_DEBUG.printf("Got NACK when requesting channel change to 0x%02x\n", currentChannel);
					}
 				} // while

 				noPacketTimeout.restart(noPacketTimeoutInterval);
 				nrf24config::setChannel(currentChannel);
			}
		}

		if(noPacketTimeout.execute())
		{
			--currentChannel;
			nrf24config::setChannel(currentChannel);
			Hardware::LedWhite::toggle();
			XPCC_LOG_INFO << "Switch back to channel " << currentChannel << xpcc::endl;
		}

		if (id == id_module_3)
		{
			if (nrf24data::getPacket(packet))
			{
				noPacketTimeout.stop();

				uint8_t rpd = nrf24phy::readRegister(nrf24phy::NrfRegister::RPD);

				Hardware::LedGreen::toggle();
				XPCC_LOG_DEBUG.printf("Received packet from 0x%02x\n", packet.src);
				XPCC_LOG_DEBUG << "RPD was " << rpd << xpcc::endl;
				XPCC_LOG_DEBUG.printf("Data: %02x %02x %02x %02x\n",
						packet.payload.data[3],
						packet.payload.data[2],
						packet.payload.data[1],
						packet.payload.data[0]);

				if ((*data != currentChannel) and (packet.src == addr_other)) {
					currentChannel = *data;
					XPCC_LOG_INFO.printf("Changing channel to %d\n", currentChannel);
					nrf24config::setChannel(currentChannel);
					Hardware::LedWhite::toggle();

					noPacketTimeout.restart(noPacketTimeoutInterval);
				}
			}
		}

		nrf24data::update();
	}
}
