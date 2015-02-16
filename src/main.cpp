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

xpcc::IODeviceWrapper< Hardware::Uart, xpcc::IOBuffer::BlockIfFull > loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::debug(loggerDevice);



typedef xpcc::Nrf24Phy<Hardware::Spi, Hardware::SpiCsn, Hardware::Ce> nrf24phy;
typedef xpcc::Nrf24Config<nrf24phy> nrf24config;
typedef xpcc::Nrf24Data<nrf24phy> nrf24data;



void dataLayerTest();
void simplePhyTest();

constexpr int payload_length_phy = 4;
constexpr int payload_length_data = payload_length_phy - sizeof(nrf24data::header_t);

MAIN_FUNCTION
{
    defaultSystemClock::enable();
    xpcc::cortex::SysTickTimer::enable();

    XPCC_LOG_INFO << "[log-start] nrf-beacon" << xpcc::endl;

    Hardware::initialize();
    nrf24phy::initialize(payload_length_phy);	// 4 byte payload

    dataLayerTest();

    return 0;
}

void
dataLayerTest()
{
    uint32_t id = Hardware::getUniqueId();

    nrf24data::packet_t* packet = nrf24data::allocatePacket(payload_length_data);

	XPCC_LOG_INFO.printf("packet.data: %08x\n", packet->data);

    nrf24config::setChannel(10);

    xpcc::PeriodicTimer sendTimer(500);

    if(id == id_module_1)
    {
    	// will be transmitter
    	XPCC_LOG_INFO << "I'm module #1, transmitter" << xpcc::endl;

    	nrf24data::initialize(base_addr, addr_module_1);

    	memset(packet->data, 0xCD, payload_length_data);

    	XPCC_LOG_INFO.printf("packet.data AFTER MEMSET: %08x\n", packet->data);

    	packet->dest = addr_module_3;
    	packet->length = payload_length_data;

    	XPCC_LOG_INFO << "Data length " << packet->length << xpcc::endl;

    	XPCC_LOG_INFO << "Still alive" << xpcc::endl;

    	while(1)
    	{
    		if(sendTimer.execute())
    		{
				if(nrf24data::sendPacket(*packet))
				{
					XPCC_LOG_INFO << "Packet sent" << xpcc::endl;
				} else
				{
					XPCC_LOG_INFO << "Packet not sent" << xpcc::endl;
				}

				// wait for feedback
				while(nrf24data::getSendingFeedback() == nrf24data::SendingState::Busy)
				{
					XPCC_LOG_INFO << "Waiting for feedback" << xpcc::endl;
					xpcc::delayMilliseconds(1);
				}

				nrf24data::SendingState feedback = nrf24data::getSendingFeedback();

				switch(feedback)
				{
				case nrf24data::SendingState::FinishedAck:
					XPCC_LOG_INFO << "ACK" << xpcc::endl;
					break;
				case nrf24data::SendingState::FinishedNack:
					XPCC_LOG_INFO << "NACK!" << xpcc::endl;
					break;
				case nrf24data::SendingState::DontKnow:
					XPCC_LOG_INFO << "don't know" << xpcc::endl;
					break;
				case nrf24data::SendingState::Failed:
					XPCC_LOG_INFO << "failed" << xpcc::endl;
					break;
				default:
					XPCC_LOG_INFO << "error" << xpcc::endl;
					break;
				}

				*((uint16_t*) packet->data) += 1;
    		}
    	}

    } else if(id == id_module_3)
    {
    	// will be transmitter
    	XPCC_LOG_INFO << "I'm module #3, receiver" << xpcc::endl;

    	nrf24data::initialize(base_addr, addr_module_3);

    	while(1)
    	{
    		if(nrf24data::isPacketAvailable())
    		{
    			nrf24data::getPacket(*packet);

    			XPCC_LOG_INFO << "Received packet" << xpcc::endl;
    			XPCC_LOG_INFO.printf("Data: %x %x\n",
    			                     packet->data[1],
    			                     packet->data[0]);
    		}
    		/*
    		else {
    			nrf24phy::pulseCe();
    		}
			*/
    		if(sendTimer.execute())
    		{
    			XPCC_LOG_INFO << "Still alive" << xpcc::endl;
    		}
    	}
    }


}


void
simplePhyTest()
{
    nrf24config::setSpeed(nrf24config::Speed::kBps250);
    nrf24config::setChannel(10);
    nrf24config::setCrc(nrf24config::Crc::Crc1Byte);
    nrf24config::setAutoRetransmitDelay(nrf24config::AutoRetransmitDelay::us1000);
    nrf24config::setRfPower(nrf24config::RfPower::dBm0);
    nrf24config::setAutoRetransmitCount(nrf24config::AutoRetransmitCount::Retransmit15);

    uint8_t data[4] = {0,0,0,0};

    uint32_t id = Hardware::getUniqueId();

    if(id == id_module_1)
    {
    	XPCC_LOG_INFO << "I'm module #1" << xpcc::endl;
    	XPCC_LOG_INFO << "I'm going to be a receiver" << xpcc::endl;

    	nrf24phy::setRxAddress(nrf24phy::Pipe::PIPE_1, base_addr | addr_module_1);
    	nrf24config::enablePipe(nrf24config::Pipe::PIPE_1, true);

    	nrf24config::setMode(nrf24config::Mode::Rx);
    	nrf24config::powerUp();
    	Hardware::Ce::set();

    	xpcc::PeriodicTimer RpdReadout(1000);


    	while(1)
    	{
    		if( (!(nrf24phy::readFifoStatus() & (uint8_t)nrf24phy::FifoStatus::RX_EMPTY)) ||
    		    (nrf24phy::readStatus() & ((uint8_t)nrf24phy::Status::RX_DR)))
    		{
    			// read payload
    			nrf24phy::readRxPayload(data);

				/* Clear RX_DR flag after payload is read */
    			nrf24phy::setBits(nrf24phy::NrfRegister::STATUS, nrf24phy::Status::RX_DR);

    			XPCC_LOG_INFO.printf("Received payload %x %x %x %x\n", data[3], data[2], data[1], data[0]);

    			Hardware::LedGreen::toggle();
    		}

    		if(RpdReadout.execute())
    		{
    			XPCC_LOG_INFO.printf("Rpd is: %d \n", nrf24phy::readRegister(nrf24phy::NrfRegister::RPD));

    		}
    	}
    } else if(id == id_module_2)
    {
    	XPCC_LOG_INFO << "I'm module #2" << xpcc::endl;
    	while(1)
    	{

    	}
    } else if(id == id_module_3)
    {
    	XPCC_LOG_INFO << "I'm module #3" << xpcc::endl;
    	XPCC_LOG_INFO << "I'm going to be a transmitter" << xpcc::endl;

    	nrf24config::setMode(nrf24config::Mode::Tx);
    	nrf24config::powerUp();
    	Hardware::Ce::set();

    	nrf24phy::setRxAddress(nrf24phy::Pipe::PIPE_0, base_addr | addr_module_1);
    	nrf24config::enablePipe(nrf24config::Pipe::PIPE_0, true);

    	// send to module #1
    	nrf24phy::setTxAddress(base_addr | addr_module_1);

    	/* Timer to send packets every 1000ms */
		xpcc::PeriodicTimer sendPacket(1000);

    	while(1)
    	{
    		if(sendPacket.execute())
    		{
				nrf24phy::writeTxPayload(data, 4);

				// increment data
				*(( uint32_t* ) data) += 1;
    		}

    		/* Check if packet was sent successfully  */
    		if(nrf24phy::readStatus() & ( (uint8_t)nrf24phy::Status::TX_DS | (uint8_t)nrf24phy::Status::MAX_RT ))
    		{
    			XPCC_LOG_INFO.printf("ARC: %d\n", nrf24phy::readRegister(nrf24phy::NrfRegister::OBSERVE_TX) & 0x0F);

    			if(nrf24phy::readStatus() & (uint8_t)nrf24phy::Status::MAX_RT)
    			{
    				XPCC_LOG_INFO.printf("Packet lost, MAX_RT reached\n");
    				nrf24phy::setBits(nrf24phy::NrfRegister::STATUS, nrf24phy::Status::MAX_RT);

    				Hardware::LedWhite::toggle();
    				Hardware::LedGreen::reset();

    				xpcc::delayMilliseconds(300);

    			} else
    			{
    				XPCC_LOG_INFO.printf("Packet successfully sent\n");
    				nrf24phy::setBits(nrf24phy::NrfRegister::STATUS, nrf24phy::Status::TX_DS);

    				Hardware::LedWhite::reset();

    				Hardware::LedGreen::toggle();
    			}


    		}
    	}
    } else {
    	XPCC_LOG_INFO.printf("Unknown id: %x\n", id);
    }
}
