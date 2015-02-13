#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_config.hpp>

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


typedef xpcc::Nrf24Phy<Hardware::Spi, Hardware::SpiCsn, Hardware::Ce> nrf24phy;
typedef xpcc::Nrf24Config<nrf24phy> nrf24config;

MAIN_FUNCTION
{
    defaultSystemClock::enable();
    xpcc::cortex::SysTickTimer::enable();

    XPCC_LOG_INFO << "[log-start] nrf-beacon" << xpcc::endl;

    Hardware::initialize();
    nrf24phy::initialize(4);	// 4 byte payload


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

    return 0;
}
