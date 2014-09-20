#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_phy.hpp>

using namespace xpcc::stm32;

#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::INFO

xpcc::IODeviceWrapper< Usart2 > loggerDevice;
xpcc::log::Logger xpcc::log::info(loggerDevice);

typedef SystemClock<Pll<ExternalClock<MHz12>, MHz72> > defaultSystemClock;


typedef GpioOutputB4 LedLeft;
typedef GpioOutputB5 LedRight;
typedef GpioOutputB2 Csn;


typedef SpiSimpleMaster1 Spi;


typedef xpcc::Nrf24Phy<Spi, Csn> nrf24phy;

typedef GpioInputB13 AdcIn0;


MAIN_FUNCTION
{
	defaultSystemClock::enable();

	LedLeft::setOutput(xpcc::Gpio::Low);
	LedRight::setOutput(xpcc::Gpio::Low);

	Csn::setOutput(xpcc::Gpio::High);

	GpioOutputA7::connect(Spi::Mosi);
	GpioInputA6::connect(Spi::Miso);
	GpioOutputA5::connect(Spi::Sck);
	Spi::initialize<defaultSystemClock, 9000000>();


	GpioOutputA2::connect(Usart2::Tx);
	GpioInputA3::connect(Usart2::Rx, Gpio::InputType::PullUp);
	Usart2::initialize<defaultSystemClock, 115200>(12);

	ADC1_2_COMMON->CCR &= ~ADC_CR_ADEN;
	ADC1_2_COMMON->CCR &= ~ADC_CR_ADDIS;
	ADC3_4_COMMON->CCR &= ~ADC34_CCR_VREFEN;
	xpcc::delayMicroseconds(100);

	ADC1_2_COMMON->CCR |= ADC12_CCR_VREFEN;



	Adc1::initialize(Adc1::ClockMode::Asynchronous, Adc1::Prescaler::Div256,
						Adc1::CalibrationMode::SingleEndedInputsMode, true);

	Adc1::setChannel(Adc1::Channel::InternalReference, Adc1::SampleTime::Cycles602);


	Adc3::initialize(Adc3::ClockMode::Asynchronous, Adc3::Prescaler::Div256,
						Adc3::CalibrationMode::SingleEndedInputsMode, true);
	AdcIn0::connect(Adc3::Channel5);
	Adc3::setChannel(AdcIn0::Adc3Channel, Adc3::SampleTime::Cycles182);

	XPCC_LOG_INFO << "Hello from nRF24-phy-test example" << xpcc::endl;

	nrf24phy::initialize();


	ADC1_2_COMMON->CCR &= ~ADC_CR_ADEN;
//	ADC1_2_COMMON->CCR &= ~ADC_CR_ADDIS;

//	ADC3_4_COMMON->CCR &= ~ADC34_CCR_VREFEN;
	xpcc::delayMicroseconds(10);

	ADC1_2_COMMON->CCR |= ADC12_CCR_VREFEN;

	ADC1_2_COMMON->CCR |= ADC_CR_ADEN;

	uint8_t rf_ch;
	uint64_t addr;

	while (1)
	{
		LedLeft::toggle();
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

		Adc1::startConversion();
		// wait for conversion to finish
		while(!Adc1::isConversionFinished);
		xpcc::delayMicroseconds(50);
		// print result
		int adcValue = Adc1::getValue();

		// ~1514 when 3.3V
		XPCC_LOG_INFO.printf("Battery voltage:            %d\n\n", adcValue);

		LedRight::toggle();
		xpcc::delayMilliseconds(500);

	}

	return 0;
}
