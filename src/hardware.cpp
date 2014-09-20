#include "hardware.hpp"

uint16_t Hardware::vrefNormal;
uint16_t Hardware::vrefWindow;

void Hardware::initializeVrefMeasurement(uint16_t normal, uint16_t window)
{
	vrefNormal = normal;
	vrefWindow = window;

	Adc1::initialize(Adc1::ClockMode::Asynchronous, Adc1::Prescaler::Div256,
						Adc1::CalibrationMode::SingleEndedInputsMode, true);

	// disable adc again
	ADC1_2_COMMON->CCR &= ~ADC_CR_ADEN;

	// enable vref measurement
	ADC1_2_COMMON->CCR |= ADC12_CCR_VREFEN;

	// reenable adc
	ADC1_2_COMMON->CCR |= ADC_CR_ADEN;


	// one dummy measurement needs to be done after configuring adc
	sampleVref();

}

uint16_t Hardware::sampleVref()
{
	// configure adc 1 to measure vref
	Adc1::setChannel(Adc1::Channel::InternalReference, Adc1::SampleTime::Cycles602);

	Adc1::startConversion();

	while(!Adc1::isConversionFinished);

	return Adc1::getValue();
}
