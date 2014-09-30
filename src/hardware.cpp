#include "hardware.hpp"

uint16_t Hardware::vrefNormal;
uint16_t Hardware::vrefWindow;

void
Hardware::initializeVrefMeasurement(uint16_t normal, uint16_t window)
{
    vrefNormal = normal;
    vrefWindow = window;

    Adc1::initialize(Adc1::ClockMode::Asynchronous, Adc1::Prescaler::Div256,
                     Adc1::CalibrationMode::SingleEndedInputsMode,
                     true);

    // disable adc again
    ADC1_2_COMMON->CCR &= ~ADC_CR_ADEN;

    // enable vref measurement
    ADC1_2_COMMON->CCR |= ADC12_CCR_VREFEN;

    // reenable adc
    ADC1_2_COMMON->CCR |= ADC_CR_ADEN;


    // one dummy measurement needs to be done after configuring adc
    sampleVref();

}

void
Hardware::initializeAdc()
{
    Adc3::initialize(Adc3::ClockMode::Asynchronous, Adc3::Prescaler::Div256,
                         Adc3::CalibrationMode::SingleEndedInputsMode,
                         true);



    Adc4::initialize(Adc4::ClockMode::Asynchronous, Adc4::Prescaler::Div256,
                         Adc4::CalibrationMode::SingleEndedInputsMode,
                         true);

    Adc35::connect(Adc3::Channel5);
    Adc44::connect(Adc4::Channel4);

    Adc3::setChannel(Adc35::Adc3Channel, Adc3::SampleTime::Cycles602);
    Adc4::setChannel(Adc44::Adc4Channel, Adc4::SampleTime::Cycles602);

    getAnalogStickRawX();
    getAnalogStickRawY();
}


uint16_t
Hardware::sampleVref()
{
    // configure adc 1 to measure vref
    Adc1::setChannel(Adc1::Channel::InternalReference, Adc1::SampleTime::Cycles602);

    Adc1::startConversion();

    while(!Adc1::isConversionFinished);

    return Adc1::getValue();
}

uint16_t
Hardware::getAnalogStickRawY()
{
    constexpr uint16_t samples = 2;

    uint32_t data = 0;

    for(uint8_t i = 0; i < samples; i++)
    {
        Adc3::startConversion();
        while(!Adc3::isConversionFinished);
        data += Adc3::getValue();
    }

    return data / samples;
}

uint16_t
Hardware::getAnalogStickRawX()
{
    constexpr uint16_t samples = 2;

    uint32_t data = 0;

    for(uint8_t i = 0; i < samples; i++)
    {
        Adc4::startConversion();
        while(!Adc4::isConversionFinished);
        data += Adc4::getValue();
    }

    return data / samples;
}
