#ifndef HARDWARE_HPP
#define HARDWARE_HPP

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

using namespace xpcc::stm32;


//constexpr int VrefNormal = 1515;
//constexpr int VrefWindow = 30;

class Hardware
{
public:

	static void
	initialize();

	/**
	 * @brief Configure Adc1 for measurement of internal reference
	 */
	static void
	initializeVrefMeasurement(
			uint16_t normal = 1514,
			uint16_t window = 30);


	static bool
	isVoltageLow();

private:

	static uint16_t
	sampleVref();


private:
	static uint16_t vrefNormal;	///< Adc value when voltage is normal
	static uint16_t vrefWindow;	///< When Vref is outside VrefNormal +- VrefWindow
};


#endif
