#ifndef HARDWARE_HPP
#define HARDWARE_HPP

#include <stdint.h>
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

using namespace xpcc::stm32;

typedef SystemClock<Pll<ExternalClock<MHz12>, MHz72> > defaultSystemClock;

constexpr uint32_t UartSpeed    = 115200;
constexpr uint32_t SpiSpeed     = 4500000;

class Hardware
{
public:

    typedef GpioOutputB5                    nLedGreen;
    typedef GpioOutputB4                    nLedWhite;
    typedef xpcc::GpioInverted<nLedGreen>   LedGreen;
    typedef xpcc::GpioInverted<nLedWhite>   LedWhite;

    typedef SpiMaster1                		Spi;
    typedef GpioOutputA5                    SpiSck;
    typedef GpioInputA6                     SpiMiso;
    typedef GpioOutputA7                    SpiMosi;
    typedef GpioOutputB2                    SpiCsn;
    typedef xpcc::GpioInverted<SpiCsn>      SpiCs;

    typedef GpioOutputB1					Ce;

    typedef Usart2                          Uart;
    typedef GpioOutputA2                    UartTx;
    typedef GpioInputA3                     UartRx;


    static void
    initialize()
    {
        initializeLeds();
        initializeSpi();
        initializeGpio();
        initializeUart();
        initializeVrefMeasurement();
    }

    static uint32_t
    getUniqueId()
    {
        return *((uint32_t*)0x1FFFF7AC);
    }

    /**
     * @brief Configure Adc1 for measurement of internal reference
     */
    static void
    initializeVrefMeasurement(
            uint16_t normal = 1514,
            uint16_t window = 30);

    static void
    initializeLeds()
    {
        LedGreen::setOutput(xpcc::Gpio::Low);
        LedWhite::setOutput(xpcc::Gpio::Low);
    }

    static void
    initializeGpio()
    {
    	Ce::setOutput();
    }

    static void
    initializeSpi()
    {
        SpiMosi::connect(Spi::Mosi);
        SpiMiso::connect(Spi::Miso);
        SpiSck::connect(Spi::Sck);
        Spi::initialize<defaultSystemClock, SpiSpeed>();

        SpiCsn::setOutput(xpcc::Gpio::High);
    }

    static void
    initializeUart()
    {
        UartTx::connect(Uart::Tx);
        UartRx::connect(Uart::Rx, Gpio::InputType::PullUp);
        Uart::initialize<defaultSystemClock, UartSpeed>(12);
    }

    static bool
    isVoltageLow()
    {
        return (sampleVref() - vrefNormal) > vrefWindow;
    }

//  static uint16_t
//  chargeCurrent();

private:

    static uint16_t
    sampleVref();


private:
    static uint16_t vrefNormal; ///< Adc value when voltage is normal
    static uint16_t vrefWindow; ///< When Vref is outside VrefNormal +- VrefWindow
};


#endif
