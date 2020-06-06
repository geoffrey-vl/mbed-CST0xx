
#ifndef CST816S_H
#define CST816S_H

#include "drivers/I2C.h"
#include "InterruptIn.h"
#include "DigitalOut.h"
#include "platform/Callback.h"
#include <vector>

#define CST816S_TOUCHDATA_SIZE          63
#define HYN_MAX_POINTS                  10

class CST816S
{
public:
    enum class Gesture : uint8_t {
        None = 0x00,
        SlideDown = 0x01,
        SlideUp = 0x02,
        SlideLeft = 0x03,
        SlideRight = 0x04,
        SingleTap = 0x05,
        DoubleTap = 0x0B,
        LongPress = 0x0C
    };

    enum class Action : uint8_t {
        Down = 0,
        Up = 1,
        Contact = 2
    };

    struct TouchInfo {
        uint16_t x;
        uint16_t y;
        Action action;
        uint8_t fingerId;
        uint8_t pressure;
        uint8_t area;
        Gesture gesture;
    };


    CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin);
    CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin, mbed::Callback<void(std::vector<CST816S::TouchInfo> )> touch_event_callback);
    
    void init();
    void setCallBack(mbed::Callback<void(std::vector<CST816S::TouchInfo>)> touch_event_callback);
    void enableIRQ();
    void disableIRQ();

protected:
    mbed::I2C *_i2c;
    mbed::InterruptIn _touchInterrupt;
    mbed::DigitalOut _reset;
    mbed::Callback<void(std::vector<CST816S::TouchInfo>)> _touch_event_callback;
    uint8_t touchData[CST816S_TOUCHDATA_SIZE];

    std::vector<CST816S::TouchInfo> getTouchData();
    void onTouchInterrupt();
    /**
     * Write to registers using I2C. 
     * @return Return 0 for a successful execution, nonzero on failure.
     */ 
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    /**
     * Read from registers using I2C. 
     * @return Return 0 for a successful execution, nonzero on failure.
     */ 
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
};

#endif // CST816S_H
