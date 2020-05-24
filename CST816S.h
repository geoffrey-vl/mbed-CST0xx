
#ifndef CST816S_H
#define CST816S_H

#include "drivers/I2C.h"
#include "InterruptIn.h"
#include "DigitalOut.h"
#include "platform/Callback.h"

class CST816S
{
public:
    enum event_type
    {
        EVENT_TYPE_TOUCH = 0x05,
        EVENT_TYPE_LONG_TOUCH = 0x0C,
        EVENT_TYPE_SWIPE_Y_POSITIVE = 0x01,
        EVENT_TYPE_SWIPE_Y_NEGATIVE = 0x02,
        EVENT_TYPE_SWIPE_X_NEGATIVE = 0x03,
        EVENT_TYPE_SWIPE_X_POSITIVE = 0x04
    };

    struct ts_event
    {
        enum event_type type;
        uint16_t x;
        uint16_t y;
    };

    CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin);
    CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin, mbed::Callback<void(struct ts_event)> touch_event_callback);
    
    void init();
    void setCallBack(mbed::Callback<void(struct ts_event)> touch_event_callback);
    void enableIRQ();
    void disableIRQ();

protected:
    mbed::I2C *_i2c;
    mbed::InterruptIn _touchInterrupt;
    mbed::DigitalOut _reset;
    mbed::Callback<void(struct ts_event)> _touch_event_callback;
    /**
     * Write to registers using I2C. 
     * @return Return 0 for a successful execution.
     */ 
    uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    /**
     * Read from registers using I2C. 
     * @return Return 0 for a successful execution.
     */ 
    uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    void onTouchInterrupt();
};

#endif // CST816S_H
