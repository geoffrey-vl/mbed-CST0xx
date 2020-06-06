#include "CST816S.h"

#include "mbed_events.h"
#include "rtos.h"
#include "platform/mbed_assert.h"

#include "mbed-trace/mbed_trace.h"
#define TRACE_GROUP "TCHP"

#define CST816S_I2C_ADDRESS                     0x15

#define FT_TOUCH_POINT_NUM                      2 
#define HYN_TOUCH_EVENT_TYPE_POS                1
#define HYN_TOUCH_X_H_POS                       3
#define HYN_TOUCH_X_L_POS                       4
#define HYN_TOUCH_Y_H_POS                       5
#define HYN_TOUCH_Y_L_POS                       6
#define HYN_TOUCH_EVENT_POS                     3
#define HYN_TOUCH_ID_POS                        5
#define HYN_TOUCH_STEP                          6
#define HYN_TOUCH_XY_POS                        7
#define HYN_TOUCH_MISC                          8


CST816S::CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin) : 
    CST816S(i2c, interruptPin, resetPin, nullptr)
{
}

CST816S::CST816S(mbed::I2C* i2c, PinName interruptPin, PinName resetPin, mbed::Callback<void(std::vector<CST816S::TouchInfo>)> touch_event_callback) : 
    _i2c(i2c),
    _touchInterrupt(interruptPin),
    _reset(resetPin, 0),
    _touch_event_callback(touch_event_callback)
{
}

void CST816S::init()
{
    _reset = 0;
    ThisThread::sleep_for(20);
    _reset = 1;

    // TODO: check Chip ID and firmware version on initialization
}

void CST816S::setCallBack(mbed::Callback<void(std::vector<CST816S::TouchInfo>)> touch_event_callback)
{
    _touch_event_callback = touch_event_callback;
}

void CST816S::enableIRQ()
{
    // Request the shared queue and attach touch handler to interrupt pin
    EventQueue *sharedEventQueue = mbed::mbed_event_queue();
    _touchInterrupt.fall(sharedEventQueue->event(this, &CST816S::onTouchInterrupt)); 
}

void CST816S::disableIRQ()
{
    // detach touch handler to interrupt pin
    _touchInterrupt.fall(nullptr); 
}

void CST816S::onTouchInterrupt()
{
    //When IRQ pin was fallen the touchpad device has wakened up.
    // So now we must query the device for the data it has captured
    std::vector<CST816S::TouchInfo> data = getTouchData();

    if (_touch_event_callback)
        _touch_event_callback(data);
}

std::vector<CST816S::TouchInfo> CST816S::getTouchData()
{
    uint16_t ret = i2c_reg_read(CST816S_I2C_ADDRESS, 0x00, touchData, CST816S_TOUCHDATA_SIZE);
    std::vector<CST816S::TouchInfo> data;
    if(ret != 0)
        return data;

    //get number of touch points
    uint8_t touch_point_num = touchData[FT_TOUCH_POINT_NUM] & 0x0F;

    //parse the configured amount of touch events
    for(int i=0; i<touch_point_num; i++)
    {
        CST816S::TouchInfo eventData;

        auto xHigh = touchData[HYN_TOUCH_X_H_POS + (HYN_TOUCH_STEP * i)] & 0x0f;
        auto xLow = touchData[HYN_TOUCH_X_L_POS + (HYN_TOUCH_STEP * i)];
        eventData.x = (xHigh << 8) | xLow;

        auto yHigh = touchData[HYN_TOUCH_Y_H_POS + (HYN_TOUCH_STEP * i)] & 0x0f;
        auto yLow = touchData[HYN_TOUCH_Y_L_POS + (HYN_TOUCH_STEP * i)];
        eventData.y = (yHigh << 8) | yLow;

        auto action = touchData[HYN_TOUCH_EVENT_POS + (HYN_TOUCH_STEP * i)] >> 6; 
        eventData.action = static_cast<Action>(action);

        eventData.fingerId = touchData[HYN_TOUCH_ID_POS + (HYN_TOUCH_STEP * i)] >> 4;
        eventData.pressure = touchData[HYN_TOUCH_XY_POS + (HYN_TOUCH_STEP * i)];
        eventData.area = touchData[HYN_TOUCH_MISC + (HYN_TOUCH_STEP * i)] >> 4;

        data.push_back(eventData);
    }

    return data;
}

uint16_t CST816S::i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
                                        uint16_t length)
{
    const int TEMP_BUF_SIZE = 32;
    MBED_ASSERT(length > TEMP_BUF_SIZE);
    
    uint8_t tmp[TEMP_BUF_SIZE];
    tmp[0] = reg_addr;
    memcpy(tmp + 1, reg_data, length);

    int ret = _i2c->write(i2c_addr << 1, (const char *)tmp, length + 1, false);
    if (ret != 0)
        tr_err("i2c_reg_write failed, code %i", ret);
    return ret;
}


uint16_t CST816S::i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, 
                                uint8_t *reg_data, uint16_t length)
{
    int ret = _i2c->write(i2c_addr << 1, (const char *)&reg_addr, 1, true);
    if(ret != 0) {
        tr_err("i2c_reg_read failed writing address, code %i.", ret);
        return ret;
    }

    ret = _i2c->read(i2c_addr << 1, (char *)reg_data, length, false);
    if (ret != 0)
        tr_err("i2c_reg_read failed reading data, code %i.", ret);
    return ret;
}