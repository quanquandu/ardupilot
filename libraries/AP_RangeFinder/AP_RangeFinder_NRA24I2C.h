#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

#define AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR   0x70
#define AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING 0x51

class AP_RangeFinder_NRA24I2C : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_NRA24I2C(RangeFinder::RangeFinder_State &_state,
    								AP_RangeFinder_Params &_params,
                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool _init(void);
    void _timer(void);

    uint16_t distance;
    bool new_distance;
    
    // start a reading
    bool start_reading(void);
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    enum class Status {
        WAITTING = 0,
        GET_HEAD_ONCE,
        GET_HEAD,
        GET_TAIL_ONCE,
        GET_TAIL,
        GET_ONE_FRAME,
        WAITTING_FOR_TAIL,
    }_reading_state = Status::WAITTING;

    uint32_t last_reading_ms = 0;
    uint16_t last_distance_cm = 0;
    uint8_t linebuf[50];
    uint8_t buffer_count = 0;
};
