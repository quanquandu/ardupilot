/*
 * 增加头文件------AP_RangeFinder_NRA24.h
 */
#ifndef __AP_RANGEFINDER_NRA24_H__

#define __AP_RANGEFINDER_NRA24_H__
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"

//定义NRA24驱动
class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend
{
public:
    //添加构造函数 constructor
    AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State& _state,
        AP_RangeFinder_Params& _params,
        uint8_t serial_instance);
    //识别驱动		             
    static bool detect(/*AP_SerialManager& serial_manager, */uint8_t serial_instance, AP_RangeFinder_Params& _params);
    //更新状态---- update state
    void update(void) override;
protected:
    enum class Status {
        WAITTING = 0,
        GET_HEAD_ONCE,
        GET_HEAD,
        GET_TAIL_ONCE,
        GET_TAIL,
        GET_ONE_FRAME,
        WAITTING_FOR_TAIL,
    }_reading_state = Status::WAITTING;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }
private:
    //获取数据
    bool send_start_command();
    bool get_reading(uint16_t& reading_cm);
    AP_HAL::UARTDriver* uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint16_t last_distance_cm = 0;
    uint8_t linebuf[50];
    uint8_t buffer_count = 0;
};
#endif /* AP_RANGEFINDER_NRA24_H_ */
