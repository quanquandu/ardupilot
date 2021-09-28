/*
 * 增加头文件------AP_RangeFinder_NRA24.h
 */
#ifndef __AP_RANGEFINDER_NRA24_H__

#define __AP_RANGEFINDER_NRA24_H__
#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
 //定义枚举类型
typedef enum NRA24_Status {
    NRA24_IDLE = 0,
    NRA24_GOT_START1 = 1,
    NRA24_GOT_START2,
    NRA24_GOT_MSGID1,
    NRA24_GOT_MSGID2,
    NRA24_GOT_DATA,
    NRA24_GOT_END1
}NRA24_parse_status;
//定义NRA24驱动
class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend
{
public:
    // constructor
    AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State& _state,
        AP_RangeFinder_Params& _params,
        uint8_t serial_instance);

    //识别驱动		             
    static bool detect(uint8_t serial_instance);
    //更新状态---- update state
    void update(void) override;
protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
private:
    //获取数据
    bool get_reading(uint16_t& reading_cm);
    AP_HAL::UARTDriver* uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[8];
    uint8_t linebuf_len;
    uint8_t snr;
    uint8_t rcs;
    uint16_t dist;
    NRA24_parse_status _parse_status;
    uint8_t range_data[1200];
    uint8_t range_data_peculiar[1200];
    uint16_t range_crc_data;
    uint16_t range_out_inf;
    uint16_t msg_id;
};
#endif /* AP_RANGEFINDER_NRA24_H_ */
