#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NRA24.h"


AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State& _state,
    AP_RangeFinder_Params& _params,
    uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    const AP_SerialManager& serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }

    //uart = hal.uartD;
    //uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24, serial_instance));
    _parse_status = NRA24_IDLE;
    linebuf_len = 0;
    snr = 0;
}

bool AP_RangeFinder_NRA24::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}


//bool AP_RangeFinder_NRA24::detect(AP_SerialManager& serial_manager,
//    uint8_t serial_instance)
//{
//    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24, serial_instance) != nullptr;
//}


uint8_t get_range_nar24 = 0;

bool AP_RangeFinder_NRA24::get_reading(uint16_t& reading_cm)
{
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t k = 0;
    uint32_t len = 0;
    //没有串口直接返回
    if (uart == nullptr)
    {
        //这种状态是未连接
        set_status(RangeFinder::RangeFinder_NotConnected);
        get_range_nar24 = 0;
        return false;
    }
    //获取有效字节数是多少
    int16_t nbytes = uart->available();
    //首先判断看是不是14字节数据，是的话表示正常数据，否则是不正常数据，然后进行拆包处理
    if (nbytes == 14)
    {
        get_range_nar24 = 2;
        for (i = 0; i < nbytes; i++)
        {
            range_data[i] = uart->read(); //读取串口数据
        }

        if ((range_data[0] == 0xaa) && (range_data[1] == 0xaa)
            && (range_data[2] == 12) && (range_data[3] == 7)
            && (range_data[12] == 0x55) && (range_data[13] == 0x55))
        {
            //获取校验数据
            range_crc_data = range_data[4] + range_data[5] + range_data[6]
                + range_data[7] + range_data[8] + range_data[9]
                + range_data[10];
            //获取校验数据的低八位数据
            range_crc_data = range_crc_data & 0xff;
            //判断校验是否通过，通过之后进行获取数据
            if (range_data[11] == range_crc_data)
            {
                //获取有效数据
                reading_cm = range_data[6] * 0x100 + range_data[7];
                //出厂测试保留
                rcs = range_data[5] * 0.5 - 50;
                return true;

            }
            else
            {
             //   uart->clear_buf();
                return false;
            }

        }
        else
        {
         //   uart->clear_buf();
            return false;
        }

    }
    //对于特殊的数据进行解析
    if (nbytes != 14)
    {
        get_range_nar24 = 3;
        //存储数据
        for (j = 0; j < nbytes; j++)
        {
            //串口读取数据
            range_data[j] = uart->read();

        }
        //读取后直接清掉缓冲区数据
     //   uart->clear_buf();
        for (k = 0; k < nbytes; k++)
        {
            if (range_data[k] == 0xaa) //防止越界
            {
                len = k;
                if (len + 14 > nbytes)
                {
                    return false;
                }
            }
            //判断是否是有效数据
            if ((range_data[k] == 0xaa) && (range_data[k + 1] == 0xaa)
                && (range_data[k + 2] == 12) && (range_data[k + 3] == 7)
                && (range_data[k + 12] == 0x55)
                && (range_data[k + 13] == 0x55))
            {
                range_crc_data = range_data[k + 4] + range_data[k + 5]
                    + range_data[k + 6] + range_data[k + 7]
                    + range_data[k + 8] + range_data[k + 9]
                    + range_data[k + 10];
                //获取校验数据的低八位数据
                range_crc_data = range_crc_data & 0xff;
                if (range_crc_data == range_data[k + 11])
                {
                    //读取有效数据
                    reading_cm = range_data[k + 6] * 256 + range_data[k + 7];
                    //出厂测试保留
                    rcs = range_data[5] * 0.5 - 50;
                    return true;
                }
            }
        }
        return false;

    }
    // no readings so return false
    return false;
}


void AP_RangeFinder_NRA24::update()
{
    if (get_reading(state.distance_cm))  //获取数据state.distance_cm
    {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    }
    else if (AP_HAL::millis() - last_reading_ms > 3000) //如果3s还没数据，提示没数据
    {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

