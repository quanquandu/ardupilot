#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "AP_RangeFinder_NRA24.h"
extern const AP_HAL::HAL& hal;

AP_RangeFinder_NRA24::AP_RangeFinder_NRA24( RangeFinder::RangeFinder_State& _state,
    AP_RangeFinder_Params& _params,
    uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params) 
{
    const AP_SerialManager& serial_manager = AP::serialmanager();
    // 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    if (_params.orientation == 25)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance));
        }
    }
    else if (_params.orientation == 0)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24_FORWARD, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24_FORWARD, serial_instance));
        }
    }
    else if (_params.orientation == 4)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_BACKWARD, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SERIALPROTOCOL_NRA24_BACKWARD, serial_instance));
        }
    }
    else if (_params.orientation == 6)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_LEFT, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SERIALPROTOCOL_NRA24_LEFT, serial_instance));
        }
    }
    else if (_params.orientation == 2)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_RIGHT, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SERIALPROTOCOL_NRA24_RIGHT, serial_instance));
        }
    }
    else if (_params.orientation == 24)
    {
        uart = serial_manager.find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_UP, serial_instance);
        if (uart != nullptr) {
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SERIALPROTOCOL_NRA24_UP, serial_instance));
        }
    }
    /* //   const AP_SerialManager& serial_manager = AP::serialmanager();
 //   uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
 //   uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance));

    const AP_SerialManager& serial_manager = AP::serialmanager();
    uart = hal.uartD;// serial_manager.find_serial(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance));
        //uart->begin(115200);
    }
*/
}


bool AP_RangeFinder_NRA24::detect(
    uint8_t serial_instance,
    AP_RangeFinder_Params& _params)
{
   // return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance) != nullptr;
    if (_params.orientation == 25)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NRA24_DOWN, serial_instance) != nullptr;
    }
    else if (_params.orientation == 0)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NRA24_FORWARD, serial_instance) != nullptr;
    }
    else if (_params.orientation == 4)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_BACKWARD, serial_instance) != nullptr;
    }
    else if (_params.orientation == 6)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_LEFT, serial_instance) != nullptr;
    }
    else if (_params.orientation == 2)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_RIGHT, serial_instance) != nullptr;
    }
    else if (_params.orientation == 24)
    {
        return AP::serialmanager().find_serial(AP_SerialManager::SERIALPROTOCOL_NRA24_UP, serial_instance) != nullptr;        
    }
    return false;
}


inline bool if_data_frame(uint8_t* buf, uint16_t& reading_cm) {

    uint8_t* payload = (buf + 4);

    uint8_t* msg_id = (buf + 2);

    if ((msg_id[0] + (((uint16_t)msg_id[1]) << 8)) != 0x70c) return false;


    //reading_cm = static_cast<uint16_t>(((payload[2] << 8) + payload[3] * 0.01) * 100);
    reading_cm = static_cast<uint16_t>(((payload[2] << 8) + payload[3]));

    return true;

    uint16_t id = msg_id[0] + msg_id[1] * 0x100;

    if (id != 0x70c)
    {

        return false;
    }

    reading_cm = payload[2] * 0x100 + payload[3];

    return true;
}

bool AP_RangeFinder_NRA24::get_reading(uint16_t& reading_cm)
{
    if (uart == nullptr) {
        return false;
    }
    uint32_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();

        switch (_reading_state)
        {
        case Status::WAITTING: {
            if (c == 0xAA)
            {
                buffer_count = 0;
                linebuf[buffer_count] = c;

                _reading_state = Status::GET_HEAD_ONCE;
            }
            break;
        }
        case Status::GET_HEAD_ONCE: {
            if (c == 0xAA) {
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::WAITTING_FOR_TAIL;
            }
            else
            {
                buffer_count++;
                linebuf[buffer_count] = 0xAA;
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::WAITTING_FOR_TAIL;
            }

            break;
        }
        case Status::WAITTING_FOR_TAIL: {
            buffer_count++;
            linebuf[buffer_count] = c;
            if (c == 0x55)
                _reading_state = Status::GET_TAIL_ONCE;
            break;
        }

        case Status::GET_TAIL_ONCE: {
            if (c == 0x55) {
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::GET_ONE_FRAME;
            }
            //		break;原文这里有问题不应该break
        }
        case Status::GET_ONE_FRAME: {
            _reading_state = Status::WAITTING;
            if (if_data_frame(linebuf, reading_cm)) return true;
            break;
        }

        default:
            break;
        }
        if (buffer_count > sizeof(linebuf)) {
            buffer_count = 0;
            _reading_state = Status::WAITTING;
        }
    }

    return false;

}

void AP_RangeFinder_NRA24::update(void)
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
