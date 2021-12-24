/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_RangeFinder_NRA24I2C.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_NRA24I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NRA24I2C::AP_RangeFinder_NRA24I2C(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

/*
   detect if a Maxbotix rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_NRA24I2C::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_NRA24I2C *sensor
        = new AP_RangeFinder_NRA24I2C(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_NRA24I2C::_init(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    if (!start_reading()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_NRA24I2C::_timer, void));

    return true;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_NRA24I2C::start_reading()
{
    uint8_t buffer[] = { 0xAA,0xAA,0x00,0x02,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0x55 };

    // send command to take reading
    return _dev->transfer(buffer, sizeof(buffer), nullptr, 0);
}

inline bool if_data_frame(uint8_t* buf, uint16_t& reading_cm) {

    uint8_t* payload = (buf + 4);

    uint8_t* msg_id = (buf + 2);

    if ((msg_id[0] + (((uint16_t)msg_id[1]) << 8)) != 0x70c)
        return false;

    reading_cm = static_cast<uint16_t>(((payload[2] << 8) + payload[3]));
    //   hal.uartF->write((uint8_t)reading_cm);//串口测试

    return true;
}

bool AP_RangeFinder_NRA24I2C::get_reading(uint16_t& reading_cm)
{
    bool bGot = false;
    //if (uart == nullptr) {
    //    return false;
    //}

    //uint32_t nbytes = uart->available();
    uint8_t c;
    //while (nbytes-- > 0) {
    while (_dev->transfer(nullptr, 0, &c, sizeof(c) == true)) {
    //    bool ret = _dev->transfer(nullptr, 0, &c, sizeof(c));

        //  hal.uartF->write(c);
        if (buffer_count > 50) {
            _reading_state = Status::WAITTING;
        }
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
                _reading_state = Status::GET_ONE_FRAME;	//		原文这里有问题不应该break
            }
            else
            {
                _reading_state = Status::WAITTING_FOR_TAIL;//还没有到结尾
                break;
            }
        }
        case Status::GET_ONE_FRAME: {
            _reading_state = Status::WAITTING;
            if (buffer_count != 13)
            {
                break;//          return false;
            }
            if (if_data_frame(linebuf, reading_cm))
            {
                bGot = true;
                //if (nbytes < 14)
                //    return true;
            }
            break;
        }

        default:
            break;
        }

        if (bGot == true)
        {
            return true;
        }

    }

    return false;

}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_NRA24I2C::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_NRA24I2C::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_cm = distance;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
