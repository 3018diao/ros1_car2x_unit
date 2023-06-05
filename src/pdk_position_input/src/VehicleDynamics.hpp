/*
 * License Notice
 *
 * This project includes software developed by Continental Engineering
 * Services GmbH. (https://www.conti-engineering.com)
 *
 * You may not use this software except in compliance with the License.
 * You may find the License attached to the offer document.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <chrono>
#include <memory>


/**
  * \ingroup vdy
  * @brief A struct defining the timestamp used by the VDY data.
  *
  */
struct CTimeStamp
{ // Posix time, i.e. time since since 00:00:00 Coordinated Universal Time (UTC), Thursday, 1 January 1970
  uint32_t Seconds{0};
  uint32_t Nanoseconds{0};

  CTimeStamp() = default;

  CTimeStamp(uint32_t Seconds_, uint32_t Nanoseconds_)
      : Seconds(Seconds_)
      , Nanoseconds(Nanoseconds_)
  {
  }
};


/**
  * \ingroup vdy
  * @brief A struct defining a single signal in the VDY data.
  *
  */
struct CVehicleDynamicsSignal
{
  enum class eSignalState
  {
    INVALID,
    VALID
  };

  eSignalState state{eSignalState::INVALID};
  float value{0.0f};

  CVehicleDynamicsSignal() = default;

  CVehicleDynamicsSignal(eSignalState state_, float value_)
      : state(state_)
      , value(value_)
  {
  }
};

/**
  * \ingroup vdy
  * @brief A struct holding all VDY values.
  *
  */
struct CVehicleDynamics
{
  CTimeStamp
      TimeStamp; // Posix time, i.e. time since since 00:00:00 Coordinated Universal Time(UTC), Thursday, 1 January 1970
  CVehicleDynamicsSignal LongVel;   /* [m/s]   */
  CVehicleDynamicsSignal YawRate;   /* [rad/s] */
  CVehicleDynamicsSignal LongAccel; /* [m/s^2] */
  CVehicleDynamicsSignal LatAccel;  /* [m/s^2] */

  // Default Constructor
  CVehicleDynamics() = default;

  // Constructor
  CVehicleDynamics(CTimeStamp TimeStamp_, CVehicleDynamicsSignal LongVel_, CVehicleDynamicsSignal YawRate_,
                   CVehicleDynamicsSignal LongAccel_, CVehicleDynamicsSignal LatAccel_)
      : TimeStamp(TimeStamp_)
      , LongVel(LongVel_)
      , YawRate(YawRate_)
      , LongAccel(LongAccel_)
      , LatAccel(LatAccel_)
  {
  }
};
