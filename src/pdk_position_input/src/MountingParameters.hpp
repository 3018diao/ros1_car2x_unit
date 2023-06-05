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
#include <cstdint>

#pragma once

/**
 * \ingroup mnt
 * @brief A struct holding the mounting parameters for a sensor.
 *
 */
struct CMountingParameters
{
    enum class eSensorOrientation
    {
        RIGHT = 0,
        LEFT = 1,
        UP = 2,
        DOWN = 3
    };

    enum class eSensorType
    {
        ARS430EO = 0,
        ARS430DI = 1,
        SRR520CO = 3
    };

    uint8_t SensorId = 0u;
    eSensorType SensorType = eSensorType::ARS430EO;
    float LongPos = 0.0f;
    float LatPos = 0.0f;
    float VertPos = 0.0f;
    float LongPosToCoG = 0.0f;
    float YawAngle = 0.0f;
    eSensorOrientation SensorOrientation = eSensorOrientation::RIGHT;

    // Default Constructor
    CMountingParameters() = default;

    // Constructor using Orientation Flag
    CMountingParameters(uint8_t SensorId_, eSensorType SensorType_, float LongPos_, float LatPos_, float VertPos_,
                        float LongPosToCoG_, float YawAngle_, eSensorOrientation SensorOrientation_)
        : SensorId(SensorId_)
        , SensorType(SensorType_)
        , LongPos(LongPos_)
        , LatPos(LatPos_)
        , VertPos(VertPos_)
        , LongPosToCoG(LongPosToCoG_)
        , YawAngle(YawAngle_)
        , SensorOrientation(SensorOrientation_)
    {
    }
    
}; // struct CMountingParameters
