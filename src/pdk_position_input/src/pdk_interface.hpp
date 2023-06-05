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

#include <functional>
#include <string>
#include <vector>

#include "MountingParameters.hpp"
#include "VehicleDynamics.hpp"

namespace PDK
{
/**
 * @ingroup misc
 * @brief The CInterface class is the customer interface for accessing data
 * related to the PDK. All methods of the interface are static and depend on a
 * correct initialization using the Init(pdk_config_file_path) function, passing
 * the PDK configuration file. After successful initialization, data can be
 * subscribed and published.
 *
 * Most of the subscribed data is passed as a serialized protobuf string. The
 * corresponding *.proto files can be found in the PDK include folder.
 */
class CInterface
{
public:

  /**
   * @ingroup misc
   * @brief Initializes the customer interface library
   *
   * @param config_path The PDK configuration file
   *
   * @return void
   */
  static void Init(const std::string &config_path);

  /**
   * @ingroup rdi
   * @brief Set the RDI callback
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::RadarDetectionImage
   *
   * @return void
   */
  static void SetRDICallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup rdi
   * @brief Stop RDI Subscriber and release callback function.
   * 
   * Stops your application subscribing the Radar Detection Image messages and
   * removes the registered callback function. After calling this function, it
   * is safe to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopRDISubscriber();

  /**
   * @ingroup rdi
   * @brief Set the Radar Status callback
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::RadarStatus
   *
   * @return void
   */
  static void SetRadarStatusCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup rdi
   * @brief Stop Radar Status Subscriber and release callback function.
   * 
   * Stops your application subscribing the Radar Status messages and removes
   * the registered callback function. After calling this function, it is safe
   * to destroy objects associated with the callback function.
   *
   * @return void
   */
  static void StopRadarStatusSubscriber();

  /**
   * @ingroup tracking
   * @brief Set the Dynamic Object Tracking callback function and bind it to the
   *        corresponding subscriber.
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::RadarObjectList
   *   form the DOT software module.
   *
   * @return void
   */
  static void SetTrackingCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup tracking
   * @brief Stop Dynamic Object Tracking subscriber and release callback function.
   * 
   * Stops your application subscribing DOT Object List messages and removes
   * the registered callback function. After calling this function, it is safe
   * to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopTrackingSubscriber();

  /**
   * @ingroup tracking
   * @brief Set the Sensor Objects Callback function and bind it to the
   *        corresponding subscriber.
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::RadarObjectList
   *   form the radar sensor.
   *
   * @return void
   */
  static void SetSensorObjectsCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup tracking
   * @brief Stop Sensor Objects Subscriber and release callback function.
   *
   * Stops your application subscribing Sensor Object List messages and removes
   * the registered callback function. After calling this function, it is safe
   * to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopSensorObjectsSubscriber();

  /**
   * @ingroup sem
   * @brief Set the Grid Callback object
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::GridImage
   *
   * @return void
   */
  static void SetGridCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup sem
   * @brief Stop Grid Image Subscriber and release callback function.
   * 
   * Stops your application subscribing Grid images and removes the registered
   * callback function. After calling this function, it is safe to destroy
   * objects associated with your callback function.
   *
   * @return void
   */
  static void StopGridSubscriber();

  /**
   * @ingroup sem
   * @brief Set the Freespace Callback object
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::FreespaceDataChannel
   *
   * @return void
   */
  static void SetFreespaceCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup sem
   * @brief Stop Freespace Subscriber and release callback function.
   * 
   * Stops your application subscribing Freespace messages and removes the
   * registered callback function. After calling this function, it is safe to
   * destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopFreespaceSubscriber();

  /**
   * @ingroup mnt
   * @brief Set the Mounting Parameters Callback object
   *
   * @param callback A function callback accepting an int and a reference of type CMountingParameters
   *   int is the sensor ID
   *   CMountingParameters are the mounting parameters for the sensor ID
   *
   * @return void
   */
  static void SetMountingParametersCallback(std::function<void(const int, const CMountingParameters &)> callback);

  /**
   * @ingroup mnt
   * @brief Stop Mounting Parameter Subscriber and release callback function.
   * 
   * Stops your application subscribing Sensor Mounting Parameter messages and
   * removes the registered callback function. After calling this function, it
   * is safe to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopMountingParametersSubscriber();

  /**
   * @ingroup vdy
   * @brief Set the Vehicle Dynamics Callback object
   *
   * @param callback A function callback accepting a reference of type CVehicleDynamics
   *   CVehicleDynamics are the current vehicle dynamics published to the system
   *
   * @return void
   */
  static void SetVehicleDynamicsCallback(std::function<void(const CVehicleDynamics &)> callback);

  /**
   * @ingroup vdy
   * @brief Stop Vehicle Dynamics Subscriber and release callback function.
   *
   * Stops your application subscribing Vehicle Dynamics messages and removes
   * the registered callback function. After calling this function, it is safe
   * to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopVehicleDynamicsSubscriber();

  /**
   * @ingroup camera
   * @brief Set the Camera Image Callback object
   *
   * @param callback A function callback accepting a std::string reference
   *   String contains serialized protobuf data of type pb::PDK::CameraImage
   *
   * @return void
   */
 static void SetCameraImageCallback(std::function<void(const std::string &)> callback);

  /**
   * @ingroup camera
   * @brief Stop the Camera Image Callback Subscriber and release callback function
   *
   * Stops your application subscribing Camera Image messages and removes
   * the registered callback function. After calling this function, it is safe
   * to destroy objects associated with your callback function.
   *
   * @return void
   */
  static void StopCameraImageCallback();

  /**
   * @ingroup mnt
   * @brief Publishes a set of mounting parameters to the ecal network once
   *
   * @param mountingParameters Parameters to publish
   *
   * @return True if success, false if failure
   */
  static bool PublishMountingParameters(const CMountingParameters &mountingParameters);

  /**
   * @ingroup mnt
   * @brief Publishes a list of mounting parameters to the ecal network once
   *
   * @param mountingParametersList Parameter list to publish
   *
   * @return True if success, false if failure
   */
  static bool PublishMountingParametersList(const std::vector<CMountingParameters> &mountingParametersList);

  /**
   * @ingroup vdy
   * @brief Publishes vehicle dynamics data to the ecal network once
   *
   * @param vehicleDynamics The data to publish
   *
   * @return True if success, false if failure
   */
  static bool PublishVehicleDynamics(const CVehicleDynamics &vehicleDynamics);
};

/**
 *@ingroup misc
 * @brief Get the version string
 *
 * @return std::string
 */
std::string GetVersion();
} // namespace PDK
