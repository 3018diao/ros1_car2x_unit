```c
GossipMessage
│
└── GossipType (Union)
    ├── ChannelBusyRatio
    │    ├── busy: unsigned int
    │    └── total: unsigned int
    ├── LinkLayerReception
    │    ├── source: array of bytes (required)
    │    ├── destination: array of bytes (required)
    │    ├── channel: unsigned int
    │    ├── power_cbm: int
    │    └── payload: array of bytes (required)
    └── FacilityLayerReception
         └── msg: FacilityLayerMessage (Union)
               └── CPMessage
                    ├── header: ItsPduHeader (required)
                    │    ├── protocol_version: unsigned int
                    │    ├── message_id: unsigned int
                    │    └── station_id: unsigned int
                    ├── generation_delta_time: unsigned long
                    └── perceived_object_container: PerceivedObjectContainer
                         └── PerceivedObject
                              ├── object_id: unsigned int
                              ├── measurement_delta_time: int
                              ├── position: CartesianPosition3dWithConfidence (required)
                              │    ├── x_cord: CartesianCoordinateWithConfidence (required)
                              │    │    ├── value: int
                              │    │    └── confidence: unsigned int
                              │    ├── y_cord: CartesianCoordinateWithConfidence (required)
                              │    │    ├── value: int
                              │    │    └── confidence: unsigned int
                              │    └── z_cord: CartesianCoordinateWithConfidence
                              │         ├── value: int
                              │         └── confidence: unsigned int
                              ├── velocity: Velocity3dWithConfidence
                              │    ├── polar_velocity: VelocityPolarWithZ
                              │    └── cartesian_velocity: VelocityCartesian
                              │         ├── x_velocity: VelocityComponent (required)
                              │         │    ├── vel_comp_value: int
                              │         │    └── speed_confidence: unsigned int
                              │         ├── y_velocity: VelocityComponent (required)
                              │         │    ├── vel_comp_value: int
                              │         │    └── speed_confidence: unsigned int
                              │         └── z_velocity: VelocityComponent
                              │              ├── vel_comp_value: int
                              │              └── speed_confidence: unsigned int
                              ├── acceleration: Acceleration3dWithConfidence
                              ├── angles: EulerAnglesWithConfidence
                              ├── z_angular_velocity: CartesianAngularVelocityComponent
                              ├── lower_triangular_correlation_matrices: array of LowerTriangularPositiveSemidefiniteMatrix
                              ├── object_dimension_z: ObjectDimension
                              ├── object_dimension_y: ObjectDimension
                              ├── object_dimension_x: ObjectDimension
                              ├── object_age: unsigned int
                              ├── object_perception_quality: unsigned int
                              ├── sensor_id_list: array of unsigned int
                              ├── classification: array of ObjectClassWithConfidence
                              └── map_position: MapPosition (required)

```