# ArduPilot Repository Overview for UAV Swarm Development

> **Purpose**: Comprehensive guide for developing autonomous UAV swarm systems and extending AP_DDS functionality in ArduPilot
> 
> **Target Audience**: Intermediate developers working on ArduCopter swarm features
> 
> **Last Updated**: March 2026

---

## Table of Contents

1. [Repository Structure](#1-repository-structure)
2. [ArduCopter Architecture](#2-arducopter-architecture)
3. [Core Libraries for Swarm Systems](#3-core-libraries-for-swarm-systems)
4. [AP_DDS Deep Dive](#4-ap_dds-deep-dive)
5. [Build System & Workflow](#5-build-system--workflow)
6. [Adding New Features](#6-adding-new-features)
7. [Swarm-Specific Development Guide](#7-swarm-specific-development-guide)
8. [Testing & Debugging](#8-testing--debugging)
9. [Best Practices](#9-best-practices)
10. [Quick Reference](#10-quick-reference)

---

## 1. Repository Structure

### 1.1 Top-Level Organization

```
ardupilot/
├── ArduCopter/          # Multirotor vehicle code
├── ArduPlane/           # Fixed-wing vehicle code
├── Rover/               # Ground vehicle code
├── ArduSub/             # Underwater vehicle code
├── AntennaTracker/      # Antenna tracker code
├── Blimp/               # Blimp vehicle code
├── libraries/           # Shared libraries (150+ libraries)
├── Tools/               # Build tools, simulation, testing
├── modules/             # Git submodules (MAVLink, etc.)
├── build/               # Build output (generated)
├── tests/               # Unit and integration tests
└── docs/                # Documentation
```

### 1.2 Key Directories for Swarm Development

| Directory | Purpose | Importance for Swarms |
|-----------|---------|----------------------|
| `ArduCopter/` | Vehicle-specific logic, flight modes | **Critical** - Main integration point |
| `libraries/AP_DDS/` | ROS 2 DDS integration | **Critical** - Communication backbone |
| `libraries/GCS_MAVLink/` | MAVLink protocol handling | **High** - Inter-vehicle comms |
| `libraries/AP_Avoidance/` | Collision avoidance | **High** - Swarm safety |
| `libraries/AP_Follow/` | Follow mode implementation | **High** - Formation flying |
| `libraries/AP_Mission/` | Mission planning & execution | **High** - Coordinated missions |
| `libraries/AP_AHRS/` | Attitude & heading reference | **Medium** - State estimation |
| `libraries/AP_GPS/` | GPS/GNSS handling | **Medium** - Positioning |
| `libraries/AP_NavEKF3/` | Navigation EKF | **Medium** - State estimation |
| `Tools/autotest/` | Automated testing | **Medium** - Swarm testing |

---

## 2. ArduCopter Architecture

### 2.1 Main Components

**File: `ArduCopter/Copter.h`** - Main class definition

```cpp
class Copter : public AP_Vehicle {
    // Core subsystems
    AP_AHRS &ahrs;                    // Attitude heading reference
    AC_AttitudeControl *attitude_control;  // Attitude controller
    AP_InertialNav inertial_nav;      // Inertial navigation
    AC_WPNav *wp_nav;                 // Waypoint navigation
    AC_Loiter *loiter_nav;            // Loiter control
    AC_PosControl *pos_control;       // Position controller
    Mode *flightmode;                 // Current flight mode
    // ... many more subsystems
};
```

### 2.2 Main Loop Architecture

**Location**: `ArduCopter/Copter.cpp`

```
Initialization → Scheduler → Fast Loop (400Hz) → Slow Loops (varying rates)
                    ↓
            ┌───────┴────────┐
            │                │
        Read Sensors    Update Control
            │                │
        Run EKF      Send Telemetry
            │                │
        Update Nav    Log Data
```

**Key Files**:
- `Copter.cpp` - Main setup and loop
- `system.cpp` - System initialization
- `Attitude.cpp` - Attitude control
- `mode*.cpp` - Flight mode implementations

### 2.3 Flight Modes (Key for Swarms)

**Location**: `ArduCopter/mode*.cpp`

| Mode | File | Swarm Relevance |
|------|------|-----------------|
| **GUIDED** | `mode_guided.cpp` | **Critical** - External control via MAVLink/DDS |
| **AUTO** | `mode_auto.cpp` | **High** - Mission execution |
| **LOITER** | `mode_loiter.cpp` | **High** - Position hold |
| **RTL** | `mode_rtl.cpp` | **High** - Return to launch |
| **FOLLOW** | `mode_follow.cpp` | **High** - Follow another vehicle |
| **AVOID_ADSB** | `mode_avoid_adsb.cpp` | **Medium** - Collision avoidance |

**Adding Custom Swarm Mode**:
1. Create `mode_swarm.cpp`
2. Inherit from `Mode` class
3. Implement required methods: `init()`, `run()`, `exit()`
4. Register in `mode.h` and `mode.cpp`

### 2.4 Scheduler System

**Location**: `libraries/AP_Scheduler/AP_Scheduler.cpp`

The scheduler runs tasks at different rates:

```cpp
// Example task table in ArduCopter
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(read_AHRS,          400,  100),  // 400Hz
    SCHED_TASK(update_flight_mode, 400,   75),  // 400Hz
    SCHED_TASK(send_heartbeat,       1,  110),  // 1Hz
    SCHED_TASK(gcs_check_input,    100,  180),  // 100Hz
    // ... your custom tasks here
};
```

**For Swarm**: Add tasks for swarm coordination, neighbor updates, formation control

---

## 3. Core Libraries for Swarm Systems

### 3.1 Communication Libraries

#### A. AP_DDS (ROS 2 Integration)
**Location**: `libraries/AP_DDS/`
**Purpose**: DDS-XRCE bridge for ROS 2 communication

**Key Files**:
- `AP_DDS_Client.cpp` - Main DDS client
- `AP_DDS_Topic_Table.h` - Published/subscribed topics
- `AP_DDS_Service_Table.h` - ROS 2 services
- `AP_DDS_config.h` - Feature configuration

**Current Topics** (see section 4 for details):
- Publishers: Pose, velocity, GPS, battery, clock, etc.
- Subscribers: External control, joy, transforms
- Services: Arm/disarm, mode switch, takeoff

#### B. GCS_MAVLink
**Location**: `libraries/GCS_MAVLink/`
**Purpose**: MAVLink protocol for GCS and vehicle-to-vehicle comms

**Key Features**:
- Message routing
- Multi-channel support
- Vehicle-to-vehicle messaging
- Custom message handling

**Swarm Use**: Inter-vehicle MAVLink for coordination when ROS 2 not available

### 3.2 Navigation & Control Libraries

#### A. AP_AHRS (Attitude Heading Reference System)
**Location**: `libraries/AP_AHRS/`
**Purpose**: Provides vehicle attitude, position, velocity estimates

```cpp
// Getting vehicle state
const Vector3f& vel_ned = ahrs.get_velocity_NED();
Location current_loc;
ahrs.get_position(current_loc);
```

#### B. AC_WPNav (Waypoint Navigation)
**Location**: `libraries/AC_WPNav/`
**Purpose**: Waypoint navigation controller

**Swarm Use**: Coordinated waypoint following, formation waypoints

#### C. AC_Avoidance
**Location**: `libraries/AC_Avoidance/`
**Purpose**: Object avoidance (Simple Avoidance and BendyRuler)

**Integration Points**:
- Proximity sensors
- ADS-B avoidance
- Custom avoidance backends

### 3.3 Sensor Libraries

#### A. AP_GPS
**Location**: `libraries/AP_GPS/`
- Multi-GPS support
- RTK GPS
- GPS blending

#### B. AP_RangeFinder
**Location**: `libraries/AP_RangeFinder/`
- Distance sensing
- Terrain following
- MAVLink rangefinder (for external data)

#### C. AP_Proximity
**Location**: `libraries/AP_Proximity/`
- 360° proximity sensing
- Obstacle detection
- Multiple sensor backends

### 3.4 Mission & Planning Libraries

#### A. AP_Mission
**Location**: `libraries/AP_Mission/`
**Purpose**: Mission storage and execution

**Key Features**:
- Mission item storage in EEPROM
- Mission upload/download
- DO_JUMP, DO_REPEAT support

**Swarm Extension**: Could add swarm-specific mission commands

#### B. AP_Follow
**Location**: `libraries/AP_Follow/`
**Purpose**: Follow another vehicle

**Current Implementation**:
- Follows via MAVLink position
- Configurable offset
- Lead vehicle tracking

**Swarm Enhancement Ideas**:
- Multiple leader following
- Formation maintenance
- Dynamic role switching

---

## 4. AP_DDS Deep Dive

### 4.1 Architecture Overview

```
ArduPilot Firmware
      ↓
AP_DDS_Client (Micro XRCE-DDS)
      ↓
Serial/UDP Transport
      ↓
Micro XRCE-DDS Agent (Companion Computer)
      ↓
ROS 2 DDS (FastDDS/CycloneDDS)
      ↓
ROS 2 Nodes (Swarm Controller, etc.)
```

### 4.2 File Structure

```
libraries/AP_DDS/
├── AP_DDS_Client.cpp           # Main client implementation
├── AP_DDS_Client.h             # Client class definition
├── AP_DDS_config.h             # Feature flags & configuration
├── AP_DDS_Topic_Table.h        # Topic definitions & callbacks
├── AP_DDS_Service_Table.h      # Service definitions & callbacks
├── AP_DDS_ExternalControl.cpp  # External control handling
├── AP_DDS_External_Odom.cpp    # External odometry (VisualOdom)
├── AP_DDS_Rangefinder.cpp      # External rangefinder data
├── AP_DDS_ObstacleAvoidance.cpp # Obstacle data ingestion
├── AP_DDS_Type_Conversions.cpp # Type conversion utilities
├── AP_DDS_Serial.cpp           # Serial transport
├── AP_DDS_UDP.cpp              # UDP transport (SITL only)
├── Idl/                        # ROS 2 message definitions
├── tests/                      # Unit tests
└── README.md                   # DDS documentation
```

### 4.3 Current Topic Table

**Location**: `libraries/AP_DDS/AP_DDS_Topic_Table.h`

#### Publishers (ArduPilot → ROS 2)

| Topic | ROS 2 Type | Rate | Config Flag | Purpose |
|-------|-----------|------|-------------|---------|
| `/ap/state` | `ardupilot_msgs/msg/State` | 10Hz | `AP_DDS_STATE_PUB_ENABLED` | System state |
| `/ap/ekf_status` | `ardupilot_msgs/msg/EKFStatus` | 10Hz | `AP_DDS_EKF_STATUS_PUB_ENABLED` | EKF health |
| `/ap/time` | `builtin_interfaces/msg/Time` | 100Hz | `AP_DDS_TIME_PUB_ENABLED` | System time |
| `/ap/clock` | `rosgraph_msgs/msg/Clock` | 100Hz | `AP_DDS_CLOCK_PUB_ENABLED` | ROS clock |
| `/ap/geopose/filtered` | `geographic_msgs/msg/GeoPoseStamped` | 30Hz | `AP_DDS_GEOPOSE_PUB_ENABLED` | Global pose |
| `/ap/navsat/nav_sat_fix` | `sensor_msgs/msg/NavSatFix` | Varies | `AP_DDS_NAVSATFIX_PUB_ENABLED` | GPS data |
| `/ap/pose/filtered` | `geometry_msgs/msg/PoseStamped` | 30Hz | `AP_DDS_LOCAL_POSE_PUB_ENABLED` | Local pose |
| `/ap/twist/filtered` | `geometry_msgs/msg/TwistStamped` | 30Hz | `AP_DDS_LOCAL_VEL_PUB_ENABLED` | Velocity |
| `/ap/imu/experimental/data` | `sensor_msgs/msg/Imu` | 50Hz | `AP_DDS_IMU_PUB_ENABLED` | IMU data |
| `/ap/battery/battery_state` | `sensor_msgs/msg/BatteryState` | 1Hz | `AP_DDS_BATTERY_STATE_PUB_ENABLED` | Battery |
| `/ap/airspeed` | `geometry_msgs/msg/Vector3Stamped` | 1Hz | `AP_DDS_AIRSPEED_PUB_ENABLED` | Airspeed |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | On change | `AP_DDS_STATIC_TF_PUB_ENABLED` | Static TFs |
| `/ap/gps_global_origin/filtered` | `geographic_msgs/msg/GeoPointStamped` | 1Hz | `AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED` | GPS origin |

#### Subscribers (ROS 2 → ArduPilot)

| Topic | ROS 2 Type | Config Flag | Purpose |
|-------|-----------|-------------|---------|
| `/ap/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `AP_DDS_VEL_CTRL_ENABLED` | Velocity control |
| `/ap/cmd_gps_pose` | `ardupilot_msgs/msg/GlobalPosition` | `AP_DDS_GLOBAL_POS_CTRL_ENABLED` | Global position cmd |
| `/ap/joy` | `sensor_msgs/msg/Joy` | `AP_DDS_JOY_SUB_ENABLED` | Joystick input |
| `/ap/tf` | `tf2_msgs/msg/TFMessage` | `AP_DDS_DYNAMIC_TF_SUB_ENABLED` | Dynamic transforms |
| `/ap/vision_odom` | `nav_msgs/msg/Odometry` | `AP_DDS_VISUALODOM_ENABLED` | Visual odometry |
| `/ap/rangefinder` | `sensor_msgs/msg/Range` | `AP_DDS_RANGEFINDER_SUB_ENABLED` | External rangefinder |
| `/ap/obstacle` | `sensor_msgs/msg/PointCloud2` | `AP_DDS_OBSTACLE_DISTANCE_SUB_ENABLED` | Obstacle data |

#### Services

| Service | ROS 2 Type | Config Flag | Purpose |
|---------|-----------|-------------|---------|
| `/ap/arm_motors` | `ardupilot_msgs/srv/ArmMotors` | `AP_DDS_ARM_SERVER_ENABLED` | Arm/disarm |
| `/ap/mode_switch` | `ardupilot_msgs/srv/ModeSwitch` | `AP_DDS_MODE_SWITCH_SERVER_ENABLED` | Change mode |
| `/ap/takeoff` | `ardupilot_msgs/srv/Takeoff` | `AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED` | Takeoff |
| `/ap/parameter` | Various | `AP_DDS_PARAMETER_SERVER_ENABLED` | Parameter get/set |
| `/ap/prearm_check` | `ardupilot_msgs/srv/PrearmCheck` | `AP_DDS_ARM_CHECK_SERVER_ENABLED` | Pre-arm check |

### 4.4 Configuration System

**File**: `libraries/AP_DDS/AP_DDS_config.h`

```cpp
// Main enable/disable
#ifndef AP_DDS_ENABLED
#define AP_DDS_ENABLED 1  // Enabled by default
#endif

// Feature-specific enables (examples)
#ifndef AP_DDS_STATE_PUB_ENABLED
#define AP_DDS_STATE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_RANGEFINDER_SUB_ENABLED
#define AP_DDS_RANGEFINDER_SUB_ENABLED AP_RANGEFINDER_ENABLED && AP_DDS_ENABLED
#endif
```

**How it Works**:
- `#ifndef` allows build-time overrides in board `hwdef.dat`
- Features with dependencies automatically disable if dependency missing
- Flash-size-dependent features (e.g., VisualOdom requires >1MB flash)

### 4.5 Adding New DDS Topics

**Example: Adding Swarm State Publisher**

#### Step 1: Define Message (If Custom)
Create or use existing ROS 2 message in `Idl/` directory

#### Step 2: Add to Config (`AP_DDS_config.h`)
```cpp
#ifndef AP_DDS_SWARM_STATE_PUB_ENABLED
#define AP_DDS_SWARM_STATE_PUB_ENABLED 1
#endif

#ifndef AP_DDS_DELAY_SWARM_STATE_TOPIC_MS
#define AP_DDS_DELAY_SWARM_STATE_TOPIC_MS 100  // 10Hz
#endif
```

#### Step 3: Add Topic Entry (`AP_DDS_Topic_Table.h`)
```cpp
#if AP_DDS_SWARM_STATE_PUB_ENABLED
{
    .topic_id = to_topic_id(topics::SWARM_STATE_PUB),
    .pub_id = to_publisher_id(publishers::SWARM_STATE_PUB),
    .dw_id = to_datawriter_id(datawriters::SWARM_STATE_PUB),
    .topic_profile_label = "swarm_state__t",
    .topic_name = "rt/ap/swarm/state",
    .type_name = "your_pkg::msg::dds_::SwarmState_",
    .qos = {
        .durability = DDS_RELIABILITY_BEST_EFFORT,
        .reliability = DDS_RELIABILITY_BEST_EFFORT,
    },
},
#endif
```

#### Step 4: Add Update Function (`AP_DDS_Client.cpp`)
```cpp
#if AP_DDS_SWARM_STATE_PUB_ENABLED
void AP_DDS_Client::update_swarm_state_topic()
{
    if (last_swarm_state_update_ms == 0 || 
        AP_HAL::millis() - last_swarm_state_update_ms >= AP_DDS_DELAY_SWARM_STATE_TOPIC_MS) {
        
        // Populate message
        your_pkg_msg_SwarmState msg {};
        msg.neighbor_count = get_neighbor_count();
        msg.formation_state = get_formation_state();
        // ... fill other fields
        
        // Publish
        const bool success = write_swarm_state_msg(msg);
        if (success) {
            last_swarm_state_update_ms = AP_HAL::millis();
        }
    }
}
#endif
```

#### Step 5: Call from Update Loop
In `AP_DDS_Client::update()`:
```cpp
#if AP_DDS_SWARM_STATE_PUB_ENABLED
    update_swarm_state_topic();
#endif
```

### 4.6 Adding New DDS Subscribers

**Example: Adding Swarm Command Subscriber**

#### Step 1: Add to Config
```cpp
#ifndef AP_DDS_SWARM_CMD_SUB_ENABLED
#define AP_DDS_SWARM_CMD_SUB_ENABLED 1
#endif
```

#### Step 2: Add Subscription Entry (`AP_DDS_Topic_Table.h`)
```cpp
#if AP_DDS_SWARM_CMD_SUB_ENABLED
{
    .topic_id = to_topic_id(topics::SWARM_CMD_SUB),
    .sub_id = to_subscriber_id(subscribers::SWARM_CMD_SUB),
    .dr_id = to_datareader_id(datareaders::SWARM_CMD_SUB),
    .topic_profile_label = "swarm_cmd__s",
    .topic_name = "rt/ap/swarm/command",
    .type_name = "your_pkg::msg::dds_::SwarmCommand_",
    .on_data_available_override = on_swarm_command,
    .qos = {
        .durability = DDS_RELIABILITY_RELIABLE,
        .reliability = DDS_RELIABILITY_RELIABLE,
    },
},
#endif
```

#### Step 3: Implement Callback
Create `AP_DDS_SwarmCommand.cpp`:
```cpp
#include "AP_DDS_Client.h"
#if AP_DDS_SWARM_CMD_SUB_ENABLED

void AP_DDS_Client::on_swarm_command(uxrSession* session,
                                      uxrObjectId object_id,
                                      uint16_t request_id,
                                      uxrStreamId stream_id,
                                      struct ucdrBuffer* ub,
                                      uint16_t length,
                                      void* args)
{
    your_pkg_msg_SwarmCommand msg;
    
    // Deserialize message
    bool success = your_pkg_msg_SwarmCommand_deserialize_topic(ub, &msg);
    
    if (success) {
        // Process command
        handle_swarm_command(msg);
    }
}

void AP_DDS_Client::handle_swarm_command(const your_pkg_msg_SwarmCommand& msg)
{
    // Implementation
    switch(msg.command_type) {
        case FORMATION_CHANGE:
            // Handle formation change
            break;
        case MISSION_UPDATE:
            // Handle mission update
            break;
    }
}

#endif // AP_DDS_SWARM_CMD_SUB_ENABLED
```

### 4.7 DDS Parameters

**Location**: `ArduCopter/Parameters.cpp` (or vehicle-specific)

```cpp
// DDS_* parameters
GSCALAR(dds_enable,       "DDS_ENABLE",      1),
GSCALAR(dds_port,         "DDS_PORT",       14550),
GSCALAR(dds_udp_port,     "DDS_UDP_PORT",   2019),
```

**Serial Protocol Configuration**:
Set `SERIALn_PROTOCOL = 45` (DDS) on desired serial port

---

## 5. Build System & Workflow

### 5.1 WAF Build System

ArduPilot uses WAF (Python-based build system)

**Key Commands**:
```bash
# Configure for SITL
./waf configure --board sitl

# Configure for specific hardware
./waf configure --board Pixhawk6X

# Build specific vehicle
./waf build --target bin/arducopter

# Build all
./waf build

# Clean
./waf clean

# List all boards
./waf list_boards
```

### 5.2 ROS 2 Integration Build

**Using colcon** (your current setup):
```bash
cd ~/ardu_ws
colcon build --packages-select ardupilot_sitl

# With compile commands for IDE
colcon build --packages-select ardupilot_sitl \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 5.3 Directory Structure After Build

```
build/
├── sitl/                    # SITL build artifacts
│   ├── bin/
│   │   ├── arducopter      # Executable
│   │   ├── arduplane
│   │   └── ...
│   ├── libraries/          # Compiled libraries
│   └── compile_commands.json
├── Pixhawk6X/              # Hardware build
│   └── bin/
│       └── arducopter.apj  # Firmware file for upload
```

### 5.4 Hardware-in-Loop (HIL) Setup

For testing before real flights:

```bash
# 1. Build SITL with HIL support
./waf configure --board sitl
./waf build --target bin/arducopter

# 2. Run with specific parameters
sim_vehicle.py -v ArduCopter --console --map

# 3. In another terminal, run your ROS 2 swarm nodes
ros2 launch your_swarm_pkg swarm_sim.launch.py
```

---

## 6. Adding New Features

### 6.1 Feature Development Workflow

```
1. Plan & Design
   ├── Define requirements
   ├── Identify integration points
   └── Design interfaces

2. Implementation
   ├── Create/modify files
   ├── Add configuration flags
   ├── Implement feature logic
   └── Add logging

3. Integration
   ├── Add to scheduler (if needed)
   ├── Add parameters
   ├── Update GCS/DDS interfaces
   └── Add to appropriate flight modes

4. Testing
   ├── Unit tests
   ├── SITL testing
   ├── HIL testing
   └── Flight testing

5. Documentation
   ├── Code comments
   ├── Parameter documentation
   └── Wiki updates
```

### 6.2 Adding a New Library

**Example: Adding AP_Swarm library**

#### Step 1: Create Directory Structure
```bash
mkdir libraries/AP_Swarm
cd libraries/AP_Swarm
```

#### Step 2: Create Files
**`AP_Swarm.h`**:
```cpp
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include "AP_Swarm_config.h"

#if AP_SWARM_ENABLED

class AP_Swarm {
public:
    AP_Swarm();
    
    // Initialize swarm system
    void init();
    
    // Update (called from scheduler)
    void update();
    
    // Add/remove neighbors
    void add_neighbor(uint8_t sysid, const Location& loc);
    void remove_neighbor(uint8_t sysid);
    
    // Formation control
    void set_formation_type(uint8_t type);
    Vector3f get_formation_offset(uint8_t position);
    
    // Get swarm state
    uint8_t get_neighbor_count() const { return _neighbor_count; }
    
    // Singleton access
    static AP_Swarm* get_singleton() { return _singleton; }
    
    // Parameters
    static const struct AP_Param::GroupInfo var_info[];
    
private:
    static AP_Swarm* _singleton;
    
    struct Neighbor {
        uint8_t sysid;
        Location location;
        Vector3f velocity;
        uint32_t last_update_ms;
        bool active;
    };
    
    Neighbor _neighbors[MAX_SWARM_NEIGHBORS];
    uint8_t _neighbor_count;
    
    // Parameters
    AP_Int8 _enable;
    AP_Int8 _max_neighbors;
    AP_Float _separation_distance;
    AP_Int8 _formation_type;
};

namespace AP {
    AP_Swarm* swarm();
};

#endif // AP_SWARM_ENABLED
```

**`AP_Swarm_config.h`**:
```cpp
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SWARM_ENABLED
#define AP_SWARM_ENABLED 1
#endif

#ifndef MAX_SWARM_NEIGHBORS
#define MAX_SWARM_NEIGHBORS 10
#endif

#ifndef AP_SWARM_UPDATE_RATE_MS
#define AP_SWARM_UPDATE_RATE_MS 100  // 10Hz
#endif
```

**`AP_Swarm.cpp`**:
```cpp
#include "AP_Swarm.h"

#if AP_SWARM_ENABLED

extern const AP_HAL::HAL& hal;

AP_Swarm* AP_Swarm::_singleton;

const AP_Param::GroupInfo AP_Swarm::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Swarm Enable
    // @Description: Enable swarm functionality
    // @Values: 0:Disabled,1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Swarm, _enable, 0, AP_PARAM_FLAG_ENABLE),
    
    // @Param: MAX_NBR
    // @DisplayName: Maximum Neighbors
    // @Description: Maximum number of swarm neighbors to track
    // @Range: 1 20
    AP_GROUPINFO("MAX_NBR", 2, AP_Swarm, _max_neighbors, MAX_SWARM_NEIGHBORS),
    
    // @Param: SEP_DIST
    // @DisplayName: Separation Distance
    // @Description: Minimum separation distance from neighbors in meters
    // @Range: 1 50
    // @Units: m
    AP_GROUPINFO("SEP_DIST", 3, AP_Swarm, _separation_distance, 5.0f),
    
    AP_GROUPEND
};

AP_Swarm::AP_Swarm()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Swarm::init()
{
    if (!_enable) {
        return;
    }
    
    // Initialize neighbor array
    for (uint8_t i = 0; i < MAX_SWARM_NEIGHBORS; i++) {
        _neighbors[i].active = false;
    }
    _neighbor_count = 0;
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Swarm: Initialized");
}

void AP_Swarm::update()
{
    if (!_enable) {
        return;
    }
    
    // Update logic here
    // - Check neighbor timeouts
    // - Update formation positions
    // - Collision avoidance checks
}

void AP_Swarm::add_neighbor(uint8_t sysid, const Location& loc)
{
    // Find existing or empty slot
    for (uint8_t i = 0; i < MAX_SWARM_NEIGHBORS; i++) {
        if (_neighbors[i].active && _neighbors[i].sysid == sysid) {
            // Update existing
            _neighbors[i].location = loc;
            _neighbors[i].last_update_ms = AP_HAL::millis();
            return;
        }
    }
    
    // Add new neighbor
    if (_neighbor_count < _max_neighbors) {
        for (uint8_t i = 0; i < MAX_SWARM_NEIGHBORS; i++) {
            if (!_neighbors[i].active) {
                _neighbors[i].sysid = sysid;
                _neighbors[i].location = loc;
                _neighbors[i].last_update_ms = AP_HAL::millis();
                _neighbors[i].active = true;
                _neighbor_count++;
                break;
            }
        }
    }
}

// Singleton access
namespace AP {
    AP_Swarm* swarm()
    {
        return AP_Swarm::get_singleton();
    }
}

#endif // AP_SWARM_ENABLED
```

**`wscript`**:
```python
#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    bld.ap_stlib(
        name='AP_Swarm',
        ap_vehicle='AP_Swarm',
        ap_libraries=[
            'AP_Common',
            'AP_HAL',
            'AP_Math',
            'AP_Param',
        ],
    )
```

#### Step 3: Integrate with Vehicle

**In `ArduCopter/Copter.h`**:
```cpp
#include <AP_Swarm/AP_Swarm.h>

class Copter : public AP_Vehicle {
    // ... other members
    
#if AP_SWARM_ENABLED
    AP_Swarm swarm;
#endif
};
```

**In `ArduCopter/system.cpp`**:
```cpp
void Copter::init_ardupilot()
{
    // ... existing initialization
    
#if AP_SWARM_ENABLED
    swarm.init();
#endif
}
```

**In `ArduCopter/Copter.cpp` scheduler**:
```cpp
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // ... existing tasks
    
#if AP_SWARM_ENABLED
    SCHED_TASK(update_swarm,        10,  100),
#endif
};

#if AP_SWARM_ENABLED
void Copter::update_swarm()
{
    swarm.update();
}
#endif
```

**In `ArduCopter/Parameters.cpp`**:
```cpp
const AP_Param::GroupInfo ParametersG2::var_info[] = {
    // ... existing parameters
    
#if AP_SWARM_ENABLED
    AP_SUBGROUPINFO(swarm, "SWARM_", 25, ParametersG2, AP_Swarm),
#endif
    
    AP_GROUPEND
};
```

### 6.3 Adding Parameters

**Parameter Documentation Format**:
```cpp
// @Param: PARAM_NAME
// @DisplayName: Human Readable Name
// @Description: Detailed description of what this parameter does
// @Range: min max
// @Units: unit (m, s, deg, etc.)
// @Increment: 0.1
// @User: Standard (Standard/Advanced)
// @Values: 0:Option1,1:Option2
AP_GROUPINFO("PARAM_NAME", index, Class, _variable, default_value),
```

**Parameter Groups**:
- Use `AP_SUBGROUPINFO` for nested parameter groups
- Index must be unique within group
- Use `ParametersG2` for new parameters to avoid EEPROM format changes

### 6.4 Logging

**Add to `ArduCopter/Log.cpp`**:

```cpp
// Define log structure
struct PACKED log_Swarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t neighbor_count;
    float separation_distance;
    uint8_t formation_type;
    float formation_error;
};

// Add to log structure list
const struct LogStructure Copter::Log_Write_structures[] = {
    // ... existing structures
    { LOG_SWARM_MSG, sizeof(log_Swarm),
      "SWRM", "QBfBf", "TimeUS,NbrCnt,SepDist,FType,FErr",
      "s#m##", "F-0--" },
};

// Write function
void Copter::Log_Write_Swarm()
{
#if AP_SWARM_ENABLED
    struct log_Swarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SWARM_MSG),
        time_us             : AP_HAL::micros64(),
        neighbor_count      : swarm.get_neighbor_count(),
        separation_distance : swarm.get_separation_distance(),
        formation_type      : swarm.get_formation_type(),
        formation_error     : swarm.get_formation_error(),
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
#endif
}
```

---

## 7. Swarm-Specific Development Guide

### 7.1 Swarm Architecture Patterns

#### Pattern 1: Centralized Control
```
Ground Station / ROS 2 Node
         ↓
    [Commands]
         ↓
Individual Vehicles (via DDS/MAVLink)
```

**Pros**: Simple, deterministic
**Cons**: Single point of failure, latency

**Implementation**: Use DDS subscribers for commands, publishers for state

#### Pattern 2: Decentralized (Consensus-based)
```
Vehicle 1 ←→ Vehicle 2
    ↕           ↕
Vehicle 4 ←→ Vehicle 3
```

**Pros**: Robust, scalable
**Cons**: Complex, requires consensus algorithms

**Implementation**: 
- MAVLink vehicle-to-vehicle messaging
- DDS for local coordination with companion computers
- Custom swarm library for consensus logic

#### Pattern 3: Hybrid (Leader-Follower)
```
    Leader Vehicle
    ↙   ↓   ↘
  V1   V2   V3  (Followers)
```

**Pros**: Balance of simplicity and robustness
**Cons**: Leader is critical

**Implementation**: Use `AP_Follow` library, extend with formation control

### 7.2 Key Swarm Capabilities to Implement

#### A. Neighbor Awareness
```cpp
// Via MAVLink
void handle_mavlink_position_update(mavlink_message_t& msg) {
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);
    
    // Update neighbor in swarm library
    Location neighbor_loc;
    neighbor_loc.lat = packet.lat;
    neighbor_loc.lng = packet.lon;
    neighbor_loc.alt = packet.alt;
    
    AP::swarm()->add_neighbor(msg.sysid, neighbor_loc);
}

// Via DDS
// Subscribe to /swarm/neighbors topic from other vehicles
```

#### B. Formation Control
```cpp
// Get formation offset for this vehicle
Vector3f formation_offset = AP::swarm()->get_formation_offset(my_position_in_formation);

// Apply offset to waypoint navigation
Location target_loc = mission_target;
target_loc.offset(formation_offset.x, formation_offset.y);
target_loc.alt += formation_offset.z * 100;  // cm

// Navigate to formation position
wp_nav->set_wp_destination(target_loc);
```

#### C. Collision Avoidance
```cpp
// In swarm update loop
void AP_Swarm::check_collisions() {
    Location my_loc;
    if (!AP::ahrs().get_position(my_loc)) {
        return;
    }
    
    for (uint8_t i = 0; i < MAX_SWARM_NEIGHBORS; i++) {
        if (!_neighbors[i].active) continue;
        
        float distance = my_loc.get_distance(_neighbors[i].location);
        
        if (distance < _separation_distance) {
            // Collision imminent!
            handle_collision_avoidance(i, distance);
        }
    }
}

void AP_Swarm::handle_collision_avoidance(uint8_t neighbor_idx, float distance) {
    // Calculate avoidance vector
    Location my_loc;
    AP::ahrs().get_position(my_loc);
    
    Vector2f avoid_vector = my_loc.get_distance_NE(_neighbors[neighbor_idx].location);
    avoid_vector.normalize();
    avoid_vector *= -1;  // Away from neighbor
    
    // Send avoidance command to AC_Avoidance
    // This integrates with existing avoidance system
}
```

#### D. Mission Synchronization
```cpp
// Synchronized mission start
void AP_Swarm::sync_mission_start() {
    if (!is_leader()) {
        // Wait for leader's start signal
        return;
    }
    
    // Leader broadcasts start time
    uint32_t start_time_ms = AP_HAL::millis() + MISSION_START_DELAY_MS;
    
    // Send via MAVLink to all neighbors
    for (uint8_t i = 0; i < _neighbor_count; i++) {
        mavlink_msg_mission_start_send(
            MAVLINK_COMM_0,
            _neighbors[i].sysid,
            MAV_COMP_ID_ALL,
            start_time_ms
        );
    }
    
    // Schedule own mission start
    _mission_start_time_ms = start_time_ms;
}
```

### 7.3 Communication Patterns

#### MAVLink Vehicle-to-Vehicle

**Enable in parameters**:
```
SR0_POSITION = 10    # Position at 10Hz
SR0_EXTRA1 = 10      # Attitude at 10Hz
```

**Custom MAVLink messages**:
Create in `modules/mavlink/ardupilotmega.xml`:
```xml
<message id="12345" name="SWARM_STATE">
  <description>Swarm state message</description>
  <field type="uint8_t" name="swarm_id">Swarm ID</field>
  <field type="uint8_t" name="vehicle_id">Vehicle ID in swarm</field>
  <field type="uint8_t" name="formation_type">Formation type</field>
  <field type="float" name="formation_error">Formation error (m)</field>
</message>
```

#### DDS Topics for Swarm

**Publish swarm state**:
```
/swarm/vehicle_N/state          # Individual state
/swarm/formation/status         # Formation status
/swarm/collision/warnings       # Collision warnings
```

**Subscribe to commands**:
```
/swarm/command/formation        # Formation commands
/swarm/command/mission          # Mission commands
/swarm/neighbors/positions      # Neighbor positions
```

### 7.4 Safety Considerations

#### Geofence for Swarms
```cpp
// Expand geofence based on swarm size
void AP_Swarm::update_geofence() {
    AC_Fence* fence = AP::fence();
    if (fence == nullptr) return;
    
    // Add safety margin based on neighbor count
    float safety_margin = _neighbor_count * _separation_distance * 0.5f;
    fence->set_boundary_margin(safety_margin);
}
```

#### Failsafe Behavior
```cpp
// In ArduCopter/failsafe.cpp
void Copter::failsafe_swarm() {
#if AP_SWARM_ENABLED
    if (swarm.is_enabled() && swarm.collision_imminent()) {
        // Emergency land or RTL
        set_mode(Mode::Number::LAND, ModeReason::SWARM_COLLISION);
        
        // Notify other swarm members
        swarm.broadcast_emergency();
    }
#endif
}
```

#### Emergency Separation
```cpp
void AP_Swarm::emergency_separate() {
    // Each vehicle moves in unique direction based on ID
    float angle = (my_id % 360) * DEG_TO_RAD;
    Vector2f escape_vector(
        cosf(angle) * EMERGENCY_SEPARATION_DISTANCE,
        sinf(angle) * EMERGENCY_SEPARATION_DISTANCE
    );
    
    // Command immediate position offset
    // This overrides normal navigation
}
```

---

## 8. Testing & Debugging

### 8.1 SITL Multi-Vehicle Testing

**Start multiple vehicles**:
```bash
# Terminal 1 - Vehicle 1
sim_vehicle.py -v ArduCopter -I0 --out=udp:127.0.0.1:14550

# Terminal 2 - Vehicle 2  
sim_vehicle.py -v ArduCopter -I1 --out=udp:127.0.0.1:14551

# Terminal 3 - Vehicle 3
sim_vehicle.py -v ArduCopter -I2 --out=udp:127.0.0.1:14552
```

**With DDS**:
```bash
# Start Micro XRCE-DDS Agent for each
MicroXRCEAgent udp4 -p 2019  # For vehicle 1
MicroXRCEAgent udp4 -p 2020  # For vehicle 2
MicroXRCEAgent udp4 -p 2021  # For vehicle 3
```

**SITL parameters for DDS**:
```
DDS_ENABLE = 1
DDS_UDP_PORT = 2019  # Different for each vehicle
```

### 8.2 Debugging Tools

#### GDB Debugging
```bash
# Build with debug symbols
./waf configure --board sitl --debug
./waf build

# Run under GDB
gdb --args build/sitl/bin/arducopter --home LAT,LON,ALT,HDG
```

#### MAVProxy
```bash
# Connect to vehicle
mavproxy.py --master=udp:127.0.0.1:14550

# Useful commands
MAV> param show SWARM_*     # Show swarm parameters
MAV> mode GUIDED            # Change mode
MAV> arm throttle           # Arm
MAV> takeoff 10             # Takeoff to 10m
```

#### ROS 2 DDS Introspection
```bash
# List topics
ros2 topic list

# Echo swarm topics
ros2 topic echo /ap/pose/filtered
ros2 topic echo /swarm/neighbors/positions

# Monitor rates
ros2 topic hz /ap/pose/filtered

# Record data
ros2 bag record -a  # Record all topics
```

### 8.3 Logging & Analysis

#### Enable logging
```cpp
// In ArduCopter
AP::logger().Write_Message("SWARM: Formation error: %.2f", error);
```

#### Download and analyze logs
```bash
# Download from vehicle
mavproxy.py --master=/dev/ttyUSB0
MAV> log list
MAV> log download 1 latest.bin

# Convert to readable format
mavlogdump.py --format csv latest.bin > latest.csv

# Plot with MAVExplorer
MAVExplorer.py latest.bin
```

#### Custom log analysis
```python
from pymavlink import mavutil

# Load log
mlog = mavutil.mavlink_connection('latest.bin')

# Extract swarm data
while True:
    msg = mlog.recv_match(type='SWRM')
    if msg is None:
        break
    print(f"Time: {msg.TimeUS}, Neighbors: {msg.NbrCnt}, Error: {msg.FErr}")
```

### 8.4 Unit Testing

**Location**: `libraries/AP_Swarm/tests/`

**Example test** (`test_swarm.cpp`):
```cpp
#include <AP_gtest.h>
#include <AP_Swarm/AP_Swarm.h>

TEST(AP_Swarm, AddNeighbor)
{
    AP_Swarm swarm;
    swarm.init();
    
    Location loc;
    loc.lat = 353947000;  // Canberra
    loc.lng = 1491238000;
    loc.alt = 58400;  // 584m
    
    swarm.add_neighbor(2, loc);
    
    EXPECT_EQ(swarm.get_neighbor_count(), 1);
}

TEST(AP_Swarm, FormationOffset)
{
    AP_Swarm swarm;
    swarm.init();
    swarm.set_formation_type(FORMATION_LINE);
    
    Vector3f offset = swarm.get_formation_offset(1);
    
    EXPECT_FLOAT_EQ(offset.x, 5.0f);  // Expected spacing
}

AP_GTEST_MAIN()
```

**Run tests**:
```bash
./waf configure --board linux
./waf tests
```

---

## 9. Best Practices

### 9.1 Code Style

**Follow ArduPilot style guide**:
- Class names: `CamelCase` (e.g., `AP_Swarm`)
- Member variables: `_leading_underscore` (e.g., `_neighbor_count`)
- Functions: `snake_case` (e.g., `get_neighbor_count()`)
- Constants: `UPPER_CASE` (e.g., `MAX_SWARM_NEIGHBORS`)
- Use `nullptr` not `NULL`
- Use `float` not `double` (unless precision critical)

**File organization**:
```cpp
// Header (.h)
#pragma once
#include statements
Forward declarations
Class declaration
Inline function implementations

// Source (.cpp)
#include "ClassName.h"
Other includes
Static member initialization
Function implementations
```

### 9.2 Memory Management

**Stack vs Heap**:
- Prefer stack allocation
- Avoid `new`/`delete` in flight-critical code
- Use fixed-size arrays, not dynamic allocation

**Flash size considerations**:
```cpp
#if BOARD_FLASH_SIZE > 1024
    // Feature only on large boards
#endif
```

### 9.3 Real-Time Considerations

**Avoid in fast loops**:
- No `printf` or string formatting
- No file I/O
- No blocking operations
- Minimize floating point (use lookup tables if possible)

**Use scheduler properly**:
- Fast tasks (<100μs) can run at 400Hz
- Medium tasks (100-500μs) at 100Hz or less
- Slow tasks (>500μs) at 10Hz or less

### 9.4 Parameter Design

**Make features configurable**:
```cpp
// Good
AP_GROUPINFO("ENABLE", 1, AP_Swarm, _enable, 0),

// Bad - hardcoded
#define SWARM_ALWAYS_ON 1
```

**Provide sensible defaults**:
- Defaults should work for most users
- Advanced features default to OFF
- Safety-critical features default to SAFE

### 9.5 Error Handling

**Check pointers**:
```cpp
AP_Swarm* swarm = AP::swarm();
if (swarm == nullptr) {
    return;
}
```

**Validate inputs**:
```cpp
void AP_Swarm::set_separation_distance(float dist) {
    if (dist < 1.0f || dist > 50.0f) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SWARM: Invalid separation distance");
        return;
    }
    _separation_distance = dist;
}
```

**Fail gracefully**:
```cpp
if (!critical_subsystem_available()) {
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SWARM: Limited functionality");
    _reduced_mode = true;
    // Continue with reduced capabilities
}
```

---

## 10. Quick Reference

### 10.1 Common Code Patterns

#### Accessing Core Systems
```cpp
// AHRS (attitude, heading, reference)
AP::ahrs().get_position(current_loc);
Vector3f vel_ned = AP::ahrs().get_velocity_NED();

// GPS
const AP_GPS &gps = AP::gps();
if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
    // Good GPS
}

// Battery
AP_BattMonitor &battery = AP::battery();
float voltage = battery.voltage();

// Compass
Compass &compass = AP::compass();
float heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned());

// Rangefinder
RangeFinder *rangefinder = AP::rangefinder();
if (rangefinder != nullptr) {
    float distance = rangefinder->distance_cm() / 100.0f;
}

// Logging
AP::logger().Write_Message("Text message");

// GCS
GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Message: %d", value);
```

#### Time Functions
```cpp
uint32_t now_ms = AP_HAL::millis();     // Milliseconds since boot
uint64_t now_us = AP_HAL::micros64();   // Microseconds since boot

// Timeouts
if (AP_HAL::millis() - last_update_ms > TIMEOUT_MS) {
    // Timeout occurred
}
```

#### Location/Position Math
```cpp
Location loc1, loc2;

// Distance between locations
float dist_m = loc1.get_distance(loc2);

// Bearing from loc1 to loc2
float bearing_deg = loc1.get_bearing_to(loc2) * RAD_TO_DEG;

// Offset location
loc1.offset(10.0f, 5.0f);  // 10m north, 5m east

// Altitude difference
float alt_diff_m = (loc2.alt - loc1.alt) / 100.0f;  // cm to m
```

### 10.2 Important Build Flags

```bash
# SITL with debugging
./waf configure --board sitl --debug

# Enable ASAN (address sanitizer)
./waf configure --board sitl --asan

# Specific optimization
./waf configure --board sitl --Werror

# Show compile commands
./waf configure --board sitl --compile-commands
```

### 10.3 Key Documentation Links

- **ArduPilot Dev Wiki**: https://ardupilot.org/dev/
- **MAVLink Protocol**: https://mavlink.io/
- **DDS Documentation**: https://ardupilot.org/dev/docs/ros2.html
- **Contributing Guide**: https://ardupilot.org/dev/docs/contributing.html
- **Code Style**: https://ardupilot.org/dev/docs/style-guide.html

### 10.4 Helpful Commands Cheat Sheet

```bash
# Build
./waf configure --board sitl && ./waf build
colcon build --packages-select ardupilot_sitl

# Clean build
./waf clean && ./waf configure --board sitl && ./waf build

# Run SITL
sim_vehicle.py -v ArduCopter --console --map

# Multiple vehicles
sim_vehicle.py -v ArduCopter -I0
sim_vehicle.py -v ArduCopter -I1
sim_vehicle.py -v ArduCopter -I2

# DDS Agent
MicroXRCEAgent udp4 -p 2019

# ROS 2 topics
ros2 topic list
ros2 topic echo /ap/pose/filtered
ros2 topic pub /ap/cmd_vel geometry_msgs/msg/TwistStamped "{...}"

# Parameter management
param show              # Show all parameters
param set SWARM_ENABLE 1   # Set parameter
param download <file>   # Save params to file
param load <file>       # Load params from file

# MAVLink
arm throttle
mode GUIDED
takeoff 10
position 10 20 -10     # x, y, z (NED)

# Logs
log list
log download latest
```

### 10.5 Swarm Development Checklist

**Phase 1: Planning**
- [ ] Define swarm architecture (centralized/decentralized/hybrid)
- [ ] Choose communication method (DDS/MAVLink/both)
- [ ] Design formation types and transitions
- [ ] Plan collision avoidance strategy
- [ ] Define failsafe behaviors

**Phase 2: Core Implementation**
- [ ] Create AP_Swarm library (or extend existing)
- [ ] Add neighbor tracking
- [ ] Implement formation control
- [ ] Add collision avoidance
- [ ] Create custom DDS topics/services
- [ ] Add parameters
- [ ] Implement logging

**Phase 3: Integration**
- [ ] Integrate with flight modes
- [ ] Add to scheduler
- [ ] Add GCS/telemetry support
- [ ] Implement failsafe integration
- [ ] Add geofence considerations

**Phase 4: Testing**
- [ ] Unit tests
- [ ] SITL single vehicle
- [ ] SITL multi-vehicle (3+)
- [ ] Hardware-in-loop
- [ ] Field testing (controlled environment)
- [ ] Full swarm test

**Phase 5: Documentation**
- [ ] Code comments
- [ ] Parameter documentation
- [ ] Wiki/user guide
- [ ] Safety procedures
- [ ] Troubleshooting guide

---

## Appendix A: Swarm Example - Formation Flying

### Complete Example Implementation

**File Structure**:
```
libraries/AP_Swarm/
├── AP_Swarm.h
├── AP_Swarm.cpp
├── AP_Swarm_config.h
├── AP_Swarm_Formation.cpp
└── wscript

libraries/AP_DDS/
├── AP_DDS_Swarm.cpp         # New
├── AP_DDS_Swarm.h           # New
└── [existing files]

ArduCopter/
├── mode_swarm.cpp            # New
└── [existing files]
```

**Key Implementation Points**:

1. **Formation Types**: Line, V-formation, Circle, Grid
2. **Leader Election**: Lowest vehicle ID or manual
3. **Position Calculation**: Based on formation type and position index
4. **State Machine**: IDLE → FORMING → FORMED → MISSION → RTL
5. **Communication**: DDS for state, MAVLink for fallback

---

## Appendix B: Troubleshooting

### Common Issues

**Issue**: DDS topics not appearing
- Check `DDS_ENABLE = 1`
- Verify serial protocol set to 45 (DDS)
- Ensure Agent is running
- Check firewall for UDP

**Issue**: Build errors with new library
- Verify `wscript` is correct
- Check all dependencies listed
- Ensure `#include` order correct
- Check for circular dependencies

**Issue**: Parameters not saving
- Check EEPROM not full
- Verify parameter index unique
- Use `param fetch` then `param set`

**Issue**: Swarm vehicles colliding
- Increase `SWARM_SEP_DIST`
- Check `AVOID_ENABLE = 1`
- Verify neighbor updates received
- Check formation calculations

---

## Conclusion

This document provides a comprehensive foundation for developing autonomous UAV swarm systems using ArduPilot. Key takeaways:

1. **ArduPilot** is highly modular - leverage existing libraries
2. **AP_DDS** provides robust ROS 2 integration for swarm communication
3. **Configuration flags** allow conditional compilation for different platforms
4. **Scheduler system** enables efficient multi-rate task execution
5. **Safety first** - always implement proper failsafes for swarm operations

**Next Steps**:
1. Experiment with SITL multi-vehicle setups
2. Extend AP_DDS with custom swarm topics
3. Implement formation control algorithms
4. Test extensively before field deployment

For questions and community support:
- ArduPilot Discord: https://ardupilot.org/discord
- ArduPilot Forum: https://discuss.ardupilot.org/

Happy swarming! 🚁🚁🚁
