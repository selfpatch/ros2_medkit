# ros2_medkit_msgs

ROS 2 message and service definitions for the ros2_medkit fault management system.

## Overview

This package provides the interface definitions used by the fault management components:

- **FaultManager** (`ros2_medkit_fault_manager`) - Central fault aggregation and lifecycle management
- **FaultReporter** (`ros2_medkit_fault_reporter`) - Client library for fault reporting
- **Gateway** (`ros2_medkit_gateway`) - REST API endpoints for fault access

## Messages

### Fault.msg

Core fault data model representing an aggregated fault condition.

| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | Global fault identifier (e.g., "MOTOR_OVERHEAT") |
| `severity` | uint8 | Severity level (use SEVERITY_* constants) |
| `description` | string | Human-readable description |
| `first_occurred` | builtin_interfaces/Time | When fault was first reported |
| `last_occurred` | builtin_interfaces/Time | When fault was last reported |
| `occurrence_count` | uint32 | Total reports aggregated across all sources |
| `status` | string | PENDING, CONFIRMED, or CLEARED |
| `reporting_sources` | string[] | List of source identifiers that reported this fault |

**Severity Levels:**
| Constant | Value | Description |
|----------|-------|-------------|
| `SEVERITY_INFO` | 0 | Informational, no action required |
| `SEVERITY_WARN` | 1 | May require attention, no impact on functionality |
| `SEVERITY_ERROR` | 2 | Impacts functionality, requires intervention |
| `SEVERITY_CRITICAL` | 3 | Severe, may compromise safety or system operation |

**Status Lifecycle:**
- `PENDING` → `CONFIRMED`: After reaching confirmation threshold (N reports in time window)
- `CONFIRMED` → `CLEARED`: Via ClearFault service call

### FaultEvent.msg

Real-time fault event notification for SSE streaming (published on `/fault_manager/events`).

| Field | Type | Description |
|-------|------|-------------|
| `event_type` | string | Event type (see constants below) |
| `fault` | Fault | The fault data (state after event) |
| `timestamp` | builtin_interfaces/Time | When the event occurred |

**Event Types:**
| Constant | Trigger |
|----------|---------|
| `EVENT_CONFIRMED` | Fault transitions PENDING → CONFIRMED |
| `EVENT_CLEARED` | Fault transitions to CLEARED |
| `EVENT_UPDATED` | Fault data changes without status transition |

## Services

### ReportFault.srv

Report a fault occurrence to the FaultManager.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | Global identifier (UPPER_SNAKE_CASE, max 64 chars) |
| `severity` | uint8 | Severity level (0-3, use Fault.SEVERITY_* constants) |
| `description` | string | Human-readable description |
| `source_id` | string | Reporting node FQN (e.g., "/powertrain/engine/temp_sensor") |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | True if fault was recorded |
| `message` | string | Status or error message |

### GetFaults.srv

Query faults with optional filtering.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `filter_by_severity` | bool | If true, filter by severity field; if false, return all severities |
| `severity` | uint8 | Severity level (0-3), only used if filter_by_severity is true |
| `statuses` | string[] | Statuses to include (empty = CONFIRMED only) |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `faults` | Fault[] | Matching faults |

**Examples:**
- Default (active faults): `filter_by_severity=false, statuses=[]` → all CONFIRMED faults
- Only errors: `filter_by_severity=true, severity=2, statuses=[]` → CONFIRMED with ERROR
- All faults: `filter_by_severity=false, statuses=["PENDING", "CONFIRMED", "CLEARED"]`
- Historical: `filter_by_severity=false, statuses=["CLEARED"]`

### ClearFault.srv

Clear/acknowledge a fault. Cleared faults are retained and queryable with `statuses=["CLEARED"]`.

**Request:**
| Field | Type | Description |
|-------|------|-------------|
| `fault_code` | string | The fault to clear |

**Response:**
| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | True if fault was cleared |
| `message` | string | Status or error message |

## Usage

### C++

```cpp
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

// Create a fault message
ros2_medkit_msgs::msg::Fault fault;
fault.fault_code = "MOTOR_OVERHEAT";
fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
```

### Python

```python
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

# Create a fault message
fault = Fault()
fault.fault_code = "MOTOR_OVERHEAT"
fault.severity = Fault.SEVERITY_ERROR
fault.status = Fault.STATUS_CONFIRMED
```

## Building

```bash
colcon build --packages-select ros2_medkit_msgs
source install/setup.bash  # or setup.zsh for zsh users
```

## Verifying

```bash
ros2 interface show ros2_medkit_msgs/msg/Fault
ros2 interface show ros2_medkit_msgs/srv/ReportFault
```

## License

Apache-2.0
