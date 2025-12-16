# REP103 Compliance Plan

## Overview

This document outlines the changes required to make the codebase compliant with [REP103 - Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html).

### REP103 Key Conventions
- **Coordinate Frame**: X+ forward, Y+ left, Z+ up (right-handed)
- **Rotation**: Counter-clockwise positive around Z-axis (right-hand rule)
- **Units**: SI units (meters, radians)

---

## Files Requiring Modification

### Priority: CRITICAL

#### 1. `fake_robot_2d.py` (Line 64)

**File**: `src/fake_rover_state_controller/fake_rover_state_controller/fake_robot_2d.py`

**Current Code**:
```python
self.position['theta'] -= UPDATE_RATE * self.vel['rotate']
```

**Issues**:
- Subtraction (`-=`) makes clockwise rotation positive (violates REP103)
- `UPDATE_RATE * velocity` is incorrect; should be `velocity / UPDATE_RATE` or `velocity * dt`

**REP103 Compliant Fix**:
```python
self.position['theta'] += self.vel['rotate'] / UPDATE_RATE
```

---

#### 2. `sim_robot_2d.py` (Lines 146-149)

**File**: `src/fake_rover_state_controller/fake_rover_state_controller/sim_robot_2d.py`

**Current Code**:
```python
# Position update (directly applying x,y velocity in world frame)
self.position['theta'] = self._wrap_to_pi(self.position['theta'] + self.current_velocity['angular'] * dt)
self.position['x'] += self.current_velocity['x'] * dt
self.position['y'] += self.current_velocity['y'] * dt
```

**Issues**:
- `Twist.linear.x/y` are in body frame, but code applies them directly to world frame
- Missing rotation transformation from body frame to world frame

**REP103 Compliant Fix**:
```python
# Position update (body frame velocity -> world frame)
theta = self.position['theta']
self.position['theta'] = self._wrap_to_pi(theta + self.current_velocity['angular'] * dt)

# Transform body velocities to world frame
vx_body = self.current_velocity['x']
vy_body = self.current_velocity['y']
self.position['x'] += (vx_body * math.cos(theta) - vy_body * math.sin(theta)) * dt
self.position['y'] += (vx_body * math.sin(theta) + vy_body * math.cos(theta)) * dt
```

---

### Priority: HIGH

#### 3. `sim_rover.py` (Lines 133-134)

**File**: `src/fake_rover_state_controller/fake_rover_state_controller/sim_rover.py`

**Current Code**:
```python
heading_mid = self.position['theta'] + 0.5 * self.current_velocity['angular'] * dt
self.position['theta'] = self._wrap_to_pi(self.position['theta'] + self.current_velocity['angular'] * dt)
self.position['x'] += self.current_velocity['linear'] * (-math.sin(heading_mid)) * dt
self.position['y'] += self.current_velocity['linear'] * math.cos(heading_mid) * dt
```

**Issues**:
- Uses Y+ forward coordinate system (`-sin` for X, `cos` for Y)
- REP103 requires X+ forward coordinate system

**REP103 Compliant Fix**:
```python
heading_mid = self.position['theta'] + 0.5 * self.current_velocity['angular'] * dt
self.position['theta'] = self._wrap_to_pi(self.position['theta'] + self.current_velocity['angular'] * dt)
self.position['x'] += self.current_velocity['linear'] * math.cos(heading_mid) * dt
self.position['y'] += self.current_velocity['linear'] * math.sin(heading_mid) * dt
```

---

#### 4. `Controller.py` (Lines 302-305)

**File**: `src/controller/controller/Controller.py`

**Current Code**:
```python
ct = math.cos(self.c[2])
st = math.sin(self.c[2])
self.cdot_des[0, 0] = x*ct - y*st
self.cdot_des[1, 0] = x*st + y*ct
```

**Issues**:
- Rotation transformation direction is ambiguous
- Missing documentation on coordinate frame convention

**REP103 Compliant Fix**:
```python
# Body frame to world frame rotation (Z-axis, counter-clockwise positive)
ct = math.cos(self.c[2])  # theta = self.c[2]
st = math.sin(self.c[2])
# Standard 2D rotation matrix: [cos -sin; sin cos] * [x; y]
self.cdot_des[0, 0] = x * ct - y * st  # world X
self.cdot_des[1, 0] = x * st + y * ct  # world Y
```

**Note**: Verify if this is body-to-world or world-to-body transformation and add appropriate comments.

---

#### 5. `Cluster.py` (Lines 64-94)

**File**: `src/cluster_node/cluster_node/Cluster.py`

**Current Code (Forward Kinematics)**:
```python
theta_c = sp.atan2(
    2/3 * r_sym[0] - 1/3 * (r_sym[3] + r_sym[6]),
    2/3 * r_sym[1] - 1/3 * (r_sym[4] + r_sym[7])
)
```

**Current Code (Inverse Kinematics)**:
```python
x_1 = c_sym[0] + r/3 * sp.sin(c_sym[2])
y_1 = c_sym[1] + r/3 * sp.cos(c_sym[2])
```

**Issues**:
- `atan2(x_component, y_component)` has reversed arguments (standard is `atan2(y, x)`)
- `sin` for X and `cos` for Y indicates Y+ forward convention

**REP103 Compliant Fix**:
```python
# Forward kinematics - cluster orientation
theta_c = sp.atan2(
    2/3 * r_sym[1] - 1/3 * (r_sym[4] + r_sym[7]),  # Y component
    2/3 * r_sym[0] - 1/3 * (r_sym[3] + r_sym[6])   # X component
)

# Inverse kinematics - robot positions (X+ forward)
x_1 = c_sym[0] + r/3 * sp.cos(c_sym[2])
y_1 = c_sym[1] + r/3 * sp.sin(c_sym[2])
```

---

#### 6. `AdaptiveNavigator.py` (Line 237)

**File**: `src/adaptive_nav/adaptive_nav/AdaptiveNavigator.py`

**Current Code**:
```python
current_bearing: float = math.pi/2 - math.atan2(grad[1], grad[0])
```

**Issues**:
- `pi/2 - atan2()` converts from X+ forward to Y+ forward reference
- REP103 uses X+ as the forward (0 angle) direction

**REP103 Compliant Fix**:
```python
# Bearing from gradient vector (X+ forward = 0 radians)
current_bearing: float = math.atan2(grad[1], grad[0])
```

---

### Priority: MEDIUM

#### 7. `adaptive_nav.py` (Lines 246-247)

**File**: `src/adaptive_nav/adaptive_nav/adaptive_nav.py`

**Current Code**:
```python
cmd_vel.linear.x = math.cos(bearing) * KV
cmd_vel.linear.y = math.sin(bearing) * KV
```

**Status**: Likely correct if bearing is defined with X+ forward = 0 radians

**Action Required**:
- Verify bearing calculation source
- Add documentation comment clarifying coordinate convention

---

## Implementation Order

1. **Phase 1 - Core Simulation** (CRITICAL)
   - [x] `fake_robot_2d.py` - COMPLETED
   - [x] `sim_robot_2d.py` - COMPLETED

2. **Phase 2 - Rover Kinematics** (HIGH)
   - [x] `sim_rover.py` - COMPLETED
   - [x] `Controller.py` - COMPLETED (comment added)

3. **Phase 3 - Cluster Kinematics** (HIGH)
   - [x] `Cluster.py` - COMPLETED
   - [x] `AdaptiveNavigator.py` - COMPLETED

4. **Phase 4 - Verification** (MEDIUM)
   - [x] `adaptive_nav.py` - VERIFIED (already REP103 compliant, comment added)
   - [ ] Add unit tests for coordinate transformations
   - [ ] Update documentation

---

## Testing Strategy

### Unit Tests
- Verify forward motion increases X position (not Y)
- Verify counter-clockwise rotation increases theta
- Verify body-to-world transformation correctness

### Integration Tests
- Run simulation with known inputs
- Compare trajectory output with expected REP103-compliant behavior

### Visual Verification
- Observe robot movement in RViz
- Confirm X+ forward, Y+ left orientation

---

## Related Files (No Changes Required)

| File | Status | Notes |
|------|--------|-------|
| `tf_broadcaster.py` | OK | Passes through Pose2D directly |
| `pioneer_template.urdf` | OK | Mesh orientation may need verification |
| `my_ros_module.py` | OK | No coordinate calculations |

---

## References

- [REP 103 - Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP 105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [ROS Wiki - Coordinate Frame Conventions](http://wiki.ros.org/geometry/CoordinateFrameConventions)
