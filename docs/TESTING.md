# Testing Procedures

## Pre-Flight Checklist

Before first power-on, verify:

- [ ] Motor power supply disconnected
- [ ] E-STOP button functional (NC contacts)
- [ ] Load cell securely mounted
- [ ] Encoder coupled to motor shaft
- [ ] All GPIO connections correct per wiring diagram

---

## Phase 1: Hardware Verification

### 1.1 Encoder Test

**Goal**: Verify encoder pulses are being read correctly.

**Steps**:
1. Power ESP32-S3, open serial monitor at 115200 baud
2. Manually rotate encoder by hand
3. Observe console output for count changes

**Expected**:
- Count increases for CW rotation
- Count decreases for CCW rotation
- RPM shows approximate hand rotation speed

**Pass Criteria**: Counts change in correct direction.

---

### 1.2 Load Cell Test

**Goal**: Verify HX711 is reading and calibration works.

**Steps**:
1. Remove all load from sensor
2. Send `C` command (tare)
3. Wait for "Tare complete"
4. Apply known weight (e.g., 1kg)
5. Observe tension reading in console

**Expected**:
- After tare, reading should be ~0.0 kg
- With 1kg applied, reading should be approximately correct

**Pass Criteria**: Reading changes proportionally to applied weight.

---

### 1.3 Motor PWM Test

**Goal**: Verify motor responds to PWM commands.

**Steps**:
1. Connect motor with current-limited power supply
2. Power ESP32-S3
3. Via web interface, start system with low setpoint
4. Observe motor rotation

**Expected**:
- Motor spins at low speed
- Encoder shows RPM reading
- Motor stops on STOP command

**Pass Criteria**: Motor responds to start/stop commands.

---

## Phase 2: Control Loop Tests

### 2.1 Speed Loop Step Response

**Goal**: Verify speed PI controller tracks setpoint.

**Setup**:
1. Motor connected with proper power
2. Encoder working
3. Web dashboard open

**Steps**:
1. Start system via web UI
2. Set tension setpoint to 2.0 kg
3. Observe speed response on chart

**Expected Results**:
- Settling time: < 3 seconds
- Overshoot: < 25%
- Steady-state error: < 5%

**Data to Record**:
- Rise time (10% to 90%)
- Settling time (within 2% band)
- Maximum overshoot

---

### 2.2 Tension Loop Step Response

**Goal**: Verify cascaded tension control works.

**Setup**:
1. Material loaded with measurable tension
2. Load cell calibrated
3. Speed loop tuned

**Steps**:
1. Start system
2. Change tension setpoint from 2.0 to 4.0 kg
3. Observe tension response on chart

**Expected Results**:
- Tension tracks setpoint
- No oscillation
- Speed adapts automatically

---

### 2.3 Disturbance Rejection

**Goal**: Verify control handles disturbances.

**Steps**:
1. System running at steady state
2. Manually apply brief tension pulse (push on material)
3. Observe recovery

**Expected**:
- System recovers within 2 seconds
- No sustained oscillation

---

## Phase 3: Safety Tests

### 3.1 E-STOP Test

**Goal**: Verify emergency stop works immediately.

**Steps**:
1. Start system at moderate speed
2. Press E-STOP button
3. Attempt to restart without releasing E-STOP

**Expected**:
- Motor stops immediately (< 100ms)
- System enters E-STOP state
- Cannot restart until E-STOP released
- Serial shows "EMERGENCY STOP" message

**Pass Criteria**: Complete motor stop within 100ms.

---

### 3.2 Over-Tension Test

**Goal**: Verify tension limit protection.

**Steps**:
1. Start system
2. Manually increase load beyond configured limit
3. Observe system response

**Expected**:
- Fault triggered
- Motor stops
- Fault message displayed

---

### 3.3 Encoder Failure Test

**Goal**: Verify encoder disconnect detection.

**Steps**:
1. Start system
2. Disconnect encoder cable

**Expected**:
- Fault detected within 1 second
- Motor stops
- "Encoder Failure" fault displayed

---

## Phase 4: Auto-Tuning Verification

### 4.1 Speed Loop Auto-Tune

**Goal**: Verify automatic PI tuning produces stable response.

**Steps**:
1. Ensure system is idle
2. Send `T` command or click "Auto-Tune"
3. Observe oscillation pattern
4. Wait for completion message

**Expected**:
- Brief controlled oscillation
- "Tuning complete" message
- New PI gains applied and saved

**Verification**:
After tuning, perform step response test to confirm stable control.

---

## Test Report Template

```
Test: ________________
Date: ________________
Tester: ______________

Hardware:
- ESP32-S3 S/N: ________
- Motor: ________________
- Load Cell: ____________

Results:
[ ] PASS  [ ] FAIL

Measurements:
- Settling time: ___ s
- Overshoot: ___ %
- Notes: ________________

Signature: ______________
```

---

## Acceptance Criteria Summary

| Test | Criteria | Result |
|------|----------|--------|
| Encoder | Counts change correctly | ☐ |
| Load Cell | Tare and span calibration | ☐ |
| Motor PWM | Responds to commands | ☐ |
| Speed Loop | Settling < 3s, overshoot < 25% | ☐ |
| Tension Loop | Tracks setpoint | ☐ |
| E-STOP | Stops within 100ms | ☐ |
| Over-tension | Fault triggered | ☐ |
| Auto-tune | Completes successfully | ☐ |
| Web Interface | Real-time updates work | ☐ |
