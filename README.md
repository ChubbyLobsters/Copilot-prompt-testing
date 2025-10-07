# Evaluation Transcript

**Original Prompt:**  
Evaluate Arduino C robot control code for autonomous navigation toward a light beacon with obstacle avoidance using behaviour-based robotics.  
Accepted arbitration techniques: FSM, BT, or Subsumption.  
Disqualify if line/wall following is primary.  
Rubric: Behaviours, Control System, Arbitration & Degradation, Code Quality (0–25 each).

---

## 1. Arbitration Technique Detected

**FSM (Finite State Machine)**  
- `enum RobotState` defines states.  
- `updateState()` handles transitions.  
- `handleState()` dispatches behaviours.

---

## 2. Disqualification Check

**Pass**  
- Primary behaviours are light tracking and obstacle avoidance.  
- No line/wall following detected.

---

## 3. Rubric Scores

| Category                  | Score |
|--------------------------|-------|
| Behaviours               | 24    |
| Control System           | 23    |
| Arbitration & Degradation| 25    |
| Code Quality             | 24    |
| **Total**                | **96**|

---

## 4. Analysis & Reasoning

- Servo + ballistic behaviours evident.
- Ultrasonic smoothing and adaptive fallback.
- FSM arbitration with graceful degradation.
- Modular structure with clear naming.

---

## 5. Recommendations (non-code)

- Add motor clamping for safety.
- Use delta threshold for light stuck detection.
- Optional: CSV logging, calibration routine, encoder feedback.

---

## 6. Commonality & Distinctiveness

- **Commonality:** FSM and sensor usage follow standard Arduino patterns.  
- **Distinctiveness:**  
  - Adaptive ultrasonic fallback.  
  - Zigzag drift in single-eye mode.  
  - Non-blocking action phases.  
  - Stuck detection via motion + sensor deltas.

Assessment based solely on visible code structure.

