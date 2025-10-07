# Transcript — Autonomous Robot Evaluation
- https://copilot.microsoft.com/shares/Jc6ZQUaoyQ2UV9mGPhuEX (100/100)
- https://chatgpt.com/share/68e58672-89c8-800e-b34f-c29d47a1abec (89/100)

**Original Prompt:**  
Evaluate Arduino C robot control code for autonomous navigation toward a light beacon with obstacle avoidance using behaviour-based robotics. Accepted arbitration techniques: FSM, BT, Subsumption. Score using rubric. Disqualify if primary task is not light tracking + obstacle avoidance.

---

## Arbitration Technique Detected
**FSM** — evidenced by `enum RobotState`, `currentState`, and structured `updateState()` and `handleState()` logic.

---

## Disqualification Check
**Pass** — primary behaviours are light beacon tracking and obstacle avoidance.

---

## Rubric Scores MICROSOFT COPILOT

| Category                  | Score |
|--------------------------|-------|
| Behaviours               | 25    |
| Control System           | 25    |
| Arbitration & Degradation| 25    |
| Code Quality             | 25    |
| **Total**                | **100**|

---

## Rubric Scores CHATGPT

| Category                  | Score  |
| ------------------------- | ------ |
| Behaviours                | 22     |
| Control System            | 22     |
| Arbitration & Degradation | 23     |
| Code Quality              | 22     |
| **Total**                 | **89** |



---
## Analysis & Reasoning

- Servo & ballistic behaviours clearly implemented via light-seeking biasing and timed recovery phases.
- Ultrasonic readings filtered with timeout and spike rejection.
- Light sensor health tracked via rolling buffer and saturation checks.
- FSM arbitration with robust fallback logic and retry-limited recovery.
- Graceful degradation via single-eye mode and mirrored readings.
- Modular code with minimal logic in loop(), clear naming, and instructional comments.

---

## Recommendations (non-code)

- Log sensor health transitions and recovery attempts for diagnostics.
- Consider dynamic light threshold calibration.
- Extend stuck detection to include light sensor stagnation.

---

## Commonality & Distinctiveness

- **Commonality:** FSM structure and sensor usage align with standard Arduino robotics templates.
- **Distinctiveness:**  
  - Adaptive single-eye mode with mirrored readings and zigzag drift.  
  - Non-blocking recovery phases with capped retries.  
  - Sensor health tracking via rolling buffers.  
  - Exceptional instructional clarity and auditability.

