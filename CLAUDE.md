# ELEC 571 Project — Handoff Context for Claude Code

> Paste or save this file as `CLAUDE.md` in the project folder (next to the paper PDF) so Claude Code reads it at the start of every session. It summarizes everything decided in the planning conversation so work can continue from where it stopped.

---

## 1. Project snapshot

- **Course:** ELEC 571 (Computational Control). Lecture topics: convex optimization, KKT, state space, dynamic programming, LQR, MPC (explicit and online). Notation used in lectures: x⁺ = Ax + Bu, batch QP view of LQR/MPC, Riccati/DARE, offset-free MPC with disturbance estimate d̂.
- **Team:** Youssef (builds the **plant / simulation**) + a partner (builds the **controller**).
- **Due:** **December 9, 2026** (report + MATLAB code).
- **Tools:** MATLAB + Simulink, all toolboxes available (Optimization, Robotics System, Control System, MPC, Statistics/ML). No ROS.
- **Base paper:** J. Wang et al., *"VisionSafeEnhanced VPC: Cautious Predictive Control with Visibility Constraints under Uncertainty for Autonomous Robotic Surgery,"* arXiv:2508.18937 (2025). File: `Wang_2025_VisionSafeEnhanced_VPC.pdf`.
- **Project scope (user's decision, final):** Recreate the paper's system, **following the paper**, in simplified form first. Apply MPC (and, in the original pitch, DeePC as a comparison). Add more of the paper's method (CBF, GPR, chance constraints) only if time allows. "We'll see along the way how complex it is."
- **Current stage:** Youssef is starting the **MATLAB/Simulink plant model**. The controller is the partner's job; the plant must expose clean ports for it.

## 2. How Youssef wants to work (important)

- **Follow the paper.** Cite the equation/section number for every modelling choice. If something is *not* in the paper, say so explicitly and explain why it's needed. Do not silently simplify or "replace" parts of the paper's model.
- **Concise and plain.** He has controls theory and strong programming background but is not an expert in visual servoing or robotics papers. Explain terms in one line; avoid walls of text.
- **Render math properly** (LaTeX with `$...$` / `$$...$$`, never raw code blocks of TeX in chat).
- He prefers his own plan as the primary structure; help navigate it and catch errors/omissions.

## 3. The system (what is being controlled)

| Thing | Role | Paper reference |
|---|---|---|
| 6-joint robot arm holding the laparoscope | **The plant.** State = joint angles q ∈ ℝ⁶ | Section II, Eq. (1) |
| Camera | Rigidly mounted at the laparoscope tip → **camera motion = arm tip motion** | Section II: "A monocular camera mounted at the laparoscope tip" |
| Control input | **Joint speeds q̇(k) ∈ ℝ⁶** (not angles) | Section III-A: "the control input is defined as u(k) = q̇(k)" |
| Surgical instrument (scissors) | Moved by the **surgeon** (teleoperated), **not** by the controller. Acts as a disturbance. | Section IV-A: "The scissors were teleoperated along the multi-target sequence, while the laparoscope autonomously adjusted its pose" |
| Output | **s = [u, v]ᵀ**: pixel position of the instrument tip in the image (an "image feature" = 2 numbers, not a whole image) | Section II, Eq. (2); Eq. (8) states s(k) ∈ ℝ² |

The camera and the instrument move separately; they only "meet" in the image. The controller sees the instrument's motion only through the measured s(k) each step.

## 4. Paper equations used (Section II–III)

**Eq. 1 — arm kinematics:** ẋ = J_r(q) q̇, with ẋ = [v_x, v_y, v_z, ω_x, ω_y, ω_z]ᵀ the camera (TCP) velocity. J_r ∈ ℝ⁶ˣ⁶ "is determined by the geometric parameters and joint configuration" (DH), ref. [22] = Siciliano et al., *Robotics: Modelling, Planning and Control* (general geometric Jacobian).

**Eq. 2 — image kinematics:** ṡ = L_s ẋ, L_s ∈ ℝ²ˣ⁶ (interaction matrix, ref. [23] = Chaumette & Hutchinson 2006).

**Eq. 3–4 — interaction matrix** (û = u − u₀, v̂ = v − v₀; f_x, f_y focal lengths in px; (u₀, v₀) principal point; d = target depth in camera frame):

$$
L_s = \begin{bmatrix} -\frac{f_x}{d} & 0 & \frac{\hat u}{d} & \frac{\hat u \hat v}{f_y} & -\left(f_x + \frac{\hat u^2}{f_x}\right) & \frac{f_x \hat v}{f_y} \\ 0 & -\frac{f_y}{d} & \frac{\hat v}{d} & f_y + \frac{\hat v^2}{f_y} & -\frac{\hat u \hat v}{f_x} & -\frac{f_y \hat u}{f_x} \end{bmatrix}
$$

Columns 1–3 multiply camera translation (divided by depth d); columns 4–6 multiply rotation (no d).

**Eq. 5:** J_s = L_s J_r ∈ ℝ²ˣ⁶ (joint speeds → pixel velocity).

**Eq. 6 — discrete model (Euler):**

$$ s(k+1) = s(k) + \delta t\, J_s\, \dot q(k) + B_d\big(d(s(k), \dot q(k)) + w(k)\big) $$

- δt sampling period; B_d disturbance distribution matrix; d(s, q̇) residual (unmodelled dynamics, later learned by GPR); w(k) ~ N(0, σ_w²) noise.
- Euler = "new position = old + velocity × δt". The uncertainty is **not** handled by Euler; it is represented by the added d and w terms and handled later by GPR/CBF.
- **Symbol clash:** d = depth (Eq. 3) vs d(s, q̇) = residual (Eq. 6). Unrelated.

**Eq. 7 — VPC cost (Classical VPC):**

$$ J = \sum_{i=1}^{N_p} (s_{k+i} - s_d)^\top Q (s_{k+i} - s_d) + \sum_{i=0}^{N_c} \dot q_{k+i}^\top R\, \dot q_{k+i} $$

s.t. q_min ≤ q_{k+i} ≤ q_max, q̇_min ≤ q̇_{k+i} ≤ q̇_max.

**Eq. 8 — FoV safe set:** h(s) = r² − ‖s − s_c‖² ≥ 0 (s_c FoV centre, r radius "parameterized by camera intrinsics").

**Eq. 9 — discrete CBF:** h(s(k+1)) − (1 − α) h(s(k)) ≥ 0, 0 < α ≤ 1 ("safety margin may shrink by at most a fraction α per step").

**Eqs. 10–20 (GPR + uncertainty, plug-in later):** residual training target y_k = B_d†(s_{k+1} − s_k − δt J_s q̇_k) (Eq. 10); GPR input z_k = [s; q] (8-D: two image features + six joint angles); predictor s̄_{k+1} = s̄_k + δt J_s q̇_k + B_d μ(z_k) (Eq. 14); covariance propagation (Eq. 15); uncertainty-adaptive CBF with shrunk radius r − βσ_s (Eqs. 16–18); chance constraints (Eqs. 19–20). Full problem: Eq. 21.

**Controller variants in the paper:**
- *Classical VPC* = Eq. 7 only (no FoV constraint). Laparoscope stays still and the scissors leave view (FoVSR 61.73%).
- *VisionSafe VPC* = + CBF FoV constraint (FoVSR 98.97%).
- *VisionSafeEnhanced VPC* (theirs) = + GPR + uncertainty-adaptive CBF + chance constraints (FoVSR 99.98%).
- *FullyTracking VPC* (experiments) = continuous IBVS centring: always visible but jittery, high motion cost.

**Metrics:**
- FoVSR = % of time the instrument is inside the FoV.
- AVSS (Eq. 22): ε_i = max(0, ‖s_i − s_c‖/r − 1), E = (1/N) Σ ε_i.
- MAE / RMSE vs the disturbance-free ground-truth trajectory.
- MAC (Eq. 24): E_k = Σ_{τ=0}^{k} q̇_τᵀ M(q_ref) q̇_τ (pseudo-kinetic energy; M = mass matrix at a reference configuration).

**Paper parameters (Table II, Section IV-A):** N_p = 10, N_c = 5, Q = I₂, R = 0.1·I₆, α = 0.5, GPR length-scale 0.5·I₆, σ_f² = 0.1, GPR init 50 samples + sliding window 30. Simulation: **10% perturbation of DH parameters**, process/measurement noise zero-mean Gaussian **std 0.001 m**. Scenario: scissors move **L1 → L2 → L3 → L1**, FoV modelled as a circle.

## 5. Plant design (Youssef's part) — agreed structure

Youssef's own flowchart (in `System_Modelling_.pdf`) was checked and is correct: q → J_r(q) q̇ → ẋ → ṡ = L_s ẋ → Euler → Eq. 6. Corrections/additions agreed:

1. Input is **q̇**, state is q. Add the joint update **q(k+1) = q(k) + δt q̇(k)** (implied by Eq. 7 constraining q while the input is q̇).
2. Label fix: ṡ is the **instrument tip's pixel velocity**, not "camera position velocity".
3. Write Eq. 5 explicitly (J_s = L_s J_r).
4. L_s is recomputed **every step**: it needs the **current s(k)** (u, v from the last Eq. 6 output, fed back through a Unit Delay), depth d, and intrinsics.
5. **Frame conversion W (not in the paper, but required with a real arm):** L_s expects camera velocity in the **camera frame**, ordered **[v; ω]**. MATLAB `geometricJacobian` returns velocity in the **base frame**, ordered **[ω; v]**. So:

$$ W(q) = \begin{bmatrix} 0 & R(q)^\top \\ R(q)^\top & 0 \end{bmatrix}, \qquad J_s = L_s\, W(q)\, J_r(q) $$

   with R(q) = camera orientation in the base frame (`getTransform`). The paper implicitly assumes J_r already gives camera-frame velocity.

**Plant blocks (Simulink, discrete fixed-step δt):**

| Block | Equation | Source |
|---|---|---|
| Saturate input (optional) | clip q̇ to [q̇_min, q̇_max] | Eq. 7 limits |
| Joint angles | q(k+1) = q(k) + δt q̇(k) | implied by Eq. 7 |
| Arm Jacobian | J_r(q) via `geometricJacobian` (UR5 + camera body) | Eq. 1, ref. [22] |
| Frame conversion | W(q) from R(q) | needed (see above) |
| Interaction matrix | L_s(s, d, f_x, f_y, u₀, v₀) | Eqs. 3–4 |
| Image Jacobian | J_s = L_s W J_r | Eq. 5 |
| Image update | s(k+1) = s(k) + δt J_s q̇(k) + B_d(d(·) + w(k)) | Eq. 6 |

Inputs to plant: q̇(k) (from controller), depth d, residual d(s, q̇) port (instrument's own pixel motion + unmodelled effects; Eq. 6 has no explicit instrument term), noise w(k).

**Ports for the partner (controller interface):**

| Port | Size | Direction | Meaning |
|---|---|---|---|
| q̇(k) | 6 | controller → plant | joint speeds (control input) |
| s(k) | 2 | plant → controller | instrument tip pixel [u, v] |
| q(k) | 6 | plant → controller | joint angles (for J_r(q) and joint limits) |
| depth d | 1 | shared | controller needs it for its own L_s |
| parameters | — | shared file | nominal DH, f_x, f_y, u₀, v₀, δt, joint limits, s_c, r |

**Model-error test (paper, Section IV-A):** plant uses DH **perturbed by 10%**; controller uses **nominal** DH. The controller is never told the true values; the test is whether it still keeps the instrument in view.

## 6. Parameter values chosen (with sources)

| Parameter | Value | Why / source |
|---|---|---|
| Arm | **UR5**, 6 revolute joints | matches J_r ∈ ℝ⁶ˣ⁶; paper's robot (MicroPort Toumai) DH not published; UR arms used in autonomous laparoscopy research (arXiv:2603.08490) |
| UR5 DH (nominal) | a = [0, −0.425, −0.39225, 0, 0, 0] m; d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823] m; α = [π/2, 0, 0, π/2, −π/2, 0] | official Universal Robots DH page |
| Plant DH | nominal ± 10% | paper Section IV-A |
| Laparoscope | rigid shaft ≈ 0.3 m fixed to UR5 flange, camera body at tip, z-axis out of lens | our choice (paper gives no length) |
| Image size | 1920 × 1080 px | endoscope in the paper's ref. [27] (Gruijthuijsen et al. 2022, arXiv:2107.02317): 60 fps, 1920 × 1080 |
| f_x = f_y | ≈ 1371 px | f = 960 / tan(70°/2); 70° = standard laparoscope FoV; square pixels |
| u₀, v₀ | 960, 540 px | image centre (ideal camera) |
| Depth d | 0.08 m constant | ref. [27] desired depth z* = 8 cm; laparoscope working range 10–150 mm |
| δt | 1/30 s (or 1/60 s) | set by camera frame rate (60 fps in ref. [27]) and MPC solve time; loop runs on a PC, not an MCU; robot's internal joint controller is much faster, so commanded joint speeds are tracked within δt |
| Noise w | σ = 0.001 | paper Section IV-A |
| B_d | I₂ (simplest) | paper does not specify |
| Units | f in px, d in m → camera velocity in m/s, rad/s → ṡ in px/s | |
| Paper's ref. [27] also reports | tooltip measurements at 9 Hz, 340 ms measurement delay | context if a delay test is added |

## 7. What the paper does NOT give (our choices / open items)

- Arm DH parameters and camera intrinsics → chosen above.
- B_d values → I₂.
- How depth is obtained → constant 0.08 m (paper treats d as known; real robot has a stereo laparoscope).
- Instrument motion → Eq. 6 has no instrument term; it enters only through the residual port. The paper's scenario L1 → L2 → L3 → L1 needs concrete target positions (choose so the instrument would leave a static camera's FoV).
- The paper never writes the pinhole projection formula or forward kinematics T(q) explicitly (Section II only says a 3D point "projects onto the image plane"). Youssef wants to stick to the paper's equations; W uses only the orientation R(q) from `getTransform`.
- Trocar / remote-centre-of-motion constraint: real laparoscopes pivot about the incision; the paper does not impose it. Note as a limitation or add later.
- Tracking target s_d: in experiments it is a weighted mix of the two instrument tips and the current lymph node (adaptive weights, ref. [27]). Simplest start: s_d = s_c (image centre). If s_d = instrument tip, the FoV constraint never activates (that's FullyTracking).

## 8. Controller side (partner) — notes

- Controller output = q̇(k) (6). Inputs = s(k), q(k), depth d, plus set values s_d, s_c, r, limits, Q, R, N_p, N_c.
- Prediction model = Eq. 6 without d, w (nominal values): ŝ_{k+i+1} = ŝ_{k+i} + δt J_s q̇_{k+i}; also predict q for joint limits.
- Open choices for the partner (not in the paper): whether J_s is held constant over the horizon (keeps it a QP) or re-linearized; the FoV circle (Eq. 8) is quadratic in s → `fmincon`, or approximate by a polygon → `quadprog`.
- Simulink: MATLAB Function block with `coder.extrinsic('quadprog')` (or `fmincon`); fixed-step discrete solver; Fast Restart for batch runs.
- Original project pitch (to keep in mind for the report): compare model-based MPC vs data-driven **DeePC** (Coulson, Lygeros, Dörfler, "Data-enabled predictive control: In the shallows of the DeePC," ECC 2019) on the same constrained problem. DeePC replaces the prediction model (J_r, L_s, J_s, Eq. 6) with a Hankel matrix of recorded (q̇, s) data; the cost and constraints stay the same. Wang et al. do **not** test DeePC; that comparison is the project's addition. The paper's model error is in the arm DH (10%); the project can additionally test camera-calibration error, noise, and (optionally) delay.

## 9. Plug-and-play later

- **GPR residual** μ(z_k), z_k = [s; q]: train on Eq. 10 target, add B_d μ in the prediction (Eq. 14). MATLAB `fitrgp`. Belongs on the controller side.
- **Uncertainty-adaptive CBF + chance constraints** (Eqs. 15–20).
- **DeePC** controller variant.
- Measurement delay port (not in paper).
- Friction / elastic deformation (named in paper's hardware results) as extra residual terms.
- Trocar (RCM) constraint.

## 10. Suggested build order (plant)

1. Parameter script (`params.m`): UR5 DH nominal + perturbed, intrinsics, δt, d, limits, s_c, r, σ_w.
2. Robot model: `robot = loadrobot("universalUR5","DataFormat","column")`, add fixed body `camera` (≈0.3 m along tool axis, z out of lens).
3. Simulink plant blocks (Section 5 table), discrete fixed-step δt; Unit Delay feeding s(k) back into L_s.
4. **Open-loop checks:** q̇ = 0 → s only changes via the residual port (instrument motion); a constant single-joint speed → s drifts in a consistent direction; compare Eq. 6's one-step prediction against itself with nominal vs perturbed DH to see the model-error gap.
5. Plot s(k) on a virtual 1920×1080 image with the FoV circle (like the paper's figures).
6. Expose ports (Section 5) and hand to partner; agree on one shared parameter file.

## 11. Files

- `Wang_2025_VisionSafeEnhanced_VPC.pdf` — base paper.
- `Laparoscope_Plant_Flowchart.pdf` — plant flowchart, block equations, W, values, sources (also online as a private claude.ai page: https://claude.ai/artifact/7LafzaLzPwbCNGiGs6HQZM).
- `System_Modelling_.pdf` — Youssef's hand-drawn plant flowchart.

## 12. Sources

- Wang et al., arXiv:2508.18937 — https://arxiv.org/abs/2508.18937
- Universal Robots DH parameters — https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
- Gruijthuijsen et al., Robotic endoscope control via autonomous instrument tracking (paper's ref. [27]) — https://arxiv.org/pdf/2107.02317
- Open-source robotics research platform for autonomous laparoscopic surgery — https://arxiv.org/pdf/2603.08490
- Wide-FoV vs conventional 70° laparoscope, Surgical Endoscopy 2023 — https://www.springermedicine.com/cholecystectomy/laparoscopy/comparing-a-novel-wide-field-of-view-laparoscope-with-convention/26071320
- Tang et al., laparoscope calibration (10–150 mm working distance) — https://journal.hep.com.cn/fitee/EN/PDF/10.1631/FITEE.1400149
- Coulson, Lygeros, Dörfler, DeePC, ECC 2019 — https://arxiv.org/abs/1811.05890
- Chaumette & Hutchinson, Visual servo control I, IEEE RAM 2006 (paper's ref. [23])
- Siciliano et al., Robotics: Modelling, Planning and Control, 2010 (paper's ref. [22])
