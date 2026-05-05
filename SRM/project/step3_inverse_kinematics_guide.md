# Step 3: Inverse Kinematics — Beginner Guide

## What is Inverse Kinematics?

**Direct Kinematics (Step 2)** answered: *"If I move the joints to these angles, where does the hand end up?"*

**Inverse Kinematics (Step 3)** answers the opposite: *"If I want the hand to be **here**, what angles should the joints be?"*

This is much harder! There can be many different joint combinations that reach the same point.

---

## The Redundancy Problem

The KUKA LBR MED has **7 joints**, but 3D space only has **6 degrees of freedom** (x, y, z position + 3 rotation angles). This means the robot has **1 extra degree of freedom** — it is **redundant**.

Think of it like this: you can touch the tip of your nose with your elbow pointing left, right, up, or down. Same destination, different arm configurations.

We resolve this by introducing the **arm angle ψ (psi)** — a single number that decides which "pose" the arm takes. Setting `psi = 0` gives one valid solution.

---

## The Big Idea: Kinematic Decoupling

Instead of solving all 7 joints at once (very hard), we **split the problem in two**:

```
                 POSITION problem          ORIENTATION problem
                 ─────────────────         ───────────────────
Joints used:     q1, q2, q3, q4           q5, q6, q7
Controls:        Where is the hand?        Which way does it point?
```

The trick that makes this work is called the **wrist centre**.

---

## Part A: Position — Finding q1, q2, q3, q4

### Step A1 — Find the Wrist Centre

The end-effector (hand) is d7 = 0.126 m beyond the wrist joint.  
So the wrist centre is:

```
p_wrist = p_desired  −  d7 × (z-axis of the desired orientation)
```

> **Why?**  
> Joints 5, 6, 7 can only rotate — they cannot translate the wrist centre.  
> So we find the wrist centre first, and use it to solve q1–q4 independently.

---

### Step A2 — How far is the shoulder from the wrist?

The shoulder is fixed at height d1 = 0.340 m above the ground: `p_shoulder = [0, 0, 0.340]`.

```
v  = p_wrist − p_shoulder   (a 3D vector)
r  = length of v             (straight-line distance)
```

---

### Step A3 — Solve q4 (Elbow Angle) — Law of Cosines

The shoulder, elbow, and wrist form a **triangle**:

```
        shoulder
        /      \
    d3 /        \ d5
      /    q4    \
   elbow ────── wrist
           r
```

Using the Law of Cosines:

```
cos(q4) = (r² − d3² − d5²) / (2 × d3 × d5)
q4 = acos( cos(q4) )
```

> **Singularity!** When `r = d3 + d5`, the arm is fully stretched. `cos(q4) = 1`, `q4 = 0`.  
> The Jacobian loses rank here — the robot cannot push further out.  
> When `r = |d3 − d5|`, the arm is fully folded. Same problem in the other direction.

---

### Step A4 — Place the Elbow (using Arm Angle ψ)

The elbow can sit anywhere on a circle around the shoulder-to-wrist axis (this is the redundancy!). The arm angle `ψ` picks a point on that circle:

```
n_se = cos(α_shoulder) × n_sw + sin(α_shoulder) × (cos(ψ) × t_ref + sin(ψ) × b_ref)
p_elbow = p_shoulder + d3 × n_se
```

Where `t_ref` and `b_ref` are two perpendicular reference axes, and `α_shoulder` comes from the law of cosines at the shoulder corner.

---

### Step A5 — Solve q1 and q2 (Shoulder Angles)

The DH model tells us the direction from shoulder to elbow is:

```
z2_direction = [ cos(q1)·sin(q2),  sin(q1)·sin(q2),  cos(q2) ]
```

We just found `p_elbow`, so we know this direction. We can invert it:

```
q1 = atan2( v_se_y,  v_se_x )         ← horizontal angle (like a compass)
q2 = atan2( sqrt(vx²+vy²),  v_se_z )  ← vertical tilt angle
```

> **Singularity!** When the elbow is directly above/below the shoulder (`vx = vy = 0`),  
> `atan2(0, 0)` is undefined. The Jacobian rank drops here too.

---

### Step A6 — Solve q3 (Arm Roll)

`q3` rotates the arm around the shoulder-to-elbow axis (like rolling your forearm).  
We compute it by looking at the wrist centre direction **from the elbow's reference frame**:

```
v_ew_in_frame2 = R12ᵀ × (p_wrist − p_elbow)
q3 = atan2( v_ew_y_local,  v_ew_x_local )
```

---

## Part B: Orientation — Finding q5, q6, q7

### Step B1 — What's left to rotate?

Now that q1–q4 are known, we can compute the rotation matrix `R_04`  
(the orientation of the arm up to joint 4).

The wrist joints (5, 6, 7) must make up the **remaining rotation**:

```
R_47 = R_04ᵀ × R_desired
```

---

### Step B2 — Decompose R_47 into q5, q6, q7 (ZYZ Euler angles)

The wrist acts like a ball-and-socket joint. Its three axes follow a **Z-Y-Z** rotation pattern:

```
q6 = atan2( sqrt(R47[1,3]² + R47[2,3]²),  R47[3,3] )
q5 = atan2( R47[2,3],  R47[1,3] )
q7 = atan2( R47[3,2], -R47[3,1] )
```

> **Singularity!** When `q6 = 0` or `q6 = π`, the wrist is in a singular configuration.  
> q5 and q7 become co-linear (they spin around the same axis), so we lose one degree of freedom.  
> The code handles this by setting q5 = 0 and putting everything into q7.

---

## Summary of Singularities

| Name | When | Why it's a problem |
|---|---|---|
| Elbow singularity | `r ≈ d3+d5` or `r ≈ \|d3−d5\|` | Arm fully stretched or folded |
| Shoulder singularity | Elbow directly above/below shoulder | `atan2(0,0)` is undefined |
| Wrist singularity | `q6 ≈ 0` or `q6 ≈ π` | q5 and q7 become co-linear |

At each singularity, the **rank of the Jacobian drops** below 6 (we verify this in Step 4).

---

## How to Run the Code

```matlab
% In the MATLAB Command Window:
cd('c:\Users\shueb\OneDrive\Documentos\SRM\SRM-2526\SRM\project')

% Test a specific pose:
R_goal = eye(3);            % identity = end-effector pointing straight up
p_goal = [0; 0; 1.266];    % home position
psi    = 0;                 % arm angle (try changing this!)

q = inverse_kinematics(R_goal, p_goal, psi)

% Validate all tests automatically:
run('validate_inverse_kinematics.m')
```

---

## Files Created

| File | Purpose |
|---|---|
| `inverse_kinematics.m` | The IK function — input: pose, output: 7 joint angles |
| `validate_inverse_kinematics.m` | Runs FK → IK → FK and checks the round-trip error |

A **position error < 1 mm** and an **orientation error < 0.01** means the IK is working correctly.
