# Step 1: Denavit-Hartenberg Parameters — Beginner Guide

## What is this step about?

Before we can do any calculations, we need to give MATLAB a mathematical description of the robot — like a "blueprint" that tells it how each joint is connected to the next. This is what the **Denavit-Hartenberg (D-H) convention** does.

---

## What is the D-H Convention?

The D-H convention is a standard way to describe any robot arm using a simple table of numbers. Each **row** of the table describes **one joint** of the robot.

The KUKA LBR MED has **7 joints**, so the table has 7 rows.

Each row has **5 columns**:

| Column | Name | What it means (in plain English) |
|--------|------|----------------------------------|
| `d` | Link offset | How far to slide along the **previous joint's axis** before reaching the next joint |
| `v` (θ) | Joint angle | The **rotation angle** of this joint (this is the variable that changes when the robot moves) |
| `a` | Link length | How far to slide **sideways** (perpendicular to the axis) to the next joint |
| `alpha` (α) | Link twist | How much to **tilt** the next joint's axis relative to the current one |
| `offset` | Home offset | A fixed extra angle added when the joint is at its "zero" position |

> **Think of it like giving directions:**  
> "Go forward 340 mm (d), then rotate the next road by 90°(α), then your joint rotates by θ."

---

## The KUKA LBR MED Robot Structure

The robot has **7 rotational joints**, connected in a chain from base to end-effector.  
All joints are **rotational** (they spin, they don't slide), so the joint variable always goes in the `v` column.

The physical dimensions of the robot (in metres) are:

```
Base  →  d1=0.340m  →  Shoulder  →  d3=0.400m  →  Elbow  →  d5=0.400m  →  Wrist  →  d7=0.126m  →  End-Effector
```

The key feature of this robot is that each joint axis is **perpendicular** to the previous one. This is captured by the `alpha` values alternating between `-π/2` and `+π/2` (i.e., -90° and +90°).

---

## The D-H Table for KUKA LBR MED

| Joint | `d` (offset) | `v` (angle) | `a` (length) | `alpha` (twist) | `offset` |
|-------|-------------|-------------|--------------|-----------------|----------|
| 1 | 0.340 m | **q1** | 0 | −π/2 | 0 |
| 2 | 0 | **q2** | 0 | +π/2 | 0 |
| 3 | 0.400 m | **q3** | 0 | −π/2 | 0 |
| 4 | 0 | **q4** | 0 | +π/2 | 0 |
| 5 | 0.400 m | **q5** | 0 | −π/2 | 0 |
| 6 | 0 | **q6** | 0 | +π/2 | 0 |
| 7 | 0.126 m | **q7** | 0 | 0 | 0 |

**Notice the pattern:**
- `a = 0` for every row — this robot has no sideways offsets, only forward offsets (`d`).
- `alpha` alternates: −90°, +90°, −90°, +90°, … — this is what makes adjacent joints perpendicular.
- Only joints 1, 3, 5, 7 have a non-zero `d` value (the physical link lengths).

---

## What does the Toolbox expect?

The `DHTransf.m` function in the toolbox takes one row of this table and builds a **4×4 transformation matrix** from it. Specifically, it builds:

```
A = [ R  p ]
    [ 0  1 ]
```

Where `R` is a 3×3 rotation and `p` is a 3×1 position offset. This matrix describes how to go from one joint's frame to the next.

The `DKin.m` function then **multiplies all 7 of these matrices together**:

```
T = A1 × A2 × A3 × A4 × A5 × A6 × A7
```

This gives the full transformation from the robot base to the end-effector (used in Step 2).

---

## How to Run the Code

```matlab
% In the MATLAB Command Window:
cd('c:\Users\shueb\OneDrive\Documentos\SRM\SRM-2526\SRM\project')

% Load the D-H table
Robot = KukaLBR()
```

**What you should see:** A 7×5 symbolic matrix. Each row is one joint. The `q1` through `q7` entries are symbolic variables — MATLAB treats them as unknowns until you substitute real numbers.

---

## Files for This Step

| File | Purpose |
|------|---------|
| `KukaLBR.m` | Defines the D-H table for the KUKA LBR MED. This is the starting point for all other steps. |
| `toolbox/DHTransf.m` | (Provided) Converts one row of the D-H table into a 4×4 matrix. |
| `toolbox/DKin.m` | (Provided) Multiplies all D-H matrices to get the full kinematics. |

---

## Key Takeaway

The D-H table is the **single source of truth** for the robot's geometry. If this table is wrong, every subsequent step will give wrong results. The validation in Step 2 is what confirms this table is correct.
