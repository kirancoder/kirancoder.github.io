---
title: "Handling Non-Linear Constraints in Indirect Optimal Control"
layout: default
toc: true
permalink: /notes/Indirect_non_linear_constrains/
---


# Handling Non-Linear Constraints in Indirect (Pontryagin) Optimal Control  
## For Launch Vehicle (LV) / Re-Entry Trajectory Optimization

Launch and re-entry trajectory problems naturally include **non-linear inequality constraints**, for example:

- **Path constraints:**  
  - Heating rate: $\dot{q}(x,u) \le \dot{q}_{\max}$  
  - Dynamic pressure: $q(x,u) \le q_{\max}$  
  - g-load: $n(x,u) \le n_{\max}$  
  - Angle-of-attack (AoA) limits  
  - Flight corridor boundaries  

- **State constraints:**  
  - Altitude limits  
  - Velocity bounds  

- **Control bounds:**  
  - Bank angle limits  
  - Thrust limits  
  - AoA bounds  

Indirect (Pontryagin) methods **can** handle these constraints, but only when the full Pontryagin + KKT conditions are applied.

---

# 1. Pure Control Constraints

Example:

$$
u \in [u_{\min}, u_{\max}]
$$

**Steps:**

1. Compute interior optimal control:

$$
\frac{\partial H}{\partial u} = 0
$$

2. If it violates the bounds, saturate:

$$
u^* = \mathrm{sat}(u_{\text{opt}})
$$

This typically produces **bang–bang** or **bang–singular–bang** behavior.

---

# 2. Path Constraints

General form:

$$
g(x(t), u(t)) \le 0
$$

---

## Introduce a multiplier

$$
H = L + \lambda^\top f + \mu(t)\, g(x,u)
$$

---

## Complementarity conditions

$$
\mu(t) \ge 0
$$

$$
g(x,u) \le 0
$$

$$
\mu(t)\, g(x,u) = 0
$$

These produce two types of arcs:

---

## **Unconstrained Arc**

$$
g(x,u) < 0, \quad \mu = 0
$$

Control satisfies:

$$
\frac{\partial H}{\partial u} = 0
$$

---

## **Active (Constrained) Arc**

$$
g(x,u) = 0, \quad \mu > 0
$$

Optimality condition:

$$
\frac{\partial H}{\partial u}
= \frac{\partial L}{\partial u}
+ \lambda^\top \frac{\partial f}{\partial u}
+ \mu\,\frac{\partial g}{\partial u}
= 0
$$

This becomes a **nonlinear algebraic equation** in $u$ and $\lambda$.

---

## Constraint derivative condition

To stay on the constraint surface:

$$
\frac{d}{dt}g(x,u) = 0
$$

Expand:

$$
\nabla_x g \cdot f(x,u)
+ \nabla_u g \cdot \dot{u}
= 0
$$

For most re-entry constraints (heating rate, dynamic pressure, g-load),  
$g$ does **not** depend on control $u$:

$$
\frac{d}{dt} g(x,u)
= \nabla_x g \cdot f(x,u)
= 0
$$

This equation is used to solve for the control on constrained arcs.

---

# 3. State Inequality Constraints

Example:

$$
h_{\min} \le h(t) \le h_{\max}
$$

Convert them into path constraints:

$$
g_1 = h - h_{\max} \le 0
$$

$$
g_2 = h_{\min} - h \le 0
$$

Then treat each via the path-constraint approach.

---

# 4. Multiple-Arc Structure

Typical re-entry trajectory structure:

- Unconstrained arc  
- Heating-limited arc ($g = 0$)  
- Dynamic-pressure-limited arc ($g = 0$)  
- Final unconstrained arc  

At each switching point:

- State is continuous  
- Costate is continuous  
- Hamiltonian is continuous (if final time is free)

Switching times become **unknowns** in the TPBVP.

---

# 5. Practical Implementation Steps

1. Derive Hamiltonian and costate equations.  
2. Identify active constraints (heating, $q$-bar, g-load, AoA).  
3. For each constraint, compute:  
   - $\partial g / \partial x$  
   - The constrained-arc algebraic control equation  
4. Split trajectory into arcs.  
5. Solve using:  
   - Single shooting  
   - Double shooting  
   - Multiple shooting  
6. Newton-solve for:  
   - Initial costates  
   - Switching times  
   - Multipliers  

---

# 6. Why This Is Hard in Practice

Re-entry constraints are **highly nonlinear** and often activate together  
(heating, dynamic pressure, g-load).

This makes the indirect TPBVP:

- Stiff  
- Sensitive to initial guesses  
- Hard to converge  
- Dependent on continuation strategies  

Hence modern re-entry guidance usually uses **direct methods** (pseudospectral),  
while indirect methods are used for **structure analysis** and **verification**.

---

# 7. Summary Table

| Constraint Type | Representation | Indirect Method Handling |
|-----------------|----------------|---------------------------|
| Control bounds | $u \in [u_{\min},u_{\max}]$ | Saturation / bang–bang |
| Path constraint | $g(x,u) \le 0$ | Add multiplier, apply $\mu g=0$ |
| Active arc | $g=0$, $\mu>0$ | Solve $\partial H / \partial u = 0$ |
| Feasibility | derivative constraint | $\frac{d}{dt}g = 0$ |
| Switching | arc transitions | continuity of $x$, $\lambda$, $H$ |

---

# 8. Key References

- Bryson & Ho — *Applied Optimal Control*  
- Betts — *Practical Methods for Optimal Control*  
- Re-entry trajectory papers using indirect (Pontryagin) methods  

