---
title: "Handling Non-Linear Constraints in Indirect Optimal Control"
layout: default
toc: true
permalink: /notes/nonlinear-constraints/
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

Indirect (Pontryagin) methods **can** enforce these constraints, but only when the full Pontryagin + KKT structure is used.

---

# 1. Pure Control Constraints

Example:  
$$
u \in [u_{\min}, u_{\max}]
$$

**Steps:**

1. Compute the interior optimal control from  
   $$
   \frac{\partial H}{\partial u} = 0.
   $$

2. If the control violates bounds, saturate:  
   $$
   u^* = \mathrm{sat}(u_{\text{opt}})
   $$
   yielding **bang–bang** or **bang–singular–bang** structure.

---

# 2. Path Constraints

General form:
$$
g(x(t), u(t)) \le 0.
$$

## Introduce a multiplier
$$
H = L + \lambda^\top f + \mu(t)\, g(x,u)
$$

## Complementarity conditions
$$
\mu(t) \ge 0, \quad g(x,u) \le 0, \quad \mu(t)\,g(x,u) = 0.
$$

This creates two types of arcs:

---

## **Unconstrained Arc**
$$
g(x,u) < 0, \quad \mu = 0.
$$

Control satisfies:
$$
\frac{\partial H}{\partial u} = 0.
$$

---

## **Active (Constrained) Arc**
$$
g(x,u) = 0, \quad \mu > 0.
$$

Then optimality becomes:
$$
\frac{\partial H}{\partial u}
= \frac{\partial L}{\partial u}
+ \lambda^\top \frac{\partial f}{\partial u}
+ \mu\,\frac{\partial g}{\partial u}
= 0.
$$

This is a **nonlinear algebraic equation** for $u$ and $\lambda$.

---

## Constraint derivative condition

To remain on the constraint manifold:
$$
\frac{d}{dt}g(x,u) = 0.
$$

Expanding:
$$
\nabla_x g \cdot f(x,u) + \nabla_u g \cdot \dot{u} = 0.
$$

For most re-entry constraints (heating rate, dynamic pressure, g-load),  
$g$ does **not** depend on $u$:

$$
\frac{d}{dt}g(x,u) = \nabla_x g \cdot f(x,u) = 0.
$$

This becomes an **algebraic control constraint** used to solve for $u$ on the active arc.

---

# 3. State Inequality Constraints

Example:
$$
h_{\min} \le h(t) \le h_{\max}
$$

Convert to path constraints:
$$
g_1 = h - h_{\max} \le 0,
\qquad
g_2 = h_{\min} - h \le 0.
$$

---

# 4. Multiple-Arc Structure

A realistic re-entry trajectory may require segments:

- Unconstrained  
- Heating-limited ($g = 0$)  
- Dynamic-pressure-limited ($g = 0$)  
- Unconstrained to final conditions  

At switching times:

- State is continuous  
- Costates are continuous  
- Hamiltonian is continuous (for free-final-time problems)

Switching times are **unknown variables** in the TPBVP.

---

# 5. Practical Implementation Steps

1. Derive Hamiltonian, costate equations, and unconstrained control law.  
2. Identify potential active constraints (heating, $q$-bar, g-load, AoA).  
3. For each constraint:  
   - compute $\partial g / \partial x$  
   - derive constraint-arc algebraic control equation  
4. Split trajectory into arcs  
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

Re-entry constraints are **strongly nonlinear** and often activate simultaneously  
(heating, $q$-bar, g-load all peak at similar times).

Thus the indirect TPBVP is:

- stiff  
- extremely sensitive to initial guesses  
- numerically fragile  
- often requires continuation  

This is why modern re-entry GNC commonly uses **direct methods**, while indirect methods are used for **structural insight** and **verification**.

---

# 7. Summary Table

| Constraint Type | Representation | Indirect Method Handling |
|-----------------|----------------|---------------------------|
| Control bounds | $u \in [u_{\min},u_{\max}]$ | Saturation / bang–bang |
| Path constraint | $g(x,u) \le 0$ | Add $\mu$, enforce $\mu g = 0$ |
| Active arc | $g=0,\, \mu>0$ | Solve $\partial H / \partial u = 0$ |
| Feasibility | derivative constraint | $\frac{d}{dt}g = 0$ |
| Switching | arc transitions | continuity of $x,\lambda,H$ |

---

# 8. Key References

- **Bryson & Ho**, *Applied Optimal Control*  
- **Betts**, *Practical Methods for Optimal Control*  
- Re-entry trajectory papers using Pontryagin + path constraints  


