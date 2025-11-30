---
title: "Handling Non-Linear Constraints in Indirect Optimal Control"
layout: default
toc: true
permalink: /notes/nonlinear-constraints/
---

# Handling Non-Linear Constraints in Indirect (Pontryagin) Optimal Control
## For Launch Vehicle (LV) / Re-Entry Trajectory Optimization

This section covers the methods for dealing with non-linear constraints in indirect optimal control problems, specifically for launch/re-entry vehicle (LV) trajectory optimization.

## Table of Contents
* [Introduction](#introduction)
* [Pontryagin’s Principle](#pontryagins-principle)
* [Non-Linear Constraints Handling](#non-linear-constraints-handling)
* [Examples](#examples)
* [References](#references)

---

## Introduction
Launch/re-entry vehicle trajectory problems often involve **non-linear dynamics** and **state/control constraints**. Handling these constraints effectively is key for **optimal trajectory computation**.

## Pontryagin’s Principle
Pontryagin’s Maximum Principle provides the **necessary conditions** for optimality:
\[
\dot{x} = \frac{\partial H}{\partial \lambda}, \qquad
\dot{\lambda} = -\,\frac{\partial H}{\partial x}, \qquad
H = \lambda^{T} f(x, u)
\]

## Non-Linear Constraints Handling
- **Path constraints:** $g(x,u) \le 0$
- **Control bounds:** $u_{\min} \le u \le u_{\max}$
- Methods to incorporate:
  - **Indirect shooting with Lagrange multipliers**
  - **Penalty functions**
  - **Sequential continuation / homotopy methods**

## Examples
1. **Re-entry with heat-rate constraint**
2. **LV ascent with dynamic pressure limit**
3. **Multi-stage staging constraints**

## References
1. Betts, *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*
2. Bryson & Ho, *Applied Optimal Control*
