---
title: "Direct Methods in Trajectory Optimization"
date: 2025-01-10
layout: single
---

## Problem Formulation

The optimal control problem can be written as:

$$
\min_u \int_0^T L(x,u,t)\,dt
$$

subject to:

$$
\dot{x} = f(x,u,t)
$$

## Transcription

In direct methods, the control and state
are discretized and the problem is converted
into a nonlinear programming problem.
