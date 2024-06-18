# PID Tuning by Optimization

This repository contains Python scripts to optimize PID controller gains (proportional, integral, and derivative) using optimization techniques. The optimization aims to minimize the step response characteristics: overshoot, rise time, and settling time.

## Introduction

This repository provides two approaches to optimize PID controller gains:
1. **Gradient Descent**
2. **Quadratic Programming**

## Explanation
Both scripts perform the optimization of PID controller gains for a given state-space model. The gradient descent script uses a simple finite difference approach to estimate the gradient, while the quadratic programming script uses the minimize function from scipy.optimize with the SLSQP method to find the optimal PID gains.

**Gradient Descent Script:**
Defines the PID controller and closed-loop state-space model.
Computes the cost function as a combination of overshoot, rise time, and settling time.
Updates PID gains iteratively using the gradient descent algorithm.

**Quadratic Programming Script:**
Similar to the gradient descent script but uses the minimize function for optimization.
Defines bounds for the PID gains and optimizes the cost function using quadratic programming.

## Requirements

- Python 3.x
- NumPy
- SciPy

