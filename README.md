# PID Tuning by Optimization

This repository contains Python scripts to optimize PID controller gains (proportional, integral, and derivative) using optimization techniques. The optimization aims to minimize the step response characteristics: overshoot, rise time, and settling time.

## Table of Contents

- [Introduction](#introduction)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
  - [Transfer Function](#transfer-function)
    - [Gradient Descent](#gradient-descent)
    - [Quadratic Programming](#quadratic-programming)
  - [State Space Model](#state-space-model)
    - [Gradient Descent](#gradient-descent-1)
    - [Quadratic Programming](#quadratic-programming-1)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This repository provides two approaches to optimize PID controller gains:
1. **Gradient Descent**
2. **Quadratic Programming**

The optimization is applied to:
- Transfer Function models
- State Space models

## Requirements

- Python 3.x
- NumPy
- SciPy

