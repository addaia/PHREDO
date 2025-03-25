# Proactive Handling for Rapid Emergency Deceleration and Optimisation - PHREDO

This repository hosts the code and supporting materials for the research project **"Proactive Handling for Rapid Emergency
Deceleration and Optimisation"**. The study evaluates various motion tracking methods for a Pololu 3pi+ robot operating under different surface grip conditions.

## Abstract

This research evaluates the accuracy of different motion tracking methods for a Pololu 3pi+ robot moving through surfaces with different grip conditions. The Baseline and sABS methods rely solely on encoder data, while PSC and PAPIDI implement IMU feedback to detect and counteract slip. According to experimental results, encoder-based methods are unable to detect slippage. In contrast, despite their sensitivity to noise, IMU-based methods provide a deeper understanding of dynamic motion changes. Baseline provides a benchmark (3.13%). sABS and PSC achieved comparable results (1.28% and 1.50% respectively), while demonstrating consistency across trials. Whereas, PAPIDI suffered from variability, achieving the best single value in a trial, while having a low mean error (1.44%). Ultimately, combining encoder precision with IMU responsiveness enhances the robot’s positional accuracy in low traction environments.

## Overview

The project compares four distinct approaches for motion tracking:
- **Baseline**: Relies solely on encoder data.
- **Simplified Anti-Braking System (sABS)**: Introduces a modulation mechanism to reduce slippage.
- **Post-Slip Correction (PSC)**: Utilizes IMU feedback after movement to detect and correct for slippage.
- **Pro-Activate PID Implementation (PAPIDI)**: Implements a PID controller that uses real-time IMU data to proactively counteract slip.

Experimental results show that while encoder data can offer a basic distance estimate, integrating IMU data significantly improves accuracy under low-traction conditions.

## Repository Structure

- **/Code**  
  Contains all the source files and sketches required to run the experiments:
  - **.h Files**: Header files for modules such as Kinematics, Encoders, etc.
  - **.ino Files**: Arduino sketches implementing the four methods (Baseline, sABS, PSC, PAPIDI) along with additional test routines.
  
- **/notebooks**  
  - **plotter.ypnyp**: Notebook that contains all the plots and visualisations from the research report. 

## Usage

### Arduino Code
1. Open the `.ino` files using the Arduino IDE.
2. Ensure that the accompanying `.h` files (for Kinematics, Encoders, etc.) are located in the appropriate include paths.
3. Upload the sketches to your Pololu 3pi+ robot to execute experiments using the different motion tracking strategies.

### Notebook
1. Open the `plotter.ypnyp` notebook in your preferred Jupyter environment.
2. Run the notebook cells to view all plots and graphical analyses that summarize the experimental outcomes.

## Experimentation and Results

The project investigates how well encoder-based methods perform against IMU-enhanced approaches:
- **Encoder-only methods** (Baseline and sABS) provide a general distance estimate.
- **IMU-based methods** (PSC and PAPIDI) offer improved insight by detecting and compensating for slippage.

The experiments confirm that although encoders deliver high-precision tracking under normal conditions, their inability to detect slip limits their performance on low-friction surfaces. By integrating IMU data, the system can dynamically adjust and correct for errors introduced by slippage.


