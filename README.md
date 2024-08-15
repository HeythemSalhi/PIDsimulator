# PID Controller Simulation with PyQt5 and Matplotlib

This project implements a **PID (Proportional-Integral-Derivative) controller simulation** using PyQt5 for the graphical user interface (GUI) and Matplotlib for plotting the system response. It allows users to input custom PID parameters, set a desired setpoint, and visualize the system’s response in real-time.

## Features

- Input fields for tuning PID parameters (`Kp`, `Ki`, `Kd`).
- Setpoint adjustment to test the controller's response.
- Real-time system response visualization using Matplotlib.
- Start and stop the simulation dynamically.

## Requirements

- Python 3.x
- `PyQt5` for the GUI
- `matplotlib` for plotting
- `numpy` for numerical calculations

Install the dependencies using:

```bash
pip install pyqt5 matplotlib numpy


# Produce exe from python program

To produce an executable (EXE) from a Python program, you can use a tool like PyInstaller or cx_Freeze. Here's a guide using PyInstaller:

```bash
pyinstaller --onefile --windowed .\prg_name.py

the --onefile flag tells PyInstaller to bundle everything into a single executable.
the --windowed flag, if your script is a GUI application and you want to avoid opening a terminal window.

Locate the Executable:
After the process completes, you’ll find the executable in the dist folder inside your script's directory.
