# Control system for tensile testing device using low-cost hardware and open-source software
## 1. Protocols
Four measuring protocols were implemented, each serving a typical task. Every protocol is run from the command line and can be immediately terminated by pressing “ctrl-c”.

Protocol **“ADC display”** periodically (once per second) outputs the time (in µs), the raw ADC voltage reading (in V) and computed values (typically in N) from the required ADC channels to the screen. No arguments are required. This serves in checking for present signals, zero referencing the load cell amplifier and verifying the correct conversion from Volts to Newtons (as different load cells require different conversion parameters).

Protocol **“Move”** moves the motor. Speed (in mm/s) and distance (in mm) are the required arguments. Time (in µs) and position (in µm) are periodically displayed on the screen once per second. This protocol serves for moving the device to the required position.

Protocol **“Preconditioning”** moves the motor in an oscillating way. Frequency (in Hz), amplitude (in mm) and number of cycles are the required arguments. Time (in µs), position (in µm), load cell force (in N) and the motor operating regime (requested speed in mm/s and requested distance in mm) are periodically displayed on screen once per second. This protocol serves for dynamic loading of measuring sample or for the required preconditioning of the soft tissue as described by the authors of [66].

Protocol **“Test (Preload)”** moves the motor with a specified speed until either a specified force or distance is reached. Speed (in mm/s), distance (in mm) and maximum force (in N) are the required arguments. Time (in µs), position (in µm) and load cell strain (in N) are periodically displayed on screen once per second. This protocol serves for either preloading a measuring sample or conducting a shear test.

## 2. Data saving and processing
Every protocol can be run with an additional argument – file name. If this argument is present, the acquired data is saved to a file after the protocol execution is completed or terminated. Three row-based tab-separated files are created with file name suffixes “.encoder.txt”, “.adc.txt” and “.motor.txt”.
* “.encoder.txt” contains time (in µs) and position (in µm) samples in each line.
* “.adc.txt” contains time (in µs), load cell raw voltage and force (in V and in N) and aux channel (in V).
* “.motor.txt” contains time (in µs) and a brief description of the requested motor parameters.

Because the data acquisition system runs on the RPi’s Operating System (OS), the sample times are not equally spaced in time. In this configuration, the samples are usually approximately 250 µs apart, but the test show that the actual interval can range from 200-300 µs. The samples from ADC, encoder and motor are also not sampled at the same time. To remedy this design limitation, a data processing “Equalize” protocol was implemented.

The **“Equalize”** protocol, run with a time step (in µs) and file name arguments, takes the data from the three row-based tab-separated files and produces a new row-based tab-separated file suffixed by “.eq.txt” containing equally spaced time series data from all three files. This works by taking the latest available sample before each time step while discarding all the other samples in between [81]. In the future, instead of discarding the samples, calculating a moving average could be implemented. A time step of 1000 µs proved to be a reliable choice producing stable results.
