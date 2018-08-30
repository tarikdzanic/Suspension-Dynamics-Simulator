###################
SUSPENSION DYNAMICS SIMULATOR
Author: Tarik Dzanic
Princeton Racing Electric

Using input suspension points and parameters, calculates the suspension geometry changes during wheel movement and simulates the yaw response to a step steering input. 
###################

File descriptions:
	suspension_points.csv	- Input to the codes. XYZ points of all static suspension points and some physical parameters.
	kinematics.py		- Based on the static suspension points, calculates the camber curves and roll center curves. Outputs FRONT_curves.csv and REAR_curves.csv
	transient_response.py	- Calculates the transient yaw response to a step steering input. Calls kinematics.py from within the script. Outputs the yaw gradient, roll, and tire normal forces over time to output.csv.
	postprocess.m		- MATLAB script for plotting the output of transient_response.py


Directions for use:
1) Fill out all the parameters in suspension_points.csv
2) Run transient_response.py ("python transient_response.py" in console with python3)
3) Run postporcess.m in MATLAB

Alternatively, you can run kinematics.py by itself just for the kinematic curves.

Tire data fitting is explained in the README in the TireFit folder.

