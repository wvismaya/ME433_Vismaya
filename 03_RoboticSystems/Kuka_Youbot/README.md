#########################################################################
Assgn 5
#########################################################################


a) Configuration presented is right.

#########################################################################

b)
EXPLAINATION:
-------------
Here, the inherent sigularitiess due to the configuration of the arm will result in 
velocity twists that are unattainable.
Using the mobile base can only help overcome position sigularities as it adds 3 more DOF.
It however does NOT contribute to the EE frame velocity and thus
not of use in overcoming uattainable twists as they are only contributed to by the robot arm angle joints.

#########################################################################

c)	prob13partc.py
	prob13PartC.png: Screenshot to simulate in VREP and show that Endeffector reaches desired configuration.
						Please note EE frame x, y, z values in top right corner.
	outputPartC.csv [To run in VREP]
	[ VREP scene used is YouBot_csv.ttt ]

#########################################################################
Please increase world frame x and y joints limits to 8 each,
to see proper simulation of robot for each file below
#########################################################################

d)	prob13partd.py
	prob13PartD_sim1.avi: Simulation of robot to ~ position (1, 3, 0.5) [ Visible in EE Frame ]
	prob13PartDsim1.png:	Screenshot of controller with Kp = 0.9 and Ki = 0.1.
							Overshoot is clearly visible and reaches steady-state towards the very end
							Less positional error observed
	outputPartD.csv [To run in VREP]

#########################################################################
The output csv file when Kp-Ki are changed remains the same
as in above question
#########################################################################

e) Various Kp-Ki value pairs used for this part of exercise were:
All had setting time greater than or equal to 1 - 2 seconds.

----------------------------------------------------
Commented code line presented in prob13partd.py 
for testing changes in values
----------------------------------------------------

Each plot and corresponding videos are named as: 
Controller plot: prob13PartDsim(i).png
Controller VREp: prob13PartD_sim(i).avi

SIMULATION - 1:
Kp = 0.9
Ki = 0.1

Comment: 
	WORKS THE BEST
	Negative error observed in z-axis.
	X and Y axis face a marginal overshoot

SIMULATION - 2:
Kp = 0
Ki = 0

Comment:
	Pertubations seen towards settling of output
	The controller essentially doesn't function and raw values are passed on

SIMULATION - 3:
Kp = 18
Ki = 35

Comment:
	MOST POOR PERFORMANCE
	Chattering of values at the end.
	High value of Ki gives an extremely large rise time and settling time

SIMULATION - 4:
Kp = 3
Ki = 1

Comment:
	Gives a reasonable steady state response but the offset is quite less too.
	However there is a visible angular offset

#########################################################################