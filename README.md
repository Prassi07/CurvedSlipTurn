# CurvedSlipTurn
 Source Codes for Curved Path Slip Turn using Slip Phenomenon
 Created as part of Paper titled,  "Generating Curved Path Walking Gaits for Biped Robots with Deficient Degrees of Freedom"
 Authors: Prasanna Venkatesan K S, Prajwal Rajendra Mahendrakar. This paper has been accpeted at the 2021 IEEE/SICE International Symposium for System Integration, 11th-14th Jan 2021, Iwaki, Japan. 


The source code mainly consists of five files and a sample result file and are described below.
......................................................................
(a)GenerateCustomTurn.m
Function which generates Angles of the Actuators for robots to perform Curved Turn using Slip Phenomenon
Inputs	:   Value of Radius of Circle in mm.
	    Value of Cycle Distance L in mm.
	    Direction of Turn - 1 for Counter ClockWise, 
            	              - anything else for clockwise
Outputs :   Doesnot return any value.
	    Creates a file "Smooth_Turn.txt" consiting all the angles
            Each line has 10 angles, first 5 are for right leg and other
            for left leg
            Each 5 angles are for - Hip Roll, Hip Pitch, knee Pitch, Ankle
            Roll and Ankle Pitch
.......................................................................
(b)PredictTurnParameters.m
Function to Calculate Slip Distance and Step Length for Curved Path Walk Parameters
Inputs	:   Value of Cycle Distance L in mm
            Value of Radius of Circle in mm.
Outputs	:   Returns an array of 2 elements,
            First Element is Step Length L1 in mm
            Second Element is Slip Distance DelY in mm
........................................................................
(c)SetRobotParameters.m
Function which Sets the Robot Dimensions and Parameters required for Trajectory
Generation of Curved Path Turn.
Inputs:	    Robot Link Lengths l1,l2,l3,l4,l5 in mm
            Pelvis Distance of the robot in mm
            The CoM plane for 3-D LIP model, Zo in mm
            The Step Height H of the Robot in mm
            Robot's foot dimensions lx and ly, in mm
Outputs	:   Return none, but sets the value of global Variables for Robot Parameters with the
            Data Received
.........................................................................
(d)SetWalkParameters.m
Function which Sets the Walking Parameters required for Trajectory
Generation of Curved Path Turn.
Inputs	:   Value of TimePeriod T1, T2, T3 to be set.
            Value of the InterStepDistance of the Slip phase
            Value of Xinit during the Slip Turn.
            [Xinit - Distance between Centers of both foot in Lateral Plane]
Outputs	:   Return none, but sets the value of global Variables with the
            Data Received
..........................................................................
(e)SolveIK.m
Function to Solve inverse kinematics of 5-dof leg of humanoind robot
Inputs	:   Foot Co-ordinates of the robot leg - xf,yf,zf
            Hip Co-ordinates of the robot leg - x1,y1,z1
            Character l or r, for left and right leg, only for displaying
            purposes
Output	:   Returns 5 angles - hip roll, hip pitch, knee pitch, ankle pitch, ankle roll
..............................................................................
(f)Smooth_Turn.txt
A sample file containing actuator angles for curved path walk.
Each line of this file contains tab separted integer values. The first five values are the actuator angles for the right leg in the order Hip Roll, Hip Pitch, Knee Pitch, Ankle Pitch, Ankle Roll. The next five values are the actuator angles for the left leg in the exact same order as above.
..............................................................................
To run the Source Code

Step 1 : Open the file GenerateCustomTurn.m
step 2 : Set the robot parameters, Link Lengths (l1,l2,l3,l4,l5), PelvisDistance, CoM plane height, Step Height H and FootDimensions(lx,ly) in the function call "SetRobotParameters" at line 27. (if using a different robot)
step 3 : Set the walk parameters periods of walk phase(T1), slip phase(T2) and restoration phase(T3),InterStepDistance and Xint in the function call "SetWalkParameters" at line 29. 
step 4 : Invoke function GenerateCustomTurn passing arguments radius, cycleStepLength and turnDirection of your choice
step 5 : This generates a Smooth_Turn.txt file which contains the actuator angles of the robot's right and left leg. Each line of this file has 10 angles, first 5 for right leg and latter for the left leg. The conventions are shown above.
