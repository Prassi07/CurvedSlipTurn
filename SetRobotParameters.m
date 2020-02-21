% Function which Sets the Robot Dimensions and Parameters required for Trajectory
% Generation of Curved Path Turn.
% Inputs: Robot Link Lengths l1,l2,l3,l4,l5 in mm
%         Pelvis Distance of the robot in mm
%         The CoM plane for 3-D LIP model, Zo in mm
%         The Step Height H of the Robot in mm
%         Robot's foot dimensions lx and ly, in mm
% Outputs: Return none, but sets the value of global Variables for Robot Parameters with the
%          Data Received
% Authors: Prasanna Venkatesan K S and Prajwal Rajendra Mahendrakar

function SetRobotParameters(newl1, newl2, newl3, newl4, newl5, newPelvisDistance, newZo, newH, newlx, newly)
    global l1
    global l2
    global l3
    global l4
    global l5
    global K
    global Zo
    global H
    global lx
    global ly
    l1 = newl1;
    l2 = newl2;
    l3 = newl3;
    l4 = newl4;
    l5 = newl5;
    K = newPelvisDistance/2;
    Zo = newZo;
    H = newH;
    lx = newlx;
    ly = newly;
end