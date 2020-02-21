% Function which Sets the Walking Parameters required for Trajectory
% Generation of Curved Path Turn.
% Inputs: Value of TimePeriod T1, T2, T3 to be set.
%         Value of the InterStepDistance of the Slip phase
%         Value of Xinit during the Slip Turn.
%          [Xinit - Distance between Centers of both foot in Lateral Plane]
% Outputs: Return none, but sets the value of global Variables with the
%          Data Received
% Authors: Prasanna Venkatesan K S
function SetWalkParameters(newT1, newT2, newT3, newInterStepSize, newXinit)
global T1 
global T2 
global T3,
global InterStepSize,
global Xinit

T1 = newT1;
T2 = newT2;
T3 = newT3;
InterStepSize = newInterStepSize;
Xinit = newXinit;
end
