% Function to Calculate Slip Distance and Step Length for Curved Path Walk
% Inputs: Value of Cycle Distance L in mm
%         Value of Radius of Circle in mm.
% Outputs: Returns an array of 2 elements,
%           First Element is Step Length L1 in mm
%           Second Element is Slip Distance DelY in mm
% Authors: Prajwal Rajendra Mahendrakar
function [L1, DelY] = predictTurnParameters(L,R)
    global Xinit
    global lx
    global ly
    theta = (L * 180)/(R * pi); %Calculate theta from length of arc formula
    disp(theta);
    %Calculate Slip Distance using the Slip Prediction Equation
    DelY = (sqrt((((12*Xinit^2) + lx^2 +ly^2 )/12)))*tand(((theta/Xinit))*sqrt((((12*Xinit^2) + lx^2 +ly^2 )/12)));
    %Calculate Step length.
    L1 = L + DelY;
end
