% Function which generates Angles of Actuators for robots to perform Curved
% Turn using Slip Phenomenon
% Inputs: Value of Radius of Circle in mm.
%         Value of Cycle Distance L in mm
%         Direction of Turn - 1 for Counter ClockWise, 
%                               anything else for clockwise
% Outputs: Doesnot return any value.
%          Creates a file "Smooth_Turn.txt" consiting all the angles
%           Each line has 10 angles, first 5 are for right leg and other
%           for left leg
%           Each 5 angles are for - Hip Roll, Hip Pitch, knee Pitch, Ankle
%           Roll and Ankle Pitch
%  Source Code written based on manuscirpt written for submission of Robotics
%  and Automation Letters titled 
% "Generating Curved Path Walking Gaits for Biped Robots with Deficient
% Degree of Freedom"
% Authors: Prasanna Venkatesan K S and Prajwal Rajendra Mahendrakar
function GenerateCustomTurn(radius, cycleStepLength, turnDirection)
    global T1
    global T2
    global T3
    global InterStepSize
    global Zo
    global H
    global K
    factor =1.2;
    %Set Robot Parameters (l1,l2,l3,l4,l5,PelvisDistance(2K),CoMPlaneHeight, StepHeight, FootDimension(lx), FootDimension(ly))
    %SetRobotParameters(45,65,65,50,17,65,210,20,122,87); %FOR TONY
    SetRobotParameters(1,93,93,1,33.5,90,205,20,104,66); %FOR Simulation Darwin-OP
    %Set Walk Parameters(Walk Phase Period(T1), Slip Phase Period(T2), Restoration Phase Period(T3), InterStepDistance, Xinit)
    SetWalkParameters(1,1,1,5,40);
    %Calculate Turn Parameters using Cycle Length and radius
    [L1 ,DelY] = predictTurnParameters(cycleStepLength,radius);
    fileID = fopen('Smooth_Turn_Sim.txt','w');
    t=0;
    while (t<=T1)
        %Walk Phase, by a Distance of L1
        Const1 = (L1 * sech(sqrt(K) * T1 /2))/4;
        Const2 = (- L1 * sech(sqrt(K)* T1 /2))/4;
        xr1 =(K*(sin(2*pi*t/(2*T1))+1));
        yr1 =(Const1*exp(sqrt(K)*(t/2)))+(Const2*exp(-1*sqrt(K)*(t/2)));
        zr1 =Zo ;
        xrf =K ;
        yrf =L1/4 ;
        zrf =0 ;
        xl1 =(K*(sin(2*pi*t/(T1*2))-1));
        yl1 =((Const1*exp(sqrt(K)*(t/2)))+(Const2*exp(-1*sqrt(K)*(t/2))));
        zl1 =Zo ;
        xlf = - K ;
        ylf =((L1/2)*(sin((2*pi*((t/2)+T1/2)/(T1))+pi/2))) +  L1/4;
        zlf =(H *( 1 + (sin( 4 * pi *( (t/2)+(T1/2)/(T1) ) - pi/2)))/2);
        %Choose legs based on Turning Direction
        if(turnDirection == 1)
             %Turn Counter-Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'l');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'r');
             thetas = [te6 te7 te8 te9 te10 te1 te2 te3 te4 te5];
        else
             %Turn Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'r');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'l');
             thetas = [te1*factor te2 te3 te4 te5*factor te6*factor te7 te8 te9 te10*factor];
        end
        fprintf(fileID,'%d\t',round(thetas));
        fprintf(fileID,'\n');
        t = t+0.1;
    end
    
    %Start of SLIP Phase 
    t = 0;
    N = DelY/InterStepSize;
    while (t<=T2)
    %SLIP by distance of DelY
      if (t == 0)
                xrf = K;
                yrf = L1/4;
                zrf = 0;
                xlf = -K;
                ylf = 3*L1/4;
                zlf = 0;
      else 
                xrf = K;
                yrf = L1/4+(t * DelY/T2);
                zrf = 0;
                xlf = -K;
                ylf =((3/4)*L1)  - ( t * DelY/T2);
                zlf = 0;
      end
      t = t +(T2/N);
        if(turnDirection == 1)
             %Turn Counter-Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'l');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'r');
             thetas = [te6 te7 te8 te9 te10 te1 te2 te3 te4 te5];
        else
             %Turn Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'r');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'l');
             thetas = [te1*factor te2 te3 te4 te5*factor te6*factor te7 te8 te9 te10*factor];
        end
      fprintf(fileID,'%d\t',round(thetas));
      fprintf(fileID,'\n'); 
    end  
    %Start of Restoration Phase in case of DelY <= L1/4
    if(DelY<=L1/4)   
        t = 0;
        L2 =( L1 - (4*DelY));
        while(t<=T3)
            if (t<= (T3/2))
                Const1 = (L2 * sech(sqrt(K) * T3 /2))/4;
                Const2 = (- L2 * sech(sqrt(K)* T3 /2))/4;
                xr1 = (K*(sin(2*pi*(t/2 + (T3/2))/T3)+1));
                yr1 =  Const1*exp(sqrt(K)*((t/2 + (T3/2))-T3/2))+Const2*exp(-1*sqrt(K)*((t/2 + (T3/2))-T3/2))+ L1/2 ;
                zr1 = Zo;
                xrf = K;
                yrf = ((L2/2)*sin(2*pi*(t/2 + (T3/2))/T3 + pi/2))+ (3*L1/4 - DelY);
                zrf = (H)*(1+sin(4*pi*(t/2 + (T3/2))/T3 - pi/2))/2;
                xl1 = (K*(sin(2*pi*(t/2 + (T3/2))/T3)-1));
                yl1 = Const1*exp(sqrt(K)*((t/2 + (T3/2))-T3/2))+Const2*exp(-1*sqrt(K)*((t/2 + (T3/2))-T3/2))+L1/2;
                zl1 =  Zo;
                xlf = -K;
                ylf = (3 * L1 /4) - DelY;
                zlf = 0;  
            else
                Const1 = (L1 * sech(sqrt(K) * T3 /2))/4;
                Const2 = (- L1 * sech(sqrt(K)* T3 /2))/4;
                xr1 = (K*(sin(2*pi*(t/2 + (T3/2))/T3)+1));
                yr1 =  Const1*exp(sqrt(K)*((t/2 + (T3/2))-T3/2))+Const2*exp(-1*sqrt(K)*((t/2 + (T3/2))-T3/2))+ L1/2;
                zr1 = Zo;
                xrf = K;
                yrf = ((L1/2)*sin(2*pi*(t/2 + (T3/2))/T3 + pi/2))+ (3*L1/4 - DelY);
                zrf = (H)*(1+sin(4*pi*(t/2 + (T3/2))/T3 - pi/2))/2;
                xl1 = (K*(sin(2*pi*(t/2 + (T3/2))/T3)-1));
                yl1 = Const1*exp(sqrt(K)*((t/2 + (T3/2))-T3/2))+Const2*exp(-1*sqrt(K)*((t/2 + (T3/2))-T3/2))+ L1/2;
                zl1 =  Zo;
                xlf = -K;
                ylf = (3 * L1 /4) - DelY;
                zlf = 0;
            end
            t = t +0.1;
        if(turnDirection == 1)
             %Turn Counter-Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'l');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'r');
             thetas = [te6 te7 te8 te9 te10 te1 te2 te3 te4 te5];
        else
             %Turn Clockwise
             [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'r');
             [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'l');
             thetas = [te1*factor te2 te3 te4 te5*factor te6*factor te7 te8 te9 te10*factor];
        end
        fprintf(fileID,'%d\t',round(thetas));
        fprintf(fileID,'\n');
        end
    end
    %Start of Restoration Phase in case of DelY > L1/4 and DelY <= L1/2
    if(DelY>L1/4 && DelY<L1/2)
        t = 0;
        L2 = abs( L1 - (4*DelY));
        while(t<=T3)
            if( t <= T3/2)
                Const1 = ( L2 * sech(sqrt(K) * T3 /2))/4;
                Const2 = (- L2 * sech(sqrt(K)* T3 /2))/4;
                xr1 =(K*(sin(2*pi*t/T3)+1));
                yr1 =(Const1*exp(sqrt(K)*t))+(Const2*exp(-1*sqrt(K)*t))+L1/2;
                zr1 = Zo ;
                xrf = K ;
                yrf = L1/4 + DelY ;
                zrf = 0 ;
                xl1 =(K*(sin(2*pi*t/T3)-1));
                yl1 =((Const1*exp(sqrt(K)*t))+(Const2*exp(-1*sqrt(K)*t)) +L1/2);
                zl1 = Zo ;
                xlf = - K ;
                ylf =((L2/2)*(sin((2*pi*(t+T3/2)/T3)+pi/2))) + L1/4 + DelY;
                zlf =(H *( 1 + (sin( 4 * pi *( t+(T3/2)/T3 ) - pi/2)))/2);
            end
            if ((t>T3/2) && (t<= (3*T3/4)))
                Const1 = (L2 * sech(sqrt(K) * T3 /2))/4;
                Const2 = (- L2 * sech(sqrt(K)* T3 /2))/4;
                xr1 = (K*(sin(2*pi*t/T3)+1));
                yr1 =  Const1*exp(sqrt(K)*(t-T3/2))+Const2*exp(-1*sqrt(K)*(t-T3/2))+ L1/2+ (L2/2);
                zr1 = Zo;
                xrf = K;
                yrf = ((L2/2)*sin(2*pi*t/T3 + pi/2))+ (3*DelY - (L1/4));
                zrf = (H)*(1+sin(4*pi*t/T3 - pi/2))/2;
                xl1 = (K*(sin(2*pi*t/T3)-1));
                yl1 = Const1*exp(sqrt(K)*(t-T3/2))+Const2*exp(-1*sqrt(K)*(t-T3/2))+ L1/2 + (L2/2);
                zl1 =  Zo;
                xlf = -K;
                ylf = 3*DelY - (L1/4);
                zlf = 0;
            end
            if ((t>(3*T3/4)) && (t<=T3))
                Const1 = (L1 * sech(sqrt(K) * T3 /2))/4;
                Const2 = (- L1 * sech(sqrt(K)* T3 /2))/4;
                xr1 = (K*(sin(2*pi*t/T3)+1));
                yr1 =  Const1*exp(sqrt(K)*(t-T3/2))+Const2*exp(-1*sqrt(K)*(t-T3/2))+ L1/2 + (L2/2);
                zr1 = Zo;
                xrf = K;
                yrf = ((L1/2)*sin(2*pi*t/T3 + pi/2))+ (3*DelY - L1/4);
                zrf = (H)*(1+sin(4*pi*t/T3 - pi/2))/2;
                xl1 = (K*(sin(2*pi*t/T3)-1));
                yl1 = Const1*exp(sqrt(K)*(t-T3/2))+Const2*exp(-1*sqrt(K)*(t-T3/2))+ L1/2 + (L2/2);
                zl1 =  Zo;
                xlf = -K;
                ylf = 3*DelY - L1/4;
                zlf = 0;
            end
            t = t +0.05;
            if(turnDirection == 1)
                 %Turn Counter-Clockwise
                 [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'l');
                 [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'r');
                 thetas = [te6 te7 te8 te9 te10 te1 te2 te3 te4 te5];
            else
                 %Turn Clockwise
                 [te1,te2,te3,te4,te5]=SolveIK(xrf,yrf,zrf,xr1,yr1,zr1,'r');
                 [te6,te7,te8,te9,te10]=SolveIK(xlf,ylf,zlf,xl1,yl1,zl1,'l');
                 thetas = [te1*factor te2 te3 te4 te5*factor te6*factor te7 te8 te9 te10*factor];
            end
        fprintf(fileID,'%d\t',round(thetas));
        fprintf(fileID,'\n');
        end
    end
fclose(fileID);
end
