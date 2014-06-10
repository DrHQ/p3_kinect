% Capt Joshua S Lum, USMC
% EC4310 - Lab 5
% Template program for EC4310 lab 5.
clear
clc

% sonar location matrix
sonar_location = [
      0.069  0.136  90;
      0.114  0.119  50;
      0.148  0.078  30;
      0.166  0.027  10;
      0.166 -0.027 -10;
      0.148 -0.078 -30;
      0.114 -0.119 -50;
      0.069 -0.136 -90;
     -0.157 -0.136 -90; 
     -0.203 -0.119 -130; 
     -0.237 -0.078 -150; 
     -0.255 -0.027 -170; 
     -0.255  0.027  170;
     -0.237  0.078  150; 
     -0.203  0.119  130; 
     -0.157  0.136  90 ];

% first connect to the robot
p3_start
pause(5)

%get the basic robot parameters
robotParams = p3_getRobotInfo;
numSonars = robotParams(4);
numFrontBumpers = robotParams(5);
numRearBumpers = robotParams(6);


% define minimum clearance for stopping the robot.
MIN_CLEARANCE = 250;   
rangeClearance = true;  % for use in while-loop

Pgoal = input('input [x;y] of goal:  ');
while(p3_bumpersClear(numFrontBumpers, numRearBumpers) && rangeClearance)
%     % get the sonar readings
    sonarRanges = p3_getAllSonarRange(numSonars);
 eta=10000; % 1.3*10^7
 ro=4500; % 4500
    for i = 1:8
      x_sn(i) = sonar_location(i,1);
      y_sn(i) = sonar_location(i,2);
      alpha(i) = sonar_location(i,3)*pi/180;
      
      % object position in the sonar i coordinate in meters
      p_sn = [sonarRanges(i)/1000 0 0 1]';
   
      % transform of sonar i frame relative to the robot frame
      T_r_sn = [cos(alpha(i)) -sin(alpha(i)) 0 x_sn(i);  
                sin(alpha(i))  cos(alpha(i)) 0 y_sn(i); 
                0 0 1 0; 
                0 0 0 1]; 
      % object position in the robot frame
      p_r(:,i) =  T_r_sn * p_sn;      

        di=sqrt((p_r(1,i))^2 + (p_r(2,i))^2);
        Frep(:,i)=-eta*((1/di)-(1/ro))*(1/(di^2))*[cos(alpha(i));sin(alpha(i))]
        if di>ro
            Frep(:,i)=[0;0]
        end           
     end

    % Establish Attractive Force
    Posit = p3_getXYHeading; 
    Posit(3)=Posit(3)*pi/180;% needs to be in rad for T_r_w
    Catt = 1;
    Fattreal = [Posit(1)-Pgoal(1);Posit(2)-Pgoal(2);0]
    
    % Stop robot if we get within 5cm of goal
    if norm(Fattreal)<50
        break
    end
    
    R_r_w=[cos(Posit(3)),sin(Posit(3)),0
        -sin(Posit(3)),cos(Posit(3)),0
        0,0,1];

    Fattrob = -Catt*(R_r_w*Fattreal)

    %initialize the sum vector
    Frep_x = 0.0;
    Frep_y = 0.0;

    %sum up the 8 front sonar vectors
    for i=1:8  
        Frep_x = sum(Frep(1,:)); 
        Frep_y = sum(Frep(2,:));
    end

 forward_vel = .3*(Fattrob(1)+Frep_x);
 if abs(forward_vel)>600
     forward_vel = sign(forward_vel)*600;
 end
 
 turning_vel = .03*(Fattrob(2)+Frep_y);
 if abs(turning_vel) > 100
     turning_vel = sign(turning_vel)*100;
 end

    % set the motion velocity
    p3_setTransVel(forward_vel)
    p3_setRotVel(turning_vel)  
   
    % make sure we are not too close to an obstacle.  If we are, then set
    % the "rangeClearance" to False so that we break out of the while loop
    index = find(sonarRanges < MIN_CLEARANCE);
    if(~isempty(index))
        rangeClearance = false;
        myString = 'Not enough clearance. Disconnecting from robot.';
        myString2 = 'Sonar number:  ';
        disp(myString);
        disp(myString2);
        index-1
    end
    
    % or if we are using MobileSim and we just want to break out of this
    % loop, we can press the space bar (32)
    userInput = keyinfo;
    if(userInput(1) == 32)
        rangeClearance = 0;
        myString = 'Ending the program...';
        disp(myString);
    end
    
    % wait a little bit for robot to catch up with Matlab
    pause(1);
    
end
    % set the motion velocity
    p3_setTransVel(0);
    p3_setRotVel(0);   
    
% stop and disconect from the robot
p3_end