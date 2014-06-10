% lab4.m  
% Template program for EC4310 lab 4.
close all; 
clear all; 
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

% get the basic robot parameters
robotParams = p3_getRobotInfo;
numSonars = robotParams(4);
numFrontBumpers = robotParams(5);
numRearBumpers = robotParams(6);


% define minimum clearance for stopping the robot.
MIN_CLEARANCE = 250;   
rangeClearance = true;  % for use in while-loop
play=1; 
p_rr=[]; 
for play=1; 
% while(p3_bumpersClear(numFrontBumpers, numRearBumpers) && rangeClearance)

    % get the sonar readings
    sonarRanges = p3_getAllSonarRange(numSonars);
 sonarRanges(sonarRanges>=5000)=NaN;
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
            
            %Robot to world
            myData=zeros(4,4,1); %1 picture to begin with

            j=1;
            
            Trw = eye(4);
%             Trw(:,:,j)=[cos(myData(j,3)*pi/180),-sin(myData(j,3)*pi/180),0,0;...
%            sin(myData(j,3)*pi/180),cos(myData(j,3)*pi/180),0,0; ...
%            0, 0, 1, 0; ...
%            0, 0, 0, 1];
      % object position in the robot frame
      
      p_r(:,i) =  T_r_sn * p_sn;
      p_rr(:,i) =  Trw * p_r(:,i);
      %p_rr(:,i)=Trw*T_r_sn*p_sn; 
      
 
     

end
  plot(p_rr(1,:),p_rr(2,:),'o')
    xlabel('X position (meters)')
    ylabel('Y position (meters)')
    title('2D Plot of Sonar Returns of Test Scenario ')
    hold on
    axis equal
    plot(0,0,'go'); 
    % initialize the sum vector
%     sum_x = 0.0;
%     sum_y = 0.0;
% 
%     % sum up the 8 front sonar vectors
%     for i=1:8 
%         sum_x = sum(p_r(1,:));
%         sum_y = sum(p_r(2,:));
%     end
% 
%     k1=15;
%     k2=4;
%     offset=5;
% 
%     forward_vel = k1*(sum_x-offset);
%     turning_vel = k2*sum_y;
% 
%     if hypot(forward_vel,turning_vel)<3.0
%         turning_vel=7;
%     end
% 
%     % set the motion velocity
%     p3_setTransVel(forward_vel);
%     p3_setRotVel(turning_vel);   
%     
 
 
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

% stop and disconect from the robot
p3_end