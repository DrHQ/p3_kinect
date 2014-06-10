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
 
 
% define minimum clearance for stopping the robot.
MIN_CLEARANCE = 200;   
sonar_clearance= 500;
rangeClearance = true;  % for use in while-loop
sonar_check=0; 





% CONNECT TO THE ROBOT
p3_start
pause(5)
    %Then connect to the camera
    vid = videoinput('kinect', 2, 'Depth_640x480');
    %vidrgb=videoinput('kinect',1); 

    src = getselectedsource(vid);
    %srcrgb=getselectedsource(vidrgb); %color video

    vid.FramesPerTrigger = 1;
    %vidrgb.FramesPerTrigger=1;

    src.CameraElevationAngle = 14;
    %srcrgb.CameraElevationAngle= 15; 
    triggerconfig(vid, 'manual');
    start(vid);%takes half a second to start
    pause(1);
preview(vid); 



% get the basic robot parameters
robotParams = p3_getRobotInfo;
numSonars = robotParams(4);
numFrontBumpers = robotParams(5);
numRearBumpers = robotParams(6);

while(p3_bumpersClear(numFrontBumpers, numRearBumpers) && rangeClearance)
    
    %Checks what sonar ranges are;
    sonarRanges = p3_getAllSonarRange(numSonars);
    sonar_check=~isempty(find(sonarRanges<sonar_clearance)); 
    sonar_check=0; 
 if sonar_check==1; %check to see if less than min distance
      % get the sonar readings
      disp('depthmode: sonar')
                                

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
                                end

                                % initialize the sum vector
                                sum_x = 0.0;
                                sum_y = 0.0;

                                % sum up the 8 front sonar vectors
                                for i=1:8 
                                    sum_x = sum(p_r(1,:));
                                    sum_y = sum(p_r(2,:));
                                end

                                k1=3.5;
                                k2=1.2;
                                offset=0;

                                forward_vel = k1*(sum_x-offset);
                                turning_vel = k2*sum_y;

                                if hypot(forward_vel,turning_vel)<3.0
                                    turning_vel=7;
                                end

                                % set the motion velocity
                                p3_setTransVel(forward_vel);
                                p3_setRotVel(turning_vel);   



                                % make sure we are not too close to an obstacle.  If we are, then set
                                % the "rangeClearance" to False so that we break out of the while loop
                                index = find(sonarRanges < MIN_CLEARANCE);
                                sonar_check=~isempty(find(sonarRanges<sonar_clearance));

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
    
 else 
     
    %get the sonar readings
    %get sensor readings there are 57 of them.   
        pause(1)
        %%%Collect 1 snap of video data


        %start(vidrgb); 

        imgDepth = getsnapshot(vid);
        %rgb=getdata(vidrgb); 

        imgDepth=fliplr(imgDepth); 
        layers={250:480};
            %im=imgDepth(layers{u},8:634);
            im=imgDepth(layers{1},:);%,171:640-171);

            %each 11.2281 pixels is 1 degree field of view


            %im(im==0)=[]; 

            hCol=1; 
            count=0;



            %for angle_bin=1:57 %For Angles in Image
            lengthAng=57;
            for angle_bin=1:lengthAng; 
                for hCol=hCol:hCol+11%for columns in image 
                    imCol=im(:,hCol); %For all rows in column
                    imCol(imCol==0)=[]; %if column returns zero (non return) input empty matrix 
                    temp=mean(imCol); %Take the mean of all the depths in the column
                    count=count+1;
                    depthPixel(count)=temp; 
                    %average columns
                end 
                depth(angle_bin)=mean(depthPixel);
                count=0; 
            end 




as=-28:28; 
    
    
    
    
    
    %sonarRanges = p3_getAllSonarRange(numSonars);
    %SonarRanges equivalent to get depth
    
    
    for i = 1:57
      x_sn(i) = 0; %For XBOX KINECT
      y_sn(i) = 0;  %For XBOX KINECT
      alpha(i) = as(i); %LOOKING STRAIGHT AHEAD
   
      %object position in the sonar i coordinate in meters
      p_sn = [depth(i)/1000 0 0 1]';
   
      %transform of sonar i frame relative to the robot frame
      T_r_sn = [cosd(alpha(i)) -sind(alpha(i)) 0 x_sn(i);  
                sind(alpha(i))  cosd(alpha(i)) 0 y_sn(i); 
                0 0 1 0; 
                0 0 0 1]; 
      %object position in the robot frame
      p_r(:,i) =  T_r_sn * p_sn; %4x8 double (8 sensors
      
      %scatter(p_r(1,:),p_r(2,:)); 
      %hold on
     % disp('Capture')
      %toc
    end




    % initialize the sum vector
    sum_x = 0.0;
    sum_y = 0.0;

    % sum up the 8 front sonar vectors
    for i=1:57 
        sum_x = nansum(p_r(1,:));
        sum_y = nansum(p_r(2,:));
    end

    k1=3.4;
    k2=1.0;
    offset=0;

    forward_vel = k1*(sum_x-offset);
    turning_vel = k2*sum_y;

    if hypot(forward_vel,turning_vel)<3.0
        turning_vel=4;
    end

    % set the motion velocity
    p3_setTransVel(forward_vel);
    p3_setRotVel(turning_vel);   
    
 
 
    % make sure we are not too close to an obstacle.  If we are, then set
    % the "rangeClearance" to False so that we break out of the while loop
    index = find(depth < MIN_CLEARANCE);
    sonar_check=~isempty(find(depth<sonar_clearance));
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
 end%if sonar check     
end

% stop and disconect from the robot
p3_end
%stop and disconnect from the camera
stop(vid)