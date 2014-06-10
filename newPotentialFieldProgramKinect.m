 % potentialField.m - Potential Field Program
% A script to make the robot navigate around obstacles in order to reach
% predetermined coordinates set by the user.

clear all
TRANS_VELOCITY = 500;    % translational velocity of the robot
ROT_VELOCITY = 20;     % rotational velocity of the robot
my_goal = [4000,0]; %coordinates of goal (adjustable)
dc = 5750.0; %cut off distance
rho = 250.0; %attractive force threshold distance/min distance to goal
zeta = .8; %weight of attractive force (adjustable) .7
eta = 7e7; %weight of repulsive force 9e7 (adjustable)
gain_tvel = 1.1 ; %translational velocity gain (adjustable)1.0
gain_svel = 10.0 ; %rotational velocity gain(adjustable) 10.0

attForce = []; % 
attForceD = []; %attractive force is initialized
repForce = []; %repellant force is initialized
totForce = []; %sum of all forces (repForce and attForceD)

% first connect to the robot
p3_start
pause(5)

% get the basic robot parameters
robotParams = p3_getRobotInfo;
numSonars = 57; %SET TO ANGLE IN FIELD OF VIEW
numFrontBumpers = robotParams(5);
numRearBumpers = robotParams(6);


% define minimum clearance
MIN_CLEARANCE = 300;   % 300 mm
rangeClearance = true;  % for use in while-loop
minSonarID = 1;    % this has the sonar with the min reading
minRange = 3*MIN_CLEARANCE;  % initialize to something
phi = [-28:28]*pi/180; %angle of each sonar in radians

%Intialize Computer Kinect Components 
 vid = videoinput('kinect', 2, 'Depth_640x480');
 src = getselectedsource(vid);
 vid.FramesPerTrigger = 1;
 src.CameraElevationAngle = 17;
 triggerconfig(vid, 'manual');
 start(vid);%takes half a second to start
 pause(1);
 preview(vid); 
 
 
while(p3_bumpersClear(numFrontBumpers, numRearBumpers) && rangeClearance)

     sonarRanges = p3_getAllSonarRange(numSonars);
     
     % print sonar ranges
     %sonarRanges'
    
     %Get Single Snapshot From Kinect
        imgDepth = getsnapshot(vid);
        imgDepth=fliplr(imgDepth); 
        im=imgDepth(250:480,:); %Crop Image
        
        %Initializations for For Loop 
        hCol=1; 
        count=0;
     
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

%2nd Comment

        %Store Depths
        as=-28:28; %Angle Information Total of 57 degrees

    % look at the forward sonars (0-7) to see where we have minimum
    % clearance
    minRange = 10*MIN_CLEARANCE;  % reset to something large each time
    for ii = 0:56 %57 sensors
        if(depth(ii+1) < minRange)
            minSonarID = ii;
            minRange = sonarRanges(ii+1);
        end
    end
    
    minSonarID;
    minRange;
     
  
    % determine robot movement
    my_Data = p3_getXYHeading; %assigns start out position of robot(x,y) to my_Data variable
    
    attForce(1) = my_Data(1) - my_goal(1); %calculates distance from robot position to robot goal in the x direction
    attForce(2) = my_Data(2) - my_goal(2); %calculates distance from robot position to robot goal in the y direction
    attForceMag = sqrt((attForce(1))^2 + (attForce(2))^2); %calculates magnitude of distance from robot to goal
    theta = my_Data(3) * (pi/180); %robot steering angle in robot coordinates 
    
    % see equation 2 in "Yun '97"
    if(attForceMag <= rho)
        attForce(1) = -1 * zeta *attForce(1); % World coordinate system
        attForce(2) = -1 * zeta *attForce(2); % World coordinate system
    else
        attForce(1) = -1 * zeta * rho * attForce(1) / attForceMag;
        attForce(2) = -1 * zeta * rho * attForce(2) / attForceMag;
    end
        
    attForceD(1) = attForce(1) * cos(theta) + attForce(2) * sin(theta); % Robot coordinate system
    attForceD(2) = -1 * attForce(1) * sin(theta) + attForce(2) * cos(theta); % Robot coordinate system
    
    attForceD 
    
    
    repForce(1) = 0.0; %initialize repForce x-component to 0
    repForce(2) = 0.0; %initialize repForce y-component to 0
    
    % gathers information from sonars and compares to cut off distance (dc)
    % in order to calculate repForce
    for ix =1:numSonars %57
        if(depth(ix) <= dc)
            temp = -1 * eta * (1/depth(ix)- 1/dc) * 1/depth(ix);
        else
            temp = 0.0;
        end
        repForce(1) = repForce(1) + temp*cos(phi(ix));
        repForce(2) = repForce(2) + temp*sin(phi(ix));          
    end
    
    repForceMag = sqrt((repForce(1))^2 + (repForce(2))^2); % repellant force magnitude is calcualted
    repForce
    
    totForce(1) = attForceD(1) + repForce(1); %.65 Total force is calculated by summing attractive and repellant (x-component)
    totForce(2) = attForceD(2) + repForce(2); %.65 Total force is calculated by summing attractive and repellant (y-component)
    totForce
    
    tempA = atan2(totForce(2), totForce(1)) * (180/pi)
    
    p3_setTransVel(gain_tvel * totForce(1)); % Translational velocity is set from x-component of totForce 
    p3_setRotVel(gain_svel * atan2(totForce(2), totForce(1))); %Rotational velocity is set
    

    % used to stop robot once it has reached the predetermined goal
    if(attForceMag <= 350)
        pause(2);
        string = 'Destination Reached, Ending Potential Field Program';
        disp(string);
        p3_setTransVel(0); 
        p3_setRotVel(0); 
        rangeClearance = 0;
    end

    
    % Used to manually stop robot. If we want to break out of this
    % loop, we can press the space bar (32)
    userInput = keyinfo;
    if(userInput(1) == 32)
        rangeClearance = 0;
        myString = 'End Program requested, Ending potential field program...';
        disp(myString);
        stop(vid)
    end
    
    % wait a little bit for robot to catch up with Matlab
    pause(1);
    
end

% stop and disconect from the robot
p3_end
stop(vid)



