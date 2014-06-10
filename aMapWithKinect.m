close all; 
clear all; 

vid = videoinput('kinect', 2, 'Depth_640x480');
vidrgb=videoinput('kinect',1); 

src = getselectedsource(vid);
srcrgb=getselectedsource(vidrgb); %color video

vid.FramesPerTrigger = 1;
vidrgb.FramesPerTrigger=1;

src.CameraElevationAngle = 16;
srcrgb.CameraElevationAngle= 16; 
triggerconfig(vid, 'manual');
triggerconfig(vidrgb,'manual'); 
start(vid);%takes half a second to start
pause(1);
for play=1
tic    

%%%Collect 1 snap of video data


start(vidrgb); 

imgDepth = getsnapshot(vid);
rgb=getsnapshot(vidrgb); 

rgb=flipdim(rgb,2);

imgDepth=fliplr(imgDepth); 


stop(vidrgb)
%%
%%
% figure()
% imshow(imgDepth);
% figure()
% imshow(rgb); 

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

%depth=.001007*(.004+depth);

%%Delay Pause(slows data down)

%as=[-28:28];
as=ceil(-lengthAng/2):ceil(lengthAng/2);
%for i=1:57 %For each individual sensor 
for i=1:lengthAng;   
Ts(:,:,i)=[cosd(as(i)),-sind(as(i)),0,0;...
           sind(as(i)),cosd(as(i)),0,0; ...
           0, 0, 1, 0; ...
           0, 0, 0, 1];
end


myData=zeros(4,4,1); %1 picture to begin with
P_worlds=[];
j=1; 
    Trw(:,:,j)=[cos(myData(j,3)),-sin(myData(j,3)),0,0;...
           sin(myData(j,3)),cos(myData(j,3)),0,0; ...
           0, 0, 1, 0; ...
           0, 0, 0, 1];
       
     % Now build map
     P_sonar=depth;
     %for s=1:57
         for s=1:lengthAng; 
        P_temp=Trw(:,:,j)*Ts(:,:,s)*[P_sonar(s);0;0;1];
        %P_temp=Trw(:,:,j)*[P_sonar(s);0;0;1];

        P_world(:,s)=P_temp(1:2,1);
     end
     P_worlds=[P_worlds,P_world];
     %Create Polar Matrices rho, and R
     for i=1:length(P_worlds)
     theta(i)=atan2(P_worlds(2,i),P_worlds(1,i));
     R(i)=sqrt(P_worlds(1,i)^2+P_worlds(2,i)^2);
     end
   
             % scatter(P_worlds(1,:),P_worlds(2,:))
              
%               axis([0 6 -6 6])
% 
%              title('Cartesian Plot of Coordinate system')
%              xlabel('x meters')
%              ylabel('y meters')
%              hold on 
     
%       polar(theta,R,'d')
%       title('Polar plot of data')
figure(1)
 scatter(P_worlds(1,:)/1000,P_worlds(2,:)/1000)
 hold on 
 plot(0,0,'go')
 title('2D Plot of Kinect Returns of Multiple Thin Poles') 
 xlabel('X (meters)')
 ylabel('Y (meters)')
 axis equal
 
toc
end
stop(vid)
delete(vid)
pause(1)
disp('end')
%%

    


newbinimage=imgDepth==0;
im = newbinimage; 
redChannel=rgb(:,:,1); 
greenChannel=rgb(:,:,2);
blueChannel=rgb(:,:,3); 

redChannel(im)=255;
greenChannel(im)=0; 
blueChannel(im)=0; 
figure(2)
newRGB=cat(3,redChannel,greenChannel,blueChannel); 

figure(3)
imshow(newRGB(250:480,:,:))
figure(4)
imshow(10*fliplr(imgDepth(250:480,:)))


