%This Script will be used to test depth values using the xbox Kinect before 
%and after calibration
vid = videoinput('kinect', 2, 'Depth_640x480');
vidrgb=videoinput('kinect',1); 

src = getselectedsource(vid);
srcrgb=getselectedsource(vidrgb); %color video

vid.FramesPerTrigger = 1;
vidrgb.FramesPerTrigger=1;

src.CameraElevationAngle = 14;
srcrgb.CameraElevationAngle= 14; 

start(vid);
start(vidrgb); 

imgDepth = getdata(vid);
rgb=getdata(vidrgb); 

imgDepth=fliplr(imgDepth); 

for u=1
layers={250:480};
%im=imgDepth(layers{u},8:634);
im=imgDepth(layers{u},:);%,171:640-171);

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
end 

imshow(10*imgDepth); 




%%
Actual=[10, 20, 30, 40, 50, 60, 70, 80, 90, 100:20:500]; % measurements in cm 
   

Sonar=[197, 206, 343, 410, 502, 610, 701, 806, 906, 1010, 1211, 1405, 1611, ...
       1809, 2000, 2207, 2411, 2612, 2818, 3009, 3209, 3411, 3613, 3814, ...
       4008, 4218, 4418, 4609, 4816, 5000];
   


%at x=297 y=344
Kinect=[0 0 0 0 0 0 0 8010 9190 1.016E4 1.221E4 1.416E4 1.622E4 1.828E4...
2.021E4 2.23E4 2.451E4 2.619E4 2.834E4 3.059e4 3.262E4 3.458E4 3.679E4 3.83E4...
3.93E4 0 0 0 0 0];

Kinect=.001007*(.004+Kinect);
% scatter(Actual, Sonar/10)
% hold on 
scatter(Actual/100, Kinect,'+','r')
hold on
scatter(Actual/100,Sonar/1000); 
grid on
hold on
plot(Actual/100,Actual/100,'LineWidth',1);
xlabel('Actual Depth (meters)','FontSize',30)
ylabel('Measured Depth (meters)','FontSize',30)
legend('Kinect Readings','Sonar Reading')
set(gca,'FontSize',15)


%[p,table,stats]=anova1(Kinect(8:25)')