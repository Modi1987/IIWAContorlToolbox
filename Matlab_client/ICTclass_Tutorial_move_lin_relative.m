%% Example of using ICT class to interface with KUKA iiwa

% First start the "ICTServer" on the smartPad
% Then run the following script in Matlab

% Note you have 60 seconds to connect to ICTServer after starting the
% application on the smartPad.

% Copyright: Mohammad SAFEEA, 02-Oct-2019

close all;clear;clc;
warning('off')
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=ICT.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=ICT.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % Transform matrix of EEF with respect to flange.
Tef_flange(1,4)=0; % X-coordinate of TCP (meter)
Tef_flange(2,4)=0; % Y-coordinate of TCP (meter)
Tef_flange(3,4)=30/1000; % Z-coordinate of TCP (meter)
toolData=[0.3,0,0,0.05]; % mass of tool is 1 kg, COMx=0 (m), COMy=0 (m),COMz=0.05 (m)
iiwa=ICT(ip,arg1,arg2,Tef_flange,toolData); % create the object

%% Start a connection with the server
flag=iiwa.net_establishConnection();
if flag==0
    return;
end
pause(1);
disp('------------')
disp('Make sure no objects are around the robot, for the robot will move')
input('Press enter to continue\n')
disp('------------')  
%% move to initial position
jPos={0,pi*20/180,0,-pi*70/180,0,pi*60/180,0}; % initial confuguration
relVel=0.15; % relative velocity
iiwa.movePTPJointSpace(jPos, relVel); % point to point motion in joint space

%% Linear relative motion of end effector, relative to base frame
deltaX=0;deltaY=0;deltaZ=100.;
Pos{1}=deltaX;
Pos{2}=deltaY;
Pos{3}=deltaZ;

vel=250; % linear velocity of end effector, mm/sec
iiwa.movePTPLineEefRelBase(Pos, vel);
Pos{3}=-deltaZ;
iiwa.movePTPLineEefRelBase(Pos, vel);

%% Linear relative motion of end effector, relative to EEF frame
Pos{1}=deltaX;
Pos{2}=deltaY;
Pos{3}=deltaZ;

vel=150; % linear velocity of end effector, mm/sec

iiwa.movePTPLineEefRelEef(Pos, vel);
Pos{3}=-deltaZ;
iiwa.movePTPLineEefRelEef(Pos, vel);


%% turn off the server
iiwa.net_turnOffServer();
warning('on');
