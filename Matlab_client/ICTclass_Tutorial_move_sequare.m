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
jPos_init={0,pi*20/180,0,-pi*70/180,0,pi*90/180,0}; % initial confuguration
VEL=0.15; % relative velocity
iiwa.movePTPJointSpace(jPos_init, VEL); % point to point motion in joint space

%% Get position roientation of end effector
disp('Cartesian position')
Pos=iiwa.getEEFPos();
disp(Pos);
z_1=Pos{3}; % save initial hight level
z0=448+3; % go to writing position
VEL=50; % velocity of the end effector, mm/sec


%% Insert the Pen at box level
Pos{3}=z0; %% first point
iiwa.movePTPLineEEF(Pos, VEL);

disp=50*2; % length of the side of the sequare

%% move in x positive direction
Pos{1}=Pos{1}+disp; 
iiwa.movePTPLineEEF(Pos, VEL)

%% move in y negative direction
Pos{2}=Pos{2}-disp; 
iiwa.movePTPLineEEF(Pos, VEL)

%% move in x negative direction
Pos{1}=Pos{1}-disp; 
iiwa.movePTPLineEEF(Pos, VEL)

%% move in y positive direction
Pos{2}=Pos{2}+disp; 
iiwa.movePTPLineEEF(Pos, VEL)

%% Go back to initial position
Pos{3}=z_1;
iiwa.movePTPLineEEF(Pos, VEL) 

%% turn off the server
iiwa.net_turnOffServer();

