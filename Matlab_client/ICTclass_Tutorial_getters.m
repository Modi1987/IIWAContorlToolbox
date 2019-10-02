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
Tef_flange=eye(4); % Transofrm matrix of EEF with respect to flange.
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
disp('** Acquiring Data from the Robot')
disp('------------')
%% move to some initial configuration
pinit={0,pi*20/180,0,-pi*70/180,0,pi*90/180,0}; % initial confuguration
relVel=0.15; % relative velocity
iiwa.movePTPJointSpace(pinit, relVel); % point to point motion in joint space

%% Get the joints positions
jPos  = iiwa.getJointsPos();
fprintf('** The joints positions of the robot are: \n');
disp(jPos);
disp('------------')
%% Get pose of end effector (Position/Orientation)
fprintf('** Cartesian position/orientation of end-effector:\n');
pos=iiwa.getEEFPos();
disp(pos);
disp('------------')
%% Get position of end effector
fprintf('** Cartesian position of end-effecotr:\n')
cpos=iiwa.getEEFCartesianPosition();
disp(cpos);
disp('------------')
%% Get orientation of end effector
fprintf('** Cartesian orientation of end-effecotr:\n')
orie=iiwa.getEEFCartesianOrientation();
fprintf('Alfa (rad): %f \n',orie{1});
fprintf('Beta (rad): %f \n',orie{2});
fprintf('Gamma (rad): %f \n',orie{3});
disp('------------')
%% Get force at end effector
fprintf('** Cartesian force acting at end effector:\n')
f=iiwa.getEEF_Force();
fprintf('Fx (N): %f \n',f{1});
fprintf('Fy (N): %f \n',f{2});
fprintf('Fz (N): %f \n',f{3});
disp('------------')
%% Get moment at end effector
fprintf('** Moment at eef\n');
m=iiwa.getEEF_Moment();
fprintf('Mx (mN): %f \n',m{1});
fprintf('My (mN): %f \n',m{2});
fprintf('Mz (mN): %f \n',m{3});
 disp('------------') 
%% turn off the server
iiwa.net_turnOffServer();



