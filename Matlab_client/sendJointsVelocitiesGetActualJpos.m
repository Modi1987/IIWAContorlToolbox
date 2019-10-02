function [ JPoS ] = sendJointsVelocitiesGetActualJpos( t_Kuka ,jvel)
%% Applicable to KST 1.6 and above

%% Syntax:
%  [ JPoS ] = sendJointsVelocitiesGetActualJpos( t_Kuka ,jvel)

%% About
% This function is used to send the target velocities of the joints for the
% direct servo motion, at the same time the server returns a feedback of
% the actual position of the joints from the robot encoders.
% Midpoint Reiman Sum is used to integrate angular velocity into angular
% position which is applied to the direct servo in the server.

%% Arreguments:
% jvel: is 7 cells array of doubles, representing joints velocities in (rad/sec)
% t_Kuka: is the TCP/IP connection object

%% Return value:
% JPoS: is 7 cell array of doubles of the actual Joints positions from robot encoders

% Copy right, Mohammad SAFEEA, 15th-April-2018

theCommand='velJDCJP_';
for i=1:7
    x=jvel{i};
    theCommand=[theCommand,sprintf('%0.5f',x),'_'];
end

fprintf(t_Kuka, theCommand);
message=fgets(t_Kuka);
[JPoS,N]=getDoubleFromString(message);
end

function [jPos,j]=getDoubleFromString(message)
n=max(max(size(message)));
j=0;
numString=[];
for i=1:n
    if message(i)=='_'
        j=j+1;
        jPos{j}=str2num(numString);
        numString=[];
    else
        numString=[numString,message(i)];
    end
end
end

