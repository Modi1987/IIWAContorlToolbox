function [y]=attachToolToFlange(t_kuka,ts,inertiaParams)
%% About
% Attach the tool specified by the transform (ts) into the flange

%% Areguments
% t_kuka: TCP/IP comunication object
% ts: transform vector [x,y,z,a,b,c] of TCP to the flange of the robot
% x,y,z are in mm
% a,b,c are XYZ fixed rotation angles in radians
% inertiaParams: the inertia parameters of the tool attached to the flange
% of the robot [m (kg),COMx (mm),COMy (mm),COMz (mm)] 

% Copyright: Mohammad SAFEEA, 02-Oct-2019

    if max(max(size(ts==6)))
    else
        y=false;
        return;
    end
%% 
    theCommand=['TFtrans']; % instruction part
    for i=1:6
        st=num2str(ts(i));
        theCommand=[theCommand,'_',st];
    end
    for i=1:4
        st=num2str(inertiaParams(i));
        theCommand=[theCommand,'_',st];
    end
    fprintf(t_kuka, theCommand);
    message=fgets(t_kuka);
    [ret]=checkAcknowledgment(message);
    y=ret;
    if(y==false)
        % turn off the server
        disp('Could not attach the tool to the Robot');
        net_turnOffServer(t_kuka);
    end
end