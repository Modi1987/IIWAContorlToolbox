function dh=constDhDataOf7R800()
%% This function returns the DH parameters 
%     for the KUKA iiwa 7R800

% The dimnstions are taken from Kuka manuals:
% [1] Medien-Flansch
%     Für Produktfamilie LBR iiwa
%     Montage- und Betriebsanleitung

%% DH PARAMETERS FOR THE ROBOT
dh.alfa={0,-pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2};
% following are "d" parameters for iiwa 7 R 800 
dh.d={0.34,0.0,0.4,0.0,0.4,0.0,0.126};
dh.a={0,0,0,0,0,0,0};
end