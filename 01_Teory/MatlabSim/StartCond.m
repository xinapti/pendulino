clear all
clc
% Pendolo param
L = 2;          % lunghezza [m]
Km = 0.7;         % Coef Acc Mag
Kattr = 0.01;    % Attritto viscoso sulla velocità angolare
degStart = 5;  % Angolo di partenza [deg]

% Phisicx
g=9.81; % Gravità [m/s^2]

%Sim Cond
y0 = [deg2rad(degStart);0]; % Start Condition (Theta, Omega)
frameRate=30; % Frame per Secondo [1/s]

% Control para
Theta_Ref_max = deg2rad(30);
K_p = 10;