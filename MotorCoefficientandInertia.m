clc;
clear all;

%Motor Stuff Calculator 
%Steps for use 
% 1) Enter Motor Specs 
% 2) Run Code 
% 3) Success!!!!!!!!!!!!!!!!!!!!!!
%Inertia?

K_t=1;    %Motor Torque Coefficient 
Om_NL=1;  %No Load Speed (rpm)
I_NL=1;   %No Load Current (Amp)

%Coefficient Calculation
Om_rads= Om_NL * ((2*pi)/60);
b= (K_t*I_NL)/Om_rads;

fprintf('Motor Friction Coefficient is %s\n', double(b));

%************************************************************************ 

