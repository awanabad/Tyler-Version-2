clc;
clear all;

%variables 
v_max=0.9; %max velocity of 0.9 m/s 
a_max= 0.1; %max acceleration 0.1 m/s^2
r_w= 50; %wheel radius (mm) 
m= 5; %rough mass (kg) estimate of bot
d_1= 160; %distance (mm) from ramp to centre of mass 
d_2= 75;  %distance (mm) from wheel to centre of mass 

%static friction coefficients 
sf_1= 0.08; %btwn ramp (ABS plastic) and floor (concrete) 
sf_2= 1; %between wheel (rubber) and floor (concrete)  

%dynamic coefficients of friction 
df_1= 0.06; %btwn ramp (ABS plastic) and floor (concrete) 
df_2= 0.8; %between wheel (rubber) and floor (concrete)  


%************************************************************************ 
%constants for calculation 
g=9.81; %grav const 

%Static Analysis 
W=m*g;

syms N1 N2;
eqns= [
%Moment Applied at Point B 
-(W*d_2*(10^-3)) + N1*((d_1+d_2)*(10^-3)) == 0, 
%Forces in y-direction 
N2 + N1 - W == 0 ]; 

[A, b] = equationsToMatrix(eqns);
vars=symvar(eqns); 
soln=solve(eqns, vars);

fprintf('Reaction Force at Ramp is: (N) %s\n', double(soln.N1));
fprintf('Reaction Force at Wheel is: (N) %s\n', double(soln.N2));
%************************************************************************ 
%Calculation of Stall Torque

%Frictional Forces 
SFr_1=soln.N1*sf_1*0.5; %per wheel and two wheels 
SFr_2=soln.N2*sf_2*0.5;

M_wheel=(r_w*10^-3)*(SFr_1+SFr_2); 

fprintf(1, '\n');
fprintf('Moment to be overcome by wheel, for movement, is: (N/m) %s\n', double(M_wheel));
fprintf('This is the stall torque (per wheel). \n');

%************************************************************************ 
%Calculation of Continous Torque

%Dynamic Analysis

DFr=m*a_max*0.5; %Friction is ma for const velocity 0.5 Per Wheel

%DFr_1=soln.N1*df_1*0.5; %Ignore
%DFr_2=soln.N2*df_2*0.5; %Ignore

M_wheel2=(r_w*10^-3)*(DFr); 

fprintf(1, '\n');
fprintf('Moment for constant motion is: (N/m) %s\n', double(M_wheel2));
fprintf('This is the continuous torque (per wheel). \n');

%************************************************************************ 
%Calculation of Angular Velocity 
%v_max=Omega*r 

Om=(v_max/r_w)*(1/(2*pi));
Om_min=Om*60;
fprintf(1, '\n');
fprintf('Calculated Angular Velocity (Based on v assumptions) is: (rev/s) %s\n', double(Om));
fprintf('In: (rev/min) %s\n', double(Om_min));

