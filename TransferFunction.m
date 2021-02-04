clc;
clear all;

%Motor Encoder Trans Function 

T_r= 0.27*10^-6;    %Motor Rise Time (s) 

Tao= 0.633333*(T_r); %Time Constant 

a=1/Tao;

fprintf('The simplified encoder transfer function is in the form \n');
fprintf('T(S) = a/(s+a)');
fprintf(1, '\n');
fprintf('where a %s\n', double(a));