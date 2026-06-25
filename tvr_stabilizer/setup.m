% setup.m - run this BEFORE opening/running the model
clear; clc;
p = params(); % load parameters

assignin('base', 'p', p); % assign to base workspace for Simulink
disp('Params loaded. Now run the model.');