clc
clear all
close all

quad = Quad();
%ref = @(t,x) [-2; 0; 0; 0]; %for user defined trajectory tracking 
ref = []; %for MPC trajectory tracking
CTRL = ctrl_NMPC(quad);
sim = quad.sim(ref,CTRL);
quad.plot(sim)