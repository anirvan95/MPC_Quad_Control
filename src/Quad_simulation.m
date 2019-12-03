clc
clear all
close all
quad = Quad();
Tf = 5.0;
x0 = zeros(12,1);
u=[0.7007;0.7007;0.7007;0.7007];
sim = ode45(@(t,x) quad.f(x,u), [0,Tf],x0);
quad.plot(sim,u);


[xs,us] = quad.trim();
sys = quad.linearize(xs, us);

sys_transformed = sys * inv(quad.T) ;
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

dsys_x = c2d(sys_x,1/5);

