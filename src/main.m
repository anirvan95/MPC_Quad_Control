clc
clear all
close all

addpath(genpath('C:\Users\anirv\Documents\MATLAB\Tools'));
quad = Quad();

%% Open Loop Simulation 
Tf = 200.0; % Time to simulate for
% x0 = zeros(12,1); % Initial state
% u = [0.7007;0.7007;0.7007;0.7007]; % Input to apply
% sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0); % Solve the system ODE
%quad.plot(sim, u); % Plot the result

%% System Formulation
Ts = 1/5; %Sampling time for c2d
BIAS = 0.1; %Input bias term
[xs,us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% MPC Controllers
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

% %% Simulate individual controllers
%z
time_vect = [0];
sol.z(:,1) = [0;-2];
for i = 1:Tf
    time_vect = [time_vect;(time_vect(i)+Ts)];
    sol.uz(:,i) = mpc_z.get_u(sol.z(:,i),10);
    sol.z(:,i+1) = mpc_z.A*sol.z(:,i) + mpc_z.B*sol.uz(:,i);
end
subplot(2,2,1),plot(time_vect,sol.z(2,:),'r')
% 
%x
sol.x(:,1) = [0;0;0;2];
t = 0;
for i = 1:Tf
    sol.ux(:,i) = mpc_x.get_u(sol.x(:,i),10);
    sol.x(:,i+1) = mpc_x.A*sol.x(:,i) + mpc_x.B*sol.ux(:,i);
end

subplot(2,2,2),plot(time_vect,sol.x(4,:),'g')

%y
sol.y(:,1) = [0;0;0;0];
for i = 1:Tf
    sol.uy(:,i) = mpc_y.get_u(sol.y(:,i),0);
    sol.y(:,i+1) = mpc_y.A*sol.y(:,i) + mpc_y.B*sol.uy(:,i);
end

subplot(2,2,3), plot(time_vect,sol.y(4,:),'b')

%yaw
sol.yaw(:,1) = [0;pi/4];
for i = 1:Tf
    sol.uyaw(:,i) = mpc_yaw.get_u(sol.yaw(:,i),-pi/4);
    sol.yaw(:,i+1) = mpc_yaw.A*sol.yaw(:,i) + mpc_yaw.B*sol.uyaw(:,i);
end

subplot(2,2,4), plot(time_vect,sol.yaw(2,:),'k')
%% Complete simulation
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw,BIAS);
quad.plot(sim);
