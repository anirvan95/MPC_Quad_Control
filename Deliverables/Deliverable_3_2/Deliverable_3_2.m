clear all
clc

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%% plt-ref
ref = @(t, x) [-2; -2; -2; -pi/4];
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, 0, ref);
quad.plot(sim);

%% plt-X
ref_x = @(t, x) [-2; 0; 0; 0];
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, 0, ref_x);
quad.plot(sim);

%% plt-Y
ref_y = @(t, x) [0; -2; 0; 0];
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, 0, ref_y);
quad.plot(sim);

%% plt-Z
ref_z = @(t, x) [0; 0; -2; 0];
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, 0, ref_z);
quad.plot(sim);

%% plt-yaw
ref_yaw = @(t, x) [0; 0; 0; -pi/4];
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, 0, ref_yaw);
quad.plot(sim);