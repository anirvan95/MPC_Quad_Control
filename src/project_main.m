clc
clear all

%% TODO 1.2: Dynamics
quad = Quad();

Tf = 5.0;
x0 = zeros(12, 1);
u = [1;-1;1;-1]*2*9.81/28;

sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0);
quad.plot(sim, u);

%% TODO 2.1: Steady state and linearized model
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);

%% TODO 2.2: Change of input variables u --> v : inv(T)u
sys_transformed = sys * inv(quad.T);

%% TODO 2.3: Compute independant systems
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%% Restart
clear all
clc

%% 3. MPC control
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% MPC x controller
mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

%% Test 3.1 MPC control
x = [0; 0.0; 0; 2.0];
ux = mpc_x.get_u(x)
uy = mpc_y.get_u(x)
uz = mpc_z.get_u([0; 2.0])
uyaw = mpc_yaw.get_u([0; 0.01])

%% Test 3.2 MPC tracking
x = [0; 0.0; 0; 2.0];
uxr = mpc_x.get_u(x, 2)
uyr = mpc_y.get_u(x, 2)
uzr = mpc_z.get_u([0; 2.0], 2)
uyawr = mpc_yaw.get_u([0; 0.01], pi/4)

%% 4. Simulation with nonlinear quad
offset = -0.1;
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw, offset);
quad.plot(sim);