clear all
clc
Tf = 75;
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

time_vect = [0];
%% z
sol.z(:,1) = [0;-2];
for i = 1:Tf
    time_vect = [time_vect;(time_vect(i)+Ts)];
    sol.uz(:,i) = mpc_z.get_u(sol.z(:,i));
    sol.z(:,i+1) = mpc_z.A*sol.z(:,i) + mpc_z.B*sol.uz(:,i);
end
sol.uz(:,i+1) = sol.uz(:,i);
figure(5),
subplot(1,3,1),plot(time_vect,sol.z(1,:),'r','Linewidth',2)
grid on;
ylabel("Vel z [m/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(1,3,2),plot(time_vect,sol.z(2,:),'g','Linewidth',2)
grid on;
ylabel("z [m]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(1,3,3),plot(time_vect,sol.uz,'b','Linewidth',2)
grid on;
ylabel("F [N]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
%% x
sol.x(:,1) = [0;0;0;-2];
t = 0;
for i = 1:Tf
    sol.ux(:,i) = mpc_x.get_u(sol.x(:,i));
    sol.x(:,i+1) = mpc_x.A*sol.x(:,i) + mpc_x.B*sol.ux(:,i);
end
sol.ux(:,i+1) = sol.ux(:,i);
figure(6),
subplot(3,2,1),plot(time_vect,sol.x(1,:),'r','Linewidth',2)
grid on;
ylabel("Vel pitch [rad/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,2),plot(time_vect,sol.x(2,:),'g','Linewidth',2)
grid on;
ylabel("Pitch [rad]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,3),plot(time_vect,sol.x(3,:),'r','Linewidth',2)
grid on;
ylabel("Vel x [m/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,4),plot(time_vect,sol.x(4,:),'g','Linewidth',2)
grid on;
ylabel("x [m]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,5),plot(time_vect,sol.ux,'b','Linewidth',2)
grid on;
ylabel("M_\alpha [Nm]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')

%% y
sol.y(:,1) = [0;0;0;-2];
for i = 1:Tf
    sol.uy(:,i) = mpc_y.get_u(sol.y(:,i));
    sol.y(:,i+1) = mpc_y.A*sol.y(:,i) + mpc_y.B*sol.uy(:,i);
end

sol.uy(:,i+1) = sol.uy(:,i);
figure(7),
subplot(3,2,1),plot(time_vect,sol.y(1,:),'r','Linewidth',2)
grid on;
ylabel("Vel roll [rad/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,2),plot(time_vect,sol.y(2,:),'g','Linewidth',2)
grid on;
ylabel("Roll [rad]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,3),plot(time_vect,sol.y(3,:),'r','Linewidth',2)
grid on;
ylabel("Vel y [m/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,4),plot(time_vect,sol.y(4,:),'g','Linewidth',2)
grid on;
ylabel("y [m]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(3,2,5),plot(time_vect,sol.uy,'b','Linewidth',2)
grid on;
ylabel("M_\beta [Nm]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')


%yaw
sol.yaw(:,1) = [0;-pi/4];
for i = 1:Tf
    sol.uyaw(:,i) = mpc_yaw.get_u(sol.yaw(:,i));
    sol.yaw(:,i+1) = mpc_yaw.A*sol.yaw(:,i) + mpc_yaw.B*sol.uyaw(:,i);
end

sol.uyaw(:,i+1) = sol.uyaw(:,i);
figure(8),
subplot(1,3,1),plot(time_vect,sol.yaw(1,:),'r','Linewidth',2)
grid on;
ylabel("Vel yaw [rad/s]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(1,3,2),plot(time_vect,sol.yaw(2,:),'g','Linewidth',2)
grid on;
ylabel("Yaw [rad]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')
subplot(1,3,3),plot(time_vect,sol.uyaw,'b','Linewidth',2)
grid on
grid on;
ylabel("M_\gamma [Nm]",'FontSize',18,'FontWeight','bold')
xlabel("Time [s]",'FontSize',18,'FontWeight','bold')