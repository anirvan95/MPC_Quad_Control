function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 30; % MPC horizon [SET THIS VARIABLE]
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%
h = 0.2;
f_discrete = @(x,u) RK4(x,u,h,quad);
Q = 5*eye(12);
R = eye(4);
REF_vect = [zeros(5,1);REF(4,1);zeros(3,1);REF(1:3,1)];
F = [0 0 0 1 0 0 0 0 0 0 0 0;0 0 0 -1 0 0 0 0 0 0 0 0;0 0 0 0 1 0 0 0 0 0 0 0;0 0 0 0 -1 0 0 0 0 0 0 0];
f = [0.035;0.035;0.035;0.035];
%M = [1 0 0 0;-1 0 0 0;0 1 0 0;0 -1 0 0;0 0 1 0;0 0 -1 0;0 0 0 1;0 0 0 -1];
%m = [1.5;0;1.5;0;1.5;0;1.5;0];
%opti.set_value(X0,zeros(12,1));

cost = (X(:,1)-REF_vect)'*Q*(X(:,1)-REF_vect)+U(:,1)'*R*U(:,1);
opti.subject_to(U(:,1) <= 1.5*ones(4,1));
opti.subject_to(U(:,1) >= zeros(4,1));
opti.subject_to(F*X(:,1) < f);
opti.subject_to(X(:,2) == f_discrete(X(:,1),U(:,1)));
opti.subject_to(X(:,1)==X0);   % use initial position
for k=2:N % loop over control intervals
    opti.subject_to(X(:,k+1) == f_discrete(X(:,k),U(:,k)));
    opti.subject_to(F*X(:,k) <= f);
    opti.subject_to(U(:,k) <= 1.5*ones(4,1));
    opti.subject_to(U(:,k) >= zeros(4,1));
    cost = cost + (X(:,k)-REF_vect)'*Q*(X(:,k)-REF_vect) + U(:,k)'*R*U(:,k);
end
opti.minimize(cost);


ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end