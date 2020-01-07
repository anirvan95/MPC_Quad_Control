function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 200; % MPC horizon [SET THIS VARIABLE]
h = 0.2;
f_discrete = @(x,u) RK4(x,u,h,quad);

% −−−− decision variables −−−−−−−−−
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%
Au = [1 0 0 0; -1 0 0 0; 0 1 0 0; 0 -1 0 0; 0 0 1 0; 0 0 -1 0; 0 0 0 1; 0 0 0 -1];
bu = repmat([1.5; 0.0], 4, 1);
Q = eye(12);
R = eye(4);

bx = [0.035; 0.035];
AX = zeros(4, 12);
AX(1, 4) = 1; AX(2, 4) = -1; AX(3, 5) = 1; AX(4, 5) = -1;
bX = [bx; bx];

ref_param = [0; 0; 0; 0; 0; REF(4, 1); 0; 0; 0; REF(1:3,1)];

opti.set_value(X0, zeros(12, 1));
% WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      opti.subject_to(X(:, 1) == X0)
      opti.subject_to(X(:, 2) == f_discrete(X(:, 1), U(:, 1)))
      opti.subject_to(Au*U(:, 1) <= bu);
      obj = U(:, 1)'*R*U(:, 1);
      for i=2:N
          opti.subject_to(X(:, i+1) == f_discrete(X(:, i), U(:, i)));
          opti.subject_to(AX*X(:, i) <= bX);
          opti.subject_to(Au*U(:, i) <= bu);
          obj = obj + (X(:, i) - ref_param)'*Q*(X(:, i)- ref_param) + U(:, i)'*R*U(:, i);
      end
      % obj = obj + 0*(X(:, N+1) - ref_param)'*Q*(X(:, N+1)- ref_param);
      opti.minimize(obj);

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U)
% −−−− Set the initial state and reference −−−−
opti.set_value(X0, x);
opti.set_value(REF, ref);

% −−−− Setup solver NLP
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);

% −−−− Solve the optimization problem −−−−
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end