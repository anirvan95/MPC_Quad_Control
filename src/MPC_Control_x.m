classdef MPC_Control_x < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system
      % Constraints on states and inputs
      Ax = [0 1 0 0; 0 -1 0 0];
      bx = [0.035; 0.035];
      Au = [1; -1];
      bu = [0.3; 0.3];
      
      % Objective selection and terminal set selection
      Q = eye(n);
      R = eye(m);
      Noise = zeros(n, m);
      
      % LQR for the terminal set
      [Klqr, Slqr, ~] = dlqr(mpc.A, mpc.B, Q, R, Noise);
      Klqr = -Klqr;
      
      Ax_lqr = [Ax; Au*Klqr];
      bx_lqr = [bx; bu];
      Xf = maxInvar(mpc.A+mpc.B*Klqr, polytope(Ax_lqr,bx_lqr));
      [Af, bf] = double(Xf);      

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = (x(:, 2) == mpc.A*x(:, 1) + mpc.B*u(:, 1)) + (Au*u(:, 1) <= bu);
      obj = u(:, 1)'*R*u(:, 1);
      for i=2:N-1
          con = con + (x(:, i+1) == mpc.A*x(:, i) + mpc.B*u(:, i));
          con = con + (Ax*x(:, i) <= bx) + (Au*u(:, i) <= bu);
          obj = obj + (x(:, i) - xs)'*Q*(x(:, i)-xs) + u(:, i)'*R*u(:, i);
      end
      con = con + (Af*x(:, N) <= bf);
      obj = obj + (x(:, N)-xs)'*Slqr*(x(:, N)-xs);
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      Ax = [0 1 0 0; 0 -1 0 0];
      bx = [0.035; 0.035] + Ax*xs;
      Au = [1; -1];
      bu = [0.3; 0.3];
      
      Rs = eye(size(us, 1));
      con = (xs == (mpc.A*xs + mpc.B*us)) + (ref == mpc.C*xs);      
      con = con + (Ax*xs <= bx) + (Au*us <= bu);
      obj = us'*Rs*us;
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
