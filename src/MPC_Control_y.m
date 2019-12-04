classdef MPC_Control_y < MPC_Control
    
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
            N = 30;
            Q = eye(n);
            R = eye(m);
            
            % Predicted state and input trajectories
            x = sdpvar(n, N);
            u = sdpvar(m, N-1);
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
            %State Constraints
            F = [0 1 0 0;0 -1 0 0];
            f = [0.035;0.035];
            
            %Input Constraints
            M = [1;-1];
            m = [0.3;0.3];
            [Xf,Qf] = Compute_Terminal_Set(M,m,F,f,mpc,Q,R);
            [Ff,ff] = double(Xf);
            
            con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(:,1)) + (M*u(:,1) <= m);
            obj = u(:,1)'*R*u(:,1);
            for i = 2:N-1
                con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
                con = con + (M*u(:,i) <= m);
                obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
            end
            con = con + (Ff*x(:,N) <= ff);
            obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
            
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
            %State Constraints
            F = [0 1 0 0;0 -1 0 0];
            f = [0.035;0.035];
            
            %Input Constraints
            M = [1;-1];
            m = [0.3;0.3];
            
            con = [M*us <= m, F*xs<=f, xs == mpc.A*xs + mpc.B*us, ref == mpc.C*xs];
            obj = (ref-mpc.C*xs)'*(ref-mpc.C*xs);
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
            
        end
    end
end
