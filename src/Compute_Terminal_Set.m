function [Xf,Qf] = Compute_Terminal_Set(M,m,F,f,mpc,Q,R)
%COMPUTE TERMINAL SET and Terminal Cost

% Compute LQR controller for unconstrained system
[K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
% MATLAB defines K as -K, so invert its signal
K = -K; 
% Compute maximal invariant set
Xf = polytope([F;M*K],[f;m]);
Acl = [mpc.A+mpc.B*K];
while 1
    prevXf = Xf;
    [T,t] = double(Xf);
    preXf = polytope(T*Acl,t);
    Xf = intersect(Xf, preXf);
    if isequal(prevXf, Xf)
        break
    end
end
% disp('Plotting Projection')
% if(size(mpc.A,2)>2)
%     figure,
%     subplot(1,3,1),Xf.projection(1:2).plot('alpha',0.9,'linewidth',3,'color','r');
%     hold on
%     subplot(1,3,2),Xf.projection(2:3).plot('alpha',0.9,'linewidth',3,'color','g');
%     subplot(1,3,3),Xf.projection(3:4).plot('alpha',0.9,'linewidth',3,'color','b');
% else
%     figure,
%     Xf.plot('alpha',0.2,'linewidth',3,'color','k')
% end

end

