%% Function to compute maximum invariant set within the constraints X
function Xf = maxInvar(A, X)

while 1
  prevX = X;
  [T,t] = double(X);
  preX  = polytope(T*A,t);
  X     = intersect(X, preX);
  
  if isequal(prevX, X)
    break
  end
end
Xf = X;

end