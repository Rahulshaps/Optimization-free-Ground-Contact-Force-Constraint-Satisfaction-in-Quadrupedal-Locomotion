function [cost1,cost2] = optimizer_lin(x0, ue_opt, uj, A, Be, Bj, Q, R, p)
% A, B linear model
dt = p.dt;

% Explicit Euler: x_(k+1) = x_k + dt*f(x,u)
Ad = dt * A;
Bed = dt * Be;
Bjd = dt * Bj;
xk = x0;
cost1 = 0;
cost2 =zeros([1,4]);
for i = 1:4
  ue = ue_opt(:,i);
  xk = xk + Ad*xk + Bed*ue + Bjd*uj;
  Rb=reshape(xk(16:24),[3,3]);
  Rb_T = Rb.';
  pitch = (asin(-Rb_T(1,3)));
  yaw = (atan2(Rb_T(1,2), Rb_T(1,1)));
  roll = (atan2(Rb_T(2,3), Rb_T(3,3)));
  xe = [pitch roll yaw];
  cost1 = cost1 + xe*Q*xe' + ue'*R*ue;
  %debug 
  cost2(:,i) = cost1;
end
%debug
cost2
end