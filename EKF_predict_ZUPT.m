function [MU,SIGMA] = EKF_predict_ZUPT(mu,sigma,u,Q)
% mu - prior state estimate
% sigma - prior state covariance
% u - control input
% Q - motion model noise

% MU - predicted state estimate
% SIGMA - predicted state covariance


mu = mu';
u = u';
xr = mu(1);
yr = mu(2);
thetar = mu(3);
xl = mu(4);
yl = mu(5);
thetal = mu(6);
%% Predict new state mean
MU = mu+[ u(1)*cos(thetar+u(4))-u(2)*sin(thetar+u(4));
          u(2)*cos(thetar+u(4))+u(1)*sin(thetar+u(4));
          u(4);
          u(5)*cos(thetal+u(8))-u(6)*sin(thetal+u(8));
          u(6)*cos(thetal+u(8))+u(5)*sin(thetal+u(8));
          u(8)];
MU(3) = wrapToPi(MU(3));
MU(6) = wrapToPi(MU(6));

MU = MU';
% Jacobian of prediction model w.r.t. state
G = [1 0 -u(1)*sin(mu(3)+u(4))-u(2)*cos(mu(3)+u(4)) 0 0 0;
     0 1 -u(2)*sin(mu(3)+u(4))+u(1)*cos(mu(3)+u(4)) 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 -u(5)*sin(mu(6)+u(8))-u(6)*cos(mu(6))+u(8);
     0 0 0 0 1 -u(6)*sin(mu(6)+u(8))+u(5)*cos(mu(6)+u(8));
     0 0 0 0 0 1];
% Jacobian of prediction model w.r.t. control
V = [cos(thetar+u(4)) -sin(thetar+u(4)) -u(1)*sin(thetar+u(4))-u(2)*cos(thetar+u(4)) 0 0 0;
    sin(thetar+u(4)) cos(thetar+u(4)) -u(2)*sin(thetar+u(4))+u(1)*cos(thetar+u(4)) 0 0 0;
    0 0 1 0 0 0;
    0 0 0 cos(thetal+u(8)) -sin(thetal+u(8)) -u(5)*sin(thetal+u(8))-u(6)*cos(thetal+u(8));
    0 0 0 sin(thetal+u(8)) cos(thetal+u(8)) -u(6)*sin(thetal+u(8))+u(5)*cos(thetal+u(8));
    0 0 0 0 0 1];
 
%% Predict new state covariance
SIGMA = G*sigma*G'+V*Q*V';
          