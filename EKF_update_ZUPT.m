function [MU,SIGMA] = EKF_update_ZUPT(mu,sigma,z,R)
% mu - input state estimate
% sigma - input state covariance
% z - current measurement
% R - sensor noise


% MU - measurement corrected state estimate
% SIGMA - measurement corrected state covariance

mu = mu';
x1=mu(1);x2=mu(2);x4=mu(4);x5=mu(5);

% Compute expected observation and Jacobian
dx = x1-x4;
dy = x2-x5;
c6 = cos(mu(6)); s6 = sin(mu(6));
pred_z = [-(dy*c6-dx*s6)-0.14; dx*c6+dy*s6];
H = [s6, -c6, 0, -s6, c6, dy*s6+dx*c6;
     c6, s6, 0, -c6, -s6, -dx*s6+dy*c6];


% Kalman gain
K = sigma*H'*inv(H*sigma*H'+R);
% Correction
MU = mu+K*(z-pred_z);
MU(3) = wrapToPi(MU(3));
MU(6) = wrapToPi(MU(6));
MU = MU';

SIGMA = (eye(6)-K*H)*sigma;
