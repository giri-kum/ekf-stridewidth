function RESULTS = Get_cont_intial_meas(R_P,R_FF,L_P,L_FF,Measure)
% This function generates control inputs for the EKF formulation from
% individual right and left foot trajectory estimates from ZUPT-based foot
% trajectory algorithms. It also gets rid of measurements obtained during
% turning, where the measurement model is not accurate in our EKF
% formulation. It also provides an initial guess to the state and gets rid
% of measurements, etc. taken during standing at the beginning or end of a
% trial. 

[R_control,R_theta,R_D_theta R_SL] = Get_Control(R_P,R_FF);
[L_control,L_theta, L_D_theta,L_SL] = Get_Control(L_P,L_FF);

% Find index of first step greater than 0.5 meter
minSL = 0.5;
control_start = min([find(R_SL>minSL&R_SL<1.5);find(L_SL>minSL&L_SL<1.5)]); % Fix this!!!
control_end = max([find(R_SL>minSL&R_SL<1.5);find(L_SL>minSL&L_SL<1.5)]);

% Don't use measurements during turning
dtheta_thresh = 0.2;
for ii = 1:length(Measure)
    if abs(R_D_theta(ii))>dtheta_thresh | abs(L_D_theta(ii))>dtheta_thresh
        Measure(ii,:) = [0 0];
    end
end

new_meas = Measure(control_start:control_end,:);

RESULTS.control = [R_control(control_start:control_end,:),L_control(control_start:control_end,:)];
RESULTS.Meas = new_meas;
RESULTS.Initial = [0 -0.2 R_theta(control_start) 0 0 L_theta(control_start)];


function [control, theta_long, d_theta_long, SL] = Get_Control(P, FF)
% P is 3 axis position data
% F is locations of footfalls
x_raw = P(:,1);
y_raw = P(:,2);
z_raw = P(:,3);

ff_indices = find(FF);
ff_indices = [1;ff_indices;length(x_raw)];
ff_indices = unique(ff_indices);
theta = zeros(length(ff_indices)-1,1);


dx = zeros(length(x_raw)-1,1);
dy = zeros(length(y_raw)-1,1);
dz = zeros(length(z_raw)-1,1);
dtheta = zeros(length(x_raw)-1,1);

theta_long = [];
SL = zeros(length(x_raw),1);
for ii = 1:length(ff_indices)-1
    delta_x = x_raw(ff_indices(ii+1))-x_raw(ff_indices(ii));
    delta_y = y_raw(ff_indices(ii+1))-y_raw(ff_indices(ii));
    theta(ii) = atan2(delta_y,delta_x); % Heading angle of current stride
    theta_long = [theta_long;theta(ii)*ones(ff_indices(ii+1)-ff_indices(ii),1)]; % Stride heading angle at moment
    SL(ff_indices(ii)) = sqrt(delta_x^2+delta_y^2);
end
theta_long = [theta(1);theta_long]; % Check if this should be tacked on at beginning or end

for jj = 1:length(x_raw)-1
    delta_x = x_raw(jj+1)-x_raw(jj);
    delta_y = y_raw(jj+1)-y_raw(jj);
    dx(jj) = delta_x*cos(theta_long(jj+1))+delta_y*sin(theta_long(jj+1));
    dy(jj) = delta_y*cos(theta_long(jj+1))-delta_x*sin(theta_long(jj+1));
    dz(jj) = z_raw(jj+1)-z_raw(jj);
    dtheta(jj) = theta_long(jj+1)-theta_long(jj);
end
control = [dx,dy,dz,dtheta];

d_theta_long = zeros(length(x_raw),1);
d_theta_long(1) = dtheta(1);
for kk = 2:length(dtheta)
    if dtheta(kk)~=0
        d_theta_long(kk) = dtheta(kk);
    else
        d_theta_long(kk) = d_theta_long(kk-1);
    end
end

