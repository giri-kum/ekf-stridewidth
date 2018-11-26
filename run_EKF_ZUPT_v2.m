function STATE = run_EKF_ZUPT_v2(r_data,l_data,Meas)
% INPUTS
% -r_data is a structure with information about the right foot's trajectory
% estimate. It must contain r_data.P and r_data.FF
% --r_data.P is an array of the x, y, and z position estimates of the right
% foot trajectory. It has 3 columns (x,y,z) and the length is the number of
% samples
% --r_data.FF is a single column array of the same length as r_data.P. It
% has 1's for any sample that is a zero-velocity update point and 0's
% everyewhere else
% -l_data is similar to r_data, but with the left foot's estimates
% -Meas is a two column vector that is the length of r_data.P. The second
% column is all 0's. The first column contains a range measurement whenever
% one is obtained. All other entries are 0.

% NOTE: This code assumes that r_data.P,l_data.P,and Meas are all the same
% length and are all synchronized

% OUTPUTS
% -STATE - The rows of STATE are the estimated state at each sample. Column
% 1 is the x position of the right foot, column 2 is the y position of the
% right foot, column 3 is the heading angle of the right stride. Columns
% 4-6 are similar but for the left foot


RESULTS = Get_cont_initial_meas(r_data.P,r_data.FF,l_data.P,l_data.FF,Meas);
Meas = RESULTS.Meas;

%% Initial guess of state and uncertainty
STATE = RESULTS.Initial;

SIGMA = diag([0.5 0.5 deg2rad(5) 0.5 0.5 deg2rad(5)]);
% Combined control input from right foot and left foot
control = RESULTS.control;

% Noise in motion model
f_t_noise = 8e-5;
s_t_noise = 8e-5;
rot_noise = deg2rad(0.001);

Q = diag([f_t_noise s_t_noise rot_noise f_t_noise s_t_noise rot_noise])^2; % Motion model noise
R = diag([0.00185, 0.1])^2; % Measurement model noise

% R = diag([0.00185, 0.1])^2; % Measurement model noise

% sw is vector of instantaneous stride width estimate
sw = [-((STATE(2)-STATE(5))*cos(STATE(6))-(STATE(1)-STATE(4))*sin(STATE(6)))]; % initial estimated stride width
%% Run filter
for kk = 1:size(control,1)
    % Prediction Update
    [predState,predSigma] = EKF_predict_ZUPT(STATE(end,:),SIGMA,control(kk,:),Q);
    % Measurement Update if data is available
    if Meas(kk,1) ~=0 
       [updateState,updateSigma] = EKF_update_ZUPT(predState,predSigma,Meas(kk,:)',R);
       SIGMA = updateSigma;
       STATE = [STATE;updateState];
    else
        SIGMA = predSigma;
        STATE = [STATE;predState];
    end
    

    % Compute instantaneous stride width
    x1=STATE(kk+1,1);x2=STATE(kk+1,2);x4=STATE(kk+1,4);x5=STATE(kk+1,5);
    dx = x1-x4;
    dy = x2-x5;
    c6 = cos(STATE(kk+1,6)); s6 = sin(STATE(kk+1,6));
    sw = [sw;-(dy*c6-dx*s6)];
end
% Plot Estimate
figure
plot(STATE(:,1),STATE(:,2))
hold on
plot(STATE(:,4),STATE(:,5))
axis equal
legend('Right Foot','Left Foot')
% Plot original
figure
plot(r_data.P(:,1),r_data.P(:,2))
axis equal
hold on
plot(l_data.P(:,1),l_data.P(:,2))
legend('Right Foot','Left Foot')