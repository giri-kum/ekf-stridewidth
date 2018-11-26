The primary algorithm is called "run_EKF_ZUPT_v2". The expected inputs and outputs are listed below 

INPUTS:
-r_data is a structure with information about the right foot's trajectory estimate. It must contain r_data.P and r_data.FF
--r_data.P is an array of the x, y, and z position estimates of the right foot trajectory. It has 3 columns (x,y,z) and the length is the number of samples
--r_data.FF is a single column array of the same length as r_data.P. It has 1's for any sample that is a zero-velocity update point and 0's everyewhere else
-l_data is similar to r_data, but with the left foot's estimates
-Meas is a two column vector that is the length of r_data.P. The second column is all 0's. The first column contains a range measurement whenever one is obtained. All other entries are 0.

NOTE: This code assumes that r_data.P,l_data.P,and Meas are all the same length and are all synchronized

OUTPUTS:
-STATE - The rows of STATE are the estimated state at each sample. Column 1 is the x position of the right foot, column 2 is the y position of the right foot, column 3 is the heading angle of the right stride. Columns 4-6 are similar but for the left foot

The system state is based x, y, and theta from both the left and right legs, which results in a 6x1 sized matrix. The control input, u, is also 6x1 and can be represented by delta x, delta y, and delta z of both the left and right legs.

A function"Get_cont_initial_meas" is initially called to generate control inputs for the EKF formulation from individual right and left foot trajectory estimates (R_P and L_P respectively) and footfalls (R_FF and L_FF respectively) from ZUPT-based foot trajectory algorithms. It also gets rid of measurements obtained during turning, where the measurement model is not accurate in our EKF formulation. It also provides an initial guess to the state and gets rid of measurements, etc. taken during standing at the beginning or end of a trial. 

In calculating the prediction mu and sigma, the motion model is needed. The "EKF_predict_ZUPT" function is referenced. Here, the motion model is calculated by the previous "mu" plus the relevant parameters from the input and state to get "MU". The theta values must be kept within the bounds of -pi to pi, hence the need for the "minimzedAngle" function. The Jacobian is also found and is represented by "G", which is used to calculate the predicted "SIGMA".

Next, the update step occurs with "EKF_update_ZUPT". The measurement model, "pred_z", takes in the state variables, and the Jacobian of this matrix is also found ("H"). The innovation matrix, "K", is found next, followed by "MU" (no longer the prediction MU) and after some minimization of the angles, then the covariance, "SIGMA", is found. The variables "MU" and "SIGMA" are returned at time t and are used for the next iteration of the EKF process, now at t-1.


Additionally, the function "Compute_double_support.m" is provided. This requires the same r_data and l_data structures necessary for the other files. It also requires as input two structures r_events and l_events. r_events.HS being an array of indices of identified right heel strikes and r_events. TO being an array of indices of identified right toe offs. l_events is a similar structure but for the left foot. The ouput is an array of indices of double support, used for our validation.

The function "Get_error_validation" computes the mean and standard deviation of the stride metrics for the three sections of the experimental trial. It requires the estimated state vector in the complete trajectory along with output of Compute_double_support().

For validation purposes, the estimated trajectories can plotted and recorded every 50 data samples using the "record_video" function. The input to this function is the estimated state vector for the trajectory.

Sample data is provided also provided. Rfoot_data.mat, Lfoot_data.mat, and Meas.mat provide the required r_data, l_data, and Meas arrays necessary as inputs for the "run_EKF_ZUPT_v2" function.

Additionally r_events.mat and l_events.mat provide data structures with r_events.HS being an array of indices of identified right heel strikes and r_events. TO being an array of indices of identified right toe offs. l_events is a similar structure but for the left foot. These are used to identify instances of double support using the function "Compute_double_support".

The Arduino code for the distance measurement device is called "Ultrasonic_distance.ino". To run the ultrasonic sensor with data recording in the SD card, the TRIG and ECHO pins of the ultrasonic sensor needs to connect with port 2 and 3 respectively; the MOSI, MISO, CLK and CS pins of the SD shield need to connect with port 11, 12, 13 and 10 respectively.