function double_support = Compute_double_support(r_data,l_data,r_events,l_events)
% Code computes instances of double support for use in our validation.
% INPUTS
% -r_data - structure with r_data.P as an array of the x, y, and z position 
% estimates of the right
% foot trajectory. It has 3 columns (x,y,z) and the length is the number of
% samples
% -l_data = similar to r_data but for the left foot
% -r_events - structure with r_events.HS being an array of indices of all
% right heel strikes and r_events.TO being an array of indices of all right
% toe offs
% -l_events - similar to r_events but for left foot


% OUTPUTS
% double_support - array of indices of all times of identified double
% support

r_stance  = zeros(size(r_data.P,1),1);
l_stance  = zeros(size(l_data.P,1),1);

r_swing = 0;
l_swing = 0;

r_events.HS = r_events.HS+5;
l_events.HS = l_events.HS+5;
r_events.TO = r_events.TO-10;
l_events.TO = l_events.TO-10;
r_events.TO(r_events.TO<1) = 1;
l_events.TO(l_events.TO<1) = 1;


all_HS = [r_events.HS;l_events.HS];
start = min(all_HS);

for i = start:length(r_stance)
    if ismember(i,r_events.HS)
        r_swing = 0;
    elseif ismember(i,r_events.TO)
        r_swing = 1;
    end
    
    if r_swing == 0
        r_stance(i) = 1;
    end
    
    if ismember(i,l_events.HS)
        l_swing = 0;
    elseif ismember(i,l_events.TO)
        l_swing = 1;
    end
    
    if l_swing == 0
        l_stance(i) = 1;
    end

end

r_stance(1:r_events.HS(2)) = 0;
l_stance(1:l_events.HS(2)) = 0;
r_stance(r_events.TO(end-1):end) = 0;
l_stance(l_events.TO(end-1):end) = 0;

r_st = find(r_stance);
l_st = find(l_stance);

double_support = intersect(r_st,l_st);

% Check if this is what you want
figure
plot(r_data.P(:,1),r_data.P(:,2))
hold on;
plot(r_data.P(double_support,1),r_data.P(double_support,2),'x')

plot(l_data.P(:,1),l_data.P(:,2))

plot(l_data.P(double_support,1),l_data.P(double_support,2),'x')