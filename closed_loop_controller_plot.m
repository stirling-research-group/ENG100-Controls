data = readtable('C:\Users\paul\Desktop\Work\MIT\UMichENG100\LectureControls\ENG100-Controls\logs\2021_01_30_15h12m05s_closed_loop.csv');
%%
time = data.time - data.time(1);
e = (data.theta-data.angle_ref);
timestep = .001;%(data.time-data.time_prev);
proportional_term = data.kp.*e;
intergral_term = data.ki.*data.error_sum.*data.time_diff;
derivative_term = data.kd.* (e - data.e_prev)./ data.time_diff;
current_recreated = -1*(proportional_term + intergral_term + derivative_term);
current = data.current_applied;

%%
plot(time, proportional_term,...
    time, intergral_term,...
    time, derivative_term,...
    time, current_recreated, ...
    time, current, ...
    time, e  )

legend('proportional','integral','derivative', 'current recreated', 'current', 'angle error')

%%
% plot(time, current_recreated)