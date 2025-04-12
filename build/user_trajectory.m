function [x_user, x_dot_user, x_ddot_user] = user_trajectory(t)

% step + sinusoid
x_user = 0.6*[1;sin(t);cos(t)];%(t<5) + 0*[3*sin(10*t); 2*sin(8*t); 1*sin(20*t) ]*(t>=5);
x_dot_user = 0.6*[0;cos(t);-sin(t)];%0*[30*cos(10*t); 16*cos(8*t); 20*cos(20*t)]*(t>=5);
x_ddot_user = 0.6*[0;-sin(t);-cos(t)];

