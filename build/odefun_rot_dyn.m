function [dy,u,eta_error,control_effort ,x_user,x_dot_user, control_effort_ideal] = odefun_rot_dyn(t,y,z,J,n)

    % Initialize vector of dynamical equations
    dy = zeros(length(y));

    % Z = optimzation variable
    Kp_eta = diag(z(1:3));
    Ki_eta = diag(z(4:6));
    Kd_eta = diag(z(7:9));
   % Ki_omega = diag(z(10:12));
    
    % Retrieve State of the System 
    eta = y(1:n/2);
    omega = y(n/2+1:n);
    int_e_eta = y(n+1:n+n/2);
    %int_e_omega = y(n+n/2+1:2*n);

    %input
    [x_user, x_dot_user, x_ddot_user] = user_trajectory(t);

    % Rotational Kinematic Jacobian
    H = [1 sin(eta(1))*tan(eta(2)) cos(eta(1))*tan(eta(2));...
         0 cos(eta(1)) -sin(eta(1));...
         0 sin(eta(1))/cos(eta(2)) cos(eta(1))/cos(eta(2))];
    % Inverse of Jacobian
    H_inv = [1 0 -sin(eta(2));...
             0 cos(eta(1)) sin(eta(1))*cos(eta(2));...
             0 -sin(eta(1)) cos(eta(1))*cos(eta(2))];
    
    % Rotational Kinematics ODE
    deta = H*omega; %Kinematics

    % Derivative of inverse of Jacobian
    H_dot_inv = [0 0 -cos(eta(2))*deta(2); ...
                 0 -sin(eta(1))*deta(1) -sin(eta(2))*sin(eta(1))*deta(2)+cos(eta(2))*cos(eta(1))*deta(1);...
                 0 -cos(eta(1))*deta(1) -sin(eta(1))*cos(eta(2))*deta(1)-cos(eta(1))*sin(eta(2))*deta(2)];

    % Rotational Kinematic Error
    e_eta = eta - x_user;
    e_eta_dot = deta - x_dot_user;
    
    % Desired angular velocity omega
%     w_cmd = H_inv*( -Kp_eta*e_eta - Ki_eta*int_e_eta + x_dot_user);
%     w_cmd_dot = H_dot_inv*( -Kp_eta*e_eta - Ki_eta*int_e_eta + x_dot_user ) + ...
%                 H_inv*( -Kp_eta*e_eta_dot - Ki_eta*e_eta + x_ddot_user ); 
    
    % Rotational Dynamics Error
   % e_omega = omega - w_cmd;

    % Control Input
%     u = cross(omega, J*omega) + J*( w_cmd_dot - Kp_omega*e_omega - Ki_omega*int_e_omega ) ;


    u = x_ddot_user - Kp_eta*e_eta - Kd_eta*e_eta_dot - Ki_eta*int_e_eta;

    w_cmd = H_inv*x_dot_user;
    w_cmd_dot = H_dot_inv*x_dot_user + H_inv*x_ddot_user;

    u_ideal = cross(  w_cmd, J*w_cmd ) + J*w_cmd_dot;

%     u_ideal = cross( omega , J*omega ) + J*w_cmd_dot; 

    % Dynamical model
    domega = J\( u - cross(omega,J*omega) );
    dint_e_eta = e_eta;
    %dint_e_omega = e_omega;
    
    % Define variables that help us evaluate the performance of the system
    % with a specific set of gains
    eta_error = norm(e_eta)^2; % We aim to minimize attitude error
    control_effort = norm(u,2)^2;
    control_effort_ideal = norm(u_ideal,2)^2;

    % Define Output of the function ( System's ODEs )
    dy = [deta; domega; dint_e_eta]; %dint_e_omega];
