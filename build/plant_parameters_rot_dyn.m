function [J,IC] = plant_parameters_rot_dyn()

    J = diag([1,1,1]);

    % Initial condition : remember to augment the ICs to account for the
    % integrator dynamics
    % Define Initial Conditions for ODEs
    eta0 = [0;0;0]; 
    omega0 = [0;0;0];
    int_e_eta_0 = [0;0;0];
    %int_e_omega_0 = [0;0;0];

    IC = [eta0; omega0; int_e_eta_0];% int_e_omega_0];