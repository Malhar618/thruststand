%% cost function multiobjective
function J = cost_fun_eval(gainsMatrix,J,IC,n,m,odefun)
    

    
    % Conditions for stopping integration (e.g. if it is taking too long)
    options = odeset('Events', @stopEvent); % Set event function

    % Integration of ODEs
    tic;    
    %[T,Y,te,ye,ie] = ode45(@(t,y)odefun(t,y,z,J,n), [0 15], IC, options);
    response = bash_executor(n, gainsMatrix);
    % Check if integration was stopped by the event function
%     if ~isempty(te)
%         J = 1*10^13;
%     else
% 
%         % Initilize variables to retrieve from odefun output
%         u = zeros(m,length(T));
%         eta_error = zeros(1,length(T));
%         control_effort = zeros(1,length(T));
%         control_effort_ideal = zeros(1,length(T));
% 
%         % Get Auxiliary Outputs from odefun_mck
%         for ii = 1:length(T)
% 
%             [~,u(:,ii),eta_error(:,ii), control_effort(ii), ~, ~, control_effort_ideal(ii)] = odefun(T(ii),Y(ii,:)',z,J,n);
% 
%         end
% 
%         % Compute the derivative using finite differences
% %         u_dot = (diff(u)')./ diff(T);
% %         u_dot = [u_dot; u_dot(end)];
% 
%         % Vectorial Cost Function evaluation
%         Je = sqrt(sum(eta_error));
%         Ju = sqrt(sum(control_effort));
% %         Ju_dot = sqrt(sum(u_dot.^2));
% 
%         % Not accept zero control input solutions
%         %W = diag(w_w, w_u, w_u_dot);
%         Ju_ideal = sqrt( sum( control_effort_ideal ) );
% 
%         %J = 1*Je + 0.1*Ju;
%         J = Je + abs(Ju-Ju_ideal)/Ju_ideal;
          J = load("L2norm.json")
    end
