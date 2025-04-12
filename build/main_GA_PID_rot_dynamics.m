%% **Main - GA tuning - PID - 2nd Order System **

close all
clear all
clc

%% **Plant Paremeters** %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [A,B,M,IC] = plant_parameters_threeD;
[J,IC] = plant_parameters_rot_dyn;
odefun = @odefun_rot_dyn;

n = 6;
m = 3;
% Rotational dynamics is a square system, i.e. n=m

%Open Loop response for step input
% sys = ss(A,B,eye(2),zeros(size(B)));
% step(sys)

% u = Kp*e + Kd*e_dot + Ki*\int_{t_0}^{t} e \{\rm d}\tau where Kp,Kd,Ki will be size n \times n 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% **Bound on Gains** %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HP assume in the multidimensional case, gain matrices are diagonal
Ntm = 3; % Number of tunable matrices = 2 : Ki, Kp
max_factor = 8;
lb = zeros(m*Ntm,1); % Lower bounds 
ub = ones(m*Ntm,1)*10^max_factor;   % Upper bounds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% **z = [Kp, Kd, Ki]** %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial population matrix
N = 20; % Population size
% z = [Kp, Kd, Ki] is the Optimization Variable
% Automate the first rows along indepndent directions
P0 = zeros(N,length(lb));
i = 1;
factor = -1;
while (i <= N) && (factor <= max_factor-1)
    factor = ceil(i/2) - 1; 
    P0(i,:) = ( 5*double((~mod(i,2))) + 1*double((mod(i,2))) ) %* 10^(factor); 
    i = i + 1;
end
independent_directions = i-1;
% Create other ones through random combinations of previously initilized
% individuals
for jj = independent_directions+1:N % population size
    a = randi([1 independent_directions]);
    b = randi([1 independent_directions]);
    P0(jj,:) = randomCombine(P0(a,:),P0(b,:));
end

% Set GA parameters
options = optimoptions('ga');

options.InitialPopulationMatrix = P0; % Set initial population
options.MaxGenerations = 5;         % Maximum number of generations: each generation is composed of 60 individuals. Each individual/chromosome is a set of adaptive rates
options.PopulationSize = N;          % Population size
options.MutationFcn = {@mutationadaptfeasible, 0.8}; % Mutation function and probability of mutation happening. Mutation = Random change
options.CrossoverFcn = @crossoverscattered;          % Crossover function. Individuals are created from combination of selected perents
options.SelectionFcn = 'selectionroulette';          % Selection function: each individual has a probability of being selected based on its fitness ranking position
options.EliteCount = 5;                               % Number of elite individuals: Number of best performing individuals that is kept unvaried in the following generation
options.FitnessScalingFcn = 'fitscalingrank';         % Fitness scaling function: To limit the influence of outliers, we do not use absolute fitness scores but their ranking position
options.MaxTime = 3600;                               % Maximum time in seconds
options.PlotFcn = {'gaplotbestf','gaplotselection'};  % Plot function for best fitness


disp('*****************************************************************')
disp('**Genetic Algorithm Starting****************************************')
disp('*****************************************************************')

n_resets = 0;

for i = 1:(n_resets+1)
figure
    total_time_i_ga = tic;
    if i ~= 1
        % For even numbers compute a random initial population, for odd
        % iterations initial population will be a LC of the P0 initilized
        % outside of the for cycle in order to make sure all orders of
        % magnitude are investigated
        if mod(i,2) ~= 1
            options.InitialPopulationMatrix(independent_directions+1:end-1,:) =  rand(N-independent_directions-1, length(lb)).*(10.^(randi([0, max_factor], N-independent_directions-1, length(lb)))); 
        else
            for jj = independent_directions+1:N % population size
                a = randi([1 independent_directions]);
                b = randi([1 independent_directions]);
                options.InitialPopulationMatrix(jj,:) = randomCombine(P0(a,:),P0(b,:));
            end
        end
        % At each iteration carry over the fittest individual
        load('x0_file.mat')
        options.InitialPopulationMatrix(end,:) = x;
    end

    
    % Run the genetic algorithm
    [x, fval] = ga(@(gainsMatrix)cost_fun_eval(gainsMatrix,J,IC,n,m,odefun), length(lb), [], [], [], [], lb, ub, [], options)
    
    save x0_file x

    total_time_f_ga(i) = toc(total_time_i_ga);

end

% disp('\n *****************************************************************\n')
% disp('**Simulated Annealing Starting****************************************')
% disp('\n *****************************************************************\n')


% Perform local search improve the global solution coming from GA: 5 *
% 10000 instead of 50000 at once to ensure more iterations in case the algo
% is stopped for improvement tolerance criteria

options = optimoptions('simulannealbnd', 'MaxIterations', 80, 'PlotFcn','saplotbestf');

n_resets_local = 0;
% for jj = 1:(n_resets_local+1)
%     total_time_i_sa = tic;
%     if jj~= 1
%         load('x0_file.mat', 'x');
%     end
% % Moving randomly in a neighborhood and accepting worsening solutions with
% % a certain probability that deacreases over time
%     [x, ffval] = simulannealbnd(@(z)cost_fun_eval(z,J,IC,n,m,odefun), x, lb, ub, options)
% 
%     % Save current solution for next iteration, this is where the best set
%     % of gains is loaded, send these to matlab tcp port
% 
%     save('x0_file.mat', 'x');
% 
%     total_time_f_sa(jj) = toc(total_time_i_sa);
% end

% options = optimoptions('simulannealbnd', 'MaxIterations', 10000, 'PlotFcn','saplotbestf','OutputFcn', @checkStagnation);
% 
% for jj = 1:5
%     total_time_i_sa = tic;
%     if jj ~= 1
%         load('x0_file.mat', 'x');
%     end
% 
%     % Run simulated annealing
%     [x, ffval, exitflag, output] = simulannealbnd(@(z)cost_fun_eval(z,p), x, lb, ub, options);
%     
%     % Save current solution for next iteration
%     save('x0_file.mat', 'x');
%     
%     total_time_f_sa(jj) = toc(total_time_i_sa);
% end

%%
% clear all
% [A,B,IC] = plant_parameters_oneD;
% odefun = @odefun_mck;
%[A,B,IC] = plant_parameters_threeD;

% n = size(A,1); % state dimension
% m = size(B,2); % input dimension
% Visualization
% cost_fun_eval(xx,p)
% just the optimal set of gains is loaded onto this file 
load('x0_file.mat')

[T,Y] = ode45(@(t,y)odefun(t,y,x,J,n), [0 45], IC);


for ii = 1:length(T)
%    [~,u(ii),e(ii,:),r(ii),da_sat(ii)] = odefun(T(ii),Y(ii,:)',x,p);
   [~,u(:,ii),e_eta(:,ii),control_effort(ii),x_user(:,ii),x_dot_user(:,ii)] = odefun(T(ii),Y(ii,:)',x,J,n);
end

legends = {'$$\phi(t)$$','$$\theta(t)$$', '$$\psi(t)$$'};
for i = 1:(n/2)

    figure()
    plot(T,Y(:,i), '-k', T,x_user(i,:),'--r', 'LineWidth',2)
    xlabel('$$T[s]$$',Interpreter='latex')
    ylabel('$$x(t)$$',Interpreter='latex')
    legend(legends{i},[legends{i},'$$_{\rm user}$$'], Interpreter = 'latex')


    figure()
    plot(T,u(i,:)*180/pi,'--k','Linewidth',2)%,T,da_sat*180/pi,'--r','Linewidth',1.2)
    grid on
    xlabel('$$T[s]$$',Interpreter='latex')
    ylabel('$$u(t)$$',Interpreter='latex')
%     legend('$$\delta_{a_{cmd}}$$','$$\delta_{a}$$', Interpreter = 'latex')
end

%% send the best population of gains to the flightstack on the odroid
% pass these to the software Jack made to port between the odroid and the 
% Matlab file





% figure()
% subplot(1,2,1)
% plot(T,Y(:,1), '-k', T,x_user(1,:),'--r', 'LineWidth',2)
% subplot(1,2,2)
% % plot(T,Y(:,2), '-k', T,x_dot_user,'--r', 'LineWidth',2)




% Run Time plot

% figure()
% subplot(1,2,1)
% plot(1:length(total_time_f_ga), total_time_f_ga, '*r')
% xlabel('$$Iteration$$', 'Interpreter','latex', FontSize=13)
% ylabel('$$RunTime[s]$$', 'Interpreter','latex', FontSize=13)
% grid on
% 
% subplot(1,2,2)
% plot(1:length(total_time_f_sa), total_time_f_sa, '*r')
% xlabel('$$Iteration$$', 'Interpreter','latex', FontSize=13)
% ylabel('$$RunTime[s]$$', 'Interpreter','latex',FontSize=13)
% grid on


%% **COMPARISON FUNCTION**

% if length(ub) == 3
% 
%     % Generate sample data
%     Zrand = rand(100, length(lb)).*(100.^(randi([0, 8], 10, length(lb)))); 
%     load("x0_file.mat");
% 
%     Z = [Zrand; x];
% 
%     for i = 1:length(Z(:,1))
%         c(i) = cost_fun_eval(Z(i,:),A,B,M,IC,n,m,odefun);  
%     end
%     [min_c,i_c] = min(c);
% 
%     indices = find(c<100000);
%     figure()
%     plot3(Z(end,1),Z(end,2),Z(end,3),'or',MarkerSize=15)
%     hold on
%     plot3(Z(i_c,1),Z(i_c,2),Z(i_c,3),'ob',MarkerSize=11)
% %     legend('GA best', 'Random best')
%     hold on
%     % Plot the 3D scatter plot with color representing the fourth dimension
%     for jj = 1:length(indices)
%         scatter3(Z(indices(jj),1), Z(indices(jj),2), Z(indices(jj),3), 45, c(indices(jj)), 'filled');
%         hold on
%     end
%     grid on
%     colormap('jet'); % Set the colormap (e.g., 'jet', 'hot', 'cool', etc.)
%     colorbar; % Display a color bar indicating the mapping of values to colors
%     title('3D Scatter Plot with Color representing the cost function');
%     xlabel('$$\gamma_x$$', Interpreter='latex');
%     ylabel('$$\gamma_r$$', Interpreter='latex');
%     zlabel('$$\gamma_{\theta}$$', Interpreter='latex');
% 
% end
