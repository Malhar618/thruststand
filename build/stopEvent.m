  function [value, isterminal, direction] = stopEvent(~, ~)
        % Get current real time
        currentTime = toc;
        
        % Check if the maximum real time is reached
        if currentTime >= 4% maxRealTime
            value = 0; % Stop the integration
        else
            value = 1; % Continue integration
        end
        
        isterminal = 1; % Terminate ode45 when the event is triggered
        direction = 0; % Detect event only when the function is increasing
    end