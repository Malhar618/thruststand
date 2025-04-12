function stop = checkStagnation(optimValues, ~)
    persistent stagnationCounter
    persistent previousFval
    
    if isempty(stagnationCounter)
        stagnationCounter = 0;
    end
    
    if isempty(previousFval)
        previousFval = Inf;
    end
    
    stop = false;
    if abs(optimValues.fval - previousFval) < eps % Check if fitness hasn't changed
        stagnationCounter = stagnationCounter + 1;
    else
        stagnationCounter = 0; % Reset counter if fitness changed
    end
    previousFval = optimValues.fval; % Update previous fitness value
    if stagnationCounter >= 500
        stop = true; % Stop if stagnation threshold reached
        stagnationCounter = 0;
    end
end

