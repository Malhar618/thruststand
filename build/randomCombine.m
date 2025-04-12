function combined_vector = randomCombine(v1, v2)
    % Check if the input vectors have the same length
    if numel(v1) ~= numel(v2)
        error('Input vectors must have the same length.');
    end
    
    % Initialize the combined vector
    combined_vector = zeros(size(v1));
    
    % Randomly select elements from each vector
    for i = 1:numel(v1)
        % Randomly choose from v1 or v2
        if rand() < 0.5
            combined_vector(i) = v1(i) + rand(1)*10^(floor(log10(v1(i))));
        else
            combined_vector(i) = v2(i) + rand(1)*10^(floor(log10(v2(i))));
        end
    end
end