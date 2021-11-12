
x = [0.1 0.0 0.2 0.8 0.3 0.0 0.5 0.6 0.0 0.1 0.3 0.1 0.2 0.2 0.1 0.2;
    0.7 0.5 0.2 0.1 0.0 0.4 0.0 0.3 0.5 0.6 0.2 0.5 0.0 0.6 0.7 0.4;
    0.2 0.5 0.2 0.0 0.4 0.0 0.4 0.0 0.1 0.0 0.1 0.4 0.2 0.1 0.1 0.2;
    0.0 0.0 0.4 0.1 0.3 0.6 0.1 0.1 0.4 0.3 0.4 0.0 0.6 0.1 0.1 0.2];

x = x';

m = 4;
n = 16;

R1 = ones(n, n);

% compute R
for i = 1 : n
    for j = 1 : n
        % numerator and denom
        num = 0;
        s1 = 0;
        s2 = 0;
        for k = 1 : m
            num = num + x(i, k) * x(j, k);
            s1 = s1 + x(i, k) * x(i, k);
            s2 = s2 + x(j, k) * x(j, k);
        end
        num = abs(num);
        dem = sqrt(s1 * s2);
        R1(i, j) = num/dem;
    end
end

% compute R with R1
R_curr = fuzzy_composition(R1);
while ~isequal(R_curr, R1)
    R1 = R_curr;
    R_curr = fuzzy_composition(R1);
end

%% alpha = 0.4 0.8
R4 = zeros(n, n);
R8 = zeros(n, n);
for i = 1 : n
    for j = 1 : n
        if (R1(i, j) >= 0.4)
            R4(i, j) = 1;
        end
        if (R1(i, j) >= 0.8)
            R8(i, j) = 1;
        end
    end
end

%% could be bugness
for alpha = 0.8 : 0.005 : 1.0
    R_curr = zeros(n, n);
    for i = 1 : n
        for j = 1 : n
            if (R1(i, j) >= alpha)
                R_curr(i, j) = 1;
            end
        end
    end
    % evaluate
    column_sum = cumsum(R_curr);
    if (length(unique(column_sum(end, :))) == 3)
        fprintf("Proper alpha found with %f\n", alpha);
        return;
    end
end