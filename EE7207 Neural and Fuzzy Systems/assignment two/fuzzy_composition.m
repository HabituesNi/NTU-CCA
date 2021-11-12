function [R] = fuzzy_composition(R1)

n = length(R1);
R = ones(n, n);
for i = 1 : n
    vec = R1(i, :);
    for j = 1 : n
        col = R1(j, :)';
        v_intersect = [];
        for k = 1 : n
            v_intersect = [v_intersect min(vec(k), col(k))];
        end
        R(i, j) = max(v_intersect);
    end
end
end