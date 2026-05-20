function [] = expand_matrix_multiplication_c(constantMatrix, variableName, variableResult, epsilon)
% Expand variable result array = constant matrix * variable array multiplication 

[M, N] = size(constantMatrix);

fprintf("// START - Controller created in MATLAB (epsilon = %d)\n", epsilon);

for i = 1:M
    first = true;
    for j = 1:N
        if abs(constantMatrix(i,j)) > epsilon
            if first
                fprintf("%s[%d] = %d * %s[%d];\n", variableResult, i-1, constantMatrix(i,j), variableName, i-1);
                first = false;
            else
                fprintf("%s[%d] += %d * %s[%d];\n", variableResult, i-1, constantMatrix(i,j), variableName, i-1);
            end
        end
    end
end

fprintf("// END - Controller created in MATLAB\n")
end

