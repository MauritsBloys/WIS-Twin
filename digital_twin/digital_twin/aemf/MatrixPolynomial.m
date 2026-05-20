% Basic templated created by Microsoft Copilot.

classdef MatrixPolynomial
    properties
        Coefficients
        Degree
        Size
    end
    properties (Dependent)
        rowForm
        colForm
    end
    
    methods
        function obj = MatrixPolynomial(varargin)
            if nargin < 1
                return;
            end
            s = size(varargin{1});
            if nargin == 2 && sum(s) > 1 && numel(varargin{2}) == 1
                % Corresponds to initialization using rowForm and degree
                rowForm = varargin{1};
                degree = varargin{2};
                s(2) = s(2)/(degree + 1);
                varargin = mat2cell(rowForm, s(1), s(2)*ones(1, degree+1));
            else
                for i = 2:nargin
                    if ~isequal(size(varargin{i}), s)
                        error('All matrices must be the same size.');
                    end
                end
            end
            obj.Coefficients = varargin;
            obj.Degree = length(varargin) - 1;
            obj.Size = s;
        end

        function value = get.rowForm(obj)
            value = [obj.Coefficients{:}];
        end

        function value = get.colForm(obj)
            value = vertcat(obj.Coefficients{:});
        end

        function value = barrify(obj, order)
            value = barrify(obj.Coefficients, order);
        end

        function result = plus(obj1, obj2)
            sumResult = combineCellArrays(obj1.Coefficients, obj2.Coefficients, @plus);
            result = MatrixPolynomial(sumResult{:});
        end

        function result = horzcat(obj1, obj2)
            catResult = combineCellArrays(obj1.Coefficients, obj2.Coefficients, @horzcat);
            result = MatrixPolynomial(catResult{:});
        end

        function result = minus(obj1, obj2)
            result = obj1 + (-1)*obj2;
        end

        function result = uminus(obj)
            result = (-1)*obj;
        end
        
        function result = mtimes(obj, other)
            if isnumeric(other)  % Multiplication with a scalar
                resultMult = cellfun(@(x) x*other, obj.Coefficients, 'UniformOutput', false);
                result = MatrixPolynomial(resultMult{:});
                return;
            elseif isnumeric(obj)
                resultMult = cellfun(@(x) x*obj, other.Coefficients, 'UniformOutput', false);
                result = MatrixPolynomial(resultMult{:});
                return;                
            end
            % Multiplication with other MatrixPolynomial
            if ~isequal(obj.Size(2), other.Size(1))
                error('Matrix polynomials must have compatible size.');
            end
            coeffs = cell(1,obj.Degree + other.Degree + 1);
            result = MatrixPolynomial(coeffs{:});
            for i = 1:length(obj.Coefficients)
                for j = 1:length(other.Coefficients)
                    if isempty(result.Coefficients{i+j-1})
                        result.Coefficients{i+j-1} = obj.Coefficients{i} * other.Coefficients{j};
                    else
                        result.Coefficients{i+j-1} = result.Coefficients{i+j-1} + obj.Coefficients{i} * other.Coefficients{j};
                    end
                end
            end
            result.Degree = length(result.Coefficients) - 1;
            result.Size = size(result.Coefficients{1});
        end

        function result = ctranspose(obj)
            transposedCellArray = cell(size(obj.Coefficients));
            for i = 1:numel(obj.Coefficients)
                transposedCellArray{i} = obj.Coefficients{i}';
            end
            result = MatrixPolynomial(transposedCellArray{:});
        end

        function result = clean(obj)
            cellArray = obj.Coefficients;
            cellArray = cellfun(@cleanMatrix, cellArray, 'UniformOutput', false);
            nonZeroMatrices = ~cellfun(@(x) all(x==0), cellArray); % Find matrices that are not all zeros
            lastNonZeroMatrixIndex = find(nonZeroMatrices, 1, 'last'); % Get the index of the last non-zero matrix
            if isempty(lastNonZeroMatrixIndex) % Retain the zero matrix
                cellArray{1} = 0*cellArray{1};
                lastNonZeroMatrixIndex = 1;
            end
            cleanArray = cellArray(1:lastNonZeroMatrixIndex);
            result = MatrixPolynomial(cleanArray{:});
        end
        
        function result = tf(obj, q)
            result = obj.Coefficients{1};
            for i = 1:obj.Degree
                result = result + obj.Coefficients{i+1}*q^i;
            end
        end

        function varargout = subsref(obj, S)
            switch S(1).type
                case '{}'
                    if length(S) == 1
                        % Implement obj(i) = obj.Coefficients{i}
                        varargout = {obj.Coefficients{S.subs{:}}};
                    elseif length(S) == 2
                        matrix = obj.Coefficients{S(1).subs{:}};
                        varargout = {builtin('subsref', matrix, S(2))};
                    else
                        % Use built-in subsref for dot notation
                        varargout = {builtin('subsref', obj, S)};
                    end
                otherwise
                    % Use built-in subsref for dot notation
                    [varargout{1:nargout}] = builtin('subsref', obj, S);
            end
        end

        function result = product_diff(obj)
            % Returns a cell array X of MatrixPolynomial where
            % f(t)*obj*w(t) = (sum_i X{i}df^(i)/dt)*w(t)

            % Claim: 
            % fq^n ->sum_k=0^n((-1)^(n-k)*nchoosek(n,k)*q^(n-k)f^(k)
            % Then sum up results

            result = cell(obj.Degree + 1, 1);
            for n = 1:obj.Degree+1
                result{n} = 0*obj;
            end
            for n = 0:obj.Degree
                for k = 0:n
                    % Update q^(n-k) component of f^(k) 
                    result{k+1}.Coefficients{n-k+1} = result{k+1}.Coefficients{n+1}...
                        + (-1)^(n-k)*nchoosek(n,k)*obj.Coefficients{n+1};
                end
            end
        end

        function print(obj)
            strArray = cell(obj.Size);
            for i = 1:obj.Size(1)
                for j = 1:obj.Size(2)
                    str = '';
                    for k = 1:length(obj.Coefficients)
                        if obj.Coefficients{k}(i,j) ~= 0
                            if isempty(str)
                                if k-1 == 0
                                    str = sprintf('%g ', obj.Coefficients{k}(i,j));
                                elseif k-1 == 1
                                    str = sprintf('%gq ', obj.Coefficients{k}(i,j));
                                else
                                    str = sprintf('%gq^%d ', obj.Coefficients{k}(i,j), k-1);
                                end
                            else
                                thiscoeff = obj.Coefficients{k}(i,j);
                                if thiscoeff > 0
                                    str = [str '+ '];
                                elseif thiscoeff < 0
                                    str = [str '- '];
                                    thiscoeff = abs(thiscoeff);
                                end
                                if k-1 == 1
                                    str = [str, sprintf('%gq ', thiscoeff)];
                                else
                                    str = [str, sprintf('%gq^%d ', thiscoeff, k-1)];
                                end
                            end
                        end
                    end
                    if isempty(str)
                        str = '0 ';
                    end
                    strArray{i,j} = str;
                end
            end
            printCellArrayInTable(strArray);
        end
    end
end

function printCellArrayInTable(cellArray)
    % Author: Microsoft Copilot
    % Get the number of rows and columns
    [numRows, numCols] = size(cellArray);

    % Calculate the maximum length of string in each column
    maxLen = zeros(1, numCols);
    for j = 1:numCols
        maxLen(j) = max(cellfun('length', cellArray(:, j)));
    end

    % Print the cell array in tabular format
    for i = 1:numRows
        for j = 1:numCols
            % fprintf('%-*s ', maxLen(j), cellArray{i, j}); % left-justified
            fprintf('%*s  ', maxLen(j), cellArray{i, j});
        end
        fprintf('\n');
    end
end

function sumArray = sumCellArrays(cellArray1, cellArray2)
    % Determine the size of the two cell arrays
    size1 = size(cellArray1);
    size2 = size(cellArray2);

    % Make the two cell arrays the same size by padding the smaller one with zero matrices
    if any(size1 > size2)
        cellArray2(size1(1), size1(2)) = {zeros(size(cellArray1{1}))};
    elseif any(size2 > size1)
        cellArray1(size2(1), size2(2)) = {zeros(size(cellArray2{1}))};
    end

    % Replace empty cells with zero matrices
    emptyCells1 = cellfun(@isempty, cellArray1);
    emptyCells2 = cellfun(@isempty, cellArray2);
    cellArray1(emptyCells1) = {zeros(size(cellArray1{1}))};
    cellArray2(emptyCells2) = {zeros(size(cellArray2{1}))};

    % Sum the two cell arrays element-wise
    sumArray = cellfun(@plus, cellArray1, cellArray2, 'UniformOutput', false);
end

function resArray = combineCellArrays(cellArray1, cellArray2, fun)
    % Determine the size of the two cell arrays
    size1 = size(cellArray1);
    size2 = size(cellArray2);

    % Make the two cell arrays the same size by padding the smaller one with zero matrices
    if any(size1 > size2)
        cellArray2(size1(1), size1(2)) = {zeros(size(cellArray2{1}))};
    elseif any(size2 > size1)
        cellArray1(size2(1), size2(2)) = {zeros(size(cellArray1{1}))};
    end

    % Replace empty cells with zero matrices
    emptyCells1 = cellfun(@isempty, cellArray1);
    emptyCells2 = cellfun(@isempty, cellArray2);
    cellArray1(emptyCells1) = {zeros(size(cellArray1{1}))};
    cellArray2(emptyCells2) = {zeros(size(cellArray2{1}))};

    % Sum the two cell arrays element-wise
    resArray = cellfun(fun, cellArray1, cellArray2, 'UniformOutput', false);
end

function m = cleanMatrix(m)
    m(abs(m)<eps) = 0;
end


% classdef MatrixPolynomial (originally created by Copilot)
%     properties
%         Coefficients
%         Degree
%         Size
%     end
% 
%     methods
%         function obj = MatrixPolynomial(varargin)
%             if nargin < 1
%                 error('At least one matrix must be provided.');
%             end
%             s = size(varargin{1});
%             for i = 2:nargin
%                 if ~isequal(size(varargin{i}), s)
%                     error('All matrices must be the same size.');
%                 end
%             end
%             obj.Coefficients = varargin;
%             obj.Degree = length(varargin) - 1;
%             obj.Size = s;
%         end
% 
%         function result = mtimes(obj, other)
%             if ~isequal(obj.Size, other.Size)
%                 error('Matrix polynomials must be the same size to multiply.');
%             end
%             result = MatrixPolynomial();
%             for i = 1:length(obj.Coefficients)
%                 for j = 1:length(other.Coefficients)
%                     if isempty(result.Coefficients{i+j-1})
%                         result.Coefficients{i+j-1} = obj.Coefficients{i} * other.Coefficients{j};
%                     else
%                         result.Coefficients{i+j-1} = result.Coefficients{i+j-1} + obj.Coefficients{i} * other.Coefficients{j};
%                     end
%                 end
%             end
%             result.Degree = length(result.Coefficients) - 1;
%             result.Size = obj.Size;
%         end
% 
%         function print(obj)
%             for i = 1:obj.Size(1)
%                 for j = 1:obj.Size(2)
%                     str = '';
%                     for k = 1:length(obj.Coefficients)
%                         if obj.Coefficients{k}(i,j) ~= 0
%                             if k-1 == 0
%                                 str = [str, sprintf('%f ', obj.Coefficients{k}(i,j))];
%                             elseif k-1 == 1
%                                 str = [str, sprintf('+ %f*q ', obj.Coefficients{k}(i,j))];
%                             else
%                                 str = [str, sprintf('+ %f*q^%d ', obj.Coefficients{k}(i,j), k-1)];
%                             end
%                         end
%                     end
%                     fprintf('%s\t', str);
%                 end
%                 fprintf('\n');
%             end
%         end
%     end
% end



