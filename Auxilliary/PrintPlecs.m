function PrintPlecs(M)

[nr,nc] = size(M);

fprintf('[\n');

for i = 1:nr
    
    fprintf('\t\t\t\t');   % indentation όπως στο παράδειγμά σου
    
    for j = 1:nc
        fprintf('%10.5f ', M(i,j));
    end
    
    if i < nr
        fprintf(';\n');
    else
        fprintf('];\n');
    end

end