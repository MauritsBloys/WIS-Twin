function u = my_project(u,type,U,m,zero_mean)
% Ensures u(:,i) has norm_type smaller than U*N^(1/type), i=1,...,m

if type ~= 'inf'
    U = U*(length(u/m))^(1/type);

    u = reshape(u,m,[])';
    for i = 1:m
        if zero_mean
            u(:,i) = u(:,i) - mean(u(:,i));
        end
        un = norm(u(:,i),type);
        if un > U
            u(:,i) = U*u(:,i)/un;
        end
    end
    
    u = reshape(u',[numel(u),1]);
else
    u = min(max(u,-U),U);
end