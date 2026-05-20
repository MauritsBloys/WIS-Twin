lem = 0;  % ergodic mean
for i = 1:Ntries
    M = zeros(p,p);
    for j = 1:N
        Pju = pArray(:,:,j)*u;
        M = M + Pju*Pju';
    end
    %[v,l] = eigs(M,1,'smallestreal','IsSymmetricDefinite',true);  % l contains the current objective function
    [VM,DM] = eig(M);
    [~,im] = min(diag(DM));
    l = DM(im,im);
    v = VM(:,im);
    lvec(i) = l;
    %     if (l < lold) && (i >= min(1000,Ntries/10))
    %         u = uold;
    %         break;
    %     end
    %     if (l-lold)/l < sqrt(eps)
    %         break;
    %     end
    lold = l;
    lemold = lem;
    if i == 1
        lem = l;
    else
        lem = ((i-1)*lem + l)/i;
    end 
    % Now make gradient
    g = zeros(1,N*m);
    gs = zeros(N,N*m);
    for j = 1:N
        Pjv = pArray(:,:,j)'*v;
        gs(j,:) = (2*(Pjv'*u))*Pjv;
        g = g + gs(j,:);
    end
    
    % ascent
    unew = u + tau/(tau+i)*step*g';
    % project
    %unew = 20*unew/norm(unew,2)*sqrt(N);
    unew = my_project(unew,2,bound,m,false);
    %unew = max(min(unew,1),-1);
    if norm(u - unew) <= 1e-3 || abs(lem-lemold) < 1e-5
        break
    end
    
    uold = u;
    u = unew;
end