%close all;

%%
figure; 
subplot(122)
plot(out.p(N+1:end,1), out.p(N+1:end,2:end)); 
%cc(2:3) = cc(2:3)*0;
hold on; 
plot([0 out.p(end,1)], [cc; cc], '--'); 
ylabel('Fault estimate'); 
ylim([-0.2, 0.2]); 
legend({'$\hat{f}_1$', '$\hat{f}_2$', '$\hat{f}_3$'  '$f_1$', '$f_2$', '$f_3$'}, 'Interpreter','latex','NumColumns',2); 
xlabel('Time [s]');
%title('Response of the "best" input (square wave @ 0.9 rad/s)')

%
subplot(121)
plot(out.z1(1:5*N,1), out.z1(1:5*N,2:3)); 
legend({'$u_1$', '$u_2$'}, 'Interpreter','latex'); 
xlabel('Time [s]');

%
set(gcf(),'Position', [680 458 4*200 200])

