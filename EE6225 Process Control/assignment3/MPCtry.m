clc;
clear;
close all

% set the process
G = [tf([-0.98],[12.5 1],'InputDelay',17), tf([-0.36],[15 1],'InputDelay',27), tf([-0.14],[15.2 1],'InputDelay',32);
    tf([-0.43],[14.7 1],'InputDelay',25),  tf([-0.92],[13 1],'InputDelay',16), tf([-0.11],[15.6 1],'InputDelay',33);
    tf([-0.12],[15 1],'InputDelay',31), tf([-0.16],[15 1],'InputDelay',34), tf([-1.02],[11.8 1],'InputDelay',16)]; 

inputnum = 3;

% discretize the Gs with sampling time Ts 
Ts = 5;
Gz = c2d(G,Ts);

% to get the state space model convenient for MPC design, use the
% absorbDelay command
Gz = absorbDelay(Gz);
% get the state space model 
Gz = ss(Gz);
Ap = Gz.A; Bp = Gz.B; Cp = Gz.C; Dp = Gz.D;
nx = size(Ap,1);

% get the model of plant which MPC needs
Model = ss(Ap, Bp, Cp, Dp, Ts);
A = Model.A; B = Model.B; C = Model.C;

% MPC tuning parameters
N1 = 1;
N2 = 6;
Nu = 4;
Lambda = 0.01;

% Matrix phi  output horizon
Phi = [];
for i = 1:N2-N1+1
    Phi = [Phi; C*A^i];
end

% Matrix G  control horizon
g = [];
G = [];
for i = 1:N2-N1+1
    if i < Nu
        for j = 1:i
            g = [g C*(A^(i-j))*B];
        end
        g = [g zeros(3,3*(Nu-i))];
    else
        for j = 1:Nu
            g = [g C*(A^(i-j))*B];
        end
    end
    G = [G; g];
    g = [];
end

% track time
tim = 150;
setpt = [0 ones(1,tim); zeros(1,50) ones(1,101); zeros(1,100)  ones(1,51);];
setpt = setpt';

for k=1:tim
    if (mod(k,10)==0), k, end   %track time
    wk = setpt(k,:);
    if k>1 
        xk = x;
    else 
        xk = zeros(nx,1);
    end
    % set x k  
    yk = C * xk;
    % solve y k 
    
    % unconstrained MPC
    W = ones(size(Phi,1),1) * wk;
    U = (G'*G + Lambda * eye(Nu*inputnum, Nu*inputnum))\G' * (W-Phi * xk);
    % solve the uk this moment
    uk = U(1,:);
    % apply the u(k) to the plant 
    x = Ap * xk + Bp * uk';
    
    % Save the plant nums
    y_plot(k,:) = yk;
    u_plot(k,:) = uk;
    w_plot(k,:) = wk;
    
end 

% picture
figure
subplot(211)
plot([1:tim],y_plot(:,1),'LineWidth',1)
hold on
plot([1:tim],y_plot(:,2),'LineWidth',1)
hold on
plot([1:tim],y_plot(:,3),'LineWidth',1)
grid
xlabel('Time t/sec')
ylabel('Output y')
legend('y1','y2','y3')
title('MPC response for ts=5sec, N1=1, N2=6, Nu=4, Lambda=0.01')

subplot(212)
stairs(u_plot(:,1),'LineWidth',1.5)
hold on
stairs(u_plot(:,2),'LineWidth',1.5)
hold on
stairs(u_plot(:,3),'LineWidth',1.5)
grid
ylabel('Control signal u')
xlabel('Time t/sec')
legend('u1','u2','u3')
title('Control signal vesus times t')