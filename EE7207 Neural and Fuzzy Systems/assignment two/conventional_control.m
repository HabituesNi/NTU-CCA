close all
clear 

%inputs
case_num = 4;
switch case_num
    case 1
        theta = 0;
        y = 40;
        x = 20;
    case 2
        theta = 90;
        y = -30;
        x = 10;
    case 3
        theta = -140;
        y = 30;
        x = 40;
    case 4
        theta = -10;
        y = 10;
        x = 50;
    otherwise
        disp("Error!");
        return;
end

%fixed param
L = 2.5;
T = 0.1;
v = 0.5;

%tunabel param (stopping criteria)
y_threshold = 0.1;
theta_threshold = 0.01;

indx = 0;

x_vec = x;
y_vec = y;
theta_vec = theta;
u_vec = 0;

% control param
K_y = 4.5;

figure
hold on
grid on
axis on



while ((abs(y) > y_threshold || abs(theta) > theta_threshold) && indx < 10000)
    indx = indx + 1;
    % compute error of theta and y
    e_theta = 0 - theta;
    e_y = 0 - y;
    
    e_theta = e_theta / 360;
    e_y = e_y / 200;
    
    u = K_y * (e_y + e_theta);
%     
    u_vec = [u_vec u];
%     
%     u = deg2rad(u);
    theta_rad = deg2rad(theta);
    
    %update theta,x and y
    theta_curr = theta_rad + v * T * tan(u) / L;
    x_curr = x + v * T * cos(theta_rad);
    y_curr = y + v * T * sin(theta_rad);
    
    % update theta,x,y,u
    theta = rad2deg(theta_curr);
    x = x_curr;
    y = y_curr;
    
    theta_vec = [theta_vec theta];
    x_vec = [x_vec x];
    y_vec = [y_vec y];

    if (mod(indx, 10) == 0)
        plot(x, y, 'b.-');
        pause(0.01);
    end
end

%%
figure(2);
plot (u_vec);
set(gca,'XLim',[0 4296]);
xlabel('k');
ylabel('u');
title('the curve of u')

figure(3);
plot (y_vec);
set(gca,'XLim',[0 4296]);
xlabel('k');
ylabel('y');
title('the curve of y')

figure(4);
plot (theta_vec);
set(gca,'XLim',[0 4296]);
xlabel('k');
ylabel('theta');
title('the curve of theta')