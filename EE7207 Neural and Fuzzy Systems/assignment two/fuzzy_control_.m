close all
clear
% load model
fuzzy_control_model = readfis('fuzzy_control_reasoning.fis');

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

figure
hold on
grid on
% axis equal

% theta = deg2rad(theta);
k = 1;

while (abs(y) > y_threshold || abs(theta) > theta_threshold)
    indx = indx + 1;
    %infer u
    u = evalfis(fuzzy_control_model, [y theta]);
    
    u = deg2rad(u);
    ustore(k) = u;
    
    theta_rad = deg2rad(theta);
    
    %update theta,x and y
    theta_curr = theta_rad + v * T * tan(u) / L;
    x_curr = x + v * T * cos(theta_rad);
    y_curr = y + v * T * sin(theta_rad);
    
    % update theta,x,y,u
    theta = rad2deg(theta_curr);
    x = x_curr;
    y = y_curr;
    ystore(k) = y;
    thetastore(k) = theta;
    if (mod(indx, 10) == 0)
        fprintf("y is %f\n", y);
        plot(x, y, 'b.-');
        pause(0.01)
    end
    k = k+1;
end

figure(2);
plot (ustore);
set(gca,'XLim',[0 1379]);
xlabel('k');
ylabel('u');
title('the curve of u')

figure(3);
plot (ystore);
set(gca,'XLim',[0 1379]);
xlabel('k');
ylabel('y');
title('the curve of y')

figure(4);
plot (thetastore);
set(gca,'XLim',[0 1379]);
xlabel('k');
ylabel('theta');
title('the curve of theta')
