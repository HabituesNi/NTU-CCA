clear all;
%define the original process transfer function
num = [3];
den = [1,9,27,32,16];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2;
G = zpk(z,p,k,'inputdelay',t_delay)

%define sampleing period which starts from apparent time delay
t_s_start = 3;   %sampling start time
t_s_end = 23;   %sampling stop time
Ts = 0.5;      %sampling interval
s_num = (t_s_end-t_s_start)/Ts;   %total sampling number 40
t_sample = [t_s_start:Ts:t_s_end-Ts];
[y,t] = step(G,t_sample);  %output and time 
 
% use least square method in time domain
% calculate psi,gamma for the least squares method
psi = zeros(s_num,3); %do initialization
psi(1,1) = Ts/2*y(1);
for i =2:s_num-1
      psi(i,1) = psi(i-1,1) + Ts*y(i);
end
psi(s_num,1) = psi(s_num-1,1) + Ts/2*y(s_num);
psi(:,1) = -psi(:,1);
%here use trapezoidal integration rule to calculate the intergral of continuous time signal y(¦Ó)
psi(:,2) = -1; %-A  The magnitude of the input step function is A=1.
psi(:,3) = 1*t; %tA
gamma = y; %y(t)

% use LSM to calculate the parameters theta
theta = ((psi'*psi)^-1)*psi'*gamma;
a1 = theta(1,1);
b1 = theta(3,1);
L  = theta(2,1)/theta(3,1);
 
numb = [b1];
denb  = [1,a1];
Gp = tf(numb,denb,'inputdelay',L)
step(G,Gp,35)                    %do the step test and plot
figure, nyquist(G,Gp,{0.0001,0.56});        %plot the Nyquist chart and compare


