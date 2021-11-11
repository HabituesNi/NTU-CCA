clear all;
%define the original process transfer function
num = [3];
den = [1,9,27,32,16];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2;
G = zpk(z,p,k,'inputdelay',t_delay)

%define sampleing period
t_s_start = 0;   %sampling start time
t_s_end = 20;   %sampling stop time
Ts = 0.5;      %sampling interval
s_num = (t_s_end-t_s_start)/Ts;   %total sampling number 40
t_sample = [t_s_start:Ts:t_s_end-Ts];
[y,t] = step(G,t_sample);  %output and time 

% use least square method in frequency domain
% how to calculate the G(jw)
syms w;
g(w) = (y(s_num)+w*trapz(t,(step(G,t)-y(s_num)).*sin(w*t)))+1i*w*trapz(t,(step(G,t)-y(s_num)).*cos(w*t));

%initialization
%number of the frequency response to be identified in the frequency range (0, ¦Ø c ) 
% set M = 10
% set the variable
M = 10;  
fai = zeros(1,M);
w = zeros(1,M);
psi = zeros(M,2);
gamma = zeros(M,1);

% set the initial w1 w2
W(1) = 0.0; W(2) = 0.001;
fai(1) = 0.0; fai(2) = angle(g(W(2)));

%recursive solution
%determine the w(i+1) magnitude and phase
for n=3:M
    W(n) = W(n-1)-((n-1)*pi/(M-1)+fai(n-1))*(W(n-1)-W(n-2))/(fai(n-1)-fai(n-2));
    fai(n) = phase(g(W(n)));
end

%calculate parameters a1 and b1 through LSM method.
for n=1:M
    psi(n,1) = -((real(g(W(n))))^2+(imag(g(W(n))))^2);
    psi(:,2) = 1;
    gamma(n) = ((real(g(W(n))))^2+(imag(g(W(n))))^2)*(W(n)^2);
end
theta = ((psi'*psi)^-1)*psi'*gamma;
a1 = sqrt(theta(1));
b1 = sqrt(theta(2));

%calculate parameters L through LSM method.
psi2 = W';
gamma2 = zeros(M,1);
for n=1:M
    gamma2(n) = -fai(n)-atan(W(n)/a1);
end
L = ((psi2'*psi2)^-1)*psi2'*gamma2;

num = [b1];
den  = [1,a1];
Gp = tf(num,den,'inputdelay',L)
step(G,Gp,35)                    %do the step test and plot
figure, nyquist(G,Gp,{0.0001,0.56});        %plot the Nyquist chart and compare
