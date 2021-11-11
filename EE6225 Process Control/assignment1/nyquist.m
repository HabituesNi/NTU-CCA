
%define the original process transfer function
num = [0.188];
den = [1.09,1];
% transfer the function to zpk model
[z,p,k]=tf2zp(num,den);
t_delay = 2.016;
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
M = 50;  
fai = zeros(1,M);
finalpart = zeros(1,M);

% set the initial w1 w2
W(1) = 0.0; W(2) = 0.001;
fai(1) = 0.0; fai(2) = angle(g(W(2)));

%recursive solution
%determine the w(i+1) magnitude and phase
for n=3:M
    W(n) = W(n-1)-((n-1)*pi/(M-1)+fai(n-1))*(W(n-1)-W(n-2))/(fai(n-1)-fai(n-2));
    fai(n) = phase(g(W(n)));
end

for n=1:M
    finalpart(n) = g(W(n));
end
plot (finalpart);
hold on;