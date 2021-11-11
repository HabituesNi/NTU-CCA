clc;
clear;
%select y1-u1¡¢y2-u2¡¢y3-u3 as control plan
G = [tf([-0.98],[12.5 1],'InputDelay',17), tf([-0.36],[15 1],'InputDelay',27), tf([-0.14],[15.2 1],'InputDelay',32);
    tf([-0.43],[14.7 1],'InputDelay',25),  tf([-0.92],[13 1],'InputDelay',16), tf([-0.11],[15.6 1],'InputDelay',33);
    tf([-0.12],[15 1],'InputDelay',31), tf([-0.16],[15 1],'InputDelay',34), tf([-1.02],[11.8 1],'InputDelay',16)]; 


G_size = size(G);
for i = 1:G_size(1)
    for j = 1:G_size(2)
    K(i,j) = G(i,j).num{1}(end);
    L(i,j) = G(i,j).InputDelay + G(i,j).ioDelay;
    T(i,j) = G(i,j).den;
        if size(T{i,j}) == [1 2]
            K_N(i,j) = K(i,j)/( T{i,j}(1)+L(i,j));
        else
            K_N(i,j) = K(i,j)/( T{i,j}(2)+L(i,j));
        end
    end
end

%calculate RGA and NI
Lambda = K.*(inv(K))';
NI = det(K)/(K(1,1)*K(2,2)*K(3,3));

%RNGA
Lambda_N = K_N.*(inv(K_N))';
Gamma=Lambda_N./Lambda;

%ETF With Integrity Rules
[ETF_IR, PID_IR] = eft_PID_integrity_rules(G,Lambda,Gamma);

%ETF Without Integrity Rules
ETF = eft(G,Lambda,Gamma);

sim('decentralized_simulation');
sim1=simout;
% figure(1),plot(simout);

X = zeros(3,3);
for i = 1:3
    for j = 1:3 
        X(i,j) = abs(Lambda_N(i,j)/ Lambda_N(i,i));
    end
end

sim('spases_simulation_new');
sim2=simout;
% figure(2),plot(simout);

% %Decoupling Control with Integrity Rules
% G_R=[tf([ETF_IR(1,1).num{1}],[ETF_IR(1,2).den{1}],'InputDelay',ETF_IR(1,2).InputDelay + ETF_IR(1,2).ioDelay), 0, 0;
%     0,  tf([ETF_IR(2,2).num{1}],[ETF_IR(2,2).den{1}],'InputDelay',ETF_IR(2,1).InputDelay + ETF_IR(2,1).ioDelay), 0;
%     0, 0, tf([ETF_IR(3,3).num{1}],[ETF_IR(3,3).den{1}],'InputDelay',ETF_IR(3,1).InputDelay + ETF_IR(3,1).ioDelay)]; 
% 
% for i = 1:G_size(1)
%     for j = 1:G_size(2)
%         G_hat_I(i,j) =tf(G_R(j,j).num{1}(end)/ETF_IR(j,i).num{1}(end)*ETF_IR(j,i).den{1},G_R(j,j).den{1},'InputDelay',abs(G_R(j,j).InputDelay)+abs(G_R(j,j).ioDelay) - abs(ETF_IR(j,i).InputDelay)- abs(ETF_IR(j,i).ioDelay));
%     end
% end
% 
% PID_decoupling = PID_controller(G_R);
% sim('decoupling_simulation_integrity_rules');
% sim3 = simout;
% figure(3),plot(simout);

% Decoupling Control without Integrity Rules
G_R=[tf([ETF(1,3).num{1}],[ETF(1,1).den{1}],'InputDelay',ETF(1,3).InputDelay + ETF(1,3).ioDelay), 0, 0;
    0,  tf([ETF(2,3).num{1}],[ETF(2,2).den{1}],'InputDelay',ETF(2,2).InputDelay + ETF(2,2).ioDelay), 0;
    0, 0, tf([ETF(3,2).num{1}],[ETF(3,3).den{1}],'InputDelay',ETF(3,3).InputDelay + ETF(3,3).ioDelay)]; 

for i = 1:G_size(1)
    for j = 1:G_size(2)
        G_hat_I(i,j) =tf(G_R(j,j).num{1}(end)/ETF(j,i).num{1}(end)*ETF(j,i).den{1},G_R(j,j).den{1},'InputDelay',abs(G_R(j,j).InputDelay)+abs(G_R(j,j).ioDelay) - abs(ETF(j,i).InputDelay)- abs(ETF(j,i).ioDelay));
    end
end

PID_decoupling = PID_controller(G_R);
sim('decoupling_simulation');
sim4=simout;
% figure(4),plot(simout);
% 
% %BLT Control
% figure(5),margin(-G(1,1));
% figure(6),margin(-G(2,2));
% figure(7),margin(G(3,3));
% figure(8),bode(-G(1,1));
% figure(9),bode(-G(2,2));
% figure(10),bode(G(3,3));

%Ku Wu
allmargin_ = [allmargin(-G(1,1)),allmargin(-G(2,2)),allmargin(G(3,3))];
GM = [-allmargin_(1).GainMargin(1),-allmargin_(2).GainMargin(1),allmargin_(3).GainMargin(1)];
GMF = [allmargin_(1).GMFrequency(1),allmargin_(2).GMFrequency(1),allmargin_(3).GMFrequency(1)];

% %Ku Wu
% % [-74.131024130091730,-1.074855294407441,3.147748314101317]
% % GM = [-1/10^(37.4/-20), -1/10^(0.627/-20), 1/10^(9.96/-20)];
% % GMF = [2.21, 0.154, 1.78];
% 
% K_ZN = GM./2.2;
% T_I_ZN = 2*pi./(1.2*GMF);
% T_D_ZN = 2*pi./(8*GMF);
%     
% min_error=999;
% for F=2.0:0.05:5
%     K_C = K_ZN/F;
%     T_I = F*T_I_ZN;
%     K_I = K_C./T_I;
%     max_Lc = 0;
%     Gc=[tf([K_C(1) K_I(1)],[1 0]),0,0;
%         0,tf([K_C(2) K_I(2)],[1 0]),0;
%         0,0,tf([K_C(3) K_I(3)],[1 0])];
%     for w=0.1:0.02:2.5
%         W = -1+det(eye(3)+freqresp(G*Gc,w));
%         Lc = 20*log10(abs(W/(1+W)));
%         if(Lc>max_Lc)
%              max_Lc=Lc;
%         end
%     end
%     error = abs(max_Lc-6);
%     if(error<min_error)
%     min_error=error;
%     F_match=F;
%     end 
% end
% 
% %F_match = 2.05;
% K_C = K_ZN./F_match;
% T_I = F_match.*T_I_ZN;
% K_I = K_C./T_I;

% min_error_2=999;
% for FD=2.0:0.5:5
%     T_D = T_D_ZN/FD;
%     K_D=K_C.*T_D;
%     max_Lc_2 = 0;
%     Gc=[tf([K_C(1) K_I(1)],[1 0]),0,0;
%         0,tf([K_D(2) K_C(2) K_I(2)],[0 1 0]),0;
%         0,0,tf([K_D(3) K_C(3) K_I(3)],[0 1 0])];
%     for w=0.1:0.05:2.5
%         W = -1+det(eye(3)+freqresp(G*Gc,w));
%         Lc = 20*log10(abs(W/(1+W)));
%         if(Lc>max_Lc_2)
%              max_Lc_2=Lc;
%         end
%     end
%     error = abs(max_Lc_2-6);
%     if(error<min_error_2)
%     min_error_2=error;
%     F_match_D=FD;
%     end 
% end
% 
% F_match_D = 5;
% K_C = K_ZN./F_match;
% T_I = F_match.*T_I_ZN;
% K_I = K_C./T_I;
% T_D = T_D_ZN/F_match_D;
% K_D = K_C.*T_D;
% 
% min_error_3=999;
% for F=2.0:0.05:5
%     K_C = K_ZN/F;
%     T_I = F*T_I_ZN;
%     K_I = K_C./T_I;
%     T_D = T_D_ZN/F_match_D;
%     K_D = K_C.*T_D;
%     max_Lc_3 = 0;
%      Gc=[tf([K_C(1) K_I(1)],[1 0]),0,0;
%         0,tf([K_D(2) K_C(2) K_I(2)],[0 1 0]),0;
%         0,0,tf([K_D(3) K_C(3) K_I(3)],[0 1 0])];
%     for w=0.1:0.05:2.5
%         W = -1+det(eye(3)+freqresp(G*Gc,w));
%         Lc = 20*log10(abs(W/(1+W)));
%         if(Lc>max_Lc_3)
%              max_Lc_3=Lc;
%         end
%     end
%     error = abs(max_Lc_3-6);
%     if(error<min_error_3)
%     min_error_3=error;
%     F_match_2=F;
%     end 
% end
% 
% F_match_2 =2;
% K_C = K_ZN./F_match_2;
% T_I = F_match_2.*T_I_ZN;
% K_I = K_C./T_I;
% T_D = T_D_ZN/F_match_D;
% K_D = K_C.*T_D;
% 
% min_error_4=999;
% for FD=2.0:0.5:5
%     T_D = T_D_ZN/FD;
%     K_D=K_C.*T_D;
%     
%     max_Lc_4 = 0;
%     Gc=[tf([K_C(1) K_I(1)],[1 0]),0,0;
%         0,tf([K_D(2) K_C(2) K_I(2)],[0 1 0]),0;
%         0,0,tf([K_D(3) K_C(3) K_I(3)],[0 1 0])];
%     for w=0.1:0.05:2.5
%         W = -1+det(eye(3)+freqresp(G*Gc,w));
%         Lc = 20*log10(abs(W/(1+W)));
%         if(Lc>max_Lc_4)
%              max_Lc_4=Lc;
%         end
%     end
%     error = abs(max_Lc_4-6);
%     if(error<min_error_4)
%     min_error_4=error;
%     F_match_D_2=FD;
%     end 
% end
% 
% 
% K_C = K_ZN./F_match_2;
% T_I = F_match_2.*T_I_ZN;
% K_I = K_C./T_I;
% T_D = T_D_ZN/F_match_D_2;
% K_D = K_C.*T_D;
% 
sim('decentralized_simulation_BLTtwo');
sim5=simout;
%figure(11),plot(simout);
% 
% sim('decentralized_simulation_BLT_withKD');
% figure(12),plot(simout);
% 
figure(12),plot(sim1.Time,getcolumn(sim1.Data,1),'b','LineWidth',6);hold on;
plot(sim2.Time,getcolumn(sim2.Data,1),'r','LineWidth',4);hold on;
plot(sim4.Time,getcolumn(sim4.Data,1),'g','LineWidth',3);hold on;
plot(sim5.Time,getcolumn(sim5.Data,1),'y','LineWidth',2);hold on;
legend('decentralized','sparce','decoupling','BLT') 

figure(13),plot(sim1.Time,getcolumn(sim1.Data,2),'b','LineWidth',6);hold on;
plot(sim2.Time,getcolumn(sim2.Data,2),'r','LineWidth',4);hold on;
plot(sim4.Time,getcolumn(sim4.Data,2),'g','LineWidth',3);hold on;
plot(sim5.Time,getcolumn(sim5.Data,2),'y','LineWidth',2);hold on;
legend('decentralized','sparce','decoupling','BLT') 

figure(14),plot(sim1.Time,getcolumn(sim1.Data,3),'b','LineWidth',6);hold on;
plot(sim2.Time,getcolumn(sim2.Data,3),'r','LineWidth',4);hold on;
plot(sim4.Time,getcolumn(sim4.Data,3),'g','LineWidth',3);hold on;
plot(sim5.Time,getcolumn(sim5.Data,3),'y','LineWidth',2);hold on;
legend('decentralized','sparce','decoupling','BLT') 
% 
% 
% 
