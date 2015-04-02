% Written By Matteo Fumagalli
% Calculate fdt of open loop system and validate the repetability of the
% acquisitions


clear all
close all
clc
%% Files:

%no_gommini_standardN.dat: L'identificazione e' stata fatta dal tavolo, coi
%gommini neri, N=1:5
load command_log3.txt
output1=command_log3;

%%
% il file e' composto da tutta l'acquisizione:
% counter . frequenza*10 . u . velocita' . forza

% output1(:,5)=output1(:,5)*800/4096;

Sweep1=[output1(:,1) 1./output1(:,4) output1(:,6) output1(:,7)];


N=800;

disp('files letti!!!')
% La mia funzione figa fa questo:
% divide il file in N sottomatrici, ognuna delle quali ha un forzamento a 
% k Hz.
% calcola fft dell'ingresso e delle uscite, per ogni sottomatrice
% calcola FdT considerando solo l'armonica di eccitazione 
[D1 T1 u1 f1 sysF1 sysV1 R1 R2 datas]=FdT_FSetup(Sweep1,N); 


% D  e' composta dalle sottomatrici con la freq di eccitazione i moduli di ingr, vel e fza
% u e' l'ingresso nel tempo
% f sn frequenze ogni colonna ha un valore di freq*10
% sysF e' la f5dt della fza
% sysV e' la fdt della velocita'


%% FdT

%%%% A seconda del file utilizzato, sciegliere la FdT

%% %%% case: output.txt approssimato bene da ftd di ordine 1

% sysgF1 = frd(sysF1(1,:),f1(1,:).*10);
sysgF1 = frd(sysF1(1,:),f1(1,:)./0.159);

h=figure
%set(cstprefs.tbxprefs,'FrequencyUnits','Hz')
% bode(sysgF1,'-x'),grid, legend('comando-Encoder carico')
bode(sysgF1,'-x'),grid)
% saveas(h,'test.bmp') 
% saveas(h,'test.fig') 
% 
% Fs = 1000;                 
% T = 1/Fs;                  
% L = length(T2);              
% time = (0:L-1)*T; 
% NFFT = 2^nextpow2(L); % Next power of 2 from length of y
% 
% f = [Fs/2*linspace(0,1,NFFT/2+1)]';
% G=1:100;
% T2=T3;
% for k=1:size(T2,3)
% INk(:,k)=(T2(:,1,k)-mean(T2(:,1,k)))/max(T2(:,1,k)-mean(T2(:,1,k)));
% OUTk(:,k)=(T2(:,2,k)-mean(T2(:,2,k)))/max(T2(:,2,k)-mean(T2(:,2,k)));
% Yink(:,k) = fft(INk(:,k),NFFT)/L;
% Youtk(:,k) = fft(OUTk(:,k),NFFT)/L;
