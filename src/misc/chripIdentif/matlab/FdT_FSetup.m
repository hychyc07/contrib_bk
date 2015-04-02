% function [w sysForce]=FdTSetup(Sweep)
function [D T UU FF sysF sysV R1 R2 datas]=FdTSetup(Sweep,N)

%Sweep = Sweep1;
% Nprec=1;
% count=0;
% for i=2:length(Sweep)
%     tmpdatas=[];
%     if(Sweep(i,2)>Sweep(i-1,2)) %2 is the frequency
%         count=count+1;
%         tmpdatas=Sweep(Nprec:i-1,:);
%         if(length(tmpdatas)<N)
%             disp('few datas acquired')
%         else
%             d1=length(tmpdatas);
%             tmpdatas=tmpdatas(d1-N+1:d1,:);
%             %disp('ok')
%             datas(1:N,1:4,count)=tmpdatas;
%         end
%         Nprec=i;
%     end
% end

%Sweep = Sweep1;
Nprec=1;
count=0;
Sweep1 = Sweep;
for i=2:length(Sweep1)
    tmpdatas=[];
    if(Sweep1(i,2)>Sweep1(i-1,2)) %2 is the frequency
        count=count+1;
        tmpdatas=Sweep1(Nprec:i-1,:);
        d1=length(tmpdatas);
        d2 = d1;
        if(d1<N)         
            disp('few datas acquired')
            datas(1:d2,1:4,count)=tmpdatas(1:d2,1:4);
        else
            %tmpdatas=tmpdatas(d1-N+1:d1,:);
            %disp('ok')
            %datas(1:N,1:4,count)=tmpdatas;
            disp('ok')
            datas(1:d2,1:4,count)=tmpdatas(1:d2,1:4);
        end
        Nprec=i;
    end
end

size(datas)
T = 0.010;                    % Sample time
Fs = 1/T;                     % Sampling frequency
UU=[];
FF=[];
lenn = size(datas,3)
 for j=1:lenn
%     fout=(Datas(:,3,j)-Datas(:,2,j))*pi/180;
     u=datas(:,3,j);
     F=datas(:,2,j);
     yv=datas(:,4,j);
     yf=datas(:,4,j);
%      yv=yv-mean(yv);
%      yf=yf-mean(yf);
 
     L = length(u);                     % Length of signal
     NFFT = 2^nextpow2(L); % Next power of 2 from length of y
     f = Fs/2*linspace(0,1,NFFT/2+1);
     UU=[UU u];
     FF=[FF F];
     U = fft(u-mean(u),NFFT)/L;
%      length(U)
    [Cu,Iu] = max(abs(U(1:NFFT/2+1)));
    YV = fft(yv-mean(yv),NFFT)/L;
%     length(YV)
    [Cv,Iv] = max(2*abs(YV(1:NFFT/2+1)));
    YF = fft((yf-mean(yf)),NFFT)/L;
%     length(YF)
    [Cf,If] = max(2*abs(YF(1:NFFT/2+1)));
    if(3*Iu<length(YF))
        A1=2*abs(YF(Iu));
        A2=2*abs(YF(2*Iu));
        A3=2*abs(YF(3*Iu));
    end
    R1(j)=A2/A1;
    R2(j)=A3/A2;
    FreqDatas=[f' 2*abs(U(1:NFFT/2+1)) 2*abs(YV(1:NFFT/2+1)) 2*abs(YF(1:NFFT/2+1))];
    TimeValues=[u yf];
    PowerSpectrum(:,:,j)=FreqDatas;
    TimeDatas(:,:,j)=TimeValues;
    sysF(j)=YF(Iu)/U(Iu);
    sysV(j)=YV(Iu)/U(Iu);
 end


 
 D=PowerSpectrum;
 T=TimeDatas;
%     
%     
%     % om(j)=2*pi*f(Iu);
% end
% disp('fase 2 completed')
% w=2*pi*fc;
% sysForce=sysF;