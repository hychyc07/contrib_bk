function ExecReuseData

load('ReuseData.mat');
load('ReuseDataExt.mat');

%size(R)

% time in contact
timeInFC       = zeros(3,4);
timeInFCStd    = zeros(3,4);

qualFC       = zeros(3,4);
qualFCStd    = zeros(3,4);


timeInContact       = zeros(3,4);
timeInContactStd    = zeros(3,4);

time2FInContact       = zeros(3,4);
time2FInContactStd    = zeros(3,4);

finger_range       = zeros(3,4,4);
finger_rangeStd    = zeros(3,4,4);

shake               = zeros(3,4);
shakeStd            = zeros(3,4);

pressureIn          = zeros(3,4);
pressureInStd       = zeros(3,4);

contactIn           = zeros(3,4);
contactInStd        = zeros(3,4);

presErrIn           = zeros(3,4);
presErrInStd        = zeros(3,4);

presErrOut          = zeros(3,4);
presErrOutStd       = zeros(3,4);

posErrIn           = zeros(3,4);
posErrInStd        = zeros(3,4);

posErrOut          = zeros(3,4);
posErrOutStd       = zeros(3,4);


%all object
for k=1:3
    %all phase
    for p=1:4
        %all trial        
        ids = (k-1)*12 + (p-1) +[1 5 9];

            timeInFC(k,p)       = mean(RExt(ids,1));
            timeInFCStd(k,p)    = std(RExt(ids,1));

            qualFC(k,p)       = mean(RExt(ids,2));
            qualFCStd(k,p)    = mean(RExt(ids,3));


            timeInContact(k,p)      = mean(R(ids,2));
            timeInContactStd(k,p)   = std(R(ids,2));

            time2FinInContact(k,p)      = mean(R(ids,3));
            time2FinInContactStd(k,p)   = std(R(ids,3));

            % each finger
            for i=0:3
                finger_range(k,p,i+1)         = mean(R(ids,4+i));
                finger_rangeStd(k,p,i+1)      = std(R(ids,4+i));
%                R(ids,4+i)
            end

            pressureIn(k,p)          = mean(R(ids,13));
            pressureInStd(k,p)       = std(R(ids,13));

            contactIn(k,p)           = mean(R(ids,14));
            contactInStd(k,p)        = mean(R(ids,15));

            shake(k,p)               = mean(reshape(R(ids,36:39),1,[]));
            shakeStd(k,p)            = std(reshape(R(ids,36:39),1,[]));


            presErrIn(k,p)           = mean(R(ids,16));
            presErrInStd(k,p)        = mean(R(ids,17));

            presErrOut(k,p)          = mean(R(ids,18));
            presErrOutStd(k,p)       = mean(R(ids,19));

            posErrIn(k,p)           = mean(reshape(R(ids,20:23),1,[]));
            posErrInStd(k,p)        = mean(reshape(R(ids,24:27),1,[]));

            posErrOut(k,p)          = mean(reshape(R(ids,28:31),1,[]));
            posErrOutStd(k,p)       = mean(reshape(R(ids,32:35),1,[]));


        end
    end
end

if 0
presErrMix      = [presErrIn(:,[1 2]) presErrOut(:,[1 2])];
presErrMixStd   = [presErrInStd(:,[1 2]) presErrOutStd(:,[1 2])];

posErrMix      = [posErrIn(:,[1 2]) posErrOut(:,[1 2])];
posErrMixStd   = [posErrInStd(:,[1 2]) posErrOutStd(:,[1 2])];
end



figure(1);
clf;
subplot(2,4,1);
plotBoxA('Time in contact', timeInContact,timeInContactStd ,0.5,1.1)

%subplot(2,4,2);
%plotBoxA('Time 2F in contact', time2FinInContact,time2FinInContactStd ,0.7,1.1)
    
subplot(2,4,2);
plotBoxA('Shake', shake,shakeStd,0.05,0.2,1)

subplot(2,4,3);
plotBoxA('Pressure in var', pressureIn,pressureInStd,0,4)

subplot(2,4,4);
plotBoxA('Pressure Err in', presErrIn,presErrInStd,0,20,1)


for i=1:4
    subplot(2,4,4+i);
    if (i==1)
        plotBoxA(sprintf('finger %d range',i), finger_range(:,:,i),finger_rangeStd(:,:,i),20,140,1)
    else
        plotBoxA(sprintf('finger %d range',i), finger_range(:,:,i),finger_rangeStd(:,:,i),10,70,1)
    end
end

figure(2);
clf;
subplot(2,2,1);
plotBoxA('FC in contact', timeInFC,timeInFCStd ,0.5,1.1,1);
subplot(2,2,2);
plotBoxA('Time in contact', timeInContact,timeInContactStd ,0.5,1.1)
subplot(2,2,3);
plotBoxA('FC Qual', qualFC,qualFCStd ,0,0.002);



if 0
figure(2);
clf;
subplot(1,2,1);
plotBoxA('Pressure Error (in/out)', presErrMix,presErrMixStd, 0,14)

subplot(1,2,2);
plotBoxA('Position Error (in/out)', posErrMix,posErrMixStd,0,0.7)

end

if 0
figure(2);
clf;
subplot(2,2,1);
plotBoxA('Pres err In', presErrIn,presErrInStd,0,20)

subplot(2,2,2);
plotBoxA('Pres err Out', presErrOut,presErrOutStd,0,20)

subplot(2,2,3);
plotBoxA('Pos err In', posErrIn,posErrInStd,0,1)

subplot(2,2,4);
plotBoxA('Pos err Out', posErrOut,posErrOutStd,0,1)

end


if 0
    Stat_InRatio

    Stat_TimeInContact

    Stat_MaxInPosRange
    Stat_MaxPosRange

    Stat_MeanInPressure
    Stat_StdInPressure

    Stat_MeanInContact
    Stat_StdInContact

    Stat_MeanInPressureError
    Stat_StdInPressureError
    Stat_MeanOutPressureError
    Stat_StdOutPressureError

    Stat_MeanInPosError
    Stat_StdInPosError
    Stat_MeanOutPosError
    Stat_StdOutPosError

    Stat_MeanPosShakiness
    Stat_StdPosShakiness
end



function plotMatA(name, A,AStd,miny,maxy)
    hold on;
    N = size(A,1);
    K = size(A,2);


    colors = ['b','g','r','c'];

    X = (1:K);

    for i=1:N
        plot(X,A(i,:),colors(i));
    end
    AMean = mean(A);    
    plot(X,AMean,'k','LineWidth',2);




    if(nargin>2)
        for i=1:N
            for k=1:K
                plot([X(k) X(k)],[A(i,k)-AStd(i,k) A(i,k)+AStd(i,k)],colors(i));
            end
        end
        AMeanStd = sqrt(1/N*sum(A.*A + AStd.*AStd)   - AMean.*AMean);
        for k=1:K
            plot([X(k) X(k)],[AMean(1,k)-AMeanStd(1,k) AMean(1,k)+AMeanStd(1,k)],'k');
        end
    end

    if(nargin>3)
        axis([0.5 K+0.5 miny maxy]);
    end
    title(name);
end


function plotBoxA(name, A,AStd,miny,maxy,prec)
 
   hold on;
    N = size(A,1);
    K = size(A,2);


    colors = ['b','g','r','c'];

    X = (1:K)-1;

    AMean = mean(A);    

    ObjSize     = 1/8;
    MeanSize    = 1/4;

    OffSize     = 1/16;
    SpaceSize   = 1- (OffSize + N*ObjSize + MeanSize);
    
    if(nargin<=3)
        miny = 0;
    end

    if(nargin>2)
        for i=1:N
            for k=1:K
                plotBox(X(k) - 0.5 + (SpaceSize/2+ObjSize/2) +(i-1)*(ObjSize),ObjSize,A(i,k),AStd(i,k),miny,colors(i),1)
         end
        end
        AMeanStd = sqrt(1/N*sum(A.*A + AStd.*AStd)   - AMean.*AMean);
        for k=1:K
            plotBox(X(k) - 0.5 + (SpaceSize/2+ObjSize) +(N-1)*(ObjSize) + MeanSize/2+OffSize,MeanSize,AMean(1,k),AMeanStd(1,k),miny,'k',2)
        end
    end

    if(nargin>3)
        axis([-0.5 K-0.5 miny maxy]);
    end
    title(name);
    set(gca,'XTick',[]);
    set(gca,'XTickLabel',[]);

    disp(name);    
    if(nargin>5)
        for i=1:N
            txt = sprintf('%d    ',i);        
            for k=1:K
                txt = [txt sprintf('$%.1f$&$%.1f$& ',A(i,k),AStd(i,k))];
            end
            disp(txt);
        end
        txt = 'Mean ';
        for k=1:K
            txt = [txt sprintf('$%.1f$&$%.1f$& ',AMean(1,k),AMeanStd(1,k))];
        end
        disp(txt);
    else
        for i=1:N
            txt = sprintf('%d    ',i);        
            for k=1:K
                txt = [txt sprintf('%f +- %f\t',A(i,k),AStd(i,k))];
            end
            disp(txt);
        end
        txt = 'Mean ';
        for k=1:K
            txt = [txt sprintf('%f +- %f\t',AMean(1,k),AMeanStd(1,k))];
        end
        disp(txt);
    end
    disp('');

end

function plotBox(X,W,V,Std,base,color,linewidth)
    plot([X-W/2 X+W/2 X+W/2 X-W/2 X-W/2],[base base V V base],color);
    %patch([X-W/2 X+W/2 X+W/2 X-W/2 X-W/2],[base base V V base],color);
    plot([X X],[V V+Std],color,'lineWidth',linewidth);
    plot([X-W/3 X+W/3],[V+Std V+Std],color,'lineWidth',linewidth);
end
