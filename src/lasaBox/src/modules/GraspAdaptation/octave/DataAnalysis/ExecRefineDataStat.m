function ExecRefineDataStat

load('RefinementData.mat');

N = 3;

% time in contact

timeInContact       = zeros(4*N,5);
timeInContactStd    = zeros(4*N,5);

time2FInContact       = zeros(4*N,5);
time2FInContactStd    = zeros(4*N,5);

finger_range       = zeros(4*N,5,4);
finger_rangeStd    = zeros(4*N,5,4);

shake               = zeros(4*N,5);
shakeStd            = zeros(4*N,5);

pressureIn          = zeros(4*N,5);
pressureInStd       = zeros(4*N,5);

contactIn           = zeros(4*N,5);
contactInStd        = zeros(4*N,5);

presErrIn           = zeros(4*N,5);
presErrInStd        = zeros(4*N,5);

presErrOut          = zeros(4*N,5);
presErrOutStd       = zeros(4*N,5);

posErrIn           = zeros(4*N,5);
posErrInStd        = zeros(4*N,5);

posErrOut          = zeros(4*N,5);
posErrOutStd       = zeros(4*N,5);

mtic = [];


%all object
for k=1:4
    %all phase
    for p=1:5
        %all trial        
        ids = (k-1)*15 + p+[0 5 10];
        kids = [(k-1)*N+1:k*N];


            timeInContact(kids,p)      = R(ids,2);

            % each finger
            for i=0:3
                finger_range(kids,p,i+1)         = R(ids,4+i);
            end

            pressureIn(kids,p)          = (R(ids,13));

            shake(kids,p)               = (mean(R(ids,36:39)'));

            presErrIn(kids,p)           = (R(ids,16));

        end
    end
end


CalcPVal2('Time in contact', timeInContact,"!=");
CalcPVal2('Shake          ', shake,"!=");
CalcPVal2('Pressure Err in', presErrIn,"!=");
for i=1:4
    CalcPVal2(sprintf('finger %d range ',i), finger_range(:,:,i),"!=");
end

function R = CalcAnova(name,A)
    
    Obj = repmat(reshape(repmat([1:4]',1,3)',[],1),1,5);
    Exp = repmat([1:5],12,1);
    %Obj = Obj*0+1;

    cols = [2 3;4 5; 1 3; 3 5];
    
    rows = 1:12;
    P = [];
    for i=1:4
        P = [P;anovan(reshape(A(rows,cols(i,:)),[],1)',[reshape(Obj(rows,cols(i,:)),[],1) reshape(Exp(rows,cols(i,:)),[],1) ] ,'model','interaction')'];        
    end

    str =[];
    for mr=P(:,1)'
        if(mr<0.001) 
            str = [str "+"];
        else 
            if(mr<0.01)  
                str = [str "*"];
            else
                str = [str "-"];
            end
        end
    end

    disp(sprintf("%s: %c%c %c%c",name,str))   

    disp(sprintf("\tExp.      \tObject   \tInteract.",mr))   
    i =0;
    for mr=P'
        disp(sprintf("%c\t%f\t%f\t%f",'a'+i,mr))   
        i++;
    end    

%    A   = A(1:12,[1 3]);
%    Obj = Obj(1:12,[1 3]);
%    Exp = Exp(1:12,[1 3]);

%    A   = A([1:3],[1 3]);
%    Obj = Obj([1:3],[1 3]);
%    Exp = Exp([1:3],[1 3]);

%    size(A)
%    [reshape(Exp,[],1) reshape(Obj,[],1) ]
%[reshape(Obj,[],1) reshape(Exp,[],1) ]
%    reshape(Obj,[],1)
%    anovan(reshape(A,[],1)',[reshape(Exp,[],1)] ,'model','full');
%  anovan(reshape(A,[],1)',[reshape(Obj,[],1) reshape(Exp,[],1) ] ,'model','interaction');

%   anova(reshape(A,[],1)',[reshape(Exp,[],1) ]);
    %anovan(reshape(A,[],1),[reshape(Obj,[],1) reshape(Exp,[],1) ] );
end

function R = CalcPVal2(name,A,alt)
    CalcAnova(name,A);
    disp("--------------------");
    return
    for i=0:4
        if(i<4)
            rg=i*3+[1:3];
        else
            rg=1:12;
        end

        if 1
        R = [t_test_2(A(rg,2),A(rg,3),alt) ...
             t_test_2(A(rg,4),A(rg,5),alt) ...
             t_test_2(A(rg,1),A(rg,3),alt) ...
             t_test_2(A(rg,3),A(rg,5),alt) ...
            ];
        else
        R = [welch_test(A(rg,2),A(rg,3),alt) ...
             welch_test(A(rg,4),A(rg,5),alt) ...
             welch_test(A(rg,1),A(rg,3),alt) ...
             welch_test(A(rg,3),A(rg,5),alt) ...
            ];
        end
        str =[];
        for mr=R
            if(mr<0.01) 
                str = [str "+"];
            else 
                if(mr<0.05)  
                    str = [str "*"];
                else
                    str = [str "-"];
                end
            end
        end
        disp(sprintf("%s: %c%c %c%c      %f %f %f %f",name,str,R))   

    end
    disp("--------------------");
end

function R = CalcPVal(name,A,alt)
    rg=1:12;
    R = [t_test_2(A(rg,2),A(rg,3),alt) ...
         t_test_2(A(rg,4),A(rg,5),alt) ...
         t_test_2(A(rg,3),A(rg,5),alt) ...
         t_test_2(A(rg,1),A(rg,3),alt) ...
         t_test_2(A(rg,1),A(rg,5),alt) ...
        ];

    str =[];
    for mr=R
        if(mr<0.01) 
            str = [str "#"];
        else 
            if(mr<0.05)  
                str = [str "+"];
            else
                if(mr<0.1)  
                    str = [str "*"];
                else
                    str = [str "o"];
                end
            end
        end
    end
    disp(sprintf("%s: %c%c %c %c%c      %f %f %f %f %f",name,str,R))   
end




return;

figure(1);
clf;
subplot(2,4,1);
plotBoxA('Time in contact', timeInContact,timeInContactStd ,0.5,1.1)
    
subplot(2,4,2);
plotBoxA('Shake', shake,shakeStd,0.05,0.2)

subplot(2,4,3);
plotBoxA('Pressure in var', pressureIn,pressureInStd,0,4)

subplot(2,4,4);
plotBoxA('Pressure Err in', presErrIn,presErrInStd,0,20)


for i=1:4
    subplot(2,4,4+i);
    if (i==1)
        plotBoxA(sprintf('finger %d range',i), finger_range(:,:,i),finger_rangeStd(:,:,i),0,140)
    else
        plotBoxA(sprintf('finger %d range',i), finger_range(:,:,i),finger_rangeStd(:,:,i),0,70)
    end
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


function plotBoxA(name, A,AStd,miny,maxy)
    hold on;
    N = size(A,1);
    K = size(A,2);


    colors = ['b','g','r','c'];

    X = (1:K)-1;

    AMean = mean(A);    

    ObjSize     = 1/8;
    MeanSize    = 0.25;

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
    for i=1:N
        txt = sprintf('%d   : ',i);        
        for k=1:K
            txt = [txt sprintf('%f +- %f\t',A(i,k),AStd(i,k))];
        end
        disp(txt);
    end
    txt = 'Mean: ';
    for k=1:K
        txt = [txt sprintf('%f +- %f\t',AMean(1,k),AMeanStd(1,k))];
    end
    disp(txt);
    disp('');


end

function plotBox(X,W,V,Std,base,color,linewidth)
    plot([X-W/2 X+W/2 X+W/2 X-W/2 X-W/2],[base base V V base],color);
    %patch([X-W/2 X+W/2 X+W/2 X-W/2 X-W/2],[base base V V base],color);
    plot([X X],[V V+Std],color,'lineWidth',linewidth);
    plot([X-W/3 X+W/3],[V+Std V+Std],color,'lineWidth',linewidth);

end
