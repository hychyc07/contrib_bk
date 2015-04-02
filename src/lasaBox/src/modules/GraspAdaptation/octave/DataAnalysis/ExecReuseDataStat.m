function ExecReuseDataStat

load('ReuseData.mat');

N=3;

% time in contact

timeInContact       = zeros(3*N,4);
timeInContactStd    = zeros(3*N,4);

time2FInContact       = zeros(3*N,4);
time2FInContactStd    = zeros(3*N,4);

finger_range       = zeros(3*N,4,4);
finger_rangeStd    = zeros(3*N,4,4);

shake               = zeros(3*N,4);
shakeStd            = zeros(3*N,4);

pressureIn          = zeros(3*N,4);
pressureInStd       = zeros(3*N,4);

contactIn           = zeros(3*N,4);
contactInStd        = zeros(3*N,4);

presErrIn           = zeros(3*N,4);
presErrInStd        = zeros(3*N,4);

presErrOut          = zeros(3*N,4);
presErrOutStd       = zeros(3*N,4);

posErrIn           = zeros(3*N,4);
posErrInStd        = zeros(3*N,4);

posErrOut          = zeros(3*N,4);
posErrOutStd       = zeros(3*N,4);


%all object
for k=1:3
    %all phase
    for p=1:4
        %all trial        
        ids = (k-1)*12 + (p-1) +[1 5 9];
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


CalcPVal2('Time in contact', timeInContact,"<");
CalcPVal2('Shake          ', shake,">");
CalcPVal2('Pressure Err in', presErrIn,">");
for i=1:4
    CalcPVal2(sprintf('finger %d range ',i), finger_range(:,:,i),"<");
end


return


function R = CalcAnova(name,A)
    
    Obj = repmat(reshape(repmat([1:3]',1,3)',[],1),1,4);
    Exp = repmat([1:4],9,1);
    %Obj = Obj*0+1;

    cols = [1 2;3 2];
    
    rows = 1:9;
    P = [];
    for i=1:2
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

    disp(sprintf("%s: %c%c",name,str))   

    disp(sprintf("\tExp.      \tObject   \tInteract.",mr))   
    i =0;
    for mr=P'
        disp(sprintf("%c\t%f\t%f\t%f",'a'+i,mr))   
        i++;
    end    

end


function R = CalcPVal2(name,A,alt)    
    CalcAnova(name,A);
    disp("--------------------");
    return;
    for i=0:3
        if(i<3)
            rg=i*3+[1:3];
        else
            rg=1:9;
        end
        if 0
        R = [t_test_2(A(rg,1),A(rg,2),alt) ...
             t_test_2(A(rg,3),A(rg,2),alt) ...
            ];
        else
        R = [welch_test(A(rg,1),A(rg,2),alt) ...
             welch_test(A(rg,3),A(rg,2),alt) ...
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
    %CalcAnova(name,A);
    %disp("--------------------");
end

function R = CalcPVal(name,A,alt)

    R = [t_test_2(A(:,1),A(:,2),alt) ...
         t_test_2(A(:,3),A(:,2),alt) ...
         t_test_2(A(:,3),A(:,4),alt) ...
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
    disp(sprintf("%s: %c %c %c       %f %f %f",name,str,R))   
end

