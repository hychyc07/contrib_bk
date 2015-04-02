    %%%%%%%%    
    zmpx2(n) = (sysd.C * x)';
    zmpy2(n) = (sysd.C * y)';
    ezmpx = ezmpx + (zmpx2(n)-zmpx(n));
    ezmpy = ezmpy + (zmpy2(n)-zmpy(n));
    x_h(n,:) = x';
    y_h(n,:) = y';
    z_h(n,:) = z';
    xh(n)=x_h(n,1);
    xhd(n)=x_h(n,2);
    xhdd(n)=x_h(n,3);
    yh(n)=y_h(n,1);
    yhd(n)=y_h(n,2);
    yhdd(n)=y_h(n,3);
    zh(n)=z_h(n,1);   % zh is row vector, z_h is n*3 vector 
    zhd(n)=z_h(n,2);
    zhdd(n)=z_h(n,3);

    % COM Orientation
    c = cos((rotZ_fl(n)+rotZ_fr(n))/2.0);
    s = sin((rotZ_fl(n)+rotZ_fr(n))/2.0);
    Body_R(1,1,n) = c;       Body_R(1,2,n) = -s;       Body_R(1,3,n) = 0;
    Body_R(2,1,n) = s;       Body_R(2,2,n) =  c;       Body_R(2,3,n) = 0;
    Body_R(3,1,n) = 0;       Body_R(3,2,n) = 0;        Body_R(3,3,n) = 1;
    
    Body_P(:,n)=[xh(n);yh(n);zh(n)];
    
    c1=cos(rotY_fl(n)); %Y
    s1=sin(rotY_fl(n));

    RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

    c3=cos(rotZ_fl(n)); %Z
    s3=sin(rotZ_fl(n));

    RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
    
    Foot_RL(:,:,n)=RZ*RY;
    
    Foot_PL(:,n)=[xfl(n);yfl(n);zfl(n)];

    c1=cos(rotY_fr(n)); %Y
    s1=sin(rotY_fr(n));

    RY=[[c1 0 s1];[0 1 0];[-s1 0 c1]];

    c3=cos(rotZ_fr(n)); %Z
    s3=sin(rotZ_fr(n));

    RZ=[[c3 -s3 0];[s3 c3 0];[0 0 1]];
    
    Foot_RR(:,:,n)=RZ*RY;    
    
    
    Foot_PR(:,n)=[xfr(n);yfr(n);zfr(n)];
    
    Dt=[0.0;hip_Comy;0];
    [qL(:,n)]=inversekinematics(Body_P(:,n),Body_R(:,:,n),Dt, Foot_PL(:,n), Foot_RL(:,:,n));
    Dt=[0.0;-hip_Comy;0];
    [qR(:,n)]=inversekinematics(Body_P(:,n),Body_R(:,:,n),Dt, Foot_PR(:,n), Foot_RR(:,:,n));

    [COG(:,:,n),COG_Global(:,n)]=calcCOG(Body_P(:,n),Body_R(:,:,n), Foot_PL(:,n), Foot_RL(:,:,n),Foot_PR(:,n), Foot_RR(:,:,n),qL(:,n),qR(:,n));
    
    q_abs(:,n)=[0;0;(rotZ_fl(n)+rotZ_fr(n))/2.0];

    if (n>1)
            qd_abs(:,n)=((q_abs(:,n)-q_abs(:,n-1))/T);
            qdd_abs(:,n)=((qd_abs(:,n)-qd_abs(:,n-1))/T);
            qdL(:,n)=((qL(:,n)-qL(:,n-1))/T);
            qddL(:,n)=((qdL(:,n)-qdL(:,n-1))/T);
            qdR(:,n)=((qR(:,n)-qR(:,n-1))/T);
            qddR(:,n)=((qdR(:,n)-qdR(:,n-1))/T);
            COGd(:,1,n)=((COG(:,1,n)-COG(:,1,n-1))/T);
            COGd(:,2,n)=((COG(:,2,n)-COG(:,2,n-1))/T);
            COGd(:,3,n)=((COG(:,3,n)-COG(:,3,n-1))/T);
            COGd(:,4,n)=((COG(:,4,n)-COG(:,4,n-1))/T);
            COGd(:,5,n)=((COG(:,5,n)-COG(:,5,n-1))/T);
            COGd(:,6,n)=((COG(:,6,n)-COG(:,6,n-1))/T);
            COGd(:,7,n)=((COG(:,7,n)-COG(:,7,n-1))/T);
            %COGd(:,8,n)=((COG(:,8,n)-COG(:,8,n-1))/T);
            COG_Globald(:,n)=((COG_Global(:,n)-COG_Global(:,n-1))/T);
            COGdd(:,1,n)=((COGd(:,1,n)-COGd(:,1,n-1))/T);
            COGdd(:,2,n)=((COGd(:,2,n)-COGd(:,2,n-1))/T);
            COGdd(:,3,n)=((COGd(:,3,n)-COGd(:,3,n-1))/T);
            COGdd(:,4,n)=((COGd(:,4,n)-COGd(:,4,n-1))/T);
            COGdd(:,5,n)=((COGd(:,5,n)-COGd(:,5,n-1))/T);
            COGdd(:,6,n)=((COGd(:,6,n)-COGd(:,6,n-1))/T);
            COGdd(:,7,n)=((COGd(:,7,n)-COGd(:,7,n-1))/T);
            %COGdd(:,8,n)=((COGd(:,8,n)-COGd(:,8,n-1))/T);
            COG_Globaldd(:,n)=((COG_Globald(:,n)-COG_Globald(:,n-1))/T);          
    else
            qd_abs(:,1)=[0.0;0.0;0.0];
            qdd_abs(:,1)=[0.0;0.0;0.0];
            qdL(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qddL(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qdR(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            qddR(:,1)=[0.0;0.0;0.0;0.0;0.0;0.0];
            COGd(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
            COGdd(:,:,1)=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0];[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
            COG_Globald=[0.0;0.0;0.0];
            COG_Globaldd=[0.0;0.0;0.0];            
    end

    time2(n)=time(n);