for n=tsize-2*N_L-1:tsize    
    zmpx(n)=zmpx(n-1);    
    zmpy(n)=zmpy(n-1);
    zmpz(n)=zmpz(n-1);
    zmpx_multibody(n)=zmpx_multibody(n-1);
    zmpy_multibody(n)=zmpy_multibody(n-1);
    zmpx_multibody2(n)=zmpx_multibody2(n-1);
    zmpy_multibody2(n)=zmpy_multibody2(n-1);
    xh(n)=xh(n-1);
    yh(n)=yh(n-1);
    zh(n)=zh(n-1);
    xh2(n)=xh2(n-1);
    yh2(n)=yh2(n-1);
    xfl(n)=xfl(n-1);
    yfl(n)=yfl(n-1);
    zfl(n)=zfl(n-1);
    xfr(n)=xfr(n-1);
    yfr(n)=yfr(n-1);
    zfr(n)=zfr(n-1);
    COG_Global(1,n)=COG_Global(1,n-1);
    COG_Global(2,n)=COG_Global(2,n-1);
    COG_Global(3,n)=COG_Global(3,n-1);
end