%%%%%%%%%%%%%%% for spring model %%%%%%%%%%%%%%%%%
DD=0.06;
%%%%%%%% 3D spring leg
L0 = sqrt( (lengthupperleg + lengthunderleg + DD)^2+hip_Comy^2 );
scaling=1.0;
massless_K=scaling*(4*pi*2)^2;
DampingRatio=1.5; %here rLd is not determined by natural dynamic of spring, so higher damping is unstable
viscous= 2*DampingRatio*sqrt(massless_K);
%%%%%%%%%%%%%%% coefficients %%%%%%%%%%%%%%%%%
shrink = 0.5; % for the 0 walking phase
% this shrink affects initial height a lot, thus influence the comparison
% of cart table model and sping model. i changed this to .75 that is why
% initial height is higher than COM wave. if I set this as 0.5, com wave is
% above initial height which is constant hip height in cart model 
%%%%%%%%%%%%%%%%
height_hip = lengthupperleg+lengthunderleg-G/(massless_K*2*shrink)-0.0003;
z_c=height_hip+DD;