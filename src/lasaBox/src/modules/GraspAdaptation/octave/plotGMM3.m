function plotGMM3(Mu,Sigma)
 
    D = 8;
    [x y z] = sphere(D);

    sx =reshape(x,1,[]);
    sy =reshape(y,1,[]);
    sz =reshape(z,1,[]);
    R0 = [sx;sy;sz];

    nbData = size(Mu,2);
    for j=1:nbData

        [E V] = eig(Sigma(:,:,j));
        R = E * sqrt(V) * E' *R0* 2;

        x =reshape(Mu(1,j)+R(1,:),D+1,D+1);
        y =reshape(Mu(2,j)+R(2,:),D+1,D+1);
        z =reshape(Mu(3,j)+R(3,:),D+1,D+1);
        surf(x,y,z);

        text(Mu(1,j),Mu(2,j),Mu(3,j),sprintf('G:%d',j));
    end
  
end
