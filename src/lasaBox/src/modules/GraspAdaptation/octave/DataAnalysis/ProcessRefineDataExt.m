function R = ProcessRefineDataExt

R = [];
for i=0:2
    R = [R;ProcessTestingDataExt(sprintf('can%d',i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('can%d%d',j,i))];
        R = [R;ProcessTestingDataExt(sprintf('can%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingDataExt(sprintf('bcan%d',i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('bcan%d%d',j,i))];
        R = [R;ProcessTestingDataExt(sprintf('bcan%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingDataExt(sprintf('box%d',i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('box%d%d',j,i))];
        R = [R;ProcessTestingDataExt(sprintf('box%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingDataExt(sprintf('rul%d',i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('rul%d%d',j,i))];
        R = [R;ProcessTestingDataExt(sprintf('rul%d%d_rep',j,i))];
    end
end

RExt = R;
save('RefinementDataExt.mat','RExt');

