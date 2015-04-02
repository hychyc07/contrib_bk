function R = ProcessRefineData

R = [];
for i=0:2
    R = [R;ProcessTestingData(sprintf('can%d',i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('can%d%d',j,i))];
        R = [R;ProcessTestingData(sprintf('can%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingData(sprintf('bcan%d',i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('bcan%d%d',j,i))];
        R = [R;ProcessTestingData(sprintf('bcan%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingData(sprintf('box%d',i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('box%d%d',j,i))];
        R = [R;ProcessTestingData(sprintf('box%d%d_rep',j,i))];
    end
end

for i=0:2
    R = [R;ProcessTestingData(sprintf('rul%d',i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('rul%d%d',j,i))];
        R = [R;ProcessTestingData(sprintf('rul%d%d_rep',j,i))];
    end
end

save('RefinementData.mat','R');

