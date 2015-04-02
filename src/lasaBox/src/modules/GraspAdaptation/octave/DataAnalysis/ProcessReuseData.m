function R = ProcessReuseData

R = [];

for i=0:2
    j=1;
    R = [R;ProcessTestingData(sprintf('can_bcan%d%d_rep',j,i))];
    R = [R;ProcessTestingData(sprintf('reuse_can_bcan%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('can%d%d_rep',j,i))];
    end
end


for i=0:2
    j=1;
    R = [R;ProcessTestingData(sprintf('bcan_box%d%d_rep',j,i))];
    R = [R;ProcessTestingData(sprintf('reuse_bcan_box%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('bcan%d%d_rep',j,i))];
    end
end

for i=0:2
    j=1;
    R = [R;ProcessTestingData(sprintf('box_can%d%d_rep',j,i))];
    R = [R;ProcessTestingData(sprintf('reuse_box_can%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingData(sprintf('box%d%d_rep',j,i))];
    end
end


save('ReuseData.mat','R');

