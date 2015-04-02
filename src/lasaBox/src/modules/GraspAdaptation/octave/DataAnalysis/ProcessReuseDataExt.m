function R = ProcessReuseDataExt

R = [];

for i=0:2
    j=1;
    R = [R;ProcessTestingDataExt(sprintf('can_bcan%d%d_rep',j,i))];
    R = [R;ProcessTestingDataExt(sprintf('reuse_can_bcan%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('can%d%d_rep',j,i))];
    end
end


for i=0:2
    j=1;
    R = [R;ProcessTestingDataExt(sprintf('bcan_box%d%d_rep',j,i))];
    R = [R;ProcessTestingDataExt(sprintf('reuse_bcan_box%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('bcan%d%d_rep',j,i))];
    end
end

for i=0:2
    j=1;
    R = [R;ProcessTestingDataExt(sprintf('box_can%d%d_rep',j,i))];
    R = [R;ProcessTestingDataExt(sprintf('reuse_box_can%d%d_rep',j,i))];
    for j=0:1
        R = [R;ProcessTestingDataExt(sprintf('box%d%d_rep',j,i))];
    end
end

RExt = R;

save('ReuseDataExt.mat','RExt');

