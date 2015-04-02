function [Vunf]=Unfold2by2(VecEp)
coun=1;
for i=1:20
    for j=1:50
       Vunf(i,j)=VecEp(coun);
       coun=coun+1;
    end
end

