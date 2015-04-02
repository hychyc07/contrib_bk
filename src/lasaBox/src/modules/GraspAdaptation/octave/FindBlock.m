function [vs ve] = FindBlock(A)

    N = size(A,2);

    val = A(1,:);
    minId = size(A,1);
    for i=1:N
        ids = find(abs(A(:,i)-val(i))>10.0);
        if(length(ids)>0)
            if(ids(1)<minId)
                minId = ids(1);
            end
        end
    end

    maxId = minId;
    for i=1:N
        ids = find(abs(A(1:minId,i)-val(i))<2.0);
        if(length(ids)>0)
            if(ids(end)<maxId)
                maxId = ids(end);
            end
        end
    end

    vs = maxId;
    
    A = flipud(A);
    val = A(1,:);
    minId = size(A,1);
    for i=1:N
        ids = find(abs(A(:,i)-val(i))>10.0);
        if(length(ids)>0)
            if(ids(1)<minId)
                minId = ids(1);
            end
        end
    end

    maxId = minId;
    for i=1:N
        ids = find(abs(A(1:minId,i)-val(i))<2.0);
        if(length(ids)>0)
            if(ids(end)<maxId)
                maxId = ids(end);
            end
        end
    end

    ve = size(A,1)-maxId+1;

end

