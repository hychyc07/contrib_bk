function ProcessRawData(name,mode,vstart,vend)

global GRASP_ADAPTATION_PATH
if isempty(GRASP_ADAPTATION_PATH)
    GRASP_ADAPTATION_PATH = pwd;
end

% Load data
A = load(sprintf('%s/%s.txt',GRASP_ADAPTATION_PATH,name));

if(mode==0)
    % Trunc to minimal pressure
    A(:,10:12) = max(30,A(:,10:12))*0.7;
end

% Smooth data
B = Smooth(A',15)';


if(nargin!=4)
    [vstart vend] = FindBlock(A(:,1:9));
end


T = [1:size(A,1)];
figure(1);
clf;
subplot(4,1,1);
hold on;
plot(T,A(:,1:9),'b');
plot(T([vstart:vend])-7,B([vstart:vend],1:9),'r');

subplot(4,1,2);
hold on;
plot(T,A(:,10:12),'b');
plot(T([vstart:vend])-7,B([vstart:vend],10:12),'r');



% Re-normalize directions
N = size(B,1);
for i=1:N
    for j=1:3
        V = B(i,18+(j-1)*3 + [1:3]); 
        nV = sqrt(V*V');
        B(i,18+(j-1)*3 + [1:3]) = V / nV;
    end
end

if(mode==0)

    % Set the limits and add pauses before and after
    C = [B(vstart:vend,:); flipud(B(vstart:vend,:))]; 

    C = [repmat(C(1,:),50,1);C;repmat(C(end,:),50,1)];
else
    C = B(vstart:vend,:);

    % remove bad pressure data
    ids = find( (C(:,10) > 20) & (C(:,11) > 20) & (C(:,12) > 20));
    C = C(ids,:);
end


% Plot the result
subplot(4,1,3);
plot(C(:,1:9));
subplot(4,1,4);
plot(C(:,10:12));

% Save the new data
save('-ascii',sprintf('%s/%s_p.txt',GRASP_ADAPTATION_PATH,name),'C');



