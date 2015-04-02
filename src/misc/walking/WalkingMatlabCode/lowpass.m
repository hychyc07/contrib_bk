function [x]=lowpass(lowpass,x,T)
samplingf=1/T;
fNorm = lowpass / (samplingf/2);
[b,a] = butter(10, fNorm, 'low');
x = filtfilt(b, a, x);
end