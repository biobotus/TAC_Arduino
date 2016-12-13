xlsread('C:\Users\segg2903\Documents\turbidity\sample1.xlsx',1);
N=200;
t1=t-t(1);
figure
plot(y);
yfft=fft(y);
P2 = abs(yfft/N);
P1 = P2(1:N/2+1);
P1(2:end-1) = 2*P1(2:end-1);
figure
plot(P1);