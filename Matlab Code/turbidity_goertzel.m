clc;
close all;
clear all;

%Developping and testing of the algoritm with matlab

%Sample that were taken directly from the analog read of the Arduino
%Made more than five different test but I only kept 5 for simplicity
FILE(1,:) = 'C:\Users\segg2903\Documents\turbidity\Test5\sample1.xlsx';
FILE(2,:) = 'C:\Users\segg2903\Documents\turbidity\Test5\sample2.xlsx';
FILE(3,:) = 'C:\Users\segg2903\Documents\turbidity\Test5\sample3.xlsx';
FILE(4,:) = 'C:\Users\segg2903\Documents\turbidity\Test5\sample4.xlsx';
FILE(5,:) = 'C:\Users\segg2903\Documents\turbidity\Test5\sample5.xlsx';

for f=1:5 %from first to last file
    [NUM,TXT,RAW]=xlsread(FILE(f,:)); %open file
    
    %extract data from excel file
    x = NUM(:,1);
    %t = NUM(:,2);
    y = NUM(:,3);
    %freqmoy = NUM(1,6);
    %freqmode = NUM(1,7);

    N=size(x);
    N=N(1);
    %freqmoy=freqmoy(1);
    %t1=t-t(1);
    
    %plot original data
     figure
     plot(y);
    yfft=fft(y);
    
   
    %figure
   % plot(x,yfft)
    P2 = abs(yfft/N);
    P1 = P2(1:N/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    %plot FFT of data
    figure
    plot(P1);

    %sampling_frequency=freqmoy;
    sampling_frequency=8928.571;  %À tester plus tard
    target_frequency=[0:sampling_frequency/N:sampling_frequency/2-sampling_frequency/N];
    
    %next section is Goertzel Algorithm computed for every frequency in the
    %spectrum.  That way, we can compare the results with the FFT done just before.  In the arduino, we only compute the Goertzel algorithm for
    %one frequency (469Hz which is the frequency at which we pulse the
    %LED).
    coeff=2*cos(2*pi*target_frequency/sampling_frequency);
    x = ones(1,10);
    Q0=0;
    for i = 0:100-1
        Q2=0;
        Q1=0;
        for index = 0:N-1
            Q0 = coeff(i+1)*Q1-Q2+y(index+1);
            Q2=Q1;
            Q1=Q0;
        end
        magnitude(i+1)=sqrt(Q1*Q1 + Q2*Q2 - coeff(i+1)*Q1*Q2);
        f1(i+1)=i;
    end
     
    %For each set of excel data, plot the result of the Goertzel Algorithm
     figure
     plot(f1,magnitude)
     
     %calculate where the maximum peak is between index 7 and 17 (
    [ymax(f),indmax(f)]=max(magnitude(7:17));
    
end

%Verify consistency between set of excel data by plotting maximum in
%absolute of each set of data and index of maximum
figure
stem(indmax+5)
figure
stem(ymax)

