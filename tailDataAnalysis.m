%% Import data and remove entries with no data
filename = 'tailData.txt';
delimiterIn = '\t';
data = importdata(filename,delimiterIn);

angle = data(:,1);
x = data(:,2);
y = data(:,3);

for k = length(x):-1:1
   if (x(k) == -1)
       angle(k) = [];
       x(k) = [];
       y(k) = [];
   end
end
%% Plot angle data over time
fs = 150;
T = 1/fs;
t = (0:length(angle)-1)*T;
figure
plot(t,angle)
xlabel('Time (s)')
ylabel('Angle (rad.)')

%% Denoising
xdMODWT = wden(angle,'modwtsqtwolog','s','mln',3,'db2');
figure
plot(xdMODWT), title('MODWT Denoising'); 

%% Plot spectrum of angle data
angle = angle-mean(angle)+ 1e-4;;
N = 1024;

X = fft(angle,N);
X = X(1:floor(N/2));
%f2 = fs*(0:(N/2))/N;
f = 0:fs/N:(fs-fs/N)/2;

figure
subplot(2,1,1)
plot(f,abs(X));
xlabel('Freq (Hz)')
ylabel('Magnitude')
subplot(2,1,2)
plot(f,20*log10(abs(X)));
xlabel('Freq (Hz)')
ylabel('Magnitude (dB)')