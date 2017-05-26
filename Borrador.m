%UNIVERSIDAD DE COSTA RICA
%FACULTAD DE INGENIERIA 
%ESCUELA DE INGENIERIA ELECTRICA

%IE0527 - Ingenieria de Comunicaciones
%I - 2017


%Taller de simulacion: Transformada de Fourier

%Estudiantes:
%Luis Diego Fernandez Coto
%Victoria Quiros Cordero

%Profesor:
%Teodoro Willink Castro

%23/5/17

%% 1. ESTUDIO PRELIMINAR
clear all;

%Ganancias de Ai
A1 = 0.7;
A2 = 0.3;
A3 = 0.4;

%Frecuencias fi
f1 = 440;
f2 = 3*f1;     
f3 = 5*f1;  

Fs = 32 * f1;   %frecuencia de muestreo
N = 512;    %se recomienda que sea una potencia de 2

t = 0:(1/Fs):((N-1)/Fs);    %vector para el tiempo

x = (A1*cos(2*pi*f1*t)) + (A2*sin(2*pi*f2*t)) + (A3*cos(2*pi*f3*t));

% Graficacion
% figure;
% plot(t*1000,x,'LineWidth',1.8);
% title('Gráfica de x(t)');
% xlabel('Tiempo (ms)');
% ylabel('x(t)');
% legend('x(t)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%Potencia de x
Px = bandpower(x,Fs,[0 Fs/2]);

Harmonic = {'Fundamental';'Third';'Fifth'};

Freqs = [440 1320 2200]';

Power = zeros([3 1]);
PxTotal = 0;

for k = 1:3
    Power(k) = bandpower(x,Fs,Freqs(k)+[-10 10]);
    PxTotal = PxTotal + Power(k);
end

T = table(Freqs,Power,'RowNames',Harmonic)

PxTotal

%% 2. TRANSFORMADA DE FOURIER
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); %vector de frecuencia

%Transformada de fourier X(F) de x(t)
X = 1/N*fftshift(fft(x,N)); %fftshift centra la sennal

%Graficacion de la parte real e imaginaria
% figure;
% plot (f, real(X),'LineWidth',1.8);
% hold on;
% plot (f, imag(X),'LineWidth',1.8);
% title('Gráfica de X(F)');
% xlabel('Frecuencia (Hz)');
% ylabel('X(F)');
% legend('Parte real', 'Parte imaginaria');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

% Grafiacion del espectro de Fourier de x
% figure;
% stem(f, abs(X),'LineWidth',1.8);    %espectro de Fourier
% title('Espectro de Fourier de X(F)');
% xlabel('Frecuencia (Hz)');
% ylabel('Magnitud');
% legend('Magnitud X(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%% 3. DENSIDAD ESPECTRAL DE POTENCIA
f_dB = 0:(Fs/N):((Fs/2)-(Fs/N)); %vector de frecuencia

%Obtenemos Sxx
Sxx = periodogram(x,rectwin(N),N, Fs, 'onesided');
Sxx = Fs/N*Sxx(1:N/2)';
SxxdB = 10*log10(Sxx);  %Sxx expresado en decibelios

%Graficando Sxx en dBW/Hz
% figure;
% plot(f_dB,SxxdB,'LineWidth',1.8);
% title('Gráfica de Sxx');
% xlabel('F (Hz)');
% ylaber('Magnitd (dBW)');
% legend('Sxx(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%Potencia de Sxx
PxSxx = 0;

for i = 1:1:length(Sxx)
   PxSxx = Sxx(i)+ PxSxx; 
end

PxSxx 

% Sxx a partir de FFT
Sxx1 = 2*abs(X(N/2+1:N)).^2;
SxxdB1 = 10*log10(Sxx1);  %Sxx expresado en decibelios

%Graficando Sxx1 en dBW/Hz
% figure;
% plot(f_dB,SxxdB1,'LineWidth',1.8);
% title('Gráfica de Sxx1');
% xlabel('F (Hz)');
% ylabel('Magnitd (dBW)');
% legend('Sxx1(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%Potencia de Sxx
PxSxx1 = 0;

for i = 1:1:length(Sxx1)
   PxSxx1 = Sxx(i)+ PxSxx1; 
end

PxSxx1 

%% 4. FILTRADO
%filtro FIR pasa-bajos de orden 32 con frecuencia de media potencia de fc = f 2 
D = designfilt('lowpassfir','FilterOrder',32,'HalfPowerFrequency',f2,'SampleRate',Fs);

%respuesta en frecuencia del filtro
% freqz(D, f, Fs);

%Filtrando x con el filtro disennado
y = filtfilt(D,x);

%Transformada de fourier Y(F) de y(t)
Y = 1/N*fftshift(fft(y,N)); 

%Grafiacion del espectro de Fourier de Y
% figure;
% stem(f, abs(Y),'LineWidth',1.8);    %espectro de Fourier
% title('Especto de Fourier de Y(F)');
% xlabel('Frecuencia (Hz)');
% %ylabel('X(F)');
% legend('Magnitud Y(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%% RETO: FILTRADO DE UNA SENNAL DE AUDIO
[w, Fsreto] = audioread('song.wav');

Nw = length(w);

fw = 0:(Fsreto/Nw):((Fsreto/2)-(Fsreto/Nw)); %vector de frecuencia

%Obtenemos Sxx
Sww = periodogram(w,rectwin(Nw),Nw, Fsreto, 'onesided');
Sww = Fsreto/Nw*Sww(1:floor(Nw/2))';
SwwdB = 10*log10(Sww);  %Sxx expresado en decibelios

Dw = designfilt('lowpassiir','FilterOrder',8,'HalfPowerFrequency',2000,'SampleRate',Fsreto);

v = filtfilt(Dw,w);

Svv = periodogram(v,rectwin(Nw),Nw,Fsreto,'onesided'); 
Svv = Fsreto/Nw*Svv(1:floor(Nw/2))';
SvvdB = 10*log10(Svv);

audiowrite('song_filterd.wav',v,Fsreto);


