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
f2 = 880;   %2*f1  
f3 = 1320;  %3*f1

Fs = 32 * f1;   %frecuencia de muestreo
N = 512;    %se recomienda que sea una potencia de 2

t = 0:(1/Fs):((N-1)/Fs);    %vector para el tiempo

x = (A1*cos(2*pi*f1*t)) + (A2*sin(2*pi*f2*t)) + (A3*cos(2*pi*f3*t));

% Graficacion
figure;
plot(t*1000,x);
title('Grafica de x(t)');
xlabel('Tiempo (ms)');
legend('x(t)');
grid on;

%% 2. TRANSFORMADA DE FOURIER
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); %vector de frecuencia

%Transformada de fourier X(F) de x(t)
X = 1/N*fftshift(fft(x,N)); %fftshift centra la sennal

%Graficacion de la parte real e imaginaria
figure;
plot (f, real(X));
hold on;
plot (f, imag(X));
title('Grafica de X(F)');
xlabel('Frecuencia (Hz)');
%ylabel('X(F)');
legend('Parte real', 'Parte imaginaria');
grid on;

figure;
stem(f, abs(X));    %espectro de Fourier
title('Especto de Fourier de X(F)');
xlabel('Frecuencia (Hz)');
%ylabel('X(F)');
legend('Magnitud X(F)');
grid on;

%Grafiacion del espectro de Fourier de x


%% 3. DENSIDAD ESPECTRAL DE POTENCIA
f_dB = 0:(Fs/N):((Fs/2)-(Fs/N)); %vector de frecuencia

%Obtenemos Sxx
Sxx = periodogram(x,rectwin(N),N, Fs, 'onesided');
Sxx = Fs/N*Sxx(1:N/2)';
SxxdB = 10*log10(Sxx);  %Sxx expresado en decibelios

%Graficando Sxx en dBW/Hz
figure;
plot(f_dB,SxxdB);
title('Grafica de Sxx');
xlabel('F (Hz)');
legend('Sxx(F)');
grid on;

%FALTA DETERMINAR Px de Sxx

Sxx1 = 2*abs(X(N/2+1:N)).^2;
SxxdB1 = 10*log10(Sxx1);  %Sxx expresado en decibelios

%Graficando Sxx1 en dBW/Hz
figure;
plot(f_dB,SxxdB1);
title('Grafica de Sxx1');
xlabel('F (Hz)');
legend('Sxx1(F)');
grid on;

%FALTA DETERMINAR Px de Sxx1

%%





