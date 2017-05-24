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

%% 1.ESTUDIO PRELIMINAR
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
N = 256;

t = 0:(1/Fs):((N-1)/Fs);    %vector para el tiempo

x = (A1*cos(2*pi*f1*t)) + (A2*sin(2*pi*f2*t)) + (A3*cos(2*pi*f3*t));

% Graficacion
figure;
plot(t,x);
title('Grafica de x(t)');
xlabel('Tiempo (s)');
ylabel('x(t)');
legend('x(t)');
grid on;

%% 2.TRANSFORMADA DE FOURIER
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); %vector de frecuencia

%Transformada de fourier X(F) de x(t)
X = 1/N*fftshift(fft(x,N));

%Graficacion de la parte real e imaginaria
figure;
plot (f, real(X));
hold on;
plot (f, imag(X));
title('Grafica de X(F)');
xlabel('Frecuencia (Hz)');
ylabel('X(F)');
legend('Parte real', 'Parte imaginaria');
grid on;

%Grafiacion del espectro de Fourier de x


%%





