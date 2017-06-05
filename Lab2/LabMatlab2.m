%UNIVERSIDAD DE COSTA RICA
%FACULTAD DE INGENIERIA 
%ESCUELA DE INGENIERIA ELECTRICA

%IE0527 - Ingenieria de Comunicaciones
%I - 2017

%Taller de simulacion: Modulacion analogica

%Estudiantes:
%Luis Diego Fernandez Coto, 
%Victoria Quirós Cordero, B35527

%Profesor:
%Teodoro Willink Castro

%05/06/17

%% 1. SENNALES PORTADORA Y MODULADORA
clear all;

%Ganancias Ai
A1 = 0.7;
A2 = 0.3;
A3 = 0.4;

%Frecuencias fi
f1 = 440;
f2 = 3*f1;     
f3 = 5*f1;  
fc = 38000;

Fs = 4*fc;    % Frecuencia de muestreo
N = 8192;       % Se recomienda que sea una potencia de 2

%Vectores de muestreo
t = 0:(1/Fs):((N-1)/Fs);    % Vector para el tiempo
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); % Vector de frecuencia

%Sennales de importancia
c = cos(2*pi*fc*t); %sennal portadora
x = (A1*cos(2*pi*f1*t)) + (A2*sin(2*pi*f2*t)) + (A3*cos(2*pi*f3*t)); %sennal moduladora

%Grafica de los vectores
% figure;
% plot(t*1000,x,'LineWidth',1.8);
% title('Gráfica de x(t)');
% xlabel('Tiempo (ms)');
% ylabel('x(t)');
% legend('x(t)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;
% 
% figure;
% plot(t*1000, c,'LineWidth',1.8);
% title('Gráfica de c(t)');
% xlabel('Tiempo (ms)');
% ylabel('c(t)');
% legend('c(t)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

% Transformada de fourier X(F) de x(t)
X = 1/N*fftshift(fft(x,N)); % fftshift centra x(t)

% Transformada de fourier C(F) de c(t)
C = 1/N*fftshift(fft(c,N)); % fftshift centra x(t)

% Grafica del espectro de Fourier de X
% figure;
% stem(f, abs(X),'LineWidth',1.8);    % Espectro de Fourier
% title('Espectro de Fourier de X(F)');
% xlabel('Frecuencia (Hz)');
% ylabel('Magnitud');
% legend('Magnitud X(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

% Grafica del espectro de Fourier de C
% figure;
% stem(f, abs(C),'LineWidth',1.8);    % Espectro de Fourier
% title('Espectro de Fourier de C(F)');
% xlabel('Frecuencia (Hz)');
% ylabel('Magnitud');
% legend('Magnitud C(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%% 2. MODULACION AM CONVENCIONAL
m = 0.5; %indice de modulacion

s1 = 0;

for i = 1:1:8192
    s1(i) = (1 + m * x(i))*c(i);
end 

%grafica de s1 en el tiempo
% figure;
% plot(s1,'LineWidth',1.8);
% title('Gráfica de s1(t)');
% xlabel('Tiempo (s)');
% ylabel('s1(t)');
% legend('s1(t)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

% Transformada de fourier S1(F) de s1(t)
S1 = 1/N*fftshift(fft(s1,N)); % fftshift centra x(t)

% Grafica del espectro de Fourier de S1
% figure;
% stem(f, abs(S1),'LineWidth',1.8);    % Espectro de Fourier
% title('Espectro de Fourier de S1(F)');
% xlabel('Frecuencia (Hz)');
% ylabel('Magnitud');
% legend('Magnitud S1(F)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

%% 3. MODULACION AM DSB-SC
s2 = 0;

for i = 1:1:8192
    s2(i) = x(i)*c(i);
end 

%grafica de s2 en el tiempo
% figure;
% plot(s2,'LineWidth',1.8);
% title('Gráfica de s2(t)');
% xlabel('Tiempo (s)');
% ylabel('s2(t)');
% legend('s2(t)');
% set(gcf,'color','w');
% set(gca,'fontsize', 19);
% grid on;

% Transformada de fourier S1(F) de s2(t)
S2 = 1/N*fftshift(fft(s2,N)); % fftshift centra x(t)

% Grafica del espectro de Fourier de S1
figure;
stem(f, abs(S2),'LineWidth',1.8);    % Espectro de Fourier
title('Espectro de Fourier de S2(F)');
xlabel('Frecuencia (Hz)');
ylabel('Magnitud');
legend('Magnitud S2(F)');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

%% 4. DEMODULACION AM DSB-SC

