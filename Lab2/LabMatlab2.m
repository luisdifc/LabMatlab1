% UNIVERSIDAD DE COSTA RICA
% FACULTAD DE INGENIERIA 
% ESCUELA DE INGENIERIA ELECTRICA

% IE0527 - Ingenieria de Comunicaciones
% I - 2017

% Taller de simulacion: Modulacion analogica

% Estudiantes:
% Luis Diego Fernandez Coto, 
% Victoria Quirós Cordero, B35527

% Profesor: Teodoro Willink Castro
% 05/06/17

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
N = 8192;     

%Vectores de muestreo
t = 0:(1/Fs):((N-1)/Fs);            % Vector para el tiempo
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); % Vector de frecuencia

% Sennal portadora c y sennal moduladoara x
c = cos(2*pi*fc*t); 
x = (A1*cos(2*pi*f1*t)) + (A2*sin(2*pi*f2*t)) + (A3*cos(2*pi*f3*t)); 

% Grafica de vectores c y x
figure;
plot(t*1000,x,'LineWidth',1.8);
xlabel('Tiempo (ms)');
ylabel('x(t)');
legend('x(t)');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

figure;
plot(t*1000, c,'LineWidth',1.8);
xlabel('Tiempo (ms)');
ylabel('c(t)');
legend('c(t)');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

% Transformada de Fourier X(F) de x(t)
X = 1/N*fftshift(fft(x,N)); 

% Transformada de Fourier C(F) de c(t)
C = 1/N*fftshift(fft(c,N)); 

% Espectro de Fourier de X(F)
figure;
stem(f, abs(X),'LineWidth',1.8);  
title('Espectro de Fourier de X(F)');
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|X(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

% Espectro de Fourier de C(F)
figure;
stem(f, abs(C),'LineWidth',1.8);   
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|C(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

%% 2. MODULACION AM CONVENCIONAL

m = 0.5; % Indice de modulacion
s1 = 0;

for i = 1:1:8192
    s1(i) = (1 + m * x(i))*c(i);
end 

% Grafica de s1 en el tiempo
figure;
plot(s1,'LineWidth',1.8);
xlabel('Tiempo (s)');
ylabel('s1(t)');
legend('s1(t)');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

% Transformada de Fourier S1(F) de s1(t)
S1 = 1/N*fftshift(fft(s1,N)); 

% Espectro de Fourier de S1(F)
figure;
stem(f, abs(S1),'LineWidth',1.8);  
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|S1(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

%% 3. MODULACION AM DSB-SC

s2 = 0;

for i = 1:1:8192
    s2(i) = x(i)*c(i);
end 

% Grafica de s2 en el tiempo
figure;
plot(s2,'LineWidth',1.8);
title('Gráfica de s2(t)');
xlabel('Tiempo (s)');
ylabel('s2(t)');
legend('s2(t)');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

% Transformada de fourier S2(F) de s2(t)
S2 = 1/N*fftshift(fft(s2,N)); 

% Espectro de Fourier de S2(F)
figure;
stem(f, abs(S2),'LineWidth',1.8);   
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|S2(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on;

%% 4. DEMODULACION AM DSB-SC

% Frecuencia de corte y de muestreo del filtro pasa bajos
fc_filt = 2500;
fs_filt = 4*fc_filt;

[filt_num,filt_den] = butter(32,fc_filt/(fs_filt/2));

% s1(t) demodulada
z1 = 1: 1:8192;
for i = 1:1:8192
    z1(i) = s1(i)*c(i);
end

y1 = 2.*filtfilt(filt_num,filt_den,z1); 

figure;
hold on;
plot(y1,'linestyle','-','LineWidth',1.8);
legend('y1(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;
 
% Transformada de fourier Y1(F) de y1(t)
Y1 = 1/N*fftshift(fft(y1,N)); 

% Espectro de Fourier de Y1(F)
figure;
stem(f, abs(Y1),'LineWidth',1.8);   
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|Y1(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on; 

% s2(t) demodulada
z2 = 1: 1:8192;
for i = 1:1:8192
    z2(i) = s2(i)*c(i);
end

y2 = 2.*filtfilt(filt_num,filt_den,z2); 

figure;
hold on;
plot(y2,'linestyle','-','LineWidth',1.8);
legend('y1(t)');
xlabel('Tiempo (seg)','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;
 
% Transformada de fourier Y2(F) de y2(t)
Y2 = 1/N*fftshift(fft(y2,N)); 

% Espectro de Fourier de Y2(F)
figure;
stem(f, abs(Y2),'LineWidth',1.8);   
xlabel('Frecuencia / Hz');
ylabel('Magnitud');
legend('|Y2(F)|');
set(gcf,'color','w');
set(gca,'fontsize', 19);
grid on; 

%% Reto: Multiplexacion en FM estereo 

% Vector de datos muestrados w a una tasa de muestreo Fsw
[w,Fsw] = audioread('Roxanne.wav');
wL = w(:,1);
wR = w(:,2);

figure;
hold on;
plot(wL,'linestyle','-','LineWidth',2);
legend('wL(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;

figure;
hold on;
plot(wR,'linestyle','-','LineWidth',2);
legend('wR(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;

% Aumentando frecuencia de muestro por un factor de 4
wL = interp(wL,4);
wR = interp(wR,4);
Fsw = Fsw*4;

% Vectores de tiempo y frecuencia utilizando la misma construccion de
% secciones anteriores
Nw = length(wL);
tw = 0 : (1/Fsw) :((Nw-1)/Fsw);
fw = -Fsw/2 : Fsw/Nw : ((Fsw/2)-(Fsw/Nw));

% Vector de muestras sFM de la sennal banda-base de FM estereo
Cw = 0.05*cos(2*pi*tw*19000); % Portadora
Cw2 = cos(2*pi*tw*38000);     % Sennal necesaria para modular L-R con AFM DSB-SC

sFM = 1 : 1 : Nw;
for i = 1:1:Nw
    sFM(i) = (wL(i)+wR(i)) + Cw(i) + ((wL(i)-wR(i))*Cw2(i));
end

figure;
hold on;
plot(sFM,'linestyle','-','LineWidth',2);
legend('sFM(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;

% Transformada de Fourier SFM(f) de sFM(t) 
ffsFM = fft(sFM,Nw);
Sw = 1/Nw*fftshift(ffsFM);
Sw_real = real(Sw);
Sw_im = imag(Sw);
SFM = abs(Sw_real) + abs(Sw_im);

figure;
hold on;
stem(fw,SFM,'LineWidth',1.8);
legend('|SFM(f)|');
xlabel('Frecuencia / Hz','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;

% Vector de muestras recuperado al demodular sFM
zFM = 1 : 1 : Nw;
for i = 1:1:Nw
    zFM(i) = sFM(i)*Cw2(i); % Demodulacion AM DSB-SC
end

fcw1 = 15000;  % Frecuencia de corte del filtro pasa bajos
Fsw1 = 4*fcw1; % Frecuencia de muestreo del filtro pasa bajos

% Filtro pasa bajos con frecuencia de corte fcw1 de 15 KHz
[filt1_num,filt1_den] = butter(32,fcw1/(Fsw1/2));

sFM_bajasf = filtfilt(filt1_num,filt1_den,sFM);
sFM_demod = 2*filtfilt(filt1_num,filt1_den,zFM);

% Vector de muestras del canal wL recuperado
vL = 1 : 1 : Nw;
for i = 1:1:Nw
    vL(i) = 1/2.*(sFM_bajasf(i)+sFM_demod(i));
end

% Vector de muestras del canal wR recuperado
vR = 1 : 1 : Nw;
for i = 1:1:Nw
    vR(i) = 1/2.*(sFM_bajasf(i)-sFM_demod(i));
end

% Disminuir frecuencia de muestreo por un factor de 4
vL = decimate(vL,4);
vR = decimate(vR,4); 
Fsw = Fsw/4;
 
figure;
plot(vL,'linestyle','-','LineWidth',1.8);
legend('vL(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;
    
figure;
plot(vR,'linestyle','-','LineWidth',1.8);
legend('vR(t)');
xlabel('Tiempo / s','FontSize', 24);
ylabel('Magnitud ','FontSize', 24);
grid on;

% Verificando que se recupera la cancion
v_recover(:,1) = vL;
v_recover(:,2) = vR;
audiowrite('Roxanne_recover.wav',v_recover,Fsw);
