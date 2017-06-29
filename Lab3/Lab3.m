%UNIVERSIDAD DE COSTA RICA
%FACULTAD DE INGENIERIA 
%ESCUELA DE INGENIERIA ELECTRICA

%IE0527 - Ingenieria de Comunicaciones
%I - 2017

%Taller de simulación: Modulación digital

%Estudiantes:
%Luis Diego Fernandez Coto, B22492
%Victoria Quirós Cordero, B35527

%Profesor:
%Teodoro Willink Castro

%28/06/17

%% 1. SECUENCIA DE BITS DE ENTRADA

img = imread('Squirtle.png');
imgBW = rgb2gray(img);
imgBW_bin = dec2bin(imgBW);
imgBW_v = reshape(imgBW_bin,[],2);

figure;
%imshow(imgBW);

%% 2. MODULACION 4PAM

imgBW_sim = pammod(bin2dec(imgBW_v), 4, 0, 'gray');

% Prueba de funcionamiento
% imgBW_v(125,:)
% imgBW_v(7,:)
% 
% imgBW_sim(125)
% imgBW_sim(7)
 
xI_mod = rectpulse(imgBW_sim,4);
xI_mod_muestra = xI_mod(1:4*50);

Ts = 0.002;
t = Ts/4:Ts/4:Ts*50;

% figure;
% stem(t,xI_mod_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('xI(t)');
% xlabel('Tiempo / s');

Fs = 1/Ts;
N = 500;
f = -(Fs/2):(Fs/N):((Fs/2)-(Fs/N)); % Vector de frecuencia

% Transformada de fourier xI(f) de xI(t)
XI_F = 1/N*fftshift(fft(xI_mod_muestra,N));

% Espectro de magnitud
% figure;
% stem(f, abs(XI_F),'LineWidth',1.2);  
% xlabel('Frecuencia / Hz');
% ylabel('|xI(f)|');
% axis([-250 250 0 0.7]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

%% 3. MODULACION 16QAM

k = 4;

imgBW_v1 = reshape(imgBW_bin,[],4);
imgBW_v1_dec = bin2dec(imgBW_v1);
imgBW_sim1 = qammod(imgBW_v1_dec, 16, 'gray');

% figure;
% const_nominal = scatterplot(imgBW_sim1,1,0,'*');
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% grid on;
% axis([-4 4 -4 4]);

% Prueba de funcionamiento
% imgBW_v1(125,:)
% imgBW_v1(7,:)
% 
% imgBW_sim1(125)
% imgBW_sim1(7)

% Duda, está bien simular por aparte en tiempo y frecuencia en fase y en
% cuadratura?

xI = real(imgBW_sim1);
xQ = imag(imgBW_sim1);
xI_mod = rectpulse(xI,4);
xQ_mod = rectpulse(xQ,4);
xI_mod_muestra = xI_mod(1:4*50);
xQ_mod_muestra = xQ_mod(1:4*50);

% figure;
% stem(t,xI_mod_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('xI(t)');
% xlabel('Tiempo / s');
% 
% figure;
% stem(t,xQ_mod_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('xQ(t)');
% xlabel('Tiempo / s');

% Transformada de Fourier xI(f) y xQ(f)
XI_F = 1/N*fftshift(fft(xI_mod_muestra,N));
XQ_F = 1/N*fftshift(fft(xQ_mod_muestra,N));

% Espectros de magnitud 
% figure;
% stem(f, abs(XI_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|xI(f)|');
% axis([-250 250 0 0.7]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;
% 
% figure;
% stem(f, abs(XQ_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|xQ(f)|');
% axis([-250 250 0 0.45]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

%% 4. CANAL AWGN BANDA-BASE

RI = awgn(xI_mod,20,'measured');
RQ = awgn(xQ_mod,20,'measured');
R = RI+i*RQ;

RI_mod = rectpulse(RI,1);
RQ_mod = rectpulse(RQ,1);
RI_mod_muestra = RI_mod(1:4*50);
RQ_mod_muestra = RQ_mod(1:4*50);

% figure;
% scatterplot(R,1,0,'*');
% grid on;
% axis([-4 4 -4 4]);
% 
% figure;
% stem(t,RI_mod_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('rI(t)');
% xlabel('Tiempo / s');
% 
% figure;
% stem(t,RQ_mod_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('rQ(t)');
% xlabel('Tiempo / s');

% Transformada de fourier xI(f) y xQ(f)
RI_F = 1/N*fftshift(fft(RI_mod_muestra,N));
RQ_F = 1/N*fftshift(fft(RQ_mod_muestra,N));

% Espectros de magnitud 
% figure;
% stem(f, abs(RI_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|rI(f)|');
% axis([-250 250 0 0.7]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;
% 
% figure;
% stem(f, abs(RQ_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|rQ(f)|');
% axis([-250 250 0 0.45]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

%% 5. DEMODULACION 16-QAM

% Duda sobre como se utilizó intdump y rectpulse. 
yRn_I = intdump(RI_mod,4);
yRn_Q = intdump(RQ_mod,4);
yRn = yRn_I+i*yRn_Q;

yRn_demod = qamdemod(yRn, 16, 'gray');
errores = 0;

for i=1:1:length(yRn_demod)
    if(yRn_demod(i,1) ~= imgBW_v1_dec(i,1))
        errores = errores+1;
    end 
end

yRn_demod_bin = dec2bin(yRn_demod);


%% 6. SECUENCIA DE BITS DE SALIDA

imgR_bytes = reshape(yRn_demod_bin,[],8);
imgR_bytes_dec = bin2dec(imgR_bytes);

size_imgR = size(imgBW);
imgR = reshape(imgR_bytes_dec,size_imgR );
imgR_BW = mat2gray(imgR);

figure;
%imshow(imgR_BW);

%% 7. MEDIDAS DE DESEMPEÑO

% No se si esto es cierto. 

BW_I = obw(xI_mod, Fs);
BW_Q = obw(xQ_mod, Fs);

if(BW_I < BW_Q)
    Bx = BW_Q
else
    Bx = BW_I
end

% SNR_dB = 0:k*5:k*20;
% SNR = 10.^(SNR_dB./10);
% Eb_N0 = SNR.*Ts.*Fs./k;
% 
% plot(Eb_N0,BER)
   







