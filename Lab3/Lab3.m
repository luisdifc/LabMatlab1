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

% figure;
% imshow(img);
% 
% figure;
% imshow(imgBW);

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
% scatterplot(imgBW_sim1,1,0,'*');
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

xI = real(imgBW_sim1);
xQ = imag(imgBW_sim1);
xI_mod = rectpulse(xI,4);
xQ_mod = rectpulse(xQ,4);
xI_mod_muestra = xI_mod(1:4*50);
xQ_mod_muestra = xQ_mod(1:4*50);

% xI(t) y xQ(t)
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

RI = awgn(xI_mod,0,'measured');
RQ = awgn(xQ_mod,0,'measured');
R = RI+RQ*1i;

RI_mod_muestra = RI(1:4*50);
RQ_mod_muestra = RQ(1:4*50);

% figure;
% scatterplot(R,1,0,'*');
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
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

yRn_I = intdump(RI,4);
yRn_Q = intdump(RQ,4);
yRn = yRn_I+1i*yRn_Q;

yRn_demod = qamdemod(yRn, 16, 'gray');
yRn_demod_bin = dec2bin(yRn_demod);

% BER
error_demod = 0;
    
img_error = reshape(imgBW_v1,[],1);
yRn_demod_error = reshape(yRn_demod_bin,[],1);
size_imgR = size(imgBW);

    for j=1:1:length(yRn_demod_error)
        if(yRn_demod_error(j) ~= img_error(j))
            error_demod = error_demod+1;
        end 
    end 
    
BER_demod = error_demod/(8*size_imgR(1)*size_imgR(2));    


%% 6. SECUENCIA DE BITS DE SALIDA

imgR_bytes = reshape(yRn_demod_bin,[],8);
imgR_bytes_dec = bin2dec(imgR_bytes);

imgR = reshape(imgR_bytes_dec,size_imgR );
imgR_BW = mat2gray(imgR);

figure;
imshow(imgR_BW);

%% 7. MEDIDAS DE DESEMPEÑO

BW = obw(xI_mod+1i.*xQ_mod, Fs)

SNR_dB = [0 5 10 15 20];
SNR = 10.^(SNR_dB./10);

EbN0 = SNR.*Ts.*Fs./k;
BER = [];

% Calculo de BER para diferentes SNR
for m = 1:1:5
    RI_e = awgn(xI_mod,SNR_dB(m),'measured');
    RQ_e = awgn(xQ_mod,SNR_dB(m),'measured');
    
    yRn_Ie = intdump(RI_e,4);
    yRn_Qe = intdump(RQ_e,4);
    yRne = yRn_Ie+i*yRn_Qe;

    yRne_demod = qamdemod(yRne, 16, 'gray');
    yRne_demod_bin = dec2bin(yRne_demod);
    
    error_number = 0;
    
    img_error = reshape(imgBW_v1,[],1);
    yRne_error = reshape(yRne_demod_bin,[],1);

    for j=1:1:length(yRne_error)
        if(yRne_error(j) ~= img_error(j))
            error_number = error_number+1;
        end 
    end 
    BER(m) = error_number/(8*size_imgR(1)*size_imgR(2));    
end

% figure;
% hold on;
% plot(EbN0,BER,'LineWidth',2);
% xlabel('Eb/N0');
% ylabel('BER');
% axis([0 15 0 0.14928]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

%% RETO: PULSO DE COSENO ELEVADO 

t_reto = Ts/8:Ts/8:Ts*50;

xIreto = real(imgBW_sim1);
xQreto = imag(imgBW_sim1);
xIreto_mod = rectpulse(xIreto,8);
xQreto_mod = rectpulse(xQreto,8);
Xreto_mod = xIreto_mod + 1i.*xQreto_mod;

% Filtro de raiz de coseno elevado, su respuesta al impulso y espectro de
% magnitud.
sqrt_cos_elev = rcosdesign(0.25, 16, 8, 'sqrt');

figure;
h = fvtool(sqrt_cos_elev,'Analysis','Impulse');
set(gcf,'color','w');
grid on;

% Aumentando la frecuencia de muestreo del Xreto_mod, filtrando y
% eliminando el retardo. 
x_up = upsample(Xreto_mod,8);
x_filt = conv(x_up,sqrt_cos_elev);
x = x_filt(25:length(x_filt)-24);

Xreto_Re = real(x);
Xreto_Im = imag(x);
Xreto_Re_muestra = Xreto_Re(1:8*50);
Xreto_Im_muestra = Xreto_Im(1:8*50);

% xI(t) y xQ(t)
% figure;
% plot(t_reto,Xreto_Re_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('xI(t)');
% xlabel('Tiempo / s');
% 
% figure;
% plot(t_reto,Xreto_Im_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('xQ(t)');
% xlabel('Tiempo / s');

% Transformada de Fourier xI(f) y xQ(f)
xIreto_F = 1/N*fftshift(fft(Xreto_Re_muestra,N));
xQreto_F = 1/N*fftshift(fft(Xreto_Im_muestra,N));

% Espectros de magnitud 
% figure;
% stem(f, abs(xIreto_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|xI(f)|');
% axis([-250 250 0 0.8]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;
% 
% figure;
% stem(f, abs(xQreto_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|xQ(f)|');
 %axis([-250 250 0 0.8]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

% Agregando ruido AWGN del canal 
Rreto = awgn(x,20,'measured');
Rreto_Re = real(Rreto);
Rreto_Im = imag(Rreto);
Rreto_Re_muestra = Rreto_Re(1:8*50);
Rreto_Im_muestra = Rreto_Im(1:8*50);

% rI(t) y rQ(t)
% figure;
% plot(t_reto,Rreto_Re_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('rI(t)');
% xlabel('Tiempo / s');
% 
% figure;
% plot(t_reto,Rreto_Im_muestra,'LineWidth',1.2);
% set(gca,'fontsize',22);
% set(gcf, 'color', 'w');
% ylabel('rQ(t)');
% xlabel('Tiempo / s');

% Transformada de Fourier rI(f) y rQ(f)
RretoI_F = 1/N*fftshift(fft(Rreto_Re_muestra,N));
RretoQ_F = 1/N*fftshift(fft(Rreto_Im_muestra,N));

% Espectros de magnitud 
% figure;
% stem(f, abs(RretoI_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|rI(f)|');
%axis([-250 250 0 0.8]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;
% 
% figure;
% stem(f, abs(RretoQ_F),'LineWidth',1.8);   
% xlabel('Frecuencia / Hz');
% ylabel('|rQ(f)|');
% axis([-250 250 0 0.6]);
% set(gcf,'color','w');
% set(gca,'fontsize', 22);
% grid on;

% Filtrando r(t) y demodulando
yR_reto_up = conv(Rreto,sqrt_cos_elev);
yR_reto_filt = downsample(yR_reto_up, 8);
yRn_reto = yR_reto_filt(4*8:8:length(yR_reto_filt)); % No se empieza en 1 porque retardo del primer filtrado no se habia eliminado completamente. 

yRn_reto_demod = qamdemod(yRn_reto, 16, 'gray');
yRn_reto_demod_bin = dec2bin(yRn_reto_demod);

% BER
error_reto = 0;
    
img_error = reshape(imgBW_v1,[],1);
yRn_reto_demod_error = reshape(yRn_reto_demod_bin,[],1);

for j=1:1:length(yRn_reto_demod_error)
    if(yRn_reto_demod_error(j) ~= img_error(j))
        error_reto = error_reto+1;
    end 
end 
    
BER_reto = error_reto/(8*size_imgR(1)*size_imgR(2));  

% Imagen recibida
imgR_reto_bytes = reshape(yRn_reto_demod_bin,[],8);
imgR_reto_bytes_dec = bin2dec(imgR_reto_bytes);

imgR_reto = reshape(imgR_reto_bytes_dec,size_imgR);
imgR_retoBW = mat2gray(imgR_reto);

figure;
imshow(imgR_retoBW);
