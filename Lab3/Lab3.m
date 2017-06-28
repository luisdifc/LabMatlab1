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

%% 1. ESTUDIO PRELIMINAR

img = imread('Squirtle.png');
imgBW = rgb2gray(img);
%imshow(imgBW);
imgBW_bin = dec2bin(imgBW);
imgBW_v = reshape(imgBW_bin,[],2);

%% 2. MODULACION 4PAM

imgBW_mod = pammod(bin2dec(imgBW_v), 4);

% Prueba de funcionamiento
% imgBW_v(125,:)
% imgBW_v(7,:)
% 
% imgBW_mod(125)
% imgBW_mod(7)


