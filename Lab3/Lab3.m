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

img = imread('Pokemon.jpeg');
%figure
%imshow(img);

img_gris = rgb2gray(img);
%figure
%imshow(img_gris);

img_gris_bin = dec2bin(img_gris);

img_gris_bin_size = size(img_gris_bin);
%img_gris_reshape = reshape(img_gris_bin, [1,8*img_gris_bin_size(1)])';

img_gris_reshape = reshape(img_gris_bin, [],1);

%% 2. MODULACION 4PAM

pammod_obj = modem.pammod('M', 2, 'InputType', 'bit','SymbolOrder', 'gray');
% 
double(img_gris_reshape);
% 
% % img_gris_reshape_d = double(img_gris_reshape);
img_sim = modulate(pammod_obj, double(img_gris_reshape));


% for j=1:1:length(img_gris_reshape)/2
% for i=1:2:length(img_gris_reshape)/2
%     conv = strcat(img_gris_reshape(i), img_gris_reshape(i+1))
%     str2num(conv)
%     img_gris_gray(j) = pammod(str2num(conv), 2)
% end
% end

%b = bin2gray(bin2dec(img_gris_bin),'pam', 4)






