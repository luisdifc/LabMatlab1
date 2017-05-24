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

%% MACHOTE GRAFICACION
%Hace tiempo no usaba Matlab... Entonces estaba probando cosas...
t = 0:0.01:(2*pi); 
x = cos(t);
y = sin(t);
plot(t,x);
hold on; 
plot(t,y);
title('Grafica de seno y coseno en 1 periodo');
xlabel('Eje X');
ylabel('Eje Y');
legend('Cos(t)', 'Sin(t)');
grid on;

%%




