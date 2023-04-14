clc
clear all
close all 


% % Parámetros del péndulo
% m = 1; % masa del péndulo
% l = 1; % longitud del péndulo
% g = 9.81; % aceleración debida a la gravedad
% 
% % Parámetros del controlador PID
% Kp = 10; % ganancia proporcional
% Ki = 5; % ganancia integral
% Kd = 2; % ganancia derivativa
% 
% % Condiciones iniciales del péndulo
% theta0 = pi/4; % ángulo inicial
% theta_dot0 = 0; % velocidad angular inicial
% 
% % Tiempo de simulación
% t_end = 10; % tiempo final
% dt = 0.01; % paso de tiempo
% t = 0:dt:t_end; % vector de tiempo
% 
% % Trayectoria deseada en theta y su velocidad
% theta_d = theta0 + (pi/2)*(1-cos((2*pi/t_end)*t)); % función sinusoidal
% theta_dot_d = (pi^2/t_end)*sin((2*pi/t_end)*t); % derivada de la función sinusoidal
% 
% % Inicialización de vectores de estado y controlador
% theta = zeros(size(t));
% theta_dot = zeros(size(t));
% theta(1) = theta0;
% theta_dot(1) = theta_dot0;
% u = zeros(size(t));
% 
% ierror = 0;
% 
% % Bucle de simulación
% for i = 2:length(t)
%     % Cálculo del error de posición y velocidad
%     error = theta_d(i-1) - theta(i-1);
%     error_dot = theta_dot_d(i-1) - theta_dot(i-1);
%     ierror = ierror + error *dt; 
%     
%     % Cálculo del controlador PID
%     u(i) = Kp*error + Ki*ierror + Kd*(error_dot/dt);
%     
%     % Cálculo de la aceleración angular
%     theta_dotdot = (-g/l)*sin(theta(i-1)) + (u(i)/(m*l^2));
%     
%     % Cálculo de la velocidad angular y posición angular utilizando el método de integración de Euler
%     theta_dot(i) = theta_dot(i-1) + theta_dotdot*dt;
%     theta(i) = theta(i-1) + theta_dot(i)*dt;
% end
% 
% % Gráfica de la posición angular, velocidad angular, posición deseada y velocidad deseada
% subplot(2,1,1)
% plot(t,theta,'b',t,theta_d,'r')
% ylabel('Posición Angular (rad)')
% xlabel('Tiempo (s)')
% legend('Posición Actual','Posición Deseada')
% subplot(2,1,2)
% plot(t,theta_dot,'b',t,theta_dot_d,'r')
% ylabel('Velocidad Angular (rad/s)')
% xlabel('Tiempo (s)')
% legend('Velocidad Actual','Velocidad Deseada')
% 
% 


% % Parámetros del péndulo
% m = 1; % masa del péndulo
% l = 1; % longitud del péndulo
% g = 9.81; % aceleración debida a la gravedad
% 
% % Parámetros del controlador
% K = 26; % ganancia del modo deslizante
% lambda = 16; % constante del modo deslizante
% 
% % Condiciones iniciales del péndulo
% theta0 = pi/4; % ángulo inicial
% theta_dot0 = 0; % velocidad angular inicial
% 
% % Tiempo de simulación
% t_end = 10; % tiempo final
% dt = 0.01; % paso de tiempo
% t = 0:dt:t_end; % vector de tiempo
% 
% % Trayectoria deseada en theta y su velocidad
% theta_d = theta0 + (pi/2)*(1-cos((2*pi/t_end)*t)); % función sinusoidal
% theta_dot_d = (pi^2/t_end)*sin((2*pi/t_end)*t); % derivada de la función sinusoidal
% 
% % Inicialización de vectores de estado y controlador
% theta = zeros(size(t));
% theta_dot = zeros(size(t));
% theta(1) = theta0;
% theta_dot(1) = theta_dot0;
% s = zeros(size(t));
% u = zeros(size(t));
% u_bar = zeros(size(t));
% 
% % Bucle de simulación
% for i = 2:length(t)
%     % Cálculo del error de posición y velocidad
%     error = theta(i-1) - theta_d(i-1);
%     error_dot = theta_dot(i-1) - theta_dot_d(i-1);
%     
%     % Cálculo del modo deslizante
%     s(i) = error_dot + lambda*error;
%     
%     % Cálculo de la ley de control
%     u_bar(i) = -lambda*error_dot - K* sign(s(i));
%     u(i) = (1/m*l^2)*(u_bar(i) - m*g*l*sin(theta(i-1)));
%     
%     % Cálculo de la aceleración angular
%     theta_dotdot = (-g/l)*sin(theta(i-1)) + u(i);
%     
%     % Cálculo de la velocidad angular y posición angular utilizando el método de integración de Euler
%     theta_dot(i) = theta_dot(i-1) + theta_dotdot*dt;
%     theta(i) = theta(i-1) + theta_dot(i)*dt;
% end
% 
% % Gráfica de la posición angular, velocidad angular, posición deseada y velocidad deseada
% subplot(2,1,1)
% plot(t,theta,'b',t,theta_d,'r')
% ylabel('Posición Angular (rad)')
% xlabel('Tiempo (s)')
% legend('Posición Actual','Posición Deseada')
% subplot(2,1,2)
% plot(t,theta_dot,'b',t,theta_dot_d,'r')
% ylabel('Velocidad Angular (rad/s)')
% xlabel('Tiempo (s)')
% legend('Velocidad Actual','Velocidad Deseada')


% Parámetros del péndulo
m = 1; % masa del péndulo
l = 1; % longitud del péndulo
g = 9.81; % aceleración debida a la gravedad

% Parámetros del controlador
k1 = 55; % ganancia del primer nivel del backstepping
k2 = 4; % ganancia del segundo nivel del backstepping

% Condiciones iniciales del péndulo
theta0 = pi/4; % ángulo inicial
theta_dot0 = 0; % velocidad angular inicial

% Tiempo de simulación
t_end = 10; % tiempo final
dt = 0.01; % paso de tiempo
t = 0:dt:t_end; % vector de tiempo

% Trayectoria deseada en theta y su velocidad
theta_d = theta0 + (pi/2)*(1-cos((2*pi/t_end)*t)); % función sinusoidal
theta_dot_d = (pi^2/t_end)*sin((2*pi/t_end)*t); % derivada de la función sinusoidal

% Inicialización de vectores de estado y controlador
theta = zeros(size(t));
theta_dot = zeros(size(t));
theta(1) = theta0;
theta_dot(1) = theta_dot0;
v1 = zeros(size(t));
v2 = zeros(size(t));
u = zeros(size(t));

% Bucle de simulación
for i = 2:length(t)
    % Cálculo del error de posición y velocidad
    error = theta(i-1) - theta_d(i-1);
    error_dot = theta_dot(i-1) - theta_dot_d(i-1);
    
    % Cálculo del primer nivel del backstepping
    v1(i) = - error_dot - k1*error;
    
    % Cálculo del segundo nivel del backstepping
    v2(i) = v1(i) - k2*tanh(v1(i));
    
    % Cálculo de la entrada de control
    u(i) = (1/m*l^2)*(-m*g*l*sin(theta(i-1)) + v2(i));
    
    % Cálculo de la aceleración angular
    theta_dotdot = (-g/l)*sin(theta(i-1)) + u(i);
    
    % Cálculo de la velocidad angular y posición angular utilizando el método de integración de Euler
    theta_dot(i) = theta_dot(i-1) + theta_dotdot*dt;
    theta(i) = theta(i-1) + theta_dot(i)*dt;
end

% Gráfica de la posición angular, velocidad angular, posición deseada y velocidad deseada
subplot(2,1,1)
plot(t,theta,'b',t,theta_d,'r')
ylabel('Posición Angular (rad)')
xlabel('Tiempo (s)')
legend('Posición Actual','Posición Deseada')
subplot(2,1,2)
plot(t,theta_dot,'b',t,theta_dot_d,'r')
ylabel('Velocidad Angular (rad/s)')
xlabel('Tiempo (s)')
legend('Velocidad Actual','Velocidad Deseada')

