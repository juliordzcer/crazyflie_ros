clc
clear all
close all
%% Parametros de tiempo
% Tiempo de simulación
ti = -0.01;
dt = 0.0001; % Intervalo de tiempo (s)
t_max = 100; % Tiempo máximo de simulación (s)
t = ti:dt:t_max; % Vector de tiempo

%% Vector invariante en el tiempo de formacion.
cx = -0.1;
cy = 0.15;
cz = 0;

%% Constantes, Inercias, coeficientes aerodinamicos.
% Constantes
g = 9.8; % Aceleración debido a la gravedad (m/s^2)
m=0.032; % Masa del quadrotor (kg)

ax=0.91e-9;    % kg/s
ay=0.91e-9;    % kg/s
az=0.91e-9;   % kg/s

Jx=9.827e-05;
Jy=8.185e-05;
Jz=9.613e-05;

d=1;
w1=0.4;
w2=0.6;
w3=0.8;


%% Agente 1 
% Parametros de los controladores

kp_x = 38.50;
kd_x = 10.50;
ki_x = 15.00;

kp_y = 38.50;
kd_y = 10.50;
ki_y = 15.00;

kp_z = 29.20;
kd_z = 10.00;
ki_z = 18.00;

kpphi = 3800;
kdphi = 300;
kiphi = 1800;

kptheta = 3800;
kdtheta = 300;
kitheta = 1800;

kppsi = 4000;
kdpsi = 400;
kipsi = 1200;

% Estado inicial
x1   = -0.15;   % Posición en x (m)
y1   =  0.2;   % Posición en y (m)
z1 = 0; % Posición en z (m)
xp1 = 0; % Velocidad en x (m/s)
yp1 = 0; % Velocidad en y (m/s)
zp1 = 0; % Velocidad en z (m/s)
xpp1 = 0; % Aceleracion en x (m/s^2)
ypp1 = 0; % Aceleracion en y (m/s^2)
zpp1 = 0; % Aceleracion en z (m/s^2)

phi1 = 0; % Posición en phi (deg)
theta1 = 0; % Posición en theta (deg)
psi1 = 0; % Posición en psi (deg)
phip1 = 0; % Velocidad en phi (deg/s)
thetap1 = 0; % Velocidad en theta (deg/s)
psip1 = 0; % Velocidad en psi (deg/s)
phipp1 = 0; % Aceleracion en phi (deg/s^2)
thetapp1 = 0; % Aceleracion en theta (deg/s^2)
psipp1 = 0; % Aceleracion en psi (deg/s^2)

ex_prev1 = 0; % Error previo en x (m)
iex1 = 0; % Integral del error en x (m)
ey_prev1 = 0; % Error previo en y (m)
iey1 = 0; % Integral del error en y (m)
ez_prev1 = 0; % Error previo en z (m)
iez1 = 0; % Integral del error en z (m)

ephi_prev1 = 0; % Error previo en phi (deg)
iephi1 = 0; % Integral del error en phi (deg)
etheta_prev1 = 0; % Error previo en theta (deg)
ietheta1 = 0; % Integral del error en theta (deg)
epsi_prev1 = 0; % Error previo en psi (deg)
iepsi1 = 0; % Integral del error en psi (deg)

z31 = 0;
phi31 = 0;
theta31 = 0;
psi31 = 0;

% Inicialización de vectores para almacenar los resultados
X1 = zeros(length(t), 1);
Y1 = zeros(length(t), 1);
Z1 = zeros(length(t), 1);
XP1 = zeros(length(t), 1);
YP1 = zeros(length(t), 1);
ZP1 = zeros(length(t), 1);
XPP1 = zeros(length(t), 1);
YPP1 = zeros(length(t), 1);
ZPP1 = zeros(length(t), 1);

PHI1 = zeros(length(t), 1);
THETA1 = zeros(length(t), 1);
PSI1 = zeros(length(t), 1);
PHIP1 = zeros(length(t), 1);
THETAP1 = zeros(length(t), 1);
PSIP1 = zeros(length(t), 1);
PHIPP1 = zeros(length(t), 1);
THETAPP1 = zeros(length(t), 1);
PSIPP1 = zeros(length(t), 1);

XD1 = zeros(length(t), 1);
YD1 = zeros(length(t), 1);
ZD1 = zeros(length(t), 1);
PHID1 = zeros(length(t), 1);
THETAD1 = zeros(length(t), 1);
PSID1 = zeros(length(t), 1);

EX1 = zeros(length(t), 1);
EY1 = zeros(length(t), 1);
EZ1 = zeros(length(t), 1);
EPHI1 = zeros(length(t), 1);
ETHETA1 = zeros(length(t), 1);
EPSI1 = zeros(length(t), 1);

%% Agente 2 
% Parametros de los controladores


% Estado inicial
x2   = 0; % Posición en x (m)
y2   = 0; % Posición en y (m)
z2 = 0; % Posición en z (m)
xp2 = 0; % Velocidad en x (m/s)
yp2 = 0; % Velocidad en y (m/s)
zp2 = 0; % Velocidad en z (m/s)
xpp2 = 0; % Aceleracion en x (m/s^2)
ypp2 = 0; % Aceleracion en y (m/s^2)
zpp2 = 0; % Aceleracion en z (m/s^2)

phi2 = 0; % Posición en phi (deg)
theta2 = 0; % Posición en theta (deg)
psi2 = 0; % Posición en psi (deg)
phip2 = 0; % Velocidad en phi (deg/s)
thetap2 = 0; % Velocidad en theta (deg/s)
psip2 = 0; % Velocidad en psi (deg/s)
phipp2 = 0; % Aceleracion en phi (deg/s^2)
thetapp2 = 0; % Aceleracion en theta (deg/s^2)
psipp2 = 0; % Aceleracion en psi (deg/s^2)

ex_prev2 = 0; % Error previo en x (m)
iex2 = 0; % Integral del error en x (m)
ey_prev2 = 0; % Error previo en y (m)
iey2 = 0; % Integral del error en y (m)
ez_prev2 = 0; % Error previo en z (m)
iez2 = 0; % Integral del error en z (m)

ephi_prev2 = 0; % Error previo en phi (deg)
iephi2 = 0; % Integral del error en phi (deg)
etheta_prev2 = 0; % Error previo en theta (deg)
ietheta2 = 0; % Integral del error en theta (deg)
epsi_prev2 = 0; % Error previo en psi (deg)
iepsi2 = 0; % Integral del error en psi (deg)

% Derivada de las trayectorias deseadas

xd1_prev  = 0;
xdp1_prev = 0;
yd1_prev  = 0;
ydp1_prev = 0;
zd1_prev  = 0;
zdp1_prev = 0;
psid1_prev = 0;
psidp1_prev = 0;

xd2_prev  = 0;
xdp2_prev = 0;
yd2_prev  = 0;
ydp2_prev = 0;
zd2_prev  = 0;
zdp2_prev = 0;
psid2_prev = 0;
psidp2_prev = 0;

z32 = 0;
phi32 = 0;
theta32 = 0;
psi32 = 0;



% Inicialización de vectores para almacenar los resultados
X2 = zeros(length(t), 1);
Y2 = zeros(length(t), 1);
Z2 = zeros(length(t), 1);
XP2 = zeros(length(t), 1);
YP2 = zeros(length(t), 1);
ZP2 = zeros(length(t), 1);
XPP2 = zeros(length(t), 1);
YPP2 = zeros(length(t), 1);
ZPP2 = zeros(length(t), 1);

PHI2 = zeros(length(t), 1);
THETA2 = zeros(length(t), 1);
PSI2 = zeros(length(t), 1);
PHIP2 = zeros(length(t), 1);
THETAP2 = zeros(length(t), 1);
PSIP2 = zeros(length(t), 1);
PHIPP2 = zeros(length(t), 1);
THETAPP2 = zeros(length(t), 1);
PSIPP2 = zeros(length(t), 1);

XD2 = zeros(length(t), 1);
YD2 = zeros(length(t), 1);
ZD2 = zeros(length(t), 1);
PHID2 = zeros(length(t), 1);
THETAD2 = zeros(length(t), 1);
PSID2 = zeros(length(t), 1);

EX2 = zeros(length(t), 1);
EY2 = zeros(length(t), 1);
EZ2 = zeros(length(t), 1);
EPHI2 = zeros(length(t), 1);
ETHETA2 = zeros(length(t), 1);
EPSI2 = zeros(length(t), 1);

% Senales de control
UZ2       = zeros(length(t), 1);
TAUPHI2   = zeros(length(t), 1);
TAUTHETA2 = zeros(length(t), 1);
TAUPSI2   = zeros(length(t), 1);

UZ1       = zeros(length(t), 1);
TAUPHI1   = zeros(length(t), 1);
TAUTHETA1 = zeros(length(t), 1);
TAUPSI1   = zeros(length(t), 1);

%% Almacenamiento de valores de la perturbacion.

DX = zeros(length(t), 1);
DY = zeros(length(t), 1);
DZ = zeros(length(t), 1);
DPHI = zeros(length(t), 1);
DTHETA = zeros(length(t), 1);
DPSI = zeros(length(t), 1);

%% Parametros de la trayectoria deseada.
r = 1;
f = pi/6;

%% Bucle de simulación
for i = 1:length(t)
    
    % Trayectoria deseada del agente 2 (Lider)
   
    xd2     = r*(atan(15)+atan(dt*i-15)).*cos(f*dt*i);
    xdp2    = (xd2 - xd2_prev) / dt;
    xdpp2   = (xdp2 - xdp2_prev) / dt;
    
    yd2     = r*(atan(15)+atan(dt*i-15)).*sin(f*dt*i);
    ydp2    = (yd2 - yd2_prev) / dt;
    ydpp2   = (ydp2 - ydp2_prev) / dt;
    
    zd2     = 1/2*(1+tanh(((dt*i-5)-2.5)))+0.1*(1+tanh((dt*i-35)/3));
    zdp2    = (zd2  - zd2_prev ) / dt;
    zdpp2   = (zdp2 - zdp2_prev) / dt;
    
    psid2   = sin(f*dt*i);
    psidp2    = (psid2  - psid2_prev ) / dt;
    psidpp2   = (psidp2 - psidp2_prev) / dt;
    
    % Perturbaciones 
    dx     = d * (0.3*sin(w1*dt*i));
    dy     = d * (0.3*cos(w1*dt*i));
    dz     = d * (-0.5+0.1*sin(w1*dt*i)-0.1*sin(w2*dt*i)+0.1*cos(w3*dt*i));
    dphi   = d * (-0.5+0.2*sin(w1*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w2*dt*i));
    dtheta = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w1*dt*i)+0.2*cos(w3*dt*i));
    dpsi   = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w3*dt*i));
    
    % Cálculo del error de posicion del lider
    ex2 = x2 - xd2;
    ey2 = y2 - yd2;
    ez2 = z2 - zd2;
    
    % Cálculo de la integral del error de posicion del lider
    iex2 = iex2 + ex2 * dt;
    iey2 = iey2 + ey2 * dt;
    iez2 = iez2 + ez2 * dt; 
    
    % Cálculo de la derivada del error de posicion
    exp2 = (ex2 - ex_prev2) / dt;
    eyp2 = (ey2 - ey_prev2) / dt;
    ezp2 = (ez2 - ez_prev2) / dt;
    
 
    nu_x2 = -ki_x * iex2 - kp_x * ex2 - kd_x * exp2 - ax * xp2 + xdpp2;
    nu_y2 = -ki_y * iey2 - kp_y * ey2 - kd_y * eyp2 - ay * yp2 + ydpp2;
    nu_z2 = -ki_z * iez2 - kp_z * ez2 - kd_z * ezp2 - az * zp2 + zdpp2;
    
    u2 = (sqrt(nu_x2^2 + nu_y2^2 +(nu_z2 + g)^2))*m;
    
    phid2 = asin ((m/u2)*(nu_x2*sin(psid2) - nu_y2*cos(psid2))); 
    thetad2 = atan ((nu_x2*cos(psid2) + nu_y2*sin(psid2))/(nu_z2+g));
    
    % Cálculo del error de orientacion del lider
    ephi2   = phi2 - phid2;
    etheta2 = theta2 - thetad2;
    epsi2   = psi2 - psid2;    
    
    % Cálculo de la integral del error de posicion del lider
    iephi2   = iephi2 + ephi2 * dt;
    ietheta2 = ietheta2 + etheta2 * dt;
    iepsi2   = iepsi2 + epsi2 * dt;
    
    % Cálculo de la derivada del error de posicion del lider 
    ephip2   = (ephi2 - ephi_prev2) / dt;
    ethetap2 = (etheta2 - etheta_prev2) / dt;
    epsip2   = (epsi2 - epsi_prev2) / dt;  
    
    
    % Control de posicion para el lider.
    % Control de Phi     
    tau_bar_phi2 = - kpphi*ephi2 - kiphi*iephi2 - kdphi*ephip2;
    tau_phi2=Jx*(tau_bar_phi2-((Jy-Jz)/Jx)*thetap2*psip2);

    % Control de Theta
    tau_bar_theta2 = - kptheta*etheta2 - kitheta*ietheta2 - kdtheta*ethetap2;
    tau_theta2=Jy*(tau_bar_theta2-((Jz-Jx)/Jy)*phip2*psip2);
   
    % Control de Psi
    tau_bar_psi2 = -kppsi*epsi2 - kipsi*iepsi2 - kdpsi*epsip2;
    tau_psi2 = Jz*(tau_bar_psi2-((Jx-Jy)/Jz)*thetap2*phip2+psidpp2);
       
    % Modelo dinamico del quadrotor del lider 
    xpp2     = (u2/m)*(cos(phi2) * sin(theta2)*cos(psi2)+sin(phi2)*sin(psi2))+ax*xp2+dx;
    ypp2     = (u2/m)*(cos(phi2) * sin(theta2)*sin(psi2)-sin(phi2)*cos(psi2))+ay*yp2+dy;
    zpp2     = (u2/m)*(cos(phi2) * cos(theta2))-g+az*zp2+dz;
    phipp2   = (tau_phi2/Jx)   + ((Jy-Jz)/Jx) * thetap2 *psip2+dphi;
    thetapp2 = (tau_theta2/Jy) + ((Jz-Jx)/Jy) * phip2*psip2+dtheta;
    psipp2   = (tau_psi2/Jz)   + ((Jx-Jy)/Jz) * thetap2 *phip2+dpsi;
    
    xp2 = xp2 + xpp2 * dt;
    yp2 = yp2 + ypp2 * dt;
    zp2 = zp2 + zpp2 * dt;
    phip2   = phip2 + phipp2 * dt ;
    thetap2 = thetap2 + thetapp2 * dt;
    psip2   = psip2 + psipp2 * dt;
    
    x2 = x2 + xp2 * dt;
    y2 = y2 + yp2 * dt;
    z2 = z2 + zp2 * dt;
    phi2   = phi2 + phip2 * dt ;
    theta2 = theta2 + thetap2 * dt;
    psi2   = psi2 + psip2 * dt;    
    
    % Almacenamiento de los resultados en los vectore
    
    X2(i) = x2;
    Y2(i) = y2;
    Z2(i) = z2;
    XP2(i) = xp2;
    YP2(i) = yp2;
    ZP2(i) = zp2;
    XPP2(i) = xpp2;
    YPP2(i) = ypp2;
    ZPP2(i) = zpp2;

    PHI2(i) = phi2;
    THETA2(i) = theta2;
    PSI2(i) = psi2;
    PHIP2(i) = phip2;
    THETAP2(i) = thetap2;
    PSIP2(i) = psip2;
    PHIPP2(i) = phipp2;
    THETAPP2(i) = thetapp2;
    PSIPP2(i) = psipp2;
    
    XD2(i) = xd2;
    YD2(i) = yd2;
    ZD2(i) = zd2;
    PHID2(i) = phid2;
    THETAD2(i) = thetad2;
    PSID2(i) = psid2;
    
    EX2(i) = ex2;
    EY2(i) = ey2;
    EZ2(i) = ez2;
    EPHI2(i) = ephi2;
    ETHETA2(i) = etheta2;
    EPSI2(i) = epsi2;
    
    UZ2(i)       = u2;
    TAUPHI2(i)   = tau_phi2;
    TAUTHETA2(i) = tau_theta2;
    TAUPSI2(i)   = tau_psi2;
    
    DX(i) = dx;
    DY(i) = dy;
    DZ(i) = dz;
    DPHI(i) = dphi;
    DTHETA(i) = dtheta;
    DPSI(i) = dpsi;
    
    ex_prev2 = ex2;
    ey_prev2 = ey2;
    ez_prev2 = ez2;
    ephi_prev2 = ephi2;
    etheta_prev2 = etheta2;
    epsi_prev2 = epsi2;
    
    % Derivada de las trayectorias deseadas
    xd2_prev  = xd2;
    xdp2_prev = xdp2;
    yd2_prev  = yd2;
    ydp2_prev = ydp2;
    zd2_prev  = zd2;
    zdp2_prev = zdp2;
    psid2_prev = psid2;
    psidp2_prev = psidp2;
    
        % Trayectoria deseada del agente 1 (Seguidor)    
    xd1     = x2 + cx;
    xdp1    = (xd1 - xd1_prev) / dt;
    xdpp1   = (xdp1 - xdp1_prev) / dt;
    
    yd1     = y2 + cy;
    ydp1    = (yd1 - yd1_prev) / dt;
    ydpp1   = (ydp1 - ydp1_prev) / dt;
    
    zd1     = z2 + cz;
    zdp1    = (zd1  - zd1_prev ) / dt;
    zdpp1   = (zdp1 - zdp1_prev) / dt;
    
    psid1   = psi2;
    psidp1    = (psid1  - psid1_prev ) / dt;
    psidpp1   = (psidp1 - psidp1_prev) / dt;
    
    % Perturbaciones 
    dx     = d * (0.3*sin(w1*dt*i));
    dy     = d * (0.3*cos(w1*dt*i));
    dz     = d * (-0.5+0.1*sin(w1*dt*i)-0.1*sin(w2*dt*i)+0.1*cos(w3*dt*i));
    dphi   = d * (-0.5+0.2*sin(w1*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w2*dt*i));
    dtheta = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w1*dt*i)+0.2*cos(w3*dt*i));
    dpsi   = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w3*dt*i));
    
    % Cálculo del error de posicion del seguidor 
    ex1 = x1 - xd1;
    ey1 = y1 - yd1;
    ez1 = z1 - zd1;
    
    % Cálculo de la integral del error de posicion del seguidor
    iex1 = iex1 + ex1 * dt;
    iey1 = iey1 + ey1 * dt;
    iez1 = iez1 + ez1 * dt;
    
    % Cálculo de la derivada del error de posicion del seguidor
    exp1 = (ex1 - ex_prev1) / dt;
    eyp1 = (ey1 - ey_prev1) / dt;
    ezp1 = (ez1 - ez_prev1) / dt;
    
    c1 = (u2/m)*(cos(phi2)*sin(theta2)*cos(psi2) + sin(phi2)*sin(psi2)) + dx;
    c2 = (u2/m)*(cos(phi2)*sin(theta2)*sin(psi2) - sin(phi2)*cos(psi2)) + dy;
    c3 = (u2/m)*(cos(phi2)*cos(theta2)) + az * zp2 + dz;
    
    nu_x1 = -ki_x * iex1 - kp_x * ex1 - kd_x * exp1 - ax * exp1;
    nu_y1 = -ki_y * iey1 - kp_y * ey1 - kd_y * eyp1 - ay * eyp1;
    nu_z1 = -ki_z * iez1 - kp_z * ez1 - kd_z * ezp1 - az * ezp1;
 
   
    u1 = (sqrt((nu_x1+ c1)^2 + (nu_y1+ c2)^2 +(nu_z1+ c3)^2))*m;
    
    phid1 = asin((m/u1)*((nu_x1 + c1) * sin(psid1) - (nu_y1 + c2) * cos(psid1))); 
    thetad1 = atan(((nu_x1+ c1) * cos(psid1) + (nu_y1+ c2) * sin(psid1))/(nu_z1+ c3));
    
    % Cálculo del error de orientacion del seguidor
    ephi1   = phi1 - phid1;
    etheta1 = theta1 - thetad1;
    epsi1   = psi1 - psid1;
    
    % Cálculo de la integral del error de posicion del seguidor 
    iephi1   = iephi1 + ephi1 * dt;
    ietheta1 = ietheta1 + etheta1 * dt;
    iepsi1   = iepsi1 + epsi1 * dt;
    
    % Cálculo de la derivada del error de posicion del seguidor 
    ephip1   = (ephi1 - ephi_prev1) / dt;
    ethetap1 = (etheta1 - etheta_prev1) / dt;
    epsip1   = (epsi1 - epsi_prev1) / dt;   
            
    % Control de posicion para el seguidor
    % Control de Phi    
    tau_bar_phi1 = - kpphi*ephi1 - kiphi*iephi1 - kdphi*ephip1;
    tau_phi1 =  Jx * (tau_bar_phi1-((Jy-Jz)/Jx)*thetap1*psip1);

    % Control de Theta
    tau_bar_theta1 = - kptheta*etheta1 - kitheta*ietheta1 - kdtheta*ethetap1;
    tau_theta1 = Jy*(tau_bar_theta1-((Jz-Jx)/Jy)*phip1*psip1);
   
    % Control de Psi
    tau_bar_psi1 = -kppsi*epsi1 - kipsi*iepsi1 - kdpsi*epsip1;
    tau_psi1 = Jz*(tau_bar_psi1  -((Jx-Jy)/Jz)*(thetap1*phip1));
    
    % Modelo dinamico del quadrotor del seguidor
    xpp1     = (u1/m)*(cos(phi1) * sin(theta1)*cos(psi1)+sin(phi1)*sin(psi1))+ax*xp1+dx;
    ypp1     = (u1/m)*(cos(phi1) * sin(theta1)*sin(psi1)-sin(phi1)*cos(psi1))+ay*yp1+dy;
    zpp1     = (u1/m)*(cos(phi1) * cos(theta1))-g+az*zp1+dz;
    phipp1   = (tau_phi1/Jx)   + ((Jy-Jz)/Jx) * thetap1 *psip1+dphi;
    thetapp1 = (tau_theta1/Jy) + ((Jz-Jx)/Jy) * phip1*psip1+dtheta;
    psipp1   = (tau_psi1/Jz)   + ((Jx-Jy)/Jz) * thetap1 *phip1+dpsi;
    
    xp1 = xp1 + xpp1 * dt;
    yp1 = yp1 + ypp1 * dt;
    zp1 = zp1 + zpp1 * dt;
    phip1   = phip1 + phipp1 * dt ;
    thetap1 = thetap1 + thetapp1 * dt;
    psip1   = psip1 + psipp1 * dt;
    
    x1 = x1 + xp1 * dt;
    y1 = y1 + yp1 * dt;
    z1 = z1 + zp1 * dt;
    phi1   = phi1 + phip1 * dt ;
    theta1 = theta1 + thetap1 * dt;
    psi1   = psi1 + psip1 * dt;

   
    % Almacenamiento de los resultados en los vectores
    
    X1(i) = x1;
    Y1(i) = y1;
    Z1(i) = z1;
    
    XP1(i) = xp1;
    YP1(i) = yp1;
    ZP1(i) = zp1;
    
    XPP1(i) = xpp1;
    YPP1(i) = ypp1;
    ZPP1(i) = zpp1;

    PHI1(i) = phi1;
    THETA1(i) = theta1;
    PSI1(i) = psi1;
    
    PHIP1(i) = phip1;
    THETAP1(i) = thetap1;
    PSIP1(i) = psip1;
    
    PHIPP1(i) = phipp1;
    THETAPP1(i) = thetapp1;
    PSIPP1(i) = psipp1;
    
    XD1(i) = xd1;
    YD1(i) = yd1;
    ZD1(i) = zd1;
    PHID1(i) = phid1;
    THETAD1(i) = thetad1;
    PSID1(i) = psid1;
    
    EX1(i) = ex1;
    EY1(i) = ey1;
    EZ1(i) = ez1;
    EPHI1(i) = ephi1;
    ETHETA1(i) = etheta1;
    EPSI1(i) = epsi1;
    
    % Senales de control
    UZ1(i)       = u1;
    TAUPHI1(i)   = tau_phi1;
    TAUTHETA1(i) = tau_theta1;
    TAUPSI1(i)   = tau_psi1;
    
    % Actualización del error previo
    ex_prev1 = ex1;
    ey_prev1 = ey1;
    ez_prev1 = ez1;
    ephi_prev1 = ephi1;
    etheta_prev1 = etheta1;
    epsi_prev1 = epsi1;
    
    % Derivada de las trayectorias deseadas
    xd1_prev  = xd1;
    xdp1_prev = xdp1;
    yd1_prev  = yd1;
    ydp1_prev = ydp1;
    zd1_prev  = zd1;
    zdp1_prev = zdp1;
    psid1_prev = psid1;
    psidp1_prev = psidp1;

end  


%% Extraccion de datos
pm = -ti^(-1)+1; 
t=t(pm:end);

EX1_PID     = EX1(pm:end);
EY1_PID     = EY1(pm:end); 
EZ1_PID     = EZ1(pm:end);
EPHI1_PID   = EPHI1(pm:end);
ETHETA1_PID = ETHETA1(pm:end);
EPSI1_PID   = EPSI1(pm:end);

EX2_PID = EX2(pm:end);
EY2_PID = EY2(pm:end);
EZ2_PID = EZ2(pm:end);
EPHI2_PID = EPHI2(pm:end);
ETHETA2_PID = ETHETA2(pm:end);
EPSI2_PID = EPSI2(pm:end);


X1_PID     = X1(pm:end);
Y1_PID     = Y1(pm:end); 
Z1_PID     = Z1(pm:end);
PHI1_PID   = PHI1(pm:end);
THETA1_PID = THETA1(pm:end);
PSI1_PID   = PSI1(pm:end);

X2_PID     = X2(pm:end);
Y2_PID     = Y2(pm:end);
Z2_PID     = Z2(pm:end);
PHI2_PID   = PHI2(pm:end);
THETA2_PID = THETA2(pm:end);
PSI2_PID   = PSI2(pm:end);

XD1_PID     = X1(pm:end);
YD1_PID     = Y1(pm:end); 
ZD1_PID     = Z1(pm:end);
PHID1_PID   = PHI1(pm:end);
THETAD1_PID = THETA1(pm:end);
PSID1_PID   = PSI1(pm:end);

XD2_PID     = XD2(pm:end);
YD2_PID     = YD2(pm:end);
ZD2_PID     = ZD2(pm:end);
PHID2_PID   = PHID2(pm:end);
THETAD2_PID = THETAD2(pm:end);
PSID2_PID   = PSID2(pm:end);

UZ1_PID       = UZ1(pm:end);
TAUPHI1_PID   = TAUPHI1(pm:end); 
TAUTHETA1_PID = TAUTHETA1(pm:end);
TAUPSI1_PID   = TAUPSI1(pm:end);

UZ2_PID       = UZ2(pm:end);
TAUPHI2_PID   = TAUPHI2(pm:end); 
TAUTHETA2_PID = TAUTHETA2(pm:end);
TAUPSI2_PID   = TAUPSI2(pm:end);

save('PID_Errores.mat','EX1_PID','EX2_PID','EY1_PID','EY2_PID','EZ1_PID','EZ2_PID','EPHI1_PID','EPHI2_PID','ETHETA1_PID','ETHETA2_PID','EPSI1_PID','EPSI2_PID','t','-v7.3');
save('PID_Estados.mat','X1_PID','X2_PID','Y1_PID','Y2_PID','Z1_PID','Z2_PID','PHI1_PID','PHI2_PID','THETA1_PID','THETA2_PID','PSI1_PID','PSI2_PID','-v7.3');
save('PID_Deseadas.mat','XD1_PID','XD2_PID','YD1_PID','YD2_PID','ZD1_PID','ZD2_PID','PHID1_PID','PHID2_PID','THETAD1_PID','THETAD2_PID','PSID1_PID','PSID2_PID','-v7.3')
save('PID_Control.mat','UZ1_PID','UZ2_PID','TAUPHI1_PID','TAUPHI2_PID','TAUTHETA1_PID','TAUTHETA2_PID','TAUPSI1_PID','TAUPSI2_PID','-v7.3')


figure(1)
hold on
plot(t,UZ2_PID)
plot(t,UZ1_PID)
leg1=legend('$1$','$m$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

mtr=2; ntr=3;

figure(2)
title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
subplot(mtr,ntr,1)
hold on
plot(t,XD2_PID)
plot(t,X2_PID)
plot(t,X1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$x$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
% ylim([-450 510]);
box on
hold off

subplot(mtr,ntr,2)
hold on
plot(t,YD2_PID)
plot(t,Y2_PID)
plot(t,Y1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$y$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(mtr,ntr,3)
hold on
plot(t,ZD2_PID)
plot(t,Z2_PID)
plot(t,Z1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$z$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(mtr,ntr,4)
hold on
plot(t,PHID2_PID)
plot(t,PHID1_PID)
plot(t,PHI2_PID)
plot(t,PHI1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\phi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off

subplot(mtr,ntr,5)
hold on
plot(t,THETAD2_PID)
plot(t,THETAD1_PID)
plot(t,THETA2_PID)
plot(t,THETA1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\theta$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off

subplot(mtr,ntr,6)
hold on
plot(t,PSID2_PID)
plot(t,PSI2_PID)
plot(t,PSI1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\psi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off
 
figure(3)
title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
subplot(mtr,ntr,1)
hold on
plot(t,EX2_PID)
plot(t,EX1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$x$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
% ylim([-450 510]);
box on
hold off

subplot(mtr,ntr,2)
hold on
plot(t,EY2_PID)
plot(t,EY1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$y$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(mtr,ntr,3)
hold on
plot(t,EZ2_PID)
plot(t,EZ1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$z$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(mtr,ntr,4)
hold on
plot(t,EPHI2_PID)
plot(t,EPHI1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\phi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off

subplot(mtr,ntr,5)
hold on
plot(t,ETHETA2_PID)
plot(t,ETHETA1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\theta$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off

subplot(mtr,ntr,6)
hold on
plot(t,EPSI2_PID)
plot(t,EPSI1_PID)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$\psi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');% 
box on
hold off
 