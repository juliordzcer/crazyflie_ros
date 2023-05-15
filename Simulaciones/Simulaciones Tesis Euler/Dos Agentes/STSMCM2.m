
clc
clear all
close all

%% Parametros de tiempo
% Tiempo de simulación
ti = -0.01;
dt = 0.0001; % Intervalo de tiempo (s)
t_max = 100; % Tiempo máximo de simulación (s)
t = ti:dt:t_max; % Vector de tiempo

%% Vector invariante en el tiempo de formacion
cx = 0.1;
cy = 0.3;
cz = 0;

%% Constantes, Inercias, coeficientes aerodinamicos.
% Constantes
g = 9.8; % Aceleración debido a la gravedad (m/s^2)
m=0.032; % Masa del quadrotor (kg)

ax=0.91e-9;    % kg/s
ay=0.91e-9;    % kg/s
az=0.91e-9;    % kg/s

Jx=9.827e-05;
Jy=8.185e-05;
Jz=9.613e-05;

d=1;
w1=0.4;
w2=0.6;
w3=0.8;

%% Parametros de los controladores

kpx = 8;
kdx = 7;
kix = 1;

kpy = 8;
kdy = 7;
kiy = 1;

zeta_z = 32.5;
k0_z = 3.0;
k1_z = 1.5*zeta_z^(1/2);
k2_z = 1.1*zeta_z;

zeta_phi = 52.5;
k0_phi = 2;
k1_phi = 1.5*zeta_phi^(1/2);
k2_phi = 1.1*zeta_phi;

zeta_theta = 52.5;
k0_theta = 2;
k1_theta = 1.5*zeta_theta^(1/2);
k2_theta = 1.1*zeta_theta;

zeta_psi = 45.5;
k0_psi = 2;
k1_psi = 1.5*zeta_psi^(1/2);
k2_psi = 1.1*zeta_psi;


%% Agente 1 

% Estado inicial
x1   = 0.1;   % Posición en x (m)
y1   = 0.2;   % Posición en y (m)
z1   = 0;   % Posición en z (m)
xp1  = 0;   % Velocidad en x (m/s)
yp1  = 0;   % Velocidad en y (m/s)
zp1  = 0;   % Velocidad en z (m/s)
xpp1 = 0;   % Aceleracion en x (m/s^2)
ypp1 = 0;   % Aceleracion en y (m/s^2)
zpp1 = 0;   % Aceleracion en z (m/s^2)

phi1     = 0;   % Posición en phi (deg)
theta1   = 0;   % Posición en theta (deg)
psi1     = 0;   % Posición en psi (deg)
phip1    = 0;   % Velocidad en phi (deg/s)
thetap1  = 0;   % Velocidad en theta (deg/s)
psip1    = 0;   % Velocidad en psi (deg/s)
phipp1   = 0;   % Aceleracion en phi (deg/s^2)
thetapp1 = 0;   % Aceleracion en theta (deg/s^2)
psipp1   = 0;   % Aceleracion en psi (deg/s^2)

ex_prev1 = 0;  % Error previo en x (m)
iex1     = 0;  % Integral del error en x (m)
ey_prev1 = 0;  % Error previo en y (m)
iey1     = 0;  % Integral del error en y (m)
ez_prev1 = 0;  % Error previo en z (m)
iez1     = 0;  % Integral del error en z (m)

ephi_prev1   = 0;   % Error previo en phi (deg)
iephi1       = 0;   % Integral del error en phi (deg)
etheta_prev1 = 0;   % Error previo en theta (deg)
ietheta1     = 0;   % Integral del error en theta (deg)
epsi_prev1   = 0;   % Error previo en psi (deg)
iepsi1       = 0;   % Integral del error en psi (deg)


z31     = 0;
phi31   = 0;
theta31 = 0;
psi31   = 0;


%% Agente 2 

% Estado inicial
x2   = 0.3; % Posición en x (m)
y2   = 0.1; % Posición en y (m)
z2   = 0; % Posición en z (m)
xp2  = 0; % Velocidad en x (m/s)
yp2  = 0; % Velocidad en y (m/s)
zp2  = 0; % Velocidad en z (m/s)
xpp2 = 0; % Aceleracion en x (m/s^2)
ypp2 = 0; % Aceleracion en y (m/s^2)
zpp2 = 0; % Aceleracion en z (m/s^2)

phi2     = 0; % Posición en phi (deg)
theta2   = 0; % Posición en theta (deg)
psi2     = 0; % Posición en psi (deg)
phip2    = 0; % Velocidad en phi (deg/s)
thetap2  = 0; % Velocidad en theta (deg/s)
psip2    = 0; % Velocidad en psi (deg/s)
phipp2   = 0; % Aceleracion en phi (deg/s^2)
thetapp2 = 0; % Aceleracion en theta (deg/s^2)
psipp2   = 0; % Aceleracion en psi (deg/s^2)

ex_prev2 = 0; % Error previo en x (m)
iex2     = 0; % Integral del error en x (m)
ey_prev2 = 0; % Error previo en y (m)
iey2     = 0; % Integral del error en y (m)
ez_prev2 = 0; % Error previo en z (m)
iez2     = 0; % Integral del error en z (m)

ephi_prev2   = 0; % Error previo en phi (deg)
iephi2       = 0; % Integral del error en phi (deg)
etheta_prev2 = 0; % Error previo en theta (deg)
ietheta2     = 0; % Integral del error en theta (deg)
epsi_prev2   = 0; % Error previo en psi (deg)
iepsi2       = 0; % Integral del error en psi (deg)

z32     = 0;
phi32   = 0;
theta32 = 0;
psi32   = 0;


%% Almacenamiento de valores

DX     = zeros(length(t), 1);
DY     = zeros(length(t), 1);
DZ     = zeros(length(t), 1);
DPHI   = zeros(length(t), 1);
DTHETA = zeros(length(t), 1);
DPSI   = zeros(length(t), 1);


% Inicialización de vectores para almacenar los resultados
X1   = zeros(length(t), 1);
Y1   = zeros(length(t), 1);
Z1   = zeros(length(t), 1);
XP1  = zeros(length(t), 1);
YP1  = zeros(length(t), 1);
ZP1  = zeros(length(t), 1);
XPP1 = zeros(length(t), 1);
YPP1 = zeros(length(t), 1);
ZPP1 = zeros(length(t), 1);

PHI1     = zeros(length(t), 1);
THETA1   = zeros(length(t), 1);
PSI1     = zeros(length(t), 1);
PHIP1    = zeros(length(t), 1);
THETAP1  = zeros(length(t), 1);
PSIP1    = zeros(length(t), 1);
PHIPP1   = zeros(length(t), 1);
THETAPP1 = zeros(length(t), 1);
PSIPP1   = zeros(length(t), 1);

XD1     = zeros(length(t), 1);
YD1     = zeros(length(t), 1);
ZD1     = zeros(length(t), 1);
PHID1   = zeros(length(t), 1);
THETAD1 = zeros(length(t), 1);
PSID1   = zeros(length(t), 1);

EX1     = zeros(length(t), 1);
EY1     = zeros(length(t), 1);
EZ1     = zeros(length(t), 1);
EPHI1   = zeros(length(t), 1);
ETHETA1 = zeros(length(t), 1);
EPSI1   = zeros(length(t), 1);

% Inicialización de vectores para almacenar los resultados
X2   = zeros(length(t), 1);
Y2   = zeros(length(t), 1);
Z2   = zeros(length(t), 1);
XP2  = zeros(length(t), 1);
YP2  = zeros(length(t), 1);
ZP2  = zeros(length(t), 1);
XPP2 = zeros(length(t), 1);
YPP2 = zeros(length(t), 1);
ZPP2 = zeros(length(t), 1);

PHI2     = zeros(length(t), 1);
THETA2   = zeros(length(t), 1);
PSI2     = zeros(length(t), 1);
PHIP2    = zeros(length(t), 1);
THETAP2  = zeros(length(t), 1);
PSIP2    = zeros(length(t), 1);
PHIPP2   = zeros(length(t), 1);
THETAPP2 = zeros(length(t), 1);
PSIPP2   = zeros(length(t), 1);

XD2     = zeros(length(t), 1);
YD2     = zeros(length(t), 1);
ZD2     = zeros(length(t), 1);
PHID2   = zeros(length(t), 1);
THETAD2 = zeros(length(t), 1);
PSID2   = zeros(length(t), 1);

EX2     = zeros(length(t), 1);
EY2     = zeros(length(t), 1);
EZ2     = zeros(length(t), 1);
EPHI2   = zeros(length(t), 1);
ETHETA2 = zeros(length(t), 1);
EPSI2   = zeros(length(t), 1);

% Senales de control
UZ2       = zeros(length(t), 1);
TAUPHI2   = zeros(length(t), 1);
TAUTHETA2 = zeros(length(t), 1);
TAUPSI2   = zeros(length(t), 1);

UZ1       = zeros(length(t), 1);
TAUPHI1   = zeros(length(t), 1);
TAUTHETA1 = zeros(length(t), 1);
TAUPSI1   = zeros(length(t), 1);


%% Parametros de la trayectoria deseada.
r = 1;
f = pi/6;


%% Bucle de simulación
for i = 1:length(t)
    
    % Trayectoria deseada del agente 1 (Seguidor)
    
    xd1     = x2 + cx;
    yd1     = y2 + cy;
    zd1     = z2 + cz;    
    psid1   = psi2;

    
    % Trayectoria deseada del agente 2 (Lider)
   
    xd2     = r*(atan(15)+atan(dt*i-15)).*cos(f*dt*i);
    yd2     = r*(atan(15)+atan(dt*i-15)).*sin(f*dt*i);
    zd2     = 1/2*(1+tanh(((dt*i-5)-2.5)))+0.1*(1+tanh((dt*i-35)/3));  
    psid2   =  sin(f*dt*i);

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
    
    % Cálculo del error de posicion del lider
    ex2 = x2 - xd2;
    ey2 = y2 - yd2;
    ez2 = z2 - zd2;
    
    % Cálculo de la integral del error de posicion del seguidor
    iex1 = iex1 + ex1 * dt;
    iey1 = iey1 + ey1 * dt;
    iez1 = iez1 + ez1 * dt;
    
    % Cálculo de la integral del error de posicion del lider
    iex2 = iex2 + ex2 * dt;
    iey2 = iey2 + ey2 * dt;
    iez2 = iez2 + ez2 * dt; 
    
    % Cálculo de la derivada del error de posicion del seguidor
    exp1 = (ex1 - ex_prev1) / dt;
    eyp1 = (ey1 - ey_prev1) / dt;
    ezp1 = (ez1 - ez_prev1) / dt;
    
    % Cálculo de la derivada del error de posicion
    exp2 = (ex2 - ex_prev2) / dt;
    eyp2 = (ey2 - ey_prev2) / dt;
    ezp2 = (ez2 - ez_prev2) / dt;

    
    % Calculo de phi* y theta* del seguidor
    thetad1  = -(kpx*ex1 + kix*iex1 + kdx*exp1)/g; 
    phid1    =  (kpy*ey1 + kiy*iey1 + kdy*eyp1)/g;
    
     % Calculo de phi* y theta* del lider
    thetad2  = -(kpx*ex2 + kix*iex2 + kdx*exp2)/g; 
    phid2    =  (kpy*ey2 + kiy*iey2 + kdy*eyp2)/g;
    
    % Cálculo del error de orientacion del seguidor
    ephi1   = phi1 - phid1;
    etheta1 = theta1 - thetad1;
    epsi1   = psi1 - psid1;
    
    % Cálculo del error de orientacion del lider
    ephi2   = phi2 - phid2;
    etheta2 = theta2 - thetad2;
    epsi2   = psi2 - psid2;    
    
    % Cálculo de la integral del error de posicion del seguidor 
    iephi1   = iephi1 + ephi1 * dt;
    ietheta1 = ietheta1 + etheta1 * dt;
    iepsi1   = iepsi1 + epsi1 * dt;
    
    % Cálculo de la integral del error de posicion del lider
    iephi2   = iephi2 + ephi2 * dt;
    ietheta2 = ietheta2 + etheta2 * dt;
    iepsi2   = iepsi2 + epsi2 * dt;
    
    % Cálculo de la derivada del error de posicion del seguidor 
    ephip1   = (ephi1 - ephi_prev1) / dt;
    ethetap1 = (etheta1 - etheta_prev1) / dt;
    epsip1   = (epsi1 - epsi_prev1) / dt;
    
    % Cálculo de la derivada del error de posicion del lider 
    ephip2   = (ephi2 - ephi_prev2) / dt;
    ethetap2 = (etheta2 - etheta_prev2) / dt;
    epsip2   = (epsi2 - epsi_prev2) / dt;  
            
    % Cálculo de la fuerza requerida utilizando el controlador STSMCM para el lider.     
    z_mphi2 = ezp2 + k0_z*((abs(ez2)^(2/3))*sign(ez2));
    z32 = z32 +(-k2_z*sign(z_mphi2))*dt;
    z_bar2= -k1_z*((abs(z_mphi2)^(1/2))*sign(z_mphi2)) + z32;
    
    u2 =((z_bar2+g)/(cos(theta2)*cos(phi2)))*m;
    
    % Cálculo de la fuerza requerida utilizando el controlador STSMCM para el seguidor 
    z_mphi1 = ezp1 + k0_z*((abs(ez1)^(2/3))*sign(ez1));
    z31 = z31 +(-k2_z*sign(z_mphi1))*dt;
    z_bar1= -k1_z*((abs(z_mphi1)^(1/2))*sign(z_mphi1)) + z31;
    
    u1 =m*((z_bar1+ u2*cos(theta2)*cos(phi2))/(cos(theta1)*cos(phi1)));   

    % Control de posicion para el lider.
    % Control de Phi
    
    phi_mphi2 = ephip2 + k0_phi*((abs(ephi2)^(2/3))*sign(ephi2));
    phi32 = phi32 + (-k2_phi*sign(phi_mphi2))*dt;    
    tau_bar_phi2 = -k1_phi*((abs(phi_mphi2)^(1/2))*sign(phi_mphi2)) + phi32;

    tau_phi2=Jx*(tau_bar_phi2-((Jy-Jz)/Jx)*thetap2*psip2);

    % Control de Theta

    phi_mtheta2 = ethetap2 + k0_theta*((abs(etheta2)^(2/3))*sign(etheta2));
    theta32 = theta32 + (-k2_theta*sign(phi_mtheta2))*dt;
    tau_bar_theta2 = -k1_theta*((abs(phi_mtheta2)^(1/2))*sign(phi_mtheta2)) + theta32;
    
    tau_theta2=Jy*(tau_bar_theta2-((Jz-Jx)/Jy)*phip2*psip2);
   
    % Control de Psi
    
    phi_mpsi2 = epsip2 + k0_psi*((abs(epsi2)^(2/3))*sign(epsi2));
    psi32 = psi32 + (-k2_psi*sign(phi_mpsi2))*dt;
    tau_bar_psi2 = -k1_psi*((abs(phi_mpsi2)^(1/2))*sign(phi_mpsi2)) + psi32;

    tau_psi2 = Jz*(tau_bar_psi2-((Jx-Jy)/Jz)*thetap2*phip2);
        
    
    % Control de posicion para el seguidor
    
    % Control de Phi
    
    phi_mphi1 = ephip1 + k0_phi*((abs(ephi1)^(2/3))*sign(ephi1));
    phi31 = phi31 + (-k2_phi*sign(phi_mphi1))*dt;    
    tau_bar_phi1 = -k1_phi*((abs(phi_mphi1)^(1/2))*sign(phi_mphi1)) + phi31;

    tau_phi1=Jx*(tau_bar_phi1-((Jy-Jz)/Jx)*thetap1*psip1);

    % Control de Theta

    phi_mtheta1 = ethetap1 + k0_theta*((abs(etheta1)^(2/3))*sign(etheta1));
    theta31 = theta31 + (-k2_theta*sign(phi_mtheta1))*dt;
    tau_bar_theta1 = -k1_theta*((abs(phi_mtheta1)^(1/2))*sign(phi_mtheta1)) + theta31;
    
    tau_theta1 = Jy*(tau_bar_theta1-((Jz-Jx)/Jy)*phip1*psip1);
   
    % Control de Psi
    
    phi_mpsi1 = epsip1 + k0_psi*((abs(epsi1)^(2/3))*sign(epsi1));
    psi31 = psi31 + (-k2_psi*sign(phi_mpsi1))*dt;
    tau_bar_psi1 = -k1_psi*((abs(phi_mpsi1)^(1/2))*sign(phi_mpsi1)) + psi31;

    tau_psi1 = (Jz*(tau_bar_psi1-((Jx-Jy)/Jz)*thetap1*phip1))+tau_psi2;
    

    
    
    % Modelo dinamico del quadrotor del seguidor
    xpp1     = (u1/m)*(cos(phi1) * sin(theta1)*cos(psi1)+sin(phi1)*sin(psi1))-ax*xp1+dx;
    ypp1     = (u1/m)*(cos(phi1) * sin(theta1)*sin(psi1)-sin(phi1)*cos(psi1))-ay*yp1+dy;
    zpp1     = (u1/m)*(cos(phi1) * cos(theta1))-g-az*zp1+dz;
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
    
    % Modelo dinamico del quadrotor del lider 
    xpp2     = (u2/m)*(cos(phi2) * sin(theta2)*cos(psi2)+sin(phi2)*sin(psi2))-ax*xp2+dx;
    ypp2     = (u2/m)*(cos(phi2) * sin(theta2)*sin(psi2)-sin(phi2)*cos(psi2))-ay*yp2+dy;
    zpp2     = (u2/m)*(cos(phi2) * cos(theta2))-g-az*zp2+dz;
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
    
    
    % Almacenamiento de los resultados en los vectores
    
    X1(i)   = x1;
    Y1(i)   = y1;
    Z1(i)   = abs(z1);
    XP1(i)  = xp1;
    YP1(i)  = yp1;
    ZP1(i)  = zp1;
    XPP1(i) = xpp1;
    YPP1(i) = ypp1;
    ZPP1(i) = zpp1;

    PHI1(i)     = phi1;
    THETA1(i)   = theta1;
    PSI1(i)     = psi1;
    PHIP1(i)    = phip1;
    THETAP1(i)  = thetap1;
    PSIP1(i)    = psip1;
    PHIPP1(i)   = phipp1;
    THETAPP1(i) = thetapp1;
    PSIPP1(i)   = psipp1;
    
    XD1(i)     = xd1;
    YD1(i)     = yd1;
    ZD1(i)     = zd1;
    PHID1(i)   = phid1;
    THETAD1(i) = thetad1;
    PSID1(i)   = psid1;
    
    EX1(i)     = ex1;
    EY1(i)     = ey1;
    EZ1(i)     = ez1;
    EPHI1(i)   = ephi1;
    ETHETA1(i) = etheta1;
    EPSI1(i)   = epsi1;
    
    X2(i)   = x2;
    Y2(i)   = y2;
    Z2(i)   = z2;
    XP2(i)  = xp2;
    YP2(i)  = yp2;
    ZP2(i)  = zp2;
    XPP2(i) = xpp2;
    YPP2(i) = ypp2;
    ZPP2(i) = zpp2;

    PHI2(i)     = phi2;
    THETA2(i)   = theta2;
    PSI2(i)     = psi2;
    PHIP2(i)    = phip2;
    THETAP2(i)  = thetap2;
    PSIP2(i)    = psip2;
    PHIPP2(i)   = phipp2;
    THETAPP2(i) = thetapp2;
    PSIPP2(i)   = psipp2;
    
    XD2(i)     = xd2;
    YD2(i)     = yd2;
    ZD2(i)     = zd2;
    PHID2(i)   = phid2;
    THETAD2(i) = thetad2;
    PSID2(i)   = psid2;
    
    EX2(i)     = ex2;
    EY2(i)     = ey2;
    EZ2(i)     = ez2;
    EPHI2(i)   = ephi2;
    ETHETA2(i) = etheta2;
    EPSI2(i)   = epsi2; 
    
    DX(i)     = dx;
    DY(i)     = dy;
    DZ(i)     = dz;
    DPHI(i)   = dphi;
    DTHETA(i) = dtheta;
    DPSI(i)   = dpsi;
    
    % Senales de control

    UZ1(i)       = u1;
    TAUPHI1(i)   = tau_phi1;
    TAUTHETA1(i) = tau_theta1;
    TAUPSI1(i)   = tau_psi1;
    
    UZ2(i)       = u2;
    TAUPHI2(i)   = tau_phi2;
    TAUTHETA2(i) = tau_theta2;
    TAUPSI2(i)   = tau_psi2;

    % Actualización del error previo
    ex_prev1     = ex1;
    ey_prev1     = ey1;
    ez_prev1     = ez1;
    ephi_prev1   = ephi1;
    etheta_prev1 = etheta1;
    epsi_prev1   = epsi1;
    
    ex_prev2     = ex2;
    ey_prev2     = ey2;
    ez_prev2     = ez2;
    ephi_prev2   = ephi2;
    etheta_prev2 = etheta2;
    epsi_prev2   = epsi2;
    
end
TL = 12;
%% Extraccion de datos
if ti == 0
    pm = 1;
else
    pm = -ti^(-1)+1; 
end
t=t(pm:end);

EX1_STSMCM     = EX1(pm:end);
EY1_STSMCM     = EY1(pm:end); 
EZ1_STSMCM     = EZ1(pm:end);
EPHI1_STSMCM   = EPHI1(pm:end);
ETHETA1_STSMCM = ETHETA1(pm:end);
EPSI1_STSMCM   = EPSI1(pm:end);

EX2_STSMCM = EX2(pm:end);
EY2_STSMCM = EY2(pm:end);
EZ2_STSMCM = EZ2(pm:end);
EPHI2_STSMCM = EPHI2(pm:end);
ETHETA2_STSMCM = ETHETA2(pm:end);
EPSI2_STSMCM = EPSI2(pm:end);


X1_STSMCM     = X1(pm:end);
Y1_STSMCM     = Y1(pm:end); 
Z1_STSMCM     = Z1(pm:end);
PHI1_STSMCM   = PHI1(pm:end);
THETA1_STSMCM = THETA1(pm:end);
PSI1_STSMCM   = PSI1(pm:end);

X2_STSMCM     = X2(pm:end);
Y2_STSMCM     = Y2(pm:end);
Z2_STSMCM     = Z2(pm:end);
PHI2_STSMCM   = PHI2(pm:end);
THETA2_STSMCM = THETA2(pm:end);
PSI2_STSMCM   = PSI2(pm:end);

XD1_STSMCM     = X1(pm:end);
YD1_STSMCM     = Y1(pm:end); 
ZD1_STSMCM     = Z1(pm:end);
PHID1_STSMCM   = PHI1(pm:end);
THETAD1_STSMCM = THETA1(pm:end);
PSID1_STSMCM   = PSI1(pm:end);

XD2_STSMCM     = XD2(pm:end);
YD2_STSMCM     = YD2(pm:end);
ZD2_STSMCM     = ZD2(pm:end);
PHID2_STSMCM   = PHID2(pm:end);
THETAD2_STSMCM = THETAD2(pm:end);
PSID2_STSMCM   = PSID2(pm:end);

UZ1_STSMCM       = UZ1(pm:end);
TAUPHI1_STSMCM   = TAUPHI1(pm:end); 
TAUTHETA1_STSMCM = TAUTHETA1(pm:end);
TAUPSI1_STSMCM   = TAUPSI1(pm:end);

UZ2_STSMCM       = UZ2(pm:end);
TAUPHI2_STSMCM   = TAUPHI2(pm:end); 
TAUTHETA2_STSMCM = TAUTHETA2(pm:end);
TAUPSI2_STSMCM   = TAUPSI2(pm:end);

save('STSMCM_Errores.mat','EX1_STSMCM','EX2_STSMCM','EY1_STSMCM','EY2_STSMCM','EZ1_STSMCM','EZ2_STSMCM','EPHI1_STSMCM','EPHI2_STSMCM','ETHETA1_STSMCM','ETHETA2_STSMCM','EPSI1_STSMCM','EPSI2_STSMCM','-v7.3');
save('STSMCM_Estados.mat','X1_STSMCM','X2_STSMCM','Y1_STSMCM','Y2_STSMCM','Z1_STSMCM','Z2_STSMCM','PHI1_STSMCM','PHI2_STSMCM','THETA1_STSMCM','THETA2_STSMCM','PSI1_STSMCM','PSI2_STSMCM','-v7.3');
save('STSMCM_Deseadas.mat','XD1_STSMCM','XD2_STSMCM','YD1_STSMCM','YD2_STSMCM','ZD1_STSMCM','ZD2_STSMCM','PHID1_STSMCM','PHID2_STSMCM','THETAD1_STSMCM','THETAD2_STSMCM','PSID1_STSMCM','PSID2_STSMCM','-v7.3')
save('STSMCM_Control.mat','UZ1_STSMCM','UZ2_STSMCM','TAUPHI1_STSMCM','TAUPHI2_STSMCM','TAUTHETA1_STSMCM','TAUTHETA2_STSMCM','TAUPSI1_STSMCM','TAUPSI2_STSMCM','-v7.3')


        