clc
clear all
% close all
%% Parametros de tiempo
% Tiempo de simulación
ti = -0.001;
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
az=0.91e-9;   % kg/s

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

zeta_z=5;
k1_z=25*zeta_z^(2/3);
k2_z=15*zeta_z^(1/2);
k3_z=2.3*zeta_z;
k4_z=1.1*zeta_z;

zeta_phi=.9;
k1_phi=25*zeta_phi^(2/3);
k2_phi=15*zeta_phi^(1/2);
k3_phi=2.3*zeta_phi;
k4_phi=1.1*zeta_phi;

zeta_theta=.9;
k1_theta=25*zeta_theta^(2/3);
k2_theta=15*zeta_theta^(1/2);
k3_theta=2.3*zeta_theta;
k4_theta=1.1*zeta_theta;

zeta_psi=.9;
k1_psi=25*zeta_psi^(2/3);
k2_psi=15*zeta_psi^(1/2);
k3_psi=2.3*zeta_psi;
k4_psi=1.1*zeta_psi;

%% Agente 1 

% Estado inicial
x1   = 0.1;   % Posición en x (m)
y1   = 0.1;   % Posición en y (m)
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
    
    % Trayectoria deseada del agente 1 (Seguidor)
    
    xd1     = x2 + cx;
    yd1     = y2 + cy;
    zd1     = z2 + cz;
        
    psid1   = psi2;
    psidp1  = psip2;
    psidpp1 = psipp2;
    
    % Trayectoria deseada del agente 2 (Lider)
   
    xd2     =r*(atan(15)+atan(dt*i-15)).*cos(f*dt*i);
    yd2     =r*(atan(15)+atan(dt*i-15)).*sin(f*dt*i);
    zd2     =1/2*(1+tanh(((dt*i-5)-2.5)))+0.1*(1+tanh((dt*i-35)/3));
        
    psid2   = sin(f*dt*i);
    psidp2  = f*cos(f*dt*i);
    psidpp2 = -f*f*sin(f*dt*i);
    
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
    phid1    = (kpy*ey1 + kiy*iey1 + kdy*eyp1)/g;
    
     % Calculo de phi* y theta* del lider
    thetad2  = -(kpx*ex2 + kix*iex2 + kdx*exp2)/g; 
    phid2    = (kpy*ey2 + kiy*iey2 + kdy*eyp2)/g;
    
    
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
            
    % Cálculo de la fuerza requerida utilizando el controlador TC para el lider.     
    z32 = z32 +(-k3_z*sign(ez2)-k4_z*sign(ezp2))*dt;
    z_bar2= -k1_z*(abs(ez2)^(1/3))*sign(ez2)-k2_z*(abs(ezp2)^(1/2))*sign(ezp2) + z32;
    
    u2 =((z_bar2+g)/(cos(theta2)*cos(phi2)))*m;
    
    % Cálculo de la fuerza requerida utilizando el controlador TC para el seguidor 
    z31 = z31 +(-k3_z*sign(ez1)-k4_z*sign(ezp1))*dt;
    z_bar1= -k1_z*(abs(ez1)^(1/3))*sign(ez1)-k2_z*(abs(ezp1)^(1/2))*sign(ezp1) + z31;
    
    u1 =((z_bar1+ u2*cos(theta2)*cos(phi2))/(cos(theta1)*cos(phi1)))*m;  
    
        % Control de posicion para el lider.
    % Control de Phi
    
    phi32 = phi32 + (-k3_phi*sign(ephi2)-k4_phi*sign(ephip2))*dt;    
    tau_bar_phi2 = -k1_phi*(abs(ephi2)^(1/3))*sign(ephi2)-k2_phi*(abs(ephip2)^(1/2))*sign(ephip2) + phi32;

    tau_phi2=Jx*(tau_bar_phi2-((Jy-Jz)/Jx)*thetap2*psip2);

    % Control de Theta

    theta32 = theta32 + (-k3_theta*sign(etheta2) -k4_theta*sign(ethetap2))*dt;
    tau_bar_theta2 = -k1_theta*(abs(etheta2)^(1/3))*sign(etheta2)-k2_theta*(abs(ethetap2)^(1/2))*sign(ethetap2)+ theta32;
    
    tau_theta2=Jy*(tau_bar_theta2-((Jz-Jx)/Jy)*phip2*psip2);
   
    % Control de Psi
    
    psi32 = psi32 + (-k3_psi*sign(epsi2)-k4_psi*sign(epsip2))*dt;
    tau_bar_psi2 = -k1_psi*(abs(epsi2)^(1/3))*sign(epsi2)-k2_psi*(abs(epsip2)^(1/2))*sign(epsip2) + psi32;

    tau_psi2 = Jz*(tau_bar_psi2-((Jx-Jy)/Jz)*thetap2*phip2+psidpp2);
    

    % Control de posicion para el seguidor
    
    % Control de Phi
    
    phi31 = phi31 + (-k3_phi*sign(ephi1)-k4_phi*sign(ephip1))*dt;    
    tau_bar_phi1 = -k1_phi*(abs(ephi1)^(1/3))*sign(ephi1)-k2_phi*(abs(ephip1)^(1/2))*sign(ephip1) + phi31;

    tau_phi1=Jx*(tau_bar_phi1-((Jy-Jz)/Jx)*thetap1*psip1);

    % Control de Theta

    theta31 = theta31 + (-k3_theta*sign(etheta1) -k4_theta*sign(ethetap1))*dt;
    tau_bar_theta1 = -k1_theta*(abs(etheta1)^(1/3))*sign(etheta1)-k2_theta*(abs(ethetap1)^(1/2))*sign(ethetap1)+ theta31;
    
    tau_theta1 = Jy*(tau_bar_theta1-((Jz-Jx)/Jy)*phip1*psip1);
   
    % Control de Psi
    
    psi31 = psi31 + (-k3_psi*sign(epsi1)-k4_psi*sign(epsip1))*dt;
    tau_bar_psi1 = -k1_psi*(abs(epsi1)^(1/3))*sign(epsi1)-k2_psi*(abs(epsip1)^(1/2))*sign(epsip1) + psi31;

    tau_psi1 = Jz*(tau_bar_psi1-((Jx-Jy)/Jz)*thetap1*phip1+psidpp1);
    

    
    
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
    
    X1(i) = x1;
    Y1(i) = y1;
    Z1(i) = z1;
    XP1(i) = xp1;
    YP1(i) = yp1;
    ZP1(i) = zp1;
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
    
    
    DX(i) = dx;
    DY(i) = dy;
    DZ(i) = dz;
    DPHI(i) = dphi;
    DTHETA(i) = dtheta;
    DPSI(i) = dpsi;
    
    
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
    ex_prev1 = ex1;
    ey_prev1 = ey1;
    ez_prev1 = ez1;
    ephi_prev1 = ephi1;
    etheta_prev1 = etheta1;
    epsi_prev1 = epsi1;
    
    ex_prev2 = ex2;
    ey_prev2 = ey2;
    ez_prev2 = ez2;
    ephi_prev2 = ephi2;
    etheta_prev2 = etheta2;
    epsi_prev2 = epsi2;
    
end
%% Extraccion de datos
pm = -ti^(-1)+1; 
t=t(pm:end);


EX1_TC     = EX1(pm:end);
EY1_TC     = EY1(pm:end);
EZ1_TC     = EZ1(pm:end);
EPHI1_TC     = EPHI1(pm:end);
ETHETA1_TC     = ETHETA1(pm:end);
EPSI1_TC     = EPSI1(pm:end);

EX2_TC       = EX2(pm:end);
EY2_TC       = EY2(pm:end);
EZ2_TC       = EZ2(pm:end);
EPHI2_TC     = EPHI2(pm:end);
ETHETA2_TC   = ETHETA2(pm:end);
EPSI2_TC     = EPSI2(pm:end);

X1_TC       = X1(pm:end);
Y1_TC       = Y1(pm:end);
Z1_TC       = Z1(pm:end);
PHI1_TC     = PHI1(pm:end);
THETA1_TC   = THETA1(pm:end);
PSI1_TC     = PSI1(pm:end);

X2_TC       = X2(pm:end);
Y2_TC       = Y2(pm:end);
Z2_TC       = Z2(pm:end);
PHI2_TC     = PHI2(pm:end);
THETA2_TC   = THETA2(pm:end);
PSI2_TC     = PSI2(pm:end);


XD1_TC     = XD1(pm:end);
YD1_TC     = YD1(pm:end); 
ZD1_TC     = ZD1(pm:end);
PHID1_TC   = PHID1(pm:end);
THETAD1_TC = THETAD1(pm:end);
PSID1_TC   = PSID1(pm:end);

XD2_TC      = XD2(pm:end);
YD2_TC      = YD2(pm:end);
ZD2_TC      = ZD2(pm:end);
PHID2_TC    = PHID2(pm:end);
THETAD2_TC = THETAD2(pm:end);
PSID2_TC    = PSID2(pm:end);


UZ1_TC       = UZ1(pm:end);
TAUPHI1_TC   = TAUPHI1(pm:end); 
TAUTHETA1_TC = TAUTHETA1(pm:end);
TAUPSI1_TC   = TAUPSI1(pm:end);

UZ2_TC       = UZ2(pm:end);
TAUPHI2_TC   = TAUPHI2(pm:end); 
TAUTHETA2_TC = TAUTHETA2(pm:end);
TAUPSI2_TC   = TAUPSI2(pm:end);

% 
% save('TC_Errores.mat','EX1_TC','EX2_TC','EY1_TC','EY2_TC','EZ1_TC','EZ2_TC','EPHI1_TC','EPHI2_TC','ETHETA1_TC','ETHETA2_TC','EPSI1_TC','EPSI2_TC','t','-v7.3');
% save('TC_Estados.mat','X1_TC','X2_TC','Y1_TC','Y2_TC','Z1_TC','Z2_TC','PHI1_TC','PHI2_TC','THETA1_TC','THETA2_TC','PSI1_TC','PSI2_TC','-v7.3');
% save('TC_Deseadas.mat','XD1_TC','XD2_TC','YD1_TC','YD2_TC','ZD1_TC','ZD2_TC','PHID1_TC','PHID2_TC','THETAD1_TC','THETAD2_TC','PSID1_TC','PSID2_TC','-v7.3')
% save('TC_Control.mat','UZ1_TC','UZ2_TC','TAUPHI1_TC','TAUPHI2_TC','TAUTHETA1_TC','TAUTHETA2_TC','TAUPSI1_TC','TAUPSI2_TC','-v7.3')


TL=12;

        me=6; ne=1;
        
        figure(9)
        subplot(me,ne,1)
        hold on
        plot(t,EX1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{x_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,2)
        hold on
        plot(t,EY1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{y_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,3)
        hold on
        plot(t,EZ1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{z_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,4)
        hold on
        plot(t,EPHI1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\phi_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,5)
        hold on
        plot(t,ETHETA1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\theta_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,6)
        hold on
        plot(t,EPSI1_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\psi_1-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
%         title('Error del agente 1','FontSize',TL,'interpreter','latex')
    
        %  --------------------------- errores agente 2 ------------------------
        figure(10)
        subplot(me,ne,1)
        hold on
        plot(t,EX2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{x_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,2)
        hold on
        plot(t,EY2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{y_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,3)
        hold on
        plot(t,EZ2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{z_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,4)
        hold on
        plot(t,EPHI2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\phi_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,5)
        hold on
        plot(t,ETHETA2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\theta_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,6)
        hold on
        plot(t,EPSI2_TC)
        ylabel('$m$','FontSize',TL,'interpreter','latex')  
        xlabel('$t$','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\psi_2-TC}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
