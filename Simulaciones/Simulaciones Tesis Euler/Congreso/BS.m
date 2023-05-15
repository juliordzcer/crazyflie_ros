clc
clear all
% close all

%% Parametros de la trayectoria deseada.
r = .2;
f = pi/9;
p = 15;

%% Vector invariante en el tiempo de formacion.
cx = -0.1;
cy = 0.15;
cz = 0.1;
%% Parametros de tiempo
dt = 0.001; % Intervalo de tiempo (s)
t_max = 100; % Tiempo máximo de simulación (s)

% Tiempo de simulación
ti = -0.05;
t = ti:dt:t_max; % Vector de tiempo

a = 0.109*10^(-6);
b = -210.6*10^(-6);
c = 0.154;

%% Constantes, Inercias, coeficientes aerodinamicos.
% Constantes
g = 9.8; % Aceleración debido a la gravedad (m/s^2)
m=0.032; % Masa del quadrotor (kg)

Jx=9.827e-05;
Jy=8.185e-05;
Jz=9.613e-05;

d=1;
w1=0.4;
w2=0.6;
w3=0.8;

%% Agente 1 
% Parametros de los controladores
kp_x = 28.50;
kd_x = 15.50;
ki_x = 15;

kp_y = 28.50;
kd_y = 15.50;
ki_y = 15;

kp_z = 12.50;
kd_z = 18.50;
ki_z = 0.00;

k1_phi = 265;
k2_phi = 280;

k1_theta = 265;
k2_theta = 280;

k1_psi = 265;
k2_psi = 280;

x   = 0; % Posición en x (m)
y   = 0; % Posición en y (m)
z   = 0; % Posición en z (m)
xp  = 0; % Velocidad en x (m/s)
yp  = 0; % Velocidad en y (m/s)
zp  = 0; % Velocidad en z (m/s)
xpp = 0; % Aceleracion en x (m/s^2)
ypp = 0; % Aceleracion en y (m/s^2)
zpp = 0; % Aceleracion en z (m/s^2)

phi     = 0; % Posición en phi (deg)
theta   = 0; % Posición en theta (deg)
psi     = 0; % Posición en psi (deg)
phip    = 0; % Velocidad en phi (deg/s)
thetap  = 0; % Velocidad en theta (deg/s)
psip    = 0; % Velocidad en psi (deg/s)
phipp   = 0; % Aceleracion en phi (deg/s^2)
thetapp = 0; % Aceleracion en theta (deg/s^2)
psipp   = 0; % Aceleracion en psi (deg/s^2)

ex_prev = 0; % Error previo en x (m)
iex     = 0; % Integral del error en x (m)
ey_prev = 0; % Error previo en y (m)
iey     = 0; % Integral del error en y (m)
ez_prev = 0; % Error previo en z (m)
iez     = 0; % Integral del error en z (m)

ephi_prev   = 0; % Error previo en phi (deg)
iephi       = 0; % Integral del error en phi (deg)
etheta_prev = 0; % Error previo en theta (deg)
ietheta     = 0; % Integral del error en theta (deg)
epsi_prev   = 0; % Error previo en psi (deg)
iepsi       = 0; % Integral del error en psi (deg)

phi3   = 0;
theta3 = 0;
psi3   = 0;

xd_prev  = 0;
xdp_prev = 0;
yd_prev  = 0;
ydp_prev = 0;
zd_prev  = 0;
zdp_prev = 0;
psid_prev = 0;
psidp_prev = 0;

phid_prev = 0;
phidp_prev = 0;
thetad_prev = 0;
thetadp_prev = 0;

nu_x = 0;
nu_y = 0;
nu_z = 0;

% Inicialización de vectores para almacenar los resultados
X   = zeros(length(t), 1);
Y   = zeros(length(t), 1);
Z   = zeros(length(t), 1);
XP  = zeros(length(t), 1);
YP  = zeros(length(t), 1);
ZP  = zeros(length(t), 1);
XPP = zeros(length(t), 1);
YPP = zeros(length(t), 1);
ZPP = zeros(length(t), 1);

PHI     = zeros(length(t), 1);
THETA   = zeros(length(t), 1);
PSI     = zeros(length(t), 1);
PHIP    = zeros(length(t), 1);
THETAP  = zeros(length(t), 1);
PSIP    = zeros(length(t), 1);
PHIPP   = zeros(length(t), 1);
THETAPP = zeros(length(t), 1);
PSIPP   = zeros(length(t), 1);

XD     = zeros(length(t), 1);
YD     = zeros(length(t), 1);
ZD     = zeros(length(t), 1);
PHID   = zeros(length(t), 1);
THETAD = zeros(length(t), 1);
PSID   = zeros(length(t), 1);

EX     = zeros(length(t), 1);
EY     = zeros(length(t), 1);
EZ     = zeros(length(t), 1);
EPHI   = zeros(length(t), 1);
ETHETA = zeros(length(t), 1);
EPSI   = zeros(length(t), 1);

% Senales de control
UZ_RPM   = zeros(length(t), 1);

TAUPHI   = zeros(length(t), 1);
TAUTHETA = zeros(length(t), 1);
TAUPSI   = zeros(length(t), 1);

NUX = zeros(length(t), 1);
NUY = zeros(length(t), 1);
NUZ = zeros(length(t), 1);

DPHI   = zeros(length(t), 1);
DTHETA = zeros(length(t), 1);
DPSI   = zeros(length(t), 1);

for i = 1:length(t)  
    
    xd     = r*(atan(p)+atan(dt*i-p)).*cos(f*dt*i);
    xdp    = (xd - xd_prev) / dt;
    xdpp   = (xdp - xdp_prev) / dt;

    yd     = r*(atan(p)+atan(dt*i-p)).*sin(f*dt*i);
    ydp    = (yd - yd_prev) / dt;
    ydpp   = (ydp - ydp_prev) / dt;
    
    zd     = (.4/2)*(1+tanh(((dt*i-7.5))));
    zdp    = (zd  - zd_prev ) / dt;
    zdpp   = (zdp - zdp_prev) / dt;
    
    psid   = sin(f*dt*i);
    psidp  = (psid  - psid_prev ) / dt;
    psidpp = (psidp - psidp_prev) / dt;
    
    % Perturbaciones 
    dphi   = (-0.3+0.2*sin(w1*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w2*dt*i));
    dtheta = (-0.3+0.2*cos(w3*dt*i)-0.2*sin(w1*dt*i)+0.2*cos(w3*dt*i));
    dpsi   = (-0.3+0.2*cos(w3*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w3*dt*i));
    
    % Cálculo del error de posicion del lider
    ex = x - xd;
    ey = y - yd;
    ez = z - zd;
    
    % Cálculo de la integral del error de posicion del lider
    iex = iex + ex * dt;
    iey = iey + ey * dt;
    iez = iez + ez * dt; 
    
    % Cálculo de la derivada del error de posicion
    exp = (ex - ex_prev) / dt;
    eyp = (ey - ey_prev) / dt;
    ezp = (ez - ez_prev) / dt;

    nu_x = (-ki_x * iex - kp_x * ex - kd_x * exp)+ xdpp;
    nu_y = (-ki_y * iey - kp_y * ey - kd_y * eyp)+ ydpp;
    nu_z = (-ki_z * iez - kp_z * ez - kd_z * ezp)+ zdpp;
    
    u = (sqrt((nu_x)^2 + (nu_y)^2 +(nu_z + g )^2))*m;
    
    thrust_gramo = u*( 1 / 0.00981 );
    rpm = (sqrt(4*a*(thrust_gramo-c)+b^2) - b)/(2*a);
    u2_rpm = rpm;
    
    phid     = asin ((m/u)*((nu_x)*sin(psid) - (nu_y)*cos(psid))); 
    phidp    = (phid  - phid_prev ) / dt;
    phidpp   = (phidp - phidp_prev) / dt;
    
    thetad     = atan (((nu_x)*cos(psid) + (nu_y)*sin(psid))/(nu_z+g));
    thetadp    = (thetad  - thetad_prev ) / dt;
    thetadpp   = (thetadp - thetadp_prev) / dt;
    
    % Cálculo del error de orientacion del lider
    ephi   = phid - phi;
    etheta = thetad - theta;
    epsi   = psid - psi;    
    
    % Cálculo de la integral del error de posicion del lider
    iephi   = iephi + ephi * dt;
    ietheta = ietheta + etheta * dt;
    iepsi   = iepsi + epsi * dt;
    
    % Cálculo de la derivada del error de posicion del lider 
    ephip   = (ephi - ephi_prev) / dt;
    ethetap = (etheta - etheta_prev) / dt;
    epsip   = (epsi - epsi_prev) / dt;  
    

    % Control de posicion para el lider.
    % Control de Phi 
    nu_phi = phidp + k1_phi * ephi;
    ephi2 = nu_phi - phip;
    tau_bar_phi = ephi + k1_phi * ephip + k2_phi * ephi2;   
    tau_phi = Jx * ( tau_bar_phi - ((Jy-Jz)/Jx) * thetap * psip + phidpp);

    % Control de Theta
    nu_theta = k1_theta * etheta + thetadp;
    etheta2 = nu_theta - thetap;
    tau_bar_theta = etheta + k1_theta * ethetap + k2_theta * etheta2;
    tau_theta = Jy * ( tau_bar_theta - ((Jz-Jx)/Jy) * phip * psip + thetadpp);
    
    % Control de Psi
    nu_psi = k1_psi * epsi + psidp;
    epsi2 = nu_psi - psip;
    tau_bar_psi = epsi + k1_psi * epsip + k2_psi * epsi2; 
    tau_psi = Jz * ( tau_bar_psi - ((Jx-Jy)/Jz) * thetap * phip + psidpp);
       
    % Modelo dinamico del quadrotor del lider 
    xpp     = (u/m)*(cos(phi) * sin(theta)*cos(psi)+sin(phi)*sin(psi));
    ypp     = (u/m)*(cos(phi) * sin(theta)*sin(psi)-sin(phi)*cos(psi));
    zpp     = (u/m)*(cos(phi) * cos(theta))-g;
    phipp   = (tau_phi/Jx)   + ((Jy-Jz)/Jx) * thetap * psip + dphi;
    thetapp = (tau_theta/Jy) + ((Jz-Jx)/Jy) * phip   * psip + dtheta;
    psipp   = (tau_psi/Jz)   + ((Jx-Jy)/Jz) * thetap * phip + dpsi;
    
    xp = xp + xpp * dt;
    yp = yp + ypp * dt;
    zp = zp + zpp * dt;
    phip   = phip + phipp * dt ;
    thetap = thetap + thetapp * dt;
    psip   = psip + psipp * dt;
    
    x = x + xp * dt;
    y = y + yp * dt;
    z = z + zp * dt;
    phi   = phi + phip * dt ;
    theta = theta + thetap * dt;
    psi   = psi + psip * dt;    
    
    % Almacenamiento de los resultados en los vectore
    
    X(i) = x;
    Y(i) = y;
    Z(i) = z;
    XP(i) = xp;
    YP(i) = yp;
    ZP(i) = zp;
    XPP(i) = xpp;
    YPP(i) = ypp;
    ZPP(i) = zpp;

    PHI(i) = rad2deg(phi);
    THETA(i) = rad2deg(theta);
    PSI(i) = rad2deg(psi);
    PHIP(i) = phip;
    THETAP(i) = thetap;
    PSIP(i) = psip;
    PHIPP(i) = phipp;
    THETAPP(i) = thetapp;
    PSIPP(i) = psipp;
    
    XD(i) = xd;
    YD(i) = yd;
    ZD(i) = zd;
    PHID(i) = rad2deg(phid);
    THETAD(i) = rad2deg(thetad);
    PSID(i) = rad2deg(psid);
    
    EX(i) = ex;
    EY(i) = ey;
    EZ(i) = ez;
    EPHI(i) = rad2deg(ephi);
    ETHETA(i) = rad2deg(etheta);
    EPSI(i) = rad2deg(epsi);
    
    UZ(i)       = u;
    UZ_RPM(i)   = u2_rpm;
    TAUPHI(i)   = tau_phi;
    TAUTHETA(i) = tau_theta;
    TAUPSI(i)   = tau_psi;
    
    
    DPHI(i) = dphi;
    DTHETA(i) = dtheta;
    DPSI(i) = dpsi;
    
    ex_prev = ex;
    ey_prev = ey;
    ez_prev = ez;
    ephi_prev = ephi;
    etheta_prev = etheta;
    epsi_prev = epsi;
    
    % Derivada de las trayectorias deseadas
    xd_prev  = xd;
    xdp_prev = xdp;
    yd_prev  = yd;
    ydp_prev = ydp;
    zd_prev  = zd;
    zdp_prev = zdp;
    psid_prev = psid;
    psidp_prev = psidp;
    phid_prev = phid;
    phidp_prev = phidp;
    thetad_prev = thetad;
    thetadp_prev = thetadp;
    
   NUX(i) = nu_x;
   NUY(i) = nu_y;
   NUZ(i) = nu_z;
   
    
end


% Grafica de trayectoria.
% 
% mt=2; nt=3;
% 
% figure(1)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% subplot(mt,nt,1)
% hold on
% plot(t,XD)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$x_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% % ylim([-450 510]);
% box on
% hold off
% 
% subplot(mt,nt,2)
% hold on
% plot(t,YD)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$y_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mt,nt,3)
% hold on
% plot(t,ZD)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$z_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mt,nt,4)
% hold on
% plot(t,PHID)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\phi_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mt,nt,5)
% hold on
% plot(t,THETAD)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\theta_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mt,nt,6)
% hold on
% plot(t,PSID)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\psi_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off

% Grafica de trayectoria real.

% mtr=2; ntr=3;
% 
% figure(2)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% subplot(mtr,ntr,1)
% hold on
% plot(t,X)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$x$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% % ylim([-450 510]);
% box on
% hold off
% 
% subplot(mtr,ntr,2)
% hold on
% plot(t,Y)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$y$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mtr,ntr,3)
% hold on
% plot(t,Z)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$z$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mtr,ntr,4)
% hold on
% plot(t,PHI)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\phi$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
% box on
% hold off
% 
% subplot(mtr,ntr,5)
% hold on
% plot(t,THETA)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\phi$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
% box on
% hold off
% 
% subplot(mtr,ntr,6)
% hold on
% plot(t,PSI)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\phi$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
% box on
% hold off
%  
 
 
 
 me=2; ne=3;
figure(3)
title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
subplot(me,ne,1)
hold on
plot(t,EX)
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_x$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(me,ne,2)
hold on
plot(t,EY)
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_y$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(me,ne,3)
hold on
plot(t,EZ)
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_z$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(me,ne,4)
hold on
plot(t,rad2deg(EPHI))
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_\phi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(me,ne,5)
hold on
plot(t,rad2deg(ETHETA))
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_\theta$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

subplot(me,ne,6)
hold on
plot(t,rad2deg(EPSI))
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_\psi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off



% figure(4)
% hold on
% plot3 (XD,YD,ZD)
% plot3 (X,Y,Z)
% box on
% hold off