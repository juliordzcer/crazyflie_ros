clc
clear all
clf(4)
clf(5)
% close all

% Tiempo de simulación
dt = 0.0001; % Intervalo de tiempo (s)
t_max = 100; % Tiempo máximo de simulación (s)
t = 0:dt:t_max; % Vector de tiempo


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

% Parametros de los controladores

kpx = 15;
kdx = 5;
kix = .8;

kpy = 12;
kdy = 4;
kiy = 1;

zeta_z=5.5;
k0_z=20*zeta_z^(-1/2);
k1_z=4.4*zeta_z^(2/3);
k2_z=2.5*zeta_z;

zeta_phi = 15.5;
k0_phi = 20*zeta_phi^(-1/2);
k1_phi = 4.4*zeta_phi^(2/3);
k2_phi = 2.5*zeta_phi;

zeta_theta = 15.5;
k0_theta = 20*zeta_theta^(-1/2);
k1_theta = 4.4*zeta_theta^(2/3);
k2_theta = 2.5*zeta_theta;

zeta_psi=10;
k0_psi=20*zeta_psi^(-1/2);
k1_psi=4.4*zeta_psi^(2/3);
k2_psi=2.5*zeta_psi;

% Estado inicial
x = 0; % Posición en x (m)
y = 0; % Posición en y (m)
z = 0; % Posición en z (m)
xp = 0; % Velocidad en x (m/s)
yp = 0; % Velocidad en y (m/s)
zp = 0; % Velocidad en z (m/s)
xpp = 0; % Aceleracion en x (m/s^2)
ypp = 0; % Aceleracion en y (m/s^2)
zpp = 0; % Aceleracion en z (m/s^2)

phi = 0; % Posición en phi (deg)
theta = 0; % Posición en theta (deg)
psi = 0; % Posición en psi (deg)
phip = 0; % Velocidad en phi (deg/s)
thetap = 0; % Velocidad en theta (deg/s)
psip = 0; % Velocidad en psi (deg/s)
phipp = 0; % Aceleracion en phi (deg/s^2)
thetapp = 0; % Aceleracion en theta (deg/s^2)
psipp = 0; % Aceleracion en psi (deg/s^2)

ex_prev = 0; % Error previo en x (m)
iex = 0; % Integral del error en x (m)
ey_prev = 0; % Error previo en y (m)
iey = 0; % Integral del error en y (m)
ez_prev = 0; % Error previo en z (m)
iez = 0; % Integral del error en z (m)

ephi_prev = 0; % Error previo en phi (deg)
iephi = 0; % Integral del error en phi (deg)
etheta_prev = 0; % Error previo en theta (deg)
ietheta = 0; % Integral del error en theta (deg)
epsi_prev = 0; % Error previo en psi (deg)
iepsi = 0; % Integral del error en psi (deg)


z3 = 0;
phi3 = 0;
theta3 = 0;
psi3 = 0;

% Inicialización de vectores para almacenar los resultados
X = zeros(length(t), 1);
Y = zeros(length(t), 1);
Z = zeros(length(t), 1);
XP = zeros(length(t), 1);
YP = zeros(length(t), 1);
ZP = zeros(length(t), 1);
XPP = zeros(length(t), 1);
YPP = zeros(length(t), 1);
ZPP = zeros(length(t), 1);

PHI = zeros(length(t), 1);
THETA = zeros(length(t), 1);
PSI = zeros(length(t), 1);
PHIP = zeros(length(t), 1);
THETAP = zeros(length(t), 1);
PSIP = zeros(length(t), 1);
PHIPP = zeros(length(t), 1);
THETAPP = zeros(length(t), 1);
PSIPP = zeros(length(t), 1);

XD = zeros(length(t), 1);
YD = zeros(length(t), 1);
ZD = zeros(length(t), 1);
PHID = zeros(length(t), 1);
THETAD = zeros(length(t), 1);
PSID = zeros(length(t), 1);

EX = zeros(length(t), 1);
EY = zeros(length(t), 1);
EZ = zeros(length(t), 1);
EPHI = zeros(length(t), 1);
ETHETA = zeros(length(t), 1);
EPSI = zeros(length(t), 1);

DX = zeros(length(t), 1);
DY = zeros(length(t), 1);
DZ = zeros(length(t), 1);
DPHI = zeros(length(t), 1);
DTHETA = zeros(length(t), 1);
DPSI = zeros(length(t), 1);

r = 1;
f = pi/6;

% Bucle de simulación
for i = 1:length(t)
    
    % Trayectoria deseada.
    xd     =r*(atan(15)+atan(dt*i-15)).*cos(f*dt*i);
    yd     =r*(atan(15)+atan(dt*i-15)).*sin(f*dt*i);
    zd     =1/2*(1+tanh(((dt*i-5)-2.5)))+0.1*(1+tanh((dt*i-35)/3));  
    psid   = sin(f*dt*i);
    
    % Perturbaciones 
    dx     = d * (0.3*sin(w1*dt*i));
    dy     = d * (0.3*cos(w1*dt*i));
    dz     = d * (-0.5+0.1*sin(w1*dt*i)-0.1*sin(w2*dt*i)+0.1*cos(w3*dt*i));
    dphi   = d * (-0.5+0.2*sin(w1*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w2*dt*i));
    dtheta = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w1*dt*i)+0.2*cos(w3*dt*i));
    dpsi   = d * (-0.5+0.2*cos(w3*dt*i)-0.2*sin(w2*dt*i)+0.2*cos(w3*dt*i));

    
    % Cálculo del error de posicion
    ex = x - xd;
    ey = y - yd;
    ez = z - zd;
    
    % Cálculo de la integral del error de posicion
    iex = iex + ex * dt;
    iey = iey + ey * dt;
    iez = iez + ez * dt;
    
    % Cálculo de la derivada del error de posicion
    exp = (ex - ex_prev) / dt;
    eyp = (ey - ey_prev) / dt;
    ezp = (ez - ez_prev) / dt;
    
    % Calculo de phi* y theta* 
    
    thetad  = -(kpx*ex + kix*iex + kdx*exp)/g; 
    phid    = (kpy*ey + kiy*iey + kdy*eyp)/g;
    
    % Cálculo del error de orientacion
    ephi   = phi - phid;
    etheta = theta - thetad;
    epsi   = psi - psid;
    
    % Cálculo de la integral del error de posicion
    iephi   = iephi + ephi * dt;
    ietheta = ietheta + etheta * dt;
    iepsi   = iepsi + epsi * dt;
    
    % Cálculo de la derivada del error de posicion
    ephip   = (ephi - ephi_prev) / dt;
    ethetap = (etheta - etheta_prev) / dt;
    epsip   = (epsi - epsi_prev) / dt;    
    
    % Cálculo de la fuerza requerida utilizando el controlador STSMC 
    
    z_mphi = ez + k0_z*((abs(ezp)^(3/2))*sign(ezp));
    z3 = z3 +(-k2_z*sign(z_mphi))*dt;
    z_bar= -k1_z*((abs(z_mphi)^(1/3))*sign(z_mphi)) + z3;
    
    u =((z_bar+g)/(cos(theta)*cos(phi)))*m;
    
    % Control de Phi
    phi_mphi = ephi + k0_phi*((abs(ephip)^(3/2))*sign(ephip));
    phi3 = phi3 + (-k2_phi*sign(phi_mphi))*dt;    
    tau_bar_phi = -k1_phi*((abs(phi_mphi)^(1/3))*sign(phi_mphi)) + phi3;

    tau_phi=Jx*(tau_bar_phi-((Jy-Jz)/Jx)*thetap*psip);

    % Control de Theta

    phi_mth = etheta + k0_theta*((abs(ethetap)^(3/2))*sign(ethetap));
    theta3 = theta3 + (-k2_theta*sign(phi_mth))*dt;
    tau_bar_theta = -k1_theta*((abs(phi_mth)^(1/3))*sign(phi_mth)) + theta3;
    
    tau_theta=Jy*(tau_bar_theta-((Jz-Jx)/Jy)*phip*psip);
   
    % Control de Psi
    
    phi_mpsi = epsi + k0_psi*((abs(epsip)^(3/2))*sign(epsip));
    psi3 = psi3 + (-k2_psi*sign(phi_mpsi))*dt;
    tau_bar_psi = -k1_psi*((abs(phi_mpsi)^(1/3))*sign(phi_mpsi)) + psi3;

    tau_psi = Jz*(tau_bar_psi-((Jx-Jy)/Jz)*thetap*phip);
    
    % Modelo dinamico del quadrotor
    xpp     = (u/m)*(cos(phi) * sin(theta)*cos(psi)+sin(phi)*sin(psi))-ax*xp+dx;
    ypp     = (u/m)*(cos(phi) * sin(theta)*sin(psi)-sin(phi)*cos(psi))-ay*yp+dy;
    zpp     = (u/m)*(cos(phi) * cos(theta))-g-az*zp+dz;
    phipp   = (tau_phi/Jx)   + ((Jy-Jz)/Jx) * thetap *psip+dphi;
    thetapp = (tau_theta/Jy) + ((Jz-Jx)/Jy) * phip*psip+dtheta;
    psipp   = (tau_psi/Jz)   + ((Jx-Jy)/Jz) * thetap *phip+dpsi;
    
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
    
    
    % Almacenamiento de los resultados en los vectores
    
    X(i) = x;
    Y(i) = y;
    Z(i) = z;
    XP(i) = xp;
    YP(i) = yp;
    ZP(i) = zp;
    XPP(i) = xpp;
    YPP(i) = ypp;
    ZPP(i) = zpp;

    PHI(i) = phi;
    THETA(i) = theta;
    PSI(i) = psi;
    PHIP(i) = phip;
    THETAP(i) = thetap;
    PSIP(i) = psip;
    PHIPP(i) = phipp;
    THETAPP(i) = thetapp;
    PSIPP(i) = psipp;
    
    XD(i) = xd;
    YD(i) = yd;
    ZD(i) = zd;
    PHID(i) = phid;
    THETAD(i) = thetad;
    PSID(i) = psid;
    
    EX(i) = ex;
    EY(i) = ey;
    EZ(i) = ez;
    EPHI(i) = ephi;
    ETHETA(i) = etheta;
    EPSI(i) = epsi;
    
    
    DX(i) = dx;
    DY(i) = dy;
    DZ(i) = dz;
    DPHI(i) = dphi;
    DTHETA(i) = dtheta;
    DPSI(i) = dpsi;
    
    % Actualización del error previo
    ex_prev = ex;
    ey_prev = ey;
    ez_prev = ez;
    ephi_prev = ephi;
    etheta_prev = etheta;
    epsi_prev = epsi;
    
end
% 
% 
% % Grafica de perturbaciones.
% 
% mp=2; np=3;
% 
% figure(1)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
% subplot(mp,np,1)
% hold on
% plot(t,DX)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_x$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% % ylim([-450 510]);
% box on
% hold off
% 
% subplot(mp,np,2)
% hold on
% plot(t,DY)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_y$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mp,np,3)
% hold on
% plot(t,DZ)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_z$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mp,np,4)
% hold on
% plot(t,DPHI)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$grados$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_\phi$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mp,np,5)
% hold on
% plot(t,DTHETA)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$grados$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_\theta$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mp,np,6)
% hold on
% plot(t,DPSI)
% % title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$grados$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_\psi$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% % Grafica de trayectoria.
% 
% mt=2; nt=3;
% 
% figure(2)
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
% 
% % Grafica de trayectoria real.
% 
% mtr=2; ntr=3;
% 
% figure(3)
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
 
 
 
 
 me=2; ne=3;
figure(4)
title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
subplot(me,ne,1)
hold on
plot(t,EX)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_x$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
% ylim([-450 510]);
box on
hold off

subplot(me,ne,2)
hold on
plot(t,EY)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
plot(t,EPHI)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
plot(t,ETHETA)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
plot(t,EPSI)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
ylabel('$m$','FontSize',16,'interpreter','latex')  
xlabel('$t$','FontSize',16,'interpreter','latex')
leg1=legend('$e_\psi$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off


figure(5)
hold on
plot3 (XD,YD,ZD)
plot3 (X,Y,Z)
box on
hold off

