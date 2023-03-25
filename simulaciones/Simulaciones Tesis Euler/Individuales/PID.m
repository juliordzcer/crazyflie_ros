clc
clear all
close all
clf

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
kp_x = 12;
kd_x = 4;
ki_x = 1;

kp_y = 12;
kd_y = 4;
ki_y = 1;

kp_z = 420;
kd_z = 40;
ki_z = 100;




% kpx = 15;
% kdx = 5;
% kix = .8;
% 
% kpy = 12;
% kdy = 4;
% kiy = 1;
% 
% kpz = 520;
% kdz = 40;
% kiz = 100;

kpphi = 1700;
kdphi = 850;
kiphi = 1450;

kptheta = 1700;
kdtheta = 850;
kitheta = 1450;

kppsi = 2800;
kdpsi = 400;
kipsi = 1500;




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

xd_prev  = 0;
xdp_prev = 0;

yd_prev  = 0;
ydp_prev = 0;

zd_prev  = 0;
zdp_prev = 0;
    
psid_prev = 0;
psidp_prev = 0;
        

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

U = zeros(length(t), 1);
U2 = zeros(length(t), 1);

r = 1;
f = pi/6;

% Bucle de simulación
for i = 1:length(t)
    
    % Trayectoria deseada.
    xd     = r*(atan(15)+atan(dt*i-15)).*cos(f*dt*i);
    xdp    = (xd - xd_prev) / dt;
    xdpp   = (xdp - xdp_prev) / dt;
    
    yd     =r*(atan(15)+atan(dt*i-15)).*sin(f*dt*i);
    ydp    = (yd - yd_prev) / dt;
    ydpp   = (ydp - ydp_prev) / dt;
    
    zd     =1/2*(1+tanh(((dt*i-5)-2.5)))+0.1*(1+tanh((dt*i-35)/3));
    zdp    = (zd  - zd_prev ) / dt;
    zdpp   = (zdp - zdp_prev) / dt;
    
    psid   = sin(f*dt*i);
    psidp    = (psid  - psid_prev ) / dt;
    psidpp   = (psidp - psidp_prev) / dt;

    
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
    
    nu_x = -ki_x * iex - kp_x * ex - kd_x * exp + ax * xp + xdpp;
    nu_y = -ki_y * iey - kp_y * ey - kd_y * eyp + ay * yp + ydpp;
    nu_z = -ki_z * iez - kp_z * ez - kd_z * ezp + az * zp + zdpp;
    
    
    % Cálculo de la fuerza requerida utilizando el controlador PID 
%     
%     z_bar= -kpz*ez - kiz*iez - kdz*ezp;
%     
%     u2 = (z_bar+g)/(cos(theta)*cos(phi))*m;

    u = (sqrt(nu_x^2 + nu_y^2 +(nu_z + g)^2))*m;
    
    
    
    phid = asin ((m/u)*(nu_x*sin(psid) - nu_y*cos(psid))); 
    thetad = atan ((nu_x*cos(psid) + nu_y*sin(psid))/(nu_z+g));
%     
%     thetad2  = -(kpx*ex + kix*iex + kdx*exp)/g; 
%     phid2    = (kpy*ey + kiy*iey + kdy*eyp)/g;
%     
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
    

    
    % Control de Phi
     
    tau_bar_phi = -kpphi*ephi - kiphi*iephi - kdphi*ephip;

    tau_phi=Jx*(tau_bar_phi-((Jy-Jz)/Jx)*thetap*psip);

    % Control de Theta
    tau_bar_theta = -kptheta*etheta - kitheta*ietheta - kdtheta*ethetap;
    
    tau_theta=Jy*(tau_bar_theta-((Jz-Jx)/Jy)*phip*psip);
   
    % Control de Psi
    
    tau_bar_psi = -kppsi*epsi - kipsi*iepsi - kdpsi*epsip;

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
    
    
    % Almacenami+psidppento de los resultados en los vectores
    
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
    
%     PHID2(i) = phid2;
%     THETAD2(i) = thetad2;
%     
    U(i) = u;
%     U2(i) = u2;
    
    
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
    
    % Derivada de las trayectorias deseadas
    xd_prev  = xd;
    xdp_prev = xdp;
    yd_prev  = yd;
    ydp_prev = ydp;
    zd_prev  = zd;
    zdp_prev = zdp;
    psid_prev = psid;
    psidp_prev = psidp;
        
end

% Grafica de perturbaciones.
% 
% mp=2; np=3;
% 
% figure(1)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
% subplot(mp,np,1)
% hold on
% plot(t,DX)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_x$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% ylim([-450 510]);
% box on
% hold off
% 
% subplot(mp,np,2)
% hold on
% plot(t,DY)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% Grafica de trayectoria.
% 
% mt=2; nt=3;
% 
% figure(2)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% subplot(mt,nt,1)
% hold on
% plot(t,XD)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$x_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% ylim([-450 510]);
% box on
% hold off
% 
% subplot(mt,nt,2)
% hold on
% plot(t,YD)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$y_d$');
% lgd = legend;
% lgd.NumColumns = 5;% Grafica de perturbaciones.
% 
% mp=2; np=3;
% 
% figure(1)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
% subplot(mp,np,1)
% hold on
% plot(t,DX)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$d_x$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% ylim([-450 510]);
% box on
% hold off
% 
% subplot(mp,np,2)
% hold on
% plot(t,DY)
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% title('Perturbaciones','FontSize',16,'interpreter','latex')
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
% Grafica de trayectoria.
% 
% mt=2; nt=3;
% 
% figure(2)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% subplot(mt,nt,1)
% hold on
% plot(t,XD)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$x_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% ylim([-450 510]);
% box on
% hold off
% 
% subplot(mt,nt,2)
% hold on
% plot(t,YD)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\psi_d$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off

% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');
% box on
% hold off
% 
% subplot(mt,nt,3)
% hold on
% plot(t,ZD)
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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
% title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
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

mtr=2; ntr=3;

figure(3)
title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
subplot(mtr,ntr,1)
hold on
plot(t,XD)
plot(t,X)
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
plot(t,YD)
plot(t,Y)
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
plot(t,ZD)
plot(t,Z)
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
plot(t,PHID)
plot(t,PHI)
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
plot(t,THETAD)
plot(t,THETA)
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

subplot(mtr,ntr,6)
hold on
plot(t,PSID)
plot(t,PSI)
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

% figure(1)
% mtr = 3; ntr= 1;
% subplot(mtr,ntr,1)
% hold on
% plot(t,PHID)
% plot(t,PHID2)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\phi_d$','$\phi_{d2}$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
%  ylim([-1 1]);
% box on
% hold off
% 
% subplot(mtr,ntr,2)
% hold on
% plot(t,THETAD)
% plot(t,THETAD2)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$\theta_d$','$\theta_{d2}$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
%  ylim([-1 1]);
% box on
% hold off
% 
% subplot(mtr,ntr,3)
% hold on
% plot(t,U)
% plot(t,U2)
% % title('Trayectoria Deseada','FontSize',16,'interpreter','latex')
% ylabel('$m$','FontSize',16,'interpreter','latex')  
% xlabel('$t$','FontSize',16,'interpreter','latex')
% leg1=legend('$u$','$u_{2}$');
% lgd = legend;
% lgd.NumColumns = 5;
% set(leg1,'FontSize',16,'interpreter','latex','EdgeColor','none',...
%      'Color','none');% 
%  ylim([.2 .4]);
% box on
% hold off


