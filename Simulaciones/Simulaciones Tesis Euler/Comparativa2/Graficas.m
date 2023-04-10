clc
clear all
close all 

%% Parametros de tiempo
dt = 0.001; % Intervalo de tiempo (s)
t_max = 100; % Tiempo máximo de simulación (s)

%% Parametros de la trayectoria deseada.
r = .2;
f = pi/9;
p = 15;

%% Vector invariante en el tiempo de formacion.
cx = -0.1;
cy = 0.15;
cz = 0.1;
%% Corre los controladores

% run('STSMC_J.m');
run('STSMC_R.m');

%% Carga los datos de interes de los controladores

load('STSMC_Errores_R.mat');
load('STSMC_Estados_R.mat');
load('STSMC_Deseadas_R.mat');
load('STSMC_Control_R.mat');

load('STSMC_Errores_J.mat');
load('STSMC_Estados_J.mat');
load('STSMC_Deseadas_J.mat');
load('STSMC_Control_J.mat');

%% Calcula el error cuadratico medio de los estados del Seguidor y el Lider

XS = [rms(EX1_STSMC_J),rms(EX1_STSMC_R)];
XL = [rms(EX2_STSMC_J),rms(EX2_STSMC_R)];

YS = [rms(EY1_STSMC_J),rms(EY1_STSMC_R)];
YL = [rms(EY2_STSMC_J),rms(EY2_STSMC_R)];

ZS = [rms(EZ1_STSMC_J),rms(EZ1_STSMC_R)];
ZL = [rms(EZ2_STSMC_J),rms(EZ2_STSMC_R)];

PHIS = [rms(EPHI1_STSMC_J),rms(EPHI1_STSMC_R)];
PHIL = [rms(EPHI2_STSMC_J),rms(EPHI2_STSMC_R)];

THETAS = [rms(ETHETA1_STSMC_J),rms(ETHETA1_STSMC_R)];
THETAL = [rms(ETHETA2_STSMC_J),rms(ETHETA2_STSMC_R)];

PSIS = [rms(EPSI1_STSMC_J),rms(EPSI1_STSMC_R)];
PSIL = [rms(EPSI2_STSMC_J),rms(EPSI2_STSMC_R)];

%% Calcula el error cuadratico medio de las senales de control del Seguidor y el Lider

UZS = [rms(UZ1_STSMC_J),rms(UZ1_STSMC_R)];
UZL = [rms(UZ2_STSMC_J),rms(UZ2_STSMC_R)];

TAUPHIS = [rms(TAUPHI1_STSMC_J),rms(TAUPHI1_STSMC_R)];
TAUPHIL = [rms(TAUPHI2_STSMC_J),rms(TAUPHI2_STSMC_R)];

TAUTHETAS = [rms(TAUTHETA1_STSMC_J),rms(TAUTHETA1_STSMC_R)];
TAUTHETAL = [rms(TAUTHETA2_STSMC_J),rms(TAUTHETA2_STSMC_R)];

TAUPSIS = [rms(TAUPSI1_STSMC_J),rms(TAUPSI1_STSMC_R)];
TAUPSIL = [rms(TAUPSI2_STSMC_J),rms(TAUPSI2_STSMC_R)];



%% Graficas del error cuadratico medio del error
% ---------------------------- Error cuadratico medio del error ---------------------

mecm=3; necm=2;
Label = categorical({'1.STSMC-J','2.STSMC-R'});%, '5.PID'});%

TL = 24;

%% Indices de desempeno del error de seguimiento del lider y del error de formacion del seguidor
% Indices de desempeno del error de seguimiento del lider.

figure(1)
subplot(mecm,necm,1)
hold on
bar(Label,XS)
ax = gca;
ax.FontSize = TL;
title('$X_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,2)
hold on
bar(Label,XL)
ax = gca;
ax.FontSize = TL;
title('$X_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,3)
hold on
bar(Label,YS)
ax = gca;
ax.FontSize = TL;
title('$Y_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,4)
hold on
bar(Label,YL)
ax = gca;
ax.FontSize = TL;
title('$Y_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,5)
hold on
bar(Label,ZS)
ax = gca;
ax.FontSize = TL;
title('$Z_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,6)
hold on
bar(Label,ZL)
ax = gca;
ax.FontSize = TL;
title('$Z_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

% indices de desempeno del error de seguimiento de formacion del seguidor 

mecm=3; necm=2;

figure(2)
subplot(mecm,necm,1)
hold on
bar(Label,PHIS)
ax = gca;
ax.FontSize = TL;
title('$\phi_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,2)
hold on
bar(Label,PHIL)
ax = gca;
ax.FontSize = TL;
title('$\phi_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,3)
hold on
bar(Label,THETAS)
ax = gca;
ax.FontSize = TL;
title('$\theta_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,4)
hold on
bar(Label,THETAL)
ax = gca;
ax.FontSize = TL;
title('$\theta_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,5)
hold on
bar(Label,PSIS)
ax = gca;
ax.FontSize = TL;
title('$\psi_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,6)
hold on
bar(Label,PSIL)
ax = gca;
ax.FontSize = TL;
title('$\psi_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on

%% Indices de desempeno de las senales de control usando el error cuadratico medio.

% Seguidor
figure(3)
subplot(2,2,1)
hold on
bar(Label,UZS)
ax = gca;
ax.FontSize = TL;
title('$U_{Z_{S_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[N]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
bar(Label,TAUPHIS)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\phi_{S_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
bar(Label,TAUTHETAS)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\theta_{S_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
bar(Label,TAUPSIS)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\psi_{S_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off

% Lider

figure(4)
subplot(2,2,1)
hold on
bar(Label,UZL)
ax = gca;
ax.FontSize = TL;
title('$U_{Z_{L_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[N]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
bar(Label,TAUPHIL)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\phi_{L_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
bar(Label,TAUTHETAL)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\theta_{L_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
bar(Label,TAUPSIL)
ax = gca;
ax.FontSize = TL;
title('$\tau_{\psi_{L_{RMS}}}$','FontSize',TL,'interpreter','latex')
ylabel('[Nm]','FontSize',TL,'interpreter','latex')
box on
hold off



%% Graficas de las trayectorias de la deseada y agentes de Paper Romeo


% Seguidor

mt=6; nt=1;
        
figure(5)    
title('Trayectoria del seguidor $STSMC_R$','FontSize',TL,'interpreter','latex')
        
subplot(mt,nt,1)
hold on
plot(t,XD1_STSMC_R,'LineWidth',3)
plot(t,X1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{d_{STSMC_R}}$','$x_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,2)
hold on
plot(t,YD1_STSMC_R,'LineWidth',3)
plot(t,Y1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$y_{d_{STSMC_R}}$','$y_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,3)
hold on
plot(t,ZD1_STSMC_R,'LineWidth',3)
plot(t,Z1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$z_{d_{STSMC_R}}$','$z_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,4)
hold on
plot(t,PHID1_STSMC_R,'LineWidth',3)
plot(t,PHI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\phi_{d_{STSMC_R}}$','$\phi_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,5)
hold on
plot(t,THETAD1_STSMC_R,'LineWidth',3)
plot(t,THETA1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\theta_{d_{STSMC_R}}$','$\theta_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,6)
hold on
plot(t,PSID1_STSMC_R,'LineWidth',3)
plot(t,PSI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\psi_{d_{STSMC_R}}$','$\psi_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

% Lider
      
figure(6)

subplot(mt,nt,1)
hold on
plot(t,XD2_STSMC_R,'LineWidth',3)
plot(t,X2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{d_{STSMC_R}}$','$x_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,2)
hold on
plot(t,YD2_STSMC_R,'LineWidth',3)
plot(t,Y2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$y_{d_{STSMC_R}}$','$y_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,3)
hold on
plot(t,ZD2_STSMC_R,'LineWidth',3)
plot(t,Z2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$z_{d_{STSMC_R}}$','$z_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,4)
hold on
plot(t,PHID2_STSMC_R,'LineWidth',3)
plot(t,PHI2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\phi_{STSMC_J}$','$\phi_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,5)
hold on
plot(t,THETAD2_STSMC_R,'LineWidth',3)
plot(t,THETA2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\theta_{d_{STSMC_R}}$','$\theta_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,6)
hold on
plot(t,PSID2_STSMC_R,'LineWidth',3)
plot(t,PSI2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\psi_{d_{STSMC_R}}$','$\psi_{STSMC_R}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

%% Graficas de las trayectorias de la deseada y agentes de Paper Jaime
% Seguidor

mt=6; nt=1;
        
figure(7)    
title('Trayectoria del seguidor','FontSize',TL,'interpreter','latex')
        
subplot(mt,nt,1)
hold on
plot(t,XD1_STSMC_J,'LineWidth',3)
plot(t,X1_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{d_{STSMC_J}}$','$x_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,2)
hold on
plot(t,YD1_STSMC_J,'LineWidth',3)
plot(t,Y1_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$y_{d_{STSMC_J}}$','$y_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,3)
hold on
plot(t,ZD1_STSMC_J,'LineWidth',3)
plot(t,Z1_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$z_{d_{STSMC_J}}$','$z_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,4)
hold on
plot(t,PHID1_STSMC_J,'LineWidth',3)
plot(t,PHI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\phi_{d_{STSMC_J}}$','$\phi_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,5)
hold on
plot(t,THETAD1_STSMC_J,'LineWidth',3)
plot(t,THETA1_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\theta_{d_{STSMC_J}}$','$\theta_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,6)
hold on
plot(t,PSI1_STSMC_J,'LineWidth',3)
plot(t,PSI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\psi_{d_{STSMC_J}}$','$\psi_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off

% Lider
      
figure(8)

subplot(mt,nt,1)
hold on
plot(t,XD2_STSMC_J,'LineWidth',3)
plot(t,X2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{d_{STSMC_J}}$','$x_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,2)
hold on
plot(t,YD2_STSMC_J,'LineWidth',3)
plot(t,Y2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$y_{d_{STSMC_J}}$','$y_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,3)
hold on
plot(t,ZD2_STSMC_J,'LineWidth',3)
plot(t,Z2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$z_{d_{STSMC_J}}$','$z_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,4)
hold on
plot(t,PHID2_STSMC_J,'LineWidth',3)
plot(t,PHI2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\phi_{d_{STSMC_J}}$','$\phi_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,5)
hold on
plot(t,THETAD2_STSMC_J,'LineWidth',3)
plot(t,THETA2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\theta_{d_{STSMC_J}}$','$\theta_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(mt,nt,6)
hold on
plot(t,PSID2_STSMC_J,'LineWidth',3)
plot(t,PSI2_STSMC_J,'LineWidth',3)
xlim([0 t_max])
ylabel('[grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$\psi_{d_{STSMC_J}}$','$\psi_{STSMC_J}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
 
%% Grafica de error del controlador STSMC_R y STSMC_J Respecto al tiempo

%  -------------------------- Errores del seguidor ------------------
me=6; ne=1;

figure(9)
subplot(me,ne,1)
hold on
plot(t,EX1_STSMC_J,'LineWidth',3)
plot(t,EX1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{x_1}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,2)
hold on
plot(t,EY1_STSMC_J,'LineWidth',3)
plot(t,EY1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{y_1}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
      'Color','none');
box on
hold off
        
subplot(me,ne,3)
hold on
plot(t,EZ1_STSMC_J,'LineWidth',3)
plot(t,EZ1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{z_1}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,4)
hold on
plot(t,EPHI1_STSMC_J,'LineWidth',3)
plot(t,EPHI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\phi_1}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,5)
hold on
plot(t,ETHETA1_STSMC_J,'LineWidth',3)
plot(t,ETHETA1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\theta_1}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,6)
hold on
plot(t,EPSI1_STSMC_J,'LineWidth',3)
plot(t,EPSI1_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\psi_1}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
% ------------ Error del lider ------------
figure(10)
 
subplot(me,ne,1)
hold on
plot(t,EX2_STSMC_J,'LineWidth',3)
plot(t,EX2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{x_2}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,2)
hold on
plot(t,EY2_STSMC_J,'LineWidth',3)
plot(t,EY2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{y_2}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
 set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
    'Color','none');
box on
hold off
        
subplot(me,ne,3)
hold on
plot(t,EZ2_STSMC_J,'LineWidth',3)
plot(t,EZ2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{z_2}$ [m]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,4)
hold on
plot(t,EPHI2_STSMC_J,'LineWidth',3)
plot(t,EPHI2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\phi_2}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$-STSMC_J$','$STSMC_R$');
lgd = legend;
 lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,5)
hold on
plot(t,ETHETA2_STSMC_J,'LineWidth',3)
plot(t,ETHETA2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\theta_2}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
     'Color','none');
box on
hold off
        
subplot(me,ne,6)
hold on
 plot(t,EPSI2_STSMC_J,'LineWidth',3)
 plot(t,EPSI2_STSMC_R,'LineWidth',3)
xlim([0 t_max])
ylabel('$e_{\psi_2}$ [grados]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$','$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
box on
hold off


%% Graficas en 3D de la trayectoria del controlador STSMC_R


% Grafica 3d de las trayectorias deseadas y las acutalies de los agentes
figure(11)
subplot(1,2,1)
hold on
title('Trayectoria del lider $STSMC_R$','FontSize',TL,'interpreter','latex')
plot3 (XD2_STSMC_R,YD2_STSMC_R,ZD2_STSMC_R,'LineWidth',3)
plot3 (X2_STSMC_R,Y2_STSMC_R,Z2_STSMC_R,'LineWidth',3)
ylabel('X [m]','FontSize',TL,'interpreter','latex')  
xlabel('Y [m]','FontSize',TL,'interpreter','latex')
zlabel('Z [m]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{2_{STSMC_R}}$','$x_{2d_{STSMC_R}}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
view([.5 .5 .5]);
box on
hold off

subplot(1,2,2)
hold on
title('Trayectoria del lider $STSMC_R$','FontSize',TL,'interpreter','latex')
plot3 (XD1_STSMC_R,YD1_STSMC_R,ZD1_STSMC_R,'LineWidth',3)
plot3 (X1_STSMC_R,Y1_STSMC_R,Z1_STSMC_R,'LineWidth',3)
ylabel('X [m]','FontSize',TL,'interpreter','latex')  
xlabel('Y [m]','FontSize',TL,'interpreter','latex')
zlabel('Z [m]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{2_{STSMC_R}}$','$x_{2d_{STSMC_R}}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
view([.5 .5 .5]);
box on
hold off



%% Graficas en 3D de la trayectoria del controlador STSMC_J


% Grafica 3d de las trayectorias deseadas y las acutalies de los agentes
figure(12)
subplot(1,2,1)
hold on
title('Trayectoria del lider $STSMC_J$','FontSize',TL,'interpreter','latex')
plot3 (XD2_STSMC_J,YD2_STSMC_J,ZD2_STSMC_J,'LineWidth',3)
plot3 (X2_STSMC_J,Y2_STSMC_J,Z2_STSMC_J,'LineWidth',3)
ylabel('X [m]','FontSize',TL,'interpreter','latex')  
xlabel('Y [m]','FontSize',TL,'interpreter','latex')
zlabel('Z [m]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{2_{STSMC_J}}$','$x_{2d_{STSMC_J}}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
view([.5 .5 .5]);
box on
hold off

subplot(1,2,2)
hold on
title('Trayectoria del seguidor $STSMC_J$','FontSize',TL,'interpreter','latex')
plot3 (XD1_STSMC_J,YD1_STSMC_J,ZD1_STSMC_J,'LineWidth',3)
plot3 (X1_STSMC_J,Y1_STSMC_J,Z1_STSMC_J,'LineWidth',3)
ylabel('X [m]','FontSize',TL,'interpreter','latex')  
xlabel('Y [m]','FontSize',TL,'interpreter','latex')
zlabel('Z [m]','FontSize',TL,'interpreter','latex')
leg1=legend('$x_{2_{STSMC_J}}$','$x_{2d_{STSMC_J}}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
view([.5 .5 .5]);
box on
hold off


%% Senales de control respecto al tiempo STSMC_R
        
figure(13)
subplot(2,2,1)
hold on
plot(t,UZ1_STSMC_R)
xlim([0 t_max])
ylabel('$U_{z_1}$ [N]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI1_STSMC_R)
xlim([0 t_max])
ylabel('$\tau_{\phi_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA1_STSMC_R)
xlim([0 t_max])
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\theta_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI1_STSMC_R)
xlim([0 t_max])
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\psi_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off


figure(14)
subplot(2,2,1)
hold on
plot(t,UZ2_STSMC_R)
xlim([0 t_max])
ylabel('$U_{z_2}$ [N]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI2_STSMC_R)
xlim([0 t_max])
ylabel('$\tau_{\phi_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA2_STSMC_R)
xlim([0 t_max])
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\theta_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI2_STSMC_R)
xlim([0 t_max])
leg1=legend('$STSMC_R$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\psi_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off


%% Senales de control respecto al tiempo STSMC_R
        
figure(15)
subplot(2,2,1)
hold on
plot(t,UZ1_STSMC_J)
xlim([0 t_max])
ylabel('$U_{z_1}$ [N]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI1_STSMC_J)
xlim([0 t_max])
ylabel('$\tau_{\phi_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA1_STSMC_J)
xlim([0 t_max])
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\theta_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI1_STSMC_J)
xlim([0 t_max])
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\psi_1}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off


figure(16)
subplot(2,2,1)
hold on
plot(t,UZ2_STSMC_J)
xlim([0 t_max])
ylabel('$U_{z_2}$ [N]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI2_STSMC_J)
xlim([0 t_max])
ylabel('$\tau_{\phi_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA2_STSMC_J)
xlim([0 t_max])
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\theta_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI2_STSMC_J)
xlim([0 t_max])
leg1=legend('$STSMC_J$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = TL;
ylabel('$\tau_{\psi_2}$ [Nm]','FontSize',TL,'interpreter','latex')  
xlabel('Tiempo [s]','FontSize',TL,'interpreter','latex')
box on
hold off

