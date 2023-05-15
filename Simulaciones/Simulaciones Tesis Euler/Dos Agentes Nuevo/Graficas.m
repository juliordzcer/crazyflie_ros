clc
clear all
close all 
%% Corre los controladores
% 
run('STA2.m');
run('STSMC2.m');
run('NTSMC2.m');
run('TC2.m');
run('PID2.m');

clc
clear all
close all 

%% Carga los datos de interes de los controladores

load('TC_Errores.mat');
load('TC_Estados.mat');
load('TC_Deseadas.mat');
load('TC_Control.mat');

load('PID_Errores.mat');
load('PID_Estados.mat');
load('PID_Deseadas.mat');
load('PID_Control.mat');

load('STA_Errores.mat');
load('STA_Estados.mat');
load('STA_Deseadas.mat');
load('STA_Control.mat');

load('STSMC_Errores.mat');
load('STSMC_Estados.mat');
load('STSMC_Deseadas.mat');
load('STSMC_Control.mat');

load('NTSMC_Errores.mat');
load('NTSMC_Estados.mat');
load('NTSMC_Deseadas.mat');
load('NTSMC_Control.mat');

%% Calcula el error cuadratico medio de los estados del Seguidor y el Lider

XS = [rms(EX1_NTSMC),rms(EX1_STSMC),rms(EX1_TC),rms(EX1_STA)];%];%,rms(EX1_PID)];%
XL = [rms(EX2_NTSMC),rms(EX2_STSMC),rms(EX2_TC),rms(EX2_STA)];%];%,rms(EX2_PID)];

YS = [rms(EY1_NTSMC),rms(EY1_STSMC),rms(EY1_TC),rms(EY1_STA)];%];%,rms(EY1_PID)];
YL = [rms(EY2_NTSMC),rms(EY2_STSMC),rms(EY2_TC),rms(EY2_STA)];%];%,rms(EY2_PID)];

ZS = [rms(EZ1_NTSMC),rms(EZ1_STSMC),rms(EZ1_TC),rms(EZ1_STA)];%];%,rms(EZ1_PID)];
ZL = [rms(EZ2_NTSMC),rms(EZ2_STSMC),rms(EZ2_TC),rms(EZ2_STA)];%,rms(EZ2_PID)];

PHIS = [rms(EPHI1_NTSMC),rms(EPHI1_STSMC),rms(EPHI1_TC),rms(EPHI1_STA)];%,rms(EPHI1_PID)];
PHIL = [rms(EPHI2_NTSMC),rms(EPHI2_STSMC),rms(EPHI2_TC),rms(EPHI2_STA)];%,rms(EPHI2_PID)];

THETAS = [rms(ETHETA1_NTSMC),rms(ETHETA1_STSMC),rms(ETHETA1_TC),rms(ETHETA1_STA)];%,rms(ETHETA1_PID)];
THETAL = [rms(ETHETA2_NTSMC),rms(ETHETA2_STSMC),rms(ETHETA2_TC),rms(ETHETA2_STA)];%,rms(ETHETA2_PID)];

PSIS = [rms(EPSI1_NTSMC),rms(EPSI1_STSMC),rms(EPSI1_TC),rms(EPSI1_STA)];%,rms(EPSI1_PID)];
PSIL = [rms(EPSI2_NTSMC),rms(EPSI2_STSMC),rms(EPSI2_TC),rms(EPSI2_STA)];%,rms(EPSI2_PID)];

%% Calcula el error cuadratico medio de las senales de control del Seguidor y el Lider

UZS = [rms(UZ1_NTSMC),rms(UZ1_STSMC),rms(UZ1_TC),rms(UZ1_STA)];%,rms(UZ1_PID)];
UZL = [rms(UZ2_NTSMC),rms(UZ2_STSMC),rms(UZ2_TC),rms(UZ2_STA)];%,rms(UZ2_PID)];

TAUPHIS = [rms(TAUPHI1_NTSMC),rms(TAUPHI1_STSMC),rms(TAUPHI1_TC),rms(TAUPHI1_STA)];%,rms(TAUPHI1_PID)];
TAUPHIL = [rms(TAUPHI2_NTSMC),rms(TAUPHI2_STSMC),rms(TAUPHI2_TC),rms(TAUPHI2_STA)];%,rms(TAUPHI2_PID)];

TAUTHETAS = [rms(TAUTHETA1_NTSMC),rms(TAUTHETA1_STSMC),rms(TAUTHETA1_TC),rms(TAUTHETA1_STA)];%,rms(TAUTHETA1_PID)];
TAUTHETAL = [rms(TAUTHETA2_NTSMC),rms(TAUTHETA2_STSMC),rms(TAUTHETA2_TC),rms(TAUTHETA2_STA)];%,rms(TAUTHETA2_PID)];

TAUPSIS = [rms(TAUPSI1_NTSMC),rms(TAUPSI1_STSMC),rms(TAUPSI1_TC),rms(TAUPSI1_STA)];%,rms(TAUPSI1_PID)];
TAUPSIL = [rms(TAUPSI2_NTSMC),rms(TAUPSI2_STSMC),rms(TAUPSI2_TC),rms(TAUPSI2_STA)];%,rms(TAUPSI2_PID)];



%% Graficas del error cuadratico medio del error
% ---------------------------- Error cuadratico medio del error ---------------------

mecm=6; necm=1;
Label = categorical({'1.NTSMC','2.STSMC', '3.TC', '4.STA'});%, '5.PID'});%

TL = 20;

figure(1)
subplot(mecm,necm,1)
hold on
bar(Label,XS)
ax = gca;
ax.FontSize = 16;
title('$X_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,2)
hold on
bar(Label,XL)
ax = gca;
ax.FontSize = 16;
title('$X_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,3)
hold on
bar(Label,YS)
ax = gca;
ax.FontSize = 16;
title('$Y_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,4)
hold on
bar(Label,YL)
ax = gca;
ax.FontSize = 16;
title('$Y_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,5)
hold on
bar(Label,ZS)
ax = gca;
ax.FontSize = 16;
title('$Z_S$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,6)
hold on
bar(Label,ZL)
ax = gca;
ax.FontSize = 16;
title('$Z_L$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off



mecm=6; necm=1;

figure(2)
subplot(mecm,necm,1)
hold on
bar(Label,PHIS)
ax = gca;
ax.FontSize = 16;
title('$\phi_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,2)
hold on
bar(Label,PHIL)
ax = gca;
ax.FontSize = 16;
title('$\phi_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,3)
hold on
bar(Label,THETAS)
ax = gca;
ax.FontSize = 16;
title('$\theta_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,4)
hold on
bar(Label,THETAL)
ax = gca;
ax.FontSize = 16;
title('$\theta_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,5)
hold on
bar(Label,PSIS)
ax = gca;
ax.FontSize = 16;
title('$\psi_S$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on
hold off

subplot(mecm,necm,6)
hold on
bar(Label,PSIL)
ax = gca;
ax.FontSize = 16;
title('$\psi_L$','FontSize',TL,'interpreter','latex')
ylabel('[grados]','FontSize',TL,'interpreter','latex')
% saveas(gcf,'xs','epsc')
box on

%% Error cuadratico medio de las senales de control

figure(3)
subplot(2,2,1)
hold on
bar(Label,UZS)
ax = gca;
ax.FontSize = 16;
title('$U_{Z_S}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
bar(Label,TAUPHIS)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\phi_S}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
bar(Label,TAUTHETAS)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\theta_S}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
bar(Label,TAUPSIS)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\psi_S}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off



figure(4)
subplot(2,2,1)
hold on
bar(Label,UZL)
ax = gca;
ax.FontSize = 16;
title('$U_{Z_L}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
bar(Label,TAUPHIL)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\phi_L}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
bar(Label,TAUTHETAL)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\theta_L}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
bar(Label,TAUPSIL)
ax = gca;
ax.FontSize = 16;
title('$\tau_{\psi_L}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off



        % ---------------- Trayectorias deseadas. ---------------------
        
        mt=6; nt=1;
        
        figure(5)
        
        title('Trayectoria del seguidor','FontSize',TL,'interpreter','latex')
        
        subplot(mt,nt,1)
        hold on
        plot(t,X1_NTSMC)
        plot(t,X1_STA)
        plot(t,X1_STSMC)
        plot(t,X1_TC)
        plot(t,X1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$x_{NTSMC}$','$x_{STA}$','$x_{STSMC}$','$x_{TC}$','$x_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,2)
        hold on
        plot(t,Y1_NTSMC)
        plot(t,Y1_STA)
        plot(t,Y1_STSMC)
        plot(t,Y1_TC)
        plot(t,Y1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$y_{NTSMC}$','$y_{STA}$','$y_{STSMC}$','$y_{TC}$','$y_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,3)
        hold on
        plot(t,Z1_NTSMC)
        plot(t,Z1_STA)
        plot(t,Z1_STSMC)
        plot(t,Z1_TC)
        plot(t,Z1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$z_{NTSMC}$','$z_{STA}$','$z_{STSMC}$','$z_{TC}$','$z_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,4)
        hold on
        plot(t,PHI1_NTSMC)
        plot(t,PHI1_STA)
        plot(t,PHI1_STSMC)
        plot(t,PHI1_TC)
        plot(t,PHI1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\phi_{NTSMC}$','$\phi_{STA}$','$\phi_{STSMC}$','$\phi_{TC}$','$\phi_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,5)
        hold on
        plot(t,THETA1_NTSMC)
        plot(t,THETA1_STA)
        plot(t,THETA1_STSMC)
        plot(t,THETA1_TC)
        plot(t,THETA1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\theta_{NTSMC}$','$\theta_{STA}$','$\theta_{STSMC}$','$\theta_{TC}$','$\theta_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,6)
        hold on
        plot(t,PSI1_NTSMC)
        plot(t,PSI1_STA)
        plot(t,PSI1_STSMC)
        plot(t,PSI1_TC)
        plot(t,PSI1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\psi_{NTSMC}$','$\psi_{STA}$','$\psi_{STSMC}$','$\psi_{TC}$','$\psi_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        figure(6)
        
        title('Trayectoria del lider','FontSize',TL,'interpreter','latex')
        
        subplot(mt,nt,1)
        hold on
        plot(t,X2_NTSMC)
        plot(t,X2_STA)
        plot(t,X2_STSMC)
        plot(t,X2_TC)
        plot(t,X2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$x_d$','$x_{NTSMC}$','$x_{STA}$','$x_{STSMC}$','$x_{TC}$','$x_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,2)
        hold on
        plot(t,Y2_NTSMC)
        plot(t,Y2_STA)
        plot(t,Y2_STSMC)
        plot(t,Y2_TC)
        plot(t,Y2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$y_d$','$y_{NTSMC}$','$y_{STA}$','$y_{STSMC}$','$y_{TC}$','$y_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,3)
        hold on
        plot(t,Z2_NTSMC)
        plot(t,Z2_STA)
        plot(t,Z2_STSMC)
        plot(t,Z2_TC)
        plot(t,Z2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$z_d$','$z_{NTSMC}$','$z_{STA}$','$z_{STSMC}$','$z_{TC}$','$z_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,4)
        hold on
        plot(t,PHI2_NTSMC)
        plot(t,PHI2_STA)
        plot(t,PHI2_STSMC)
        plot(t,PHI2_TC)
        plot(t,PHI2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\phi_d$','$\phi_{NTSMC}$','$\phi_{STA}$','$\phi_{STSMC}$','$\phi_{TC}$','$\phi_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,5)
        hold on
        plot(t,THETA2_NTSMC)
        plot(t,THETA2_STA)
        plot(t,THETA2_STSMC)
        plot(t,THETA2_TC)
        plot(t,THETA2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\theta_d$','$\theta_{NTSMC}$','$\theta_{STA}$','$\theta_{STSMC}$','$\theta_{TC}$','$\theta_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(mt,nt,6)
        hold on
        plot(t,PSI2_NTSMC)
        plot(t,PSI2_STA)
        plot(t,PSI2_STSMC)
        plot(t,PSI2_TC)
        plot(t,PSI2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$\psi_d$','$\psi_{NTSMC}$','$\psi_{STA}$','$\psi_{STSMC}$','$\psi_{TC}$','$\psi_{PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
      
        

        
        
        %  -------------------------- Errores agente 1 ------------------
        me=6; ne=1;
        figure(7)
        
        title('Error del seguidor','FontSize',TL,'interpreter','latex')

        subplot(me,ne,1)
        hold on
        plot(t,EX1_NTSMC)
        plot(t,EX1_STA)
        plot(t,EX1_STSMC)
        plot(t,EX1_TC)
        plot(t,EX1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{x_1-NTSMC}$','$e_{x_1-STA}$','$e_{x_1-STSMC}$','$e_{x_1-TC}$','$e_{x_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,2)
        hold on
        plot(t,EY1_NTSMC)
        plot(t,EY1_STA)
        plot(t,EY1_STSMC)
        plot(t,EY1_TC)
        plot(t,EY1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{y_1-NTSMC}$','$e_{y_1-STA}$','$e_{y_1-STSMC}$','$e_{y_1-TC}$','$e_{y_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,3)
        hold on
        plot(t,EZ1_NTSMC)
        plot(t,EZ1_STA)
        plot(t,EZ1_STSMC)
        plot(t,EZ1_TC)
        plot(t,EZ1_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{z_1-NTSMC}$','$e_{z_1-STA}$','$e_{z_1-STSMC}$','$e_{z_1-TC}$','$e_{z_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,4)
        hold on
        plot(t,EPHI1_NTSMC)
        plot(t,EPHI1_STA)
        plot(t,EPHI1_STSMC)
        plot(t,EPHI1_TC)
        plot(t,EPHI1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\phi_1-NTSMC}$','$e_{\phi_1-STA}$','$e_{\phi_1-STSMC}$','$e_{\phi_1-TC}$','$e_{\phi_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,5)
        hold on
        plot(t,ETHETA1_NTSMC)
        plot(t,ETHETA1_STA)
        plot(t,ETHETA1_STSMC)
        plot(t,ETHETA1_TC)
        plot(t,ETHETA1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\theta_1-NTSMC}$','$e_{\theta_1-STA}$','$e_{\theta_1-STSMC}$','$e_{\theta_1-TC}$','$e_{\theta_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,6)
        hold on
        plot(t,EPSI1_NTSMC)
        plot(t,EPSI1_STA)
        plot(t,EPSI1_STSMC)
        plot(t,EPSI1_TC)
        plot(t,EPSI1_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\psi_1-NTSMC}$','$e_{\psi_1-STA}$','$e_{\psi_1-STSMC}$','$e_{\psi_1-TC}$','$e_{\psi_1-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
%         
    
        %  --------------------------- errores agente 2 ------------------------
        figure(8)
        
        title('Error del lider','FontSize',TL,'interpreter','latex')
        
        subplot(me,ne,1)
        hold on
        plot(t,EX2_NTSMC)
        plot(t,EX2_STA)
        plot(t,EX2_STSMC)
        plot(t,EX2_TC)
        plot(t,EX2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{x_2-NTSMC}$','$e_{x_2-STA}$','$e_{x_2-STSMC}$','$e_{x_2-TC}$','$e_{x_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,2)
        hold on
        plot(t,EY2_NTSMC)
        plot(t,EY2_STA)
        plot(t,EY2_STSMC)
        plot(t,EY2_TC)
        plot(t,EY2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{y_2-NTSMC}$','$e_{y_2-STA}$','$e_{y_2-STSMC}$','$e_{y_2-TC}$','$e_{y_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,3)
        hold on
        plot(t,EZ2_NTSMC)
        plot(t,EZ2_STA)
        plot(t,EZ2_STSMC)
        plot(t,EZ2_TC)
        plot(t,EZ2_PID)
        ylabel('[m]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{z_2-NTSMC}$','$e_{z_2-STA}$','$e_{z_2-STSMC}$','$e_{z_2-TC}$','$e_{z_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,4)
        hold on
        plot(t,EPHI2_NTSMC)
        plot(t,EPHI2_STA)
        plot(t,EPHI2_STSMC)
        plot(t,EPHI2_TC)
        plot(t,EPHI2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\phi_2-NTSMC}$','$e_{\phi_2-STA}$','$e_{\phi_2-STSMC}$','$e_{\phi_2-TC}$','$e_{\phi_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,5)
        hold on
        plot(t,ETHETA2_NTSMC)
        plot(t,ETHETA2_STA)
        plot(t,ETHETA2_STSMC)
        plot(t,ETHETA2_TC)
        plot(t,ETHETA2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\theta_2-NTSMC}$','$e_{\theta_2-STA}$','$e_{\theta_2-STSMC}$','$e_{\theta_2-TC}$','$e_{\theta_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off
        
        subplot(me,ne,6)
        hold on
        plot(t,EPSI2_NTSMC)
        plot(t,EPSI2_STA)
        plot(t,EPSI2_STSMC)
        plot(t,EPSI2_TC)
        plot(t,EPSI2_PID)
        ylabel('[grados]','FontSize',TL,'interpreter','latex')  
        xlabel('[seg]','FontSize',TL,'interpreter','latex')
        leg1=legend('$e_{\psi_2-NTSMC}$','$e_{\psi_2-STA}$','$e_{\psi_2-STSMC}$','$e_{\psi_2-TC}$','$e_{\psi_2-PID}$');
        lgd = legend;
        lgd.NumColumns = 5;
        set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');
        box on
        hold off

%         title('Error del agente 2','FontSize',TL,'interpreter','latex')
        
        
% Grafica 3d de las trayectorias deseadas y las acutalies de los agentes
figure(9)
hold on
title('Trayectoria del lider','FontSize',TL,'interpreter','latex')
plot3 (X2_NTSMC,Y2_NTSMC,Z2_NTSMC)
plot3 (X2_PID  ,Y2_PID  ,Z2_PID)
plot3 (X2_STA  ,Y2_STA  ,Z2_STA)
plot3 (X2_STSMC,Y2_STSMC,Z2_STSMC)
plot3 (X2_TC   ,Y2_TC   ,Z2_TC)

leg1=legend('$\tau_{\theta_1-NTSMC}$','$\tau_{\theta_1-STA}$','$\tau_{\theta_1-STSMC}$','$\tau_{\theta_1-TC}$','$\tau_{\theta_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
box on
hold off

figure(10)
title('Trayectoria del siguidor','FontSize',TL,'interpreter','latex')
hold on
plot3 (X1_NTSMC,Y1_NTSMC,Z1_NTSMC)
plot3 (X1_PID,Y1_PID,Z1_PID)
plot3 (X1_STA,Y1_STA,Z1_STA)
plot3 (X1_STSMC,Y1_STSMC,Z1_STSMC)
plot3 (X1_TC,Y1_TC,Z1_TC)
leg1=legend('$\tau_{\theta_1-NTSMC}$','$\tau_{\theta_1-STA}$','$\tau_{\theta_1-STSMC}$','$\tau_{\theta_1-TC}$','$\tau_{\theta_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
box on
hold off
           
        
figure(11)
title('Se\~nales de control del seguidor','FontSize',TL,'interpreter','latex')
subplot(2,2,1)
hold on
plot(t,UZ1_NTSMC)
plot(t,UZ1_STA)
plot(t,UZ1_STSMC)
plot(t,UZ1_TC)
plot(t,UZ1_PID)
leg1=legend('$U_{z_1-NTSMC}$','$U_{z_1-STA}$','$U_{z_1-STSMC}$','$U_{z_1-TC}$','$U_{z_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$U_{z_1}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI1_NTSMC)
plot(t,TAUPHI1_STA)
plot(t,TAUPHI1_STSMC)
plot(t,TAUPHI1_TC)
plot(t,TAUPHI1_PID)
leg1=legend('$\tau_{\phi_1-NTSMC}$','$\tau_{\phi_1-STA}$','$\tau_{\phi_1-STSMC}$','$\tau_{\phi_1-TC}$','$\tau_{\phi_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\phi_1}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA1_NTSMC)
plot(t,TAUTHETA1_STA)
plot(t,TAUTHETA1_STSMC)
plot(t,TAUTHETA1_TC)
plot(t,TAUTHETA1_PID)
leg1=legend('$\tau_{\theta_1-NTSMC}$','$\tau_{\theta_1-STA}$','$\tau_{\theta_1-STSMC}$','$\tau_{\theta_1-TC}$','$\tau_{\theta_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\theta_1}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI1_NTSMC)
plot(t,TAUPSI1_STA)
plot(t,TAUPSI1_STSMC)
plot(t,TAUPSI1_TC)
plot(t,TAUPSI1_PID)
leg1=legend('$\tau_{\psi_1-NTSMC}$','$\tau_{\psi_1-STA}$','$\tau_{\psi_1-STSMC}$','$\tau_{\psi_1-TC}$','$\tau_{\psi_1-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\psi_1}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off


figure(12)
title('Se\~nales de control del lider','FontSize',TL,'interpreter','latex')
subplot(2,2,1)
hold on
plot(t,UZ2_NTSMC)
plot(t,UZ2_STA)
plot(t,UZ2_STSMC)
plot(t,UZ2_TC)
plot(t,UZ2_PID)
ax = gca;
ax.FontSize = 16;
leg1=legend('$U_{z_2-NTSMC}$','$U_{z_2-STA}$','$U_{z_2-STSMC}$','$U_{z_2-TC}$','$U_{z_2-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
title('$U_{z_2}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,2)
hold on
plot(t,TAUPHI2_NTSMC)
plot(t,TAUPHI2_STA)
plot(t,TAUPHI2_STSMC)
plot(t,TAUPHI2_TC)
plot(t,TAUPHI2_PID)
leg1=legend('$\tau_{\phi_2-NTSMC}$','$\tau_{\phi_2-STA}$','$\tau_{\phi_2-STSMC}$','$\tau_{\phi_2-TC}$','$\tau_{\phi_2-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\phi_2}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,3)
hold on
plot(t,TAUTHETA2_NTSMC)
plot(t,TAUTHETA2_STA)
plot(t,TAUTHETA2_STSMC)
plot(t,TAUTHETA2_TC)
plot(t,TAUTHETA2_PID)
leg1=legend('$\tau_{\theta_2-NTSMC}$','$\tau_{\theta_2-STA}$','$\tau_{\theta_2-STSMC}$','$\tau_{\theta_2-TC}$','$\tau_{\theta_2-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\theta_2}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off

subplot(2,2,4)
hold on
plot(t,TAUPSI2_NTSMC)
plot(t,TAUPSI2_STA)
plot(t,TAUPSI2_STSMC)
plot(t,TAUPSI2_TC)
plot(t,TAUPSI2_PID)
leg1=legend('$\tau_{\psi_2-NTSMC}$','$\tau_{\psi_2-STA}$','$\tau_{\psi_2-STSMC}$','$\tau_{\psi_2-TC}$','$\tau_{\psi_2-PID}$');
lgd = legend;
lgd.NumColumns = 5;
set(leg1,'FontSize',TL,'interpreter','latex','EdgeColor','none',...
             'Color','none');  
ax = gca;
ax.FontSize = 16;
title('$\tau_{\psi_2}$','FontSize',TL,'interpreter','latex')
ylabel('[m]','FontSize',TL,'interpreter','latex')
box on
hold off
