clear all
close all
clc

global pars u 

% User-specified parameters
u  = 1.0;      % Muscle excitation
m  = 10;       % Dumbbell weight (lbs)
p2 = 0.10;     % Biceps insertion (fraction of forearm length from elbow);
               %  this affects the biceps moment arm to elbow
T  = 0.01;     % Duration of simulation (s)
q0 = 0*pi/180; % Initial elbow angle (full extension = -90 deg)

% Other specified parameters
L1   = 13*2.54/100; % Upperarm length (m)
L2   = 13*2.54/100; % Forearm length (m)
g    = 9.81;        % Gravity (m/s^2)
p1   = 0.90;        % Biceps origin (fraction of upperarm length from elbow)
Fo   = 2309;        % Maximum isometric force (N) (Holzbaur et al., 2005)
FT   = 0.50;        % Fast-twitch fiber fraction
Lo   = 0.132;       % Optimal CC length (m) (Holzbaur et al., 2005)
Lu   = 0.192;       % Unloaded SEC length (m) (Holzbaur et al., 2005)
W    = 0.40;        % CC force-length width (Winters & Stark, 1985)
Usec = 0.04;        % SEC strain a max isometric force
Qo   = 5*pi/180;    % CC pennation angle at Lcc = Lo
Vmax = 12.0;        % CC maximum shortening velocity (Lo)
Cecc = 1.5;         % CC eccentric force plateau (Fo)
del  = 5.67;        % CC force-velocity linear transition factor (McLean et al., 2003)

% Calculated parameters
Ksec = Fo/(Usec*Lu)^2; % SEC stiffness (van Soest & Bobbert, 1993)
a = 0.1 + 0.4*FT;      % CC force-velocity Hill shape factor (Umberger et al., 2003)
b = Vmax*a;            % CC force-velocity Hill intercept factor (Umberger et al., 2003)
v1 = (Cecc - 1)*b./(a + 1); % v1,v2,v3 are just convenient collections of terms for force-velocity equations (McLean et al., 2003)
v2 = (a + 1)./(b*(del + 1).^2);
v3 = (Cecc - 1)*del^2/(del + 1)^2 + 1;
Ta = 0.080 - 0.050*FT; % Activation dynamics rise time constant (Umberger et al., 2003)
Td = 0.095 - 0.060*FT; % Activation dynamics fall time constant (Umberger et al., 2003)

% Store all parameters in a global variable for passing
pars(1) = m/2.205;
pars(2) = g;
pars(3) = L1;
pars(4) = L2;
pars(5) = p1;
pars(6) = p2;
pars(7) = Fo;
pars(8) = FT;
pars(9) = Lo;
pars(10) = Lu;
pars(11) = W;
pars(12) = Usec;
pars(13) = Qo;
pars(14) = Vmax;
pars(15) = Cecc;
pars(16) = del;
pars(17) = Ksec;
pars(18) = a;
pars(19) = b;
pars(20) = v1;
pars(21) = v2;
pars(22) = v3;
pars(23) = Ta;
pars(24) = Td;

% Time vector
dt    = 0.001;  % Timestep for reporting
tspan = 0:dt:T;

% Initial joint kinematic states
v0 = 0; % Initial elbow angular velocity (leave at zero)

% Initial muscle states
xo = 0;
yo = L1*p1;
xi = p2*L2*cos(q0);
yi = p2*L2*sin(q0);
xe = 0;
ye = 0;
aLOI = (yi - yo)./(xi - xo);
bLOI = yi - aLOI.*xi;
aMA = -1./aLOI;
bMA = ye - aMA.*xe;
xMA = (bMA - bLOI)./(aLOI - aMA);
yMA = aMA.*xMA + bMA;
r = sqrt((yMA - ye).^2 + (xMA - xe).^2);
Lmus = sqrt((yi - yo).^2 + (xi - xo).^2);
Fsec = m*g*L2*cos(q0)/r;
Lsec = sqrt(Fsec/Ksec) + Lu;
Scc0 = Lmus - Lsec;
Lcc = sqrt(Scc0^2 + (Lo*sin(Qo))^2);
cosQ = Scc0/Lcc;
Fcc = Fsec/cosQ;
FL = exp(-1/W*(Lcc/Lo - 1)^2);
FV = 1.0;
Act0 = Fcc/(Fo*FL*FV);
if (Act0 > 1.0)
    fprintf('Too heavy!  Please reduce the value of m.\n');
    return
end
x0 = [q0;v0;Act0;Scc0];

% Run simulation
[t,states] = ode23s(@elbow,tspan,x0);

% Process output
q = states(:,1);
v = states(:,2);
Act = states(:,3);
Scc = states(:,4);

% Mass coordinates
x = L2*cos(q);
y = L2*sin(q);

% Biceps origin
xo(1:length(t)) = 0;
yo(1:length(t)) = L1*p1;
xo = xo'; yo = yo';

% Biceps insertion
xi = p2*L2*cos(q);
yi = p2*L2*sin(q);

% Elbow
xe = zeros(1,length(t))';
ye = zeros(1,length(t))';

% Slope and intercept of biceps line of action
aLOI = (yi - yo)./(xi - xo);
bLOI = yi - aLOI.*xi;

% Slope and intercept of biceps moment arm
aMA = -1./aLOI;
bMA = ye - aMA.*xe;

% Intercept of biceps line of action and moment arm
xMA = (bMA - bLOI)./(aLOI - aMA);
yMA = aMA.*xMA + bMA;

% Moment arm length
r = sqrt((yMA - ye).^2 + (xMA - xe).^2);

% Muscle length
Lmus = sqrt((yi - yo).^2 + (xi - xo).^2);

% SEC length
Lsec = Lmus - Scc;

% SEC force
Fsec = Ksec*(Lsec - Lu).^2;

% CC length
Lcc = sqrt(Scc.^2 + (Lo*sin(Qo))^2);

% CC pennation angle cosine
cosQ = Scc./Lcc;

% Muscle velocity
Vmus = (2*L2*p2*sin(q).*(xo - L2*p2*cos(q)).*v - 2*L2*p2*cos(q).*(yo - L2*p2*sin(q)).*v)./sqrt(2*((xo - L2*p2*cos(q)).^2 + (yo - L2*p2*sin(q)).^2));

% Calculate CC velocity
for i = 1:length(tspan)
    if (Lsec(i) < Lu)
        Vcc(i) = Vmus;
    else
        % CC force
        Fcc = Fsec(i)/cosQ(i);

        % CC force-length factor
        FL = exp(-1/W*(Lcc(i)/Lo - 1)^2);

        % CC force-velocity factor
        FV = Fcc/(Fo*Act(i)*FL);

        % CC velocity
        if (FV < 1)
            Vcc(i) = Lo*b*(FV - 1)/(a + FV);
        elseif (FV < (Cecc*del*v1 + v1)/(del*v1 + v1))
            Vcc(i) = Lo*v1*(1 - FV)/(FV - Cecc);
        else
            Vcc(i) = Lo*(FV - v3)/v2;
        end
    end
end
    
figure()
set(gcf, 'Position',  [100, 1100, 500, 500])
for i = 1:length(t)
    clf
    hold on; box on;
    xlim([-30 50])
    ylim([-40 40])
    %axis equal
    plot([0 0],[0 L2*100],'k-','LineWidth',2)
    plot([0 x(i)]*100,[0 y(i)]*100,'k-','LineWidth',2)
    plot(x(i)*100,y(i)*100,'ro','MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',m*2)
    if (Act(i) < 0.50)
        col = [2*Act(i) 0 1];
    else
        col = [1 0 -2*Act(i) + 2];
    end
    plot([xo(i) xMA(i)]*100,[yo(i) yMA(i)]*100,'r--')
    plot([xo(i) xi(i)]*100,[yo(i) yi(i)]*100,'r-','LineWidth',0.5 + Fsec(i)/Fo*10,'Color',col)
    plot([xe(i) xMA(i)]*100,[ye(i) yMA(i)]*100,'b-','LineWidth',2)
    %legend('Upper arm','Forearm','Dumbbell','Muscle length','Muscle moment arm')
    pause(dt)
end

figure()
set(gcf, 'Position',  [600, 1100, 500, 800])
subplot(4,2,1)
hold on; box on;
plot(t,Act*100,'LineWidth',2)
plot([t(1),t(end)],[u u]*100,'k--')
xlabel('Time (s)','FontWeight','b')
ylabel('Muscle activation (%)','FontWeight','b')
ylim([0 110])
subplot(4,2,3)
hold on; box on;
plot(t,Lmus*100,'LineWidth',2)
xlabel('Time (s)','FontWeight','b')
ylabel('Muscle length (cm)','FontWeight','b')
subplot(4,2,5)
hold on; box on;
plot(t,Vmus*100,'LineWidth',2)
xlabel('Time (s)','FontWeight','b')
ylabel('Muscle velocity (cm/s)','FontWeight','b')
subplot(4,2,2)
hold on; box on;
plot(t,Fsec/Fo*100,'LineWidth',2)
plot([t(1),t(end)],[100 100],'k--')
xlabel('Time (s)','FontWeight','b')
ylabel('SEC force (%F_o)','FontWeight','b')
ylim([0 100*Cecc])
subplot(4,2,4)
hold on; box on;
plot(t,Lcc/Lo*100,'LineWidth',2)
plot([t(1),t(end)],[100 100],'k--')
xlabel('Time (s)','FontWeight','b')
ylabel('CC length (%L_o)','FontWeight','b')
subplot(4,2,6)
hold on; box on;
plot(t,Vcc/Lo,'LineWidth',2)
xlabel('Time (s)','FontWeight','b')
ylabel('CC velocity (L_o/s)','FontWeight','b')
subplot(4,2,7)
hold on; box on;
plot(t,Fsec.*Vmus,'LineWidth',2)
xlabel('Time (s)','FontWeight','b')
ylabel('Muscle power (W)','FontWeight','b')
subplot(4,2,8)
hold on; box on;
plot(t,Fcc.*Vcc,'LineWidth',2)
xlabel('Time (s)','FontWeight','b')
ylabel('CC power (W)','FontWeight','b')
