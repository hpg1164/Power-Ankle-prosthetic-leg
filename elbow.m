function xdot = elbow(t,x)

global pars u

% Parameters
m = pars(1);
g = pars(2);
L1 = pars(3);
L2 = pars(4);
p1 = pars(5);
p2 = pars(6);
Fo = pars(7);
FT = pars(8);
Lo = pars(9);
Lu = pars(10);
W = pars(11);
Usec = pars(12);
Qo = pars(13);
Vmax = pars(14);
Cecc = pars(15);
del = pars(16);
Ksec = pars(17);
a = pars(18);
b = pars(19);
v1 = pars(20);
v2 = pars(21);
v3 = pars(22);
Ta = pars(23);
Td = pars(24);

% States
q = x(1);
v = x(2);
Act = x(3);
Scc = x(4);

% Biceps origin
xo = 0;
yo = L1*p1;

% Biceps insertion
xi = p2*L2*cos(q);
yi = p2*L2*sin(q);

% Elbow
xe = 0;
ye = 0;

% Slope and intercept of biceps line of action
aLOI = (yi - yo)/(xi - xo);
bLOI = yi - aLOI*xi;

% Slope and intercept of biceps moment arm
aMA = -1/aLOI;
bMA = ye - aMA*xe;

% Intercept of biceps line of action and moment arm
xMA = (bMA - bLOI)/(aLOI - aMA);
yMA = aMA*xMA + bMA;

% Moment arm length
r = sqrt((yMA - ye)^2 + (xMA - xe)^2);

% Muscle length
Lmus = sqrt((yi - yo)^2 + (xi - xo)^2);

% Muscle velocity
Vmus = (2*L2*p2*sin(q).*(xo - L2*p2*cos(q)).*v - 2*L2*p2*cos(q).*(yo - L2*p2*sin(q)).*v)./sqrt(2*((xo - L2*p2*cos(q)).^2 + (yo - L2*p2*sin(q)).^2));

% Passive elbow torque
qmin = -70*pi/180;
qmax =  50*pi/180;
qmid =   0*pi/180;
k1 = 2.5;
k2 = 5000;
d = 1;
if (q < qmin)
    taup = -k1*(q - qmid) + k2*(q - qmin)^2 - d*v;
elseif (q <= qmax)
    taup = -k1*(q - qmid) - d*v;
elseif (q > qmax)
    taup = -k1*(q - qmid) - k2*(q - qmax)^2 - d*v;
end

% Activation dynamics
dAct = ((1/Ta - 1/Td)*u + 1/Td)*(u - Act);

% CC length
Lcc = sqrt(Scc^2 + (Lo*sin(Qo))^2);

% CC pennation angle cosine
cosQ = Scc/Lcc;

% SEC length
Lsec = Lmus - Scc;

% Calculate CC velocity
if (Lsec < Lu)
    Vcc = Vmus;
else
    % SEC force
    Fsec = Ksec*(Lsec - Lu)^2;

    % CC force
    Fcc = Fsec/cosQ;

    % CC force-length factor
    FL = exp(-1/W*(Lcc/Lo - 1)^2);

    % CC force-velocity factor
    FV = Fcc/(Fo*Act*FL);

    % CC velocity
    if (FV < 1)
        Vcc = Lo*b*(FV - 1)/(a + FV);
    elseif (FV < (Cecc*del*v1 + v1)/(del*v1 + v1))
        Vcc = Lo*v1*(1 - FV)/(FV - Cecc);
    else
        Vcc = Lo*(FV - v3)/v2;
    end
end

dScc = (Lcc*Vcc)/sqrt(Lcc^2 - (Lo*sin(Qo))^2);

% Elbow torque
tau = r*Fsec + taup;

% Equation of motion
vdot = (tau - L2*cos(q)*m*g)/(m*L2^2);

% State rates
xdot(1) = v;
xdot(2) = vdot;
xdot(3) = dAct;
xdot(4) = dScc;
xdot = xdot';

end