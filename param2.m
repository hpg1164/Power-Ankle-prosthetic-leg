clear;
close all;
clear all;
Km= 0.69688;
Ke=Km*pi/30;
M=0.9715;
L=0.01;
r=13;

% Ips= 0.034;
% b=0.01;
% Ip = Ips + Mp * L^2;


% R=1.2;
% Mw=50*10^-3;
 g=9.8;
% Iw= Mw*r^2;
% En=155;
% alpha=2*Mw+(2*Iw/r^2)+Mp;
% B=Ip+Mp*L^2;
jc= 0.0037;
d = 0.246;
k = 0.5;
kt = 0.113;
ke = 0.0115;




A21=-(M*g*d/jc+M*d^2);
A22 = -(k/jc+M*d^2);
A23 = (kt/jc+M*d^2);
A32 = -(ke/L);
A33 = -r/L;

% A43=(Mp*g*L*alpha)/(alpha*B-(Mp^2)*(L^2));
% 
% b4=(Mp*L*r+B)/(R*(alpha*B-(Mp^2)*(L^2)));
% b2=((Mp*L+r*alpha)/(R*(alpha*B-(Mp^2)*(L^2))));
b3 = 1/L;
A=[ 0   1    0   ;
    A21  A22  A23  ;
    0  A32    A33   ];

B= [0;
    0;
    b3;
    ];

C= [1 0 0];
 D= [0];


%% run the simulink model
sim("legstatespacemodel.slx")

 