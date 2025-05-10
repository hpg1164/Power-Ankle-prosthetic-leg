clear;
close all;
clear all;
Km= 0.69688;
Ke=Km*pi/30;
Mp=1.5;
L=15*10^-2;
r=3.5*10^-2;

Ips= 0.034;
b=0.01;
Ip = Ips + Mp * L^2;


R=1.2;
Mw=50*10^-3;
g=9.8;
Iw= Mw*r^2;
En=155;
alpha=2*Mw+(2*Iw/r^2)+Mp;
B=Ip+Mp*L^2;

A23=(Mp^2*g*L^2)/(alpha*B-Mp*L^2);

A43=(Mp*g*L*alpha)/(alpha*B-(Mp^2)*(L^2));

b4=(Mp*L*r+B)/(R*(alpha*B-(Mp^2)*(L^2)));
b2=((Mp*L+r*alpha)/(R*(alpha*B-(Mp^2)*(L^2))));

A=[ 0   1    0   0;
    0  0  A23  0;
    0  0    0    1;
    0 0  A43  0];

B= [0;
    2*b2;
    0;
    2*b4];

C= [0  0  1  0];
 D= [0];


%% run the simulink model
sim("legstatespacemodel.slx")

 