% iPendel Parameterdefinition
% 
% Run this file before opening the Simscape file "iPendel_3Eck_LQR"
%
% A. Gad | May 2025
%

close all
clear all

disp( ' ' );
disp( '################################################################');
disp( ' ' );
mb = 0.160;                   % Masse des Pendelkörpers inkl. Motor, Platine, usw. (System)
lb = 46.53e-3;                      % Momentarm des Systems 
mw = 92.0e-3;                      % Masse des Schwungrads
lw = 40.00e-3;                      % Momentarm des Schwungrads
M = mb*lb+mw*lw;            % Gesamtdrehmoment im System
g = 9.81;                   % m/s²
Iw = 0.00008;                  % Trägheit Schwungrads - kg*m²
Ib = 0.00015;                      % Trägheit Systems - kg*m²
I = Ib+mb*lb^2+mw*lw^2;     % Gesamtträgheit im System - kg*m²
Cb = 1.02e-3;                   % Coefficient of Friction, Pendulum Body - kg*m²/s
Cw = 0.05e-3;                   % Coefficient of Friction, Schwungrad - kg*m²/s
Km = 0.0355;                      % Torque constant of the motor N*m/A        

a21 = ((mb*lb+mw*lw)*g)/(Ib+mw*lw*lw);
a22 = -(Cb/(Ib+mw*lw*lw));
a23 = Cw/(Ib+mw*lw*lw);
a31 = -(((mb*lb+mw*lw)*g)/(Ib+mw*lw*lw));
a32 = Cb/(Ib+mw*lw*lw);
a33 = -((Cw*(Ib+Iw+mw*lw*lw))/(Iw*(Ib+mw*lw*lw)));

b12 = -(Km/(Ib+mw*lw*lw));
b13 = (Km*(Ib+Iw+mw*lw*lw))/(Iw*(Ib+mw*lw*lw));

%%%%%%%%%%%%%%%%%Systemmatrix hergeleitet anhand Lagrange-Funktion%%%%%%%%%%%%%%
A= [0,1,0;
    a21,a22,a23;
    a31,a32,a33];
B= [0; b12; b13];
c= [1,0,0;
    0,1,0;
    0,0,1];
d=[0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

% State-Space Modell
sys = ss(A,B,c,d);

poles = eig(A);

% The Q matrix

Q = c'*c;
Q(1,1) = 1;
Q(2,2) = 1;
Q(3,3) = 0.00001;


% R Value of the LQR

R = 10.0;

% Gain of the LQR

K = lqr(A,B,Q,R)

% Prepare to plot the closed-loop response in matlab

Ac = [(A-B*K)];
Bc = [B];
Cc = [c];
Dc = [d];

sys_cl = ss(Ac,Bc,Cc,Dc);

poles_cl = eig(Ac);

t = 0:0.01:1;
r = 0.2*ones(size(t));
[y,t,x] = lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(2), 'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

% open the Simscape file "iPendel_3Eck_LQR"

open_system('iPendel_3Eck_LQR');
