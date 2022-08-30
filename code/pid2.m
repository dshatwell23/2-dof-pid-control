clearvars; close('all'); clc;

Kp = 0.03;
Ti = 0.4;
Td = 0.5;
alfa = 1;
beta = 0.8;

s = tf('s');

Cr = Kp*(alfa*beta*Ti*Td*s^2 + (alfa*Td + beta*Ti)*s + 1)/(alfa*Ti*Td*s^2 + Ti*s)
Cy = Kp*(Ti*Td*(alfa + 1)*s^2 + (Ti + alfa*Td)*s + 1)/(alfa*Ti*Td*s^2 + Ti*s)