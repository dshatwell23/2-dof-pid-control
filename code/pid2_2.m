clearvars; close('all'); clc;

Kp = 0.3;
Ki = 0.095;
Kd = 0.02;
alfa = 50;
beta = 0.92;
gamma = 0;

s = tf('s');

Cr = ( (Kp*beta+Kd*alfa*gamma)*s^2 + (Kp*alfa*beta+Ki)*s + Ki*alfa)/( s^2 + alfa*s )
Cy = ( (Kp*beta+Kd*alfa)*s^2 + (Kp*alfa+Ki)*s + Ki*alfa)/( s^2 + alfa*s )

