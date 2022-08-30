clearvars; close('all'); clc; 

%% Constantes

%--------------------------- Input Parameters ----------------------------%

% Select output location, Out:
                    % "1" = Encoder #1
                    % "2" = Encoder #2
Out = 2;

% Plant configuration, Config:
                    % "1" = Drive disk only, (use only Out=1)
                    % "2" = Drive and load disk - rigid drive train
                    % "3" = Drive and load disk - flexible drive belt
Config = 3;

% Select mass parameters
mwd = 0.8; % mass of brass weights on drive disk
rwd = 0.05; % radius from center of plate on drive disk
mwl = 2; % mass of brass weights on load disk
rwl = 0.1; % radius from center of plate on load disk

% Lower (Drive) pulley: Select one of the following as appropriate:
% npd = 18; Jpd = 0.000003; % 18 tooth
npd = 24; Jpd = 0.000008; % 24 tooth
% npd = 36; Jpd = 0.000038; % 36 tooth
% npd = 72; Jpd = 0.00055; % 72 tooth

% Upper (Load) pulley pulley: Select one of the following as appropriate:
% npl = 18; Jpl = 0.000003; % 18 tooth
% npl = 24; Jpl = 0.000008; % 24 tooth
npl = 36; Jpl = 0.000038; % 36 tooth
% npl = 72; Jpl = 0.00055; % 72 tooth

% Default friction coefficients
c1 = 0.004; % if belt from drive attached
% c1 = 0.002; % if belt from drive not attached
c2 = 0.05; % if belt from drive attached
c12 = 0.017; % viscous coupling between drive and load

k = 8.45; % torsional spring constant

khw = 5.81; % Hardware gain, assume kg=1, will be corrected below if Out=2


%------------------------- The Rest is Automatic -------------------------%

gr = 6*npd/npl; % gear ratio
grprime = npd/12; % drive to SR pulley gear ratio

% First calculate known inertias
Jdd = 0.00040;
Jdl = 0.0065;
Jpbl = 0.000031; % Backlash mechanism
Jp = Jpd + Jpl + Jpbl;

% Calculate Drive inertia
rwdo = 0; % initializing
rwlo = 0;

% First find which size weight used, can use only 0, 2, or 5 weights
if mwd < 0.81
    if mwd > 0.39
        rwdo = 0.016; % smaller brass weight used
    end 
end
if mwd < 2.1
    if mwd > 0.9
        rwdo = 0.025; % larger brass weight used
    end
end

% Calculate inertia about weights own CG
Jwdo = 1/2*mwd*rwdo^2;

% Combined drive inertia:
Jd = Jdd + mwd*rwd^2 + Jwdo;

% Calculate Load inertia
if mwl < 0.81
    if mwl > 0.39
        rwlo = 0.016; % smaller brass weight used
    end 
end
if mwl < 2.1
    if mwl > 0.9
        rwlo = 0.025; % larger brass weight used
    end
end

% Calculate inertia about weights own CG
Jwlo = 1/2*mwl*rwlo^2;

% Combined Load inertia:
Jl = Jdl + mwl*rwl^2 + Jwlo;

% Build transfer functions and state space models
if Config == 1 % Drive disk only:
    % Transfer Function:
    N = khw;
    D = [Jd c1 0];
    
    % State space model
    A1 = [0 1];
    A2 = [0 -c1/Jd];
    Ao1 = [A1; A2];
    B = [0 khw/Jd]';
    C = [1 0]; % Theta1 output
end

if Config == 2 % Drive and load disks - rigid drive train
    if Out == 1 % Encoder #1 output
        Jr = Jd + Jp*grprime^(-2) + Jl*gr^(-2); % Reflected inertia at drive
        cr = c1 + c2*gr^(-2); % Reflected damping to drive
        
        % Transfer function 
        N = khw;
        D = [Jr cr 0];

        % State space model
        A1 = [0 1];
        A2 = [0 -cr/Jr];
        Ao1 = [A1; A2];
        B = [0 khw/Jr]';
        C = [1 0];
    end
    
    if Out == 2 % Encoder #2 output
        Jr = Jd*gr^2 + Jp*(gr/grprime)^2 + Jl; % Reflected inertia at load
        cr = c1*gr^2 + c2; % Reflected damping at load
        
        % Transfer function 
        N = khw*gr;
        D = [Jr cr 0];

        % State space model
        A1 = [0 1];
        A2 = [0 -cr/Jr];
        Ao1 = [A1; A2];
        B = [0 khw*gr/Jr]';
        C = [1 0];
    end
end

if Config == 3 % Drive & load disks - flexible drive train
    Jdstr = Jd + Jp*grprime^(-2); % Pulley inertias combined with drive
    
    % Transfer function:
    % The following does not include the coupled damping c12
    % N1 = khw*[J1 c2 k];
    % N2 = khw*k/gr;
    % D = [Jdstr*Jl (c2*Jdstr + c1*Jl) (k*(Jdstr + Jl/gr^2) + c1*c2) (k*(c1 + c2/gr^2)) 0]
    % The following includes c12
    N1 = khw*[Jl c2+c12 k];
    N2 = khw/gr*[c12 k];
    D = [Jdstr*Jl (c2*Jdstr + c1*Jl + (Jdstr + Jl/gr^2)*c12) (k*(Jdstr + Jl/gr^2) + c1*c2 + (c1 + c2/gr^2)*c12) (k*(c1 + c2/gr^2)) 0];
    
    % State space model:
    A1 = [0 1 0 0];
    A3 = [0 0 0 1];
    
    % The following does not include the couple damping c12
    % A2 = [-k/Jdstr/gr^2 -c1/Jdstr k/Jdstr/gr 0];
    % A4 = [k/Jl/gr 0 -k/Jl -c2/Jl];
    % The following includes c12
    A2 = [-k/gr^2/Jdstr -(c1 + c12/gr^2)/Jdstr k/gr/Jdstr c12/gr/Jdstr];
    A4 = [k/gr/Jl c12/gr/Jl -k/Jl -(c2 + c12)/Jl];
    Ao1 = [A1; A2; A3; A4];
    B = [0 khw/Jdstr 0 0]';
    if Out == 1 % Encoder #1 output
        C = [1 0 0 0];
    end
    if Out == 2 % Encoder #2 output
        C = [0 0 1 0];
    end
end

Ac = Ao1;
Bc = B;
Cc = C;
Dc = 0;

%% Control

% Plant TF
[b,a] = ss2tf(Ac,Bc,Cc,Dc);
Gp = tf(b,a);

% PID Control
PID = pidtune(Gp, 'pid');

% PID Control
PID2 = pidtune(Gp, 'pid2');
s = tf('s');
Gr = PID2.Ki/s + PID2.Kp*PID2.b;
Gy = PID2.Ki/s + PID2.Kp + PID2.Kd*s;

% Step response
t = 0:0.001:8;
y_PID = 90*step(feedback(PID*Gp, 1), t);
y_PID2 = 90*step(Gr*feedback(Gp, Gy), t);

figure;

subplot(2,1,1);
p=plot(t, 90*ones(1, length(t)), 'b', t, y_PID, 'r');
p(1).LineWidth = 1.5; 
p(2).LineWidth = 1.5; 
axis([0, 8, 0, 120]);
grid;
title('PID - Step Response', 'FontSize', 16);
xlabel('Time (s)', 'FontSize', 14);
ylabel('\theta_{2} (deg)', 'FontSize', 14)
legend({'r(t)', '\theta_2(t)'}, 'FontSize', 12);

subplot(2,1,2);
p=plot(t, 90*ones(1, length(t)), 'b', t, y_PID2, 'r');
p(1).LineWidth = 1.5; 
p(2).LineWidth = 1.5; 
axis([0, 8, 0, 120]);
xlabel('Time (s)', 'FontSize', 14);
ylabel('\theta_{2} (deg)', 'FontSize', 14);
grid;
title('2-DOF PID - Step Response', 'FontSize', 16);
legend({'r(t)', '\theta_2(t)'}, 'FontSize', 12);