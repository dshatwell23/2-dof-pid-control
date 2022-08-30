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

%% Control Predictivo LQR

Ts=0.01;

[A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,Ts);

p=10; % PREDICTION HORIZON

KCA=[C*A;C*A^2;C*A^3;C*A^4;C*A^5;C*A^6;C*A^7;C*A^8;C*A^9;C*A^10];

KCAB=[...
C*B 0 0 0 0 0 0 0 0 0
C*A*B C*B 0 0 0 0 0 0 0 0
C*A^2*B C*A*B C*B 0 0 0 0 0 0 0
C*A^3*B C*A^2*B C*A*B C*B 0 0 0 0 0 0
C*A^4*B C*A^3*B C*A^2*B C*A*B C*B 0 0 0 0 0
C*A^5*B C*A^4*B C*A^3*B C*A^2*B C*A*B C*B 0 0 0 0
C*A^6*B C*A^5*B C*A^4*B C*A^3*B C*A^2*B C*A*B C*B 0 0 0
C*A^7*B C*A^6*B C*A^5*B C*A^4*B C*A^3*B C*A^2*B C*A*B C*B 0 0
C*A^8*B C*A^7*B C*A^6*B C*A^5*B C*A^4*B C*A^3*B C*A^2*B C*A*B C*B 0
C*A^9*B C*A^8*B C*A^7*B C*A^6*B C*A^5*B C*A^4*B C*A^3*B C*A^2*B C*A*B C*B];

c=1; % FIRST COLUMN OF KCAB

G=KCAB(:,1); I=eye(10); lambda=0.0;

H=(inv(G'*G+lambda))*G';

x=[0;0;0;0]; MM=2000;

for k=1:MM
y9=C*A^9*x;
r=1; R(k)=r;
e1=r-y9;
dU= H*e1; du=dU(1);
x1=A*x+B*du; U(k)=du;
y=C*x; Y(k)=y;
x=x1;
end

% PLOTS
ejex = linspace(0,MM*Ts,MM);
subplot(2,1,1), plot(ejex,R,ejex,Y), grid,
ylabel('y(t)'), xlabel('Time (s)'),
subplot(2,1,2), plot(ejex,U), grid,
ylabel('Control u(t)'), xlabel('Time (s)')