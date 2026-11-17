close all; 
hovercraft_vel_dynamic_lin; 

%% Discretizzazioe Sistema      
Ts=0.05; %Deve essere 5/10 volte più piccolo del polo più veloce               
sysD=c2d(sys,Ts,'zoh');   %discretizzazione con ZOH 
Ad=sysD.A;
Bd=sysD.B; 


%% Optimal Planing -> Approccio basato su lsqlin 

%definizione stato iniziale e stato finale ->[v_des, w_des] 
X0=[0;0];
XT=[0.5;0.3];


% Vincoli sugli ingressi
u_lb = -0.3;
u_ub = 0.6;

% Inizializzazione
exitflag=-1; 
N=1; % numero di passi (scelta iniziale) 
deltaN=1; 
Nm=size(Bd,2); %colonne di Bd 

% Calcolo ingresso per il mntenimento a XT 
up = pinv(Bd) * (eye(size(Ad)) - Ad) * XT;

Rn = Bd;

while exitflag ~= 1 
    N=N+1; 
    Rn = [Bd Ad*Rn];
    Ccost=eye(N*Nm); 
    dcost=zeros(N*Nm , 1); 

    Aineq= [eye(N*Nm) ; - eye(N*Nm)]; 
    Bineq = [u_ub*ones(N*Nm,1) ; -u_lb*ones(N*Nm,1)] ;

    Aeq = Ad * Rn; 
    beq = XT - Ad^(N+1)*X0 - Bd*up;
    
    %calcolo controlli ottimi con lsqlin
    [u_opt_bar , ~, ~, exitflag]  = lsqlin(Ccost,dcost,Aineq,Bineq,Aeq,beq,[],[]);
   
end

%aggiunta controllo finale up (al contrario perchè non ancora flippato) 
u_opt=[up;u_opt_bar];

% Tempo totale e asse temporale
T = N * Ts;
t_total = 0:Ts:T;


% Controllo se quanto fatto sopra è andato correttamente (accetto una
% tollerenza del 0.01) 
tol=0.01;
 if(min(u_opt) <u_lb-tol || max(u_opt) > u_ub+tol)
    disp("Controli sopra i limiti!")
 else
     disp(['Controlli dentro limini. Tempo e''  pari a ' num2str(T) ] )

 end

% composizine degli ingressi in forma matriciale NxNm (=Nx2) e poi flippo 
U_opt= reshape(u_opt, 2, [])';  % righe = istanti, colonne = u1, u2
U_opt=flipud(U_opt);


%% Simulazione e Plot 

% Plot comandi 
figure;
plot(t_total, U_opt);
xlabel('Tempo [s]');
ylabel('Ingressi ottimi');
legend('u_1', 'u_2');
title('Controllo ottimo con vincoli (lsqlin)');

%imput per simulink; 
timing = Ts * (0:size(U_opt,1)-1)'; %si ottiene [0 , Ts, 2Ts, .... NTs] uno per ogni ingresso 
uSim = [timing, U_opt];

y_lin = lsim(sys,U_opt,timing,X0, 'zoh' );
ySim = [timing, y_lin];

figure;
plot(t_total,y_lin); 
legend("v","w"); 
xlabel('Tempo [s]');
