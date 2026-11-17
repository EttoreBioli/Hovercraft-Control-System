close all; 
hovercraft_vel_dynamic_lin; 

%% Discretizzazioe Sistema
Ts=0.05; %Deve essere 5/10 volte più piccolo del polo più veloce 
sysD=c2d(sys,Ts,'zoh');   %discretizzazione con ZOH 
Ad=sysD.A;
Bd=sysD.B; 

% Verifica che sysD sia raggiungibile 
Rd=ctrb(Ad,Bd); 
if rank(Rd)<size(Ad,1)
    error("sistema non raggiungibile, controllo ottimo non applicabile"); 
end    

%% Optimal Planning

%defininizione stato iniziale e stato finale 
X0=[0;0];
XT=[0.5;0.3];

N = 80; % numero di passi (scelta iniziale) 
T=N*Ts; %tempo finale 

% Calcolo up per mantenimento dello stato XT (si presuppone che tale stato
% finale sia un equilibrio) 
up = pinv(Bd) * (eye(size(Ad)) - Ad) * XT;

Rn = Bd;
for i = 2:N
    Rn = [Bd, Ad * Rn];
end

%Calcolo controlli ottimi (con aggiunta del controllo di mantenimento) 
u_opt_bar = pinv(Ad * Rn) * (XT - Ad^(N+1) * X0);


%% Tempo minimo con vincoli sul controllo -> approccio numerico

%Inizializzazione limiti su ingressi  
u_lb=-0.3; 
u_ub=0.6; 
maxStep=250; 
stepCounter=0; 

deltaTf= T*0.1 ;%incremnto/decremento tempo  

 if(min(u_opt_bar) <u_lb || max(u_opt_bar) > u_ub)
    disp("Controli iniziali fuori limite")
    %si deve aumentare il tempo 

    while (min(u_opt_bar) < u_lb || max(u_opt_bar) > u_ub)
        stepCounter = stepCounter+1; 
        T=T+deltaTf; 
        N= floor(T/Ts); %Deve essere un intero  

        Rn=Bd;  
        for i = 2:N
            Rn = [Bd, Ad * Rn];
        end
        %calcolo nuova soluzione
        u_opt_bar = pinv(Ad * Rn) * (XT - Ad^(N+1) * X0 - Bd * up); 

        if stepCounter >= maxStep
            break; 
        end 
    end
        
 else
    
    disp(['Controlli iniziali dentro limiti. Tempo e''  pari a ' num2str(T) ] )
    %si può diminuire il tempo 

    while (min(u_opt_bar) > u_lb && max(u_opt_bar) < u_ub)
        stepCounter = stepCounter+1;

        T=T-deltaTf; 
        N= floor(T/Ts); 
        Rn=Bd;  
        for i = 2:N
            Rn = [Bd, Ad * Rn];
        end
        %calcolo nuova soluzione
        u_opt_bar = pinv(Ad * Rn) * (XT - Ad^(N+1) * X0 - Bd * up); 

        if stepCounter == maxStep
            break; 
        end 

        if(min(u_opt_bar) <u_lb || max(u_opt_bar) > u_ub)
            %ultimo step fuori limite
            T=T+deltaTf; 
            N= floor(T/Ts); 
            Rn=Bd;  
            for i = 2:N
                Rn = [Bd, Ad * Rn];
            end
            %calcolo nuova soluzione
            u_opt_bar = pinv(Ad * Rn) * (XT - Ad^(N+1) * X0 - Bd * up); 
            break; 
        end

        
    end

 end

 %aggiunta controllo finale up (al contrario perchè non ancora flippato) 
u_opt=[up;u_opt_bar]; 

% Controllo se quanto fatto sopra è andato correttamente (accetto una
% tollerenza del 0.01) 
tol=0.01;
 if(min(u_opt) <u_lb-tol || max(u_opt) > u_ub+tol)
    disp("Controli sopra i limiti!")
 else
     disp(['Controlli dentro limini. Tempo e''  pari a ' num2str(T) ] )

 end




% composizione degli ingressi in forma matriciale NxNm (=Nx2) e poi flippo 
U_opt= reshape(u_opt, 2, [])';  % righe = istanti, colonne = u1, u2
%U_opt=flipud(U_opt); 

% Vettore temporale aggiornato
t_total = 0:Ts:T;  % lunghezza T+Ts, cioè N+1 istanti


%% Simuluzioni e Plot 

% Plot comandi 
figure;
plot(t_total, U_opt);
legend('u_1','u_2');
xlabel('Tempo [s]');
ylabel('Ingresso');
title('Ingressi ottimi + mantenimento');

%imput per simulink; 
timing = Ts * (0:size(U_opt,1)-1)'; %si ottiene [0 , Ts, 2Ts, .... NTs] uno per ogni ingresso 
uSim = [timing, flipud(U_opt)];

y_lin = lsim(sys,flipud(U_opt),timing,X0, 'zoh' );
ySim = [timing, y_lin];

figure;
plot(t_total,y_lin)
legend("v","w");
xlabel('Tempo [s]');

