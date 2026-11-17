%% Linearizzazione attorno al punto di equilibrio 
syms x1 x2 x3 x4 x5 x6 u1 u2 real
%syms m mu_T r J mu_R real

% Carica i parametri
hovercraft_params;

%Definizione vettore di stato e controlli 
X = [x1; x2; x3; x4; x5; x6];
u = [u1; u2];
n=6; 

%Dinamica del sistema  
f = [
    x2;
    (u1 + u2)/m * cos(x5) - (mu_T/m) * x2;
    x4;
    (u1 + u2)/m * sin(x5) - (mu_T/m) * x4;
    x6;
    (r/J)*(u1 - u2) - (mu_R/J) * x6
];

A_ = jacobian(f, X);  % ∂f/∂X
B_ = jacobian(f, u);  % ∂f/∂u

theta_e = 0; %arbitrario
equilibrio= [
    0;          % x1 arbitrario
    0;          % x2 = 0
    0;          % x3 arbitrario
    0;          % x4 = 0
    theta_e;    % x5 = theta
    0           % x6 = 0
];

u_eq = [0; 0];

A = subs(A_, [X; u], [equilibrio; u_eq]);
B = subs(B_, [X; u], [equilibrio; u_eq]);
C=[1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; 
D=zeros(3,2); 
%C = eye(6);
%D = zeros(6,2);
A=double(A); 
B=double(B);


%Analisi del Sistema Linearizzato  
fprintf("Analisi Sistema Linearizzato \n"); 
sys = ss(A,B,C,D);

% Autovalori del sistema
autovalori = eig(A);

fprintf("   1) ");
if all(real(autovalori) < 0)
    stato = "Il sistema asintoticamente stabile.";
elseif any(real(autovalori) > 0)
    stato = "Il sistema instabile.";
else
    stato = "Il sistema marginalmente stabile.";
end
fprintf("%s Poli = %s\n", stato, mat2str(autovalori', 4));

%matrice di raggiungibilità 
R=ctrb(sys); 
if rank(R) < 6
    fprintf("   2) Sistema non completamente raggiungibile. Rank(R) = %d\n", rank(R));
else
    fprintf("   2) Sistema completamente raggiungibile. Rank(R) = %d\n", rank(R));
end

O=obsv(sys); 
if rank(O) < 6
    fprintf("   3) Sistema non completamente osservabile. Rank(O) = %d\n", rank(O));
else
    fprintf("   3) Sistema completamente osservabile. Rank(O) = %d\n", rank(O));
end

%matrice funzioni di trasfeirmento 
G = zpk(minreal(tf(sys)));

%% Regolatore con K di retroazione statico 
%si hanno due ingressi, si deve usare il lemma di Heymann 
% Non siamo in grado di controllare tutti i poli. Se ci proviamo, ci viene
% restituito errore. Però, possiamo fare il placing degli stati raggiungibili

sr= [1 2 5 6]; %stati raggiungibili 
Ar=A(sr,sr); 
Br=B(sr,:);

% Verifica controllabilità 
if rank(ctrb(Ar, Br)) < length(sr)
    error("Sottosistema non completamente raggiungibile. Pole placement non applicabile");
end

poli_desiderati = [-2, -0.7, -2.0, -1.2];
Kr = place(Ar, Br, poli_desiderati);

% Ricostruisci K nella dimensione originale (6 stati, 2 ingressi)
K = zeros(2, 6);
K(:, sr) = Kr;

% Matrice A chiusa in retroazione: A - B*K
A_cl = A - B * K;

% Sistema in retroazione statica
sys_cl = ss(A_cl, B, C, D);

% Verifica 
%disp('Poli del sistema in retroazione:');
%disp(pole(sys_cl));











