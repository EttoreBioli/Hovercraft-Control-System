%% Linearizzazione attorno al punto di equilibrio 
syms x1 x2 u1 u2 real
%syms m mu_T r J mu_R real

% Carica i parametri
hovercraft_params;

%Definizione vettore di stato e controlli 
X = [x1; x2];
u = [u1; u2];
n=2; 

%Dinamica del sistema  
f = [
    1/m*(u1 + u2)- 1/m*mu_T*x1;
    r/J*(u1-u2) - 1/J*mu_R*x2; 
];

A_ = jacobian(f, X);  % ∂f/∂X
B_ = jacobian(f, u);  % ∂f/∂u

equilibrio= [
    0;          
    0;        
];

u_eq = [0; 0];

A = subs(A_, [X; u], [equilibrio; u_eq]);
B = subs(B_, [X; u], [equilibrio; u_eq]);
C=[1 0; 0 1 ]; 
D=zeros(2,2); 
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
if rank(R) < n
    fprintf("   2) Sistema non completamente raggiungibile. Rank(R) = %d\n", rank(R));
else
    fprintf("   2) Sistema completamente raggiungibile. Rank(R) = %d\n", rank(R));
end

O=obsv(sys); 
if rank(O) < n
    fprintf("   3) Sistema non completamente osservabile. Rank(O) = %d\n", rank(O));
else
    fprintf("   3) Sistema completamente osservabile. Rank(O) = %d\n", rank(O));
end

%matrice funzioni di trasfeirmento 
G = zpk(minreal(tf(sys)));