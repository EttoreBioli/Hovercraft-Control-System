close all; 
if ~exist('out', 'var')
    error('La variabile "out" non è presente nel workspace.');
end

% Estrazione dati dalla struct with time
t = out.simout.time;
x = out.simout.signals.values(:,1);
y = out.simout.signals.values(:,2);
theta = out.simout.signals.values(:,3);

% Parametri hovercraft
L = 1.5; W = 0.5;

% Setup figura
figure('Name','Animazione Hovercraft','NumberTitle','off');
axis equal
axis([-1 6 -1 6])
grid on
hold on

% Preallocazione patch per hovercraft
hPatch = patch(nan, nan, 'b');
hTrace = plot(nan, nan, 'r--'); % linea tratteggiata per traccia


% Shape hovercraft (rettangolo centrato)
shape = [-L/2 L/2 L/2 -L/2;
         -W/2 -W/2 W/2 W/2];
hCenter = plot(nan, nan, 'ko', 'MarkerFaceColor', 'k'); % puntino al centro

% Ciclo animazione
for k = 1:length(t)
    
    % Calcolo posizione e orientamento attuali
    R = [cos(theta(k)) -sin(theta(k));
         sin(theta(k))  cos(theta(k))];
    rotatedShape = R * shape;
    translatedShape = rotatedShape + [x(k); y(k)];
    
    % Aggiorna posizione hovercraft
    set(hPatch, 'XData', translatedShape(1,:), 'YData', translatedShape(2,:));
    
    % Aggiorna traccia con tutti i punti fino a k
    set(hTrace, 'XData', x(1:k), 'YData', y(1:k));

     % Aggiorna punto centrale
    set(hCenter, 'XData', x(k), 'YData', y(k));
    
    %pause(0.02);
    drawnow limitrate
end
