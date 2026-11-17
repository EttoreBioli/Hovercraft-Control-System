function out = hovercraft_kinematic(in)
    % Stati
    x = in(1);       % posizione lungo x
    y = in(2);       % posizione lungo y
    theta = in(3);   % angolo di orientazione

    % Ingressi (velocità lineare e angolare)
    v = in(4);       % velocità lineare
    omega = in(5);   % velocità angolare

    % Equazioni del modello cinematico
    dx = v * cos(theta);
    dy = v * sin(theta);
    dtheta = omega;

    % Output: derivate degli stati
    out = [dx; dy; dtheta];
end
