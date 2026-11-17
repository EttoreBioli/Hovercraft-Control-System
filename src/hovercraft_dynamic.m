function out = hovercraft_dynamic(in)  

hovercraft_params;

    x1=in(1);   %posizione lungo x
    x2=in(2);   %velocità lungo x
    x3=in(3);   %posizione lungo y
    x4=in(4);   %velocità lungo y
    x5=in(5);   % angolo di orientazione theta
    x6=in(6);   %velocità angolare

% ingressi di controllo
    u1 = in(7);  % spinta del motore DESTRO
    u2 = in(8);  % spinta del motore SINISTRO

 % Equazioni della dinamica
    dx1 = x2;
    dx2 = (u1 + u2)/m * cos(x5) - (mu_T/m) * x2;
    dx3 = x4;
    dx4 = (u1 + u2)/m * sin(x5) - (mu_T/m) * x4;
    dx5 = x6;
    dx6 = r/J * (u1 - u2) - (mu_R/J) * x6;

    % Output vettore delle derivate
    out = [dx1; dx2; dx3; dx4; dx5; dx6];

end


