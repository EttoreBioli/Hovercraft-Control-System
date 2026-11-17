function out = hovercraft_vel_dynamic(in)  

    hovercraft_params;

    x1=in(1);   %velocità lineare v
    x2=in(2);   %velocità angolare w
   
    % ingressi di controllo 
    u1 = in(3);   
    u2 = in(4);  

    % Equazioni della dinamica
    dx1 = 1/m* ( (u1 + u2)- mu_T*x1 );
    dx2 = 1/J * (r* (u1-u2) - mu_R*x2 ); 

    % Output 
    out = [dx1; dx2];

end


