function [y]=pend_fn(t,state, par)
% state = [theta, omega]
% par   = [L, Kattr, Km]
% 
% attr:= Coefficente di attrito viscoso, simula freno, >=0

[theta, omega] = deal(state(1), state(2));
[L, Kattr, Km] = deal (par(1), par(2), par(3));
g=9.81; % Gravità [m/s^2]
u = 1; % Controll always max

% Matrice dinamica interna non lineare nei parametri
F_state = [              omega;
           -g/L*sin(theta) - Kattr * omega];

% Matrice controllo non lineare nei coefficienti
Fm_x = -sign(theta)* Km * cos(theta)/(1 + abs(L*sin(theta)))^2;

B_state = [     0;
              Fm_x];

xDot= F_state + B_state * abs(u);

y = xDot;
% angAcc = (-attr*omega)-(g/L)*sin(theta);
% 
% y = [omega;angAcc,Fm_x];

end