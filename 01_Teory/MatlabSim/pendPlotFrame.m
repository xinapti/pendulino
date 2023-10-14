function pendPlotFrame(figObj, X, fm_x, par)
%PENDPLOT Summary of this function goes here
% X    = [X_mass, Y_mass]
% fm_x = Mag_Intensity_force [Vect]
% par  = [L, Km]

% Draw param
arrowScale = 0.1;
L_mass = 0.4;   % Diametro Pendente [m]
H_mass = 0.6;   % Altezza Pendente [m]


% Variable load and calc
g=9.81; % Gravità [m/s^2]
[X_mass, Y_mass] = deal(X(1), X(2));
[L, Km] = deal(par(1), par(2));
Max_Extension = L + H_mass + 0.1;
angle = atan2d(Y_mass,X_mass) + 90;

% Vect position and size
fg = [X_mass, Y_mass, 0,-g]; % Gravity force
fm = [X_mass, Y_mass, fm_x, 0] ; % Magnetic force

vectMatrix=[fg;fm];


figure(figObj)
clf

plot([-Max_Extension,Max_Extension],[0,0],'b-','LineWidth',10) % Soffitto
hold on
plot([0,0],[0,-(Max_Extension+1)],'m--','LineWidth', 2) % Asse Y

plot(0,0,'r-o','markersize',10,'markerfacecolor','r') % Perno

plot([0,X_mass],[0,Y_mass],'k-','LineWidth',3) % Corda

% Pendente
draw_rectangle(X, H_mass, L_mass, angle, 'blue')


% force print
quiver(vectMatrix(:,1),vectMatrix(:,2),vectMatrix(:,3),vectMatrix(:,4), ...
    arrowScale, "Color","red", "LineWidth", 1)

end


function[]= draw_rectangle(center_location,H,L,deg,rgb)

theta=deg2rad(deg);
center1=center_location(1);
center2=center_location(2);

R= ([cos(theta), -sin(theta); sin(theta), cos(theta)]);

X=([-L/2, L/2, L/2, -L/2]);
Y=([-H/2, -H/2, H/2, H/2]);

for i=1:4
T(:,i)=R*[X(i); Y(i)];
end

x_lower_left=center1+T(1,1);
x_lower_right=center1+T(1,2);
x_upper_right=center1+T(1,3);
x_upper_left=center1+T(1,4);

y_lower_left=center2+T(2,1);
y_lower_right=center2+T(2,2);
y_upper_right=center2+T(2,3);
y_upper_left=center2+T(2,4);

x_coor=[x_lower_left x_lower_right x_upper_right x_upper_left];
y_coor=[y_lower_left y_lower_right y_upper_right y_upper_left];

patch('Vertices',[x_coor; y_coor]','Faces',[1 2 3 4],'Edgecolor',rgb,'Facecolor','none','Linewidth',1.2);

% Rectangle Center
plot(center1,center2,rgb + "-o",'markersize',10,'markerfacecolor',rgb)

end