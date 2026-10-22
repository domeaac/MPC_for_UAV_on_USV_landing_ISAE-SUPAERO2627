function dx = quadDynamics(x, u, params)
% Non-linear quad dynamics using quaternion scalar last
%   x  = [x y z  vx vy vz  qx qy qz qw  p q r]'   13x1
%   u  = [dT tau_x tau_y tau_z]'                  4x1
%   params = [m g Ixx Iyy Izz]                    5x1

m = params(1); % mass
g = params(2); % gravity acc
I = [params(3); params(4); params(5)]; % drone moments of inertia 

v = x(4:6); 
qx = x(7); qy = x(8); qz = x(9); qw = x(10);
w = x(11:13);

T = m*g + u(1);

% Secondo le note di bernelli
%     [ 1−2(qy^2+qz^2)     2(qx qy + qw qz)    2(qx qz − qw qy) ]   
% A = [ 2(qx qy − qw qz)   1−2(qx^2+qz^2)      2(qy qz + qw qx) ] 
%     [ 2(qx qz + qw qy)   2(qy qz − qw qx)    1−2(qx^2+qy^2)   ] 

% la DCM A che porta un vettore da world a body è definita come
%     [ xw*xb yw*xb zw*xb ]   
% A = [ xw*yb yw*yb yw*yb ] 
%     [ xw*zb yw*zb zw*zb ] 
% quindi la terza riga è asse z body descritto in SDR world

zb = [2*(qx*qz + qw*qy);                % thrust axis in world frame
      2*(qy*qz - qw*qx);
      1 - 2*(qx^2 + qy^2)];

acc = T/m * zb - [0; 0; g];

% Sempre dalle note Bernelli
q_dot = 0.5 * [ qy*w(3) - qz*w(2) + qw*w(1);
                -qx*w(3) + w(1)*qz + w(2)*qw;
                qw*w(3) + qx*w(2) - qy*w(1);
               -qx*w(1) - qy*w(2) - qz*w(3)];

w_dot = (u(2:4) - cross(w, I.*w)) ./ I;  % Eulero

dx = [v; acc; q_dot; w_dot]; % Definizione
end
