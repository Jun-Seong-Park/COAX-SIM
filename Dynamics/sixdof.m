% six degree of freedom (refactored: params struct p instead of globals)

function [X, X_1dot] = sixdof(X, Force, Moment, dt, p)

eul = [X(6), X(5), X(4)];       % eul angel
linvel = X(7:9);                 % linear velocity
angvel = X(10:12);               % angular velocity

% Navigation equation NEH
X_1dot(1:3,:) = eul2rotm(eul) * linvel;

% Kinematic equation
X_1dot(4:6,:) = pqr2deul(eul) * angvel;

% Force equation
X_1dot(7:9,:) = Force / p.Mass - cross(angvel, linvel);

% Moment equation
X_1dot(10:12,:) = p.Inertia \ (Moment - cross(angvel, p.Inertia * angvel));

X = X + dt * X_1dot;

end

function R = pqr2deul(angle)

phi = angle(3);  theta = angle(2);

R(1,:) = [1, sin(phi) * tan(theta), cos(phi) * tan(theta)];
R(2,:) = [0,              cos(phi),             -sin(phi)];
R(3,:) = [0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

end
