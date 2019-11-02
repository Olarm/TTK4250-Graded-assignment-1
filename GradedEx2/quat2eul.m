function eul = quat2eul(q)

qSquared = q.^2;
qS = qSquared;
n = size(q, 2);

% φ=atan2(2(ε3ε2+ηε1),η^2−ε1^2−ε2^2+ε3^2)
% θ=asin(2(ηε2−ε1ε3))
% ψ=atan2(2(ε1ε2+ηε3),η^2+ε1^2−ε2^2−ε3^2)

phi = zeros(1, n);
theta = zeros(1, n);
psi = zeros(1, n);

for i=1:n
    phi(i) = atan2(2 * (q(4,i) * q(3,i) + q(1, i) * q(2, i)), qS(1,i) - qS(2,i) - qS(3,i) + qS(4,i));
    theta(i) = asin(2 * (q(1,i) * q(3,i) - q(2,i) * q(4,i)));
    psi(i) = atan2(2 * (q(2,i) * q(3,i) + q(1,i) * q(4,i)), qS(1,i) + qS(2,i) - qS(3,i) - qS(4,i));
end

eul = [phi; theta; psi];
end