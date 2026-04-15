function [mass,dmassdt] = massLin(t,Rocket)
%	Return the rocket mass during burn time
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - Mass      Rocket mass
%   - dMassdt   Rocket mass derivative over time

% OUTPUT:
if t > Rocket.burnTime
    mass = Rocket.emptyMass + Rocket.casingMass;
    dmassdt = 0;
else
    dmassdt = Rocket.propelMass/Rocket.burnTime;
    mass = Rocket.emptyMass+Rocket.motorMass-t*dmassdt;
end
end

