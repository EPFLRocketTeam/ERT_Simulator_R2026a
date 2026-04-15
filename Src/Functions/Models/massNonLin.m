function [mass,dmassdt] = massNonLin(t,Rocket)
%	Return the tocket mass during burn time
%   INPUT:
%   - t         Time
%   - Rocket    Structure Containing all datas
%   OUTPUT:
%   - Mass      Rocket mass
%   - dMassdt   Rocket mass derivative over time

% OUTPUT:
if t>Rocket.burnTime
    mass = Rocket.emptyMass+Rocket.motorMass-Rocket.propelMass;
    dmassdt = 0;
else
    tt = linspace(0,t,500);
    currentImpulse = trapz(tt,Thrust(tt,Rocket));
    mass = Rocket.emptyMass + Rocket.motorMass - Rocket.thrust2dMassRatio*currentImpulse;
    dmassdt = Rocket.thrust2dMassRatio*Thrust(t,Rocket);
end
end

