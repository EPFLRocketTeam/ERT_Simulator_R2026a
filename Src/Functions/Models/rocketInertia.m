function [M, dMdt, CM, I, dIdt] = RocketInertia(t, Rocket, massmodel)
% ROCKETINERTIA Compute mass, mass derivative, center of mass position from
% cone tip, Inertia moment tensor and derivative of the inertia moment
% tensor. 
% INPUTS : 
% - t           :   time [s]
% - Rocket      :   Rocket structure
% - massmodel   :   mass model selection [0 (linear), 1 (non-linear)]
% OUTPUTS
% - M           :   Mass [kg]
% - dMdt        :   Time derivative of mass [kg/s]
% - CM          :   Center of mass from cone tip [m]
% - I           :   Moment of inertia tensor in rocket coordinate system (3x3)[kgm^2]
% - dIdt        :   Time derivative of moment of inertia tensor in rocket coordinate system (3x3)[kgm^2/s]

% Compute mass values
if(massmodel)
    [M, dMdt] = Mass_Lin(t, Rocket);
else
    [M, dMdt] = Mass_Non_Lin(t, Rocket);
end

% Compute center of mass
CM = (Rocket.emptyCenterOfMass*Rocket.emptyMass + ... 
    (M-Rocket.emptyMass)*(Rocket.length-Rocket.motorLength/2))/M;

% Compute Inertia tensor
% longitudinal inertia
R_i = 0.005; % Diametre interieur grains (Tjr identique)
R_e = Rocket.motor_dia/2; % Diametre exterieur grains

I_L_Casing = Rocket.casingMass*(Rocket.motorLength^2/12 + R_e^2/2); 

Grain_Mass = M-Rocket.emptyMass-Rocket.casingMass; % Masse des grains
I_L_Grain = Grain_Mass*(Rocket.motorLength^2/12 + (R_e^2+R_i^2)/4);

I_L = Rocket.emptyInertia + I_L_Casing + I_L_Grain + ...
    (Grain_Mass+Rocket.casingMass)*...
    (Rocket.length-CM-Rocket.motorLength/2); % I + ... + Steiner
% rotational inertia

I = [I_L, 1];
% TODO : rotational inertia and inertia moment derivative.
dIdt = 0;
end