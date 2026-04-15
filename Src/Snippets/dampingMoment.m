function C2 = dampingMoment(t,Rocket,Calpha,CP,Velocity,Environnement,Altitude)
% Give damping moment based on rocket geometry & motor caracteristics
%

%--------------------------------------------------------------------------
% 1 Intrasic parameter
%--------------------------------------------------------------------------

[T, a, p, density, Nu] = stdAtmos(Altitude,Environnement);
[M,dMdt,Cm,dCmdt,I_L,dI_Ldt,I_R,dI_Rdt] = massProperties(t,Rocket,'NonLinear');

%--------------------------------------------------------------------------
% 2 Subparameter
%--------------------------------------------------------------------------
%2.1 Thrust damping coefficient 
<<<<<<< HEAD:Src/Snippets/dampingMoment.m
CR2 = dMdt*(Rocket.length-Cm).^2;
=======
CR2 = dMdt*(Rocket.length-Cm).^2;
>>>>>>> 8b6ece1c1c992ed5647f16054d68fd8a78b3021c:Src/Snippets/DampingMoment.m

%2.2 Aerodynamic damping coefficient
CNa_Total = sum(Calpha.*(CP-Cm).^2);
CA2 = density*Velocity*Rocket.maxCrossSectionArea/2*CNa_Total;

%--------------------------------------------------------------------------
% 3 Total Damping Coefficient
%--------------------------------------------------------------------------
C2 = CR2+CA2;
end

