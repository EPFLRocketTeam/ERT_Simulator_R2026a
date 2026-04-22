classdef massNonLinTest < matlab.unittest.TestCase
    % Unit tests for massNonLin.m

    properties
        AbsTol = 1e-10;
        AddedPath
    end

    methods (Test)

        %% -------------------------------------------------------------
        %   HYBRID = 0  (STANDARD MOTOR)
        %% -------------------------------------------------------------

        function testNonLinear_nonHybrid_midBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime/2;

            [M,dMdt] = massNonLin(t,R);

            tt = linspace(0,t,500);
            currentImpulse = trapz(tt,Thrust(tt,R));
            exp_M = R.emptyMass + R.motorMass - R.thrust2dMassRatio*currentImpulse;
            exp_dMdt = R.thrust2dMassRatio*Thrust(t,R);

            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
        end


        function testNonLinear_nonHybrid_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massNonLin(t,R);

            exp_M = R.emptyMass + R.motorMass - R.propelMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end


        %% -------------------------------------------------------------
        %   HYBRID = 1  (HYBRID MOTOR)
        %% -------------------------------------------------------------

        function testNonLinear_hybrid_afterBurn(testCase)
            R = rocketHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massNonLin(t,R,'NonLinear');

            exp_M = R.emptyMass + R.casingMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        %% -------------------------------------------------------------
        %   GENERAL SANITY CHECKS
        %% -------------------------------------------------------------


        function testMass(testCase)
            R = rocketNonHybrid();
            t = linspace(0, R.burnTime, 1000);
            [M,dMdt,~,~,~,~,~,~] = massNonLin(t,R,'NonLinear');
            testCase.verifyGreaterThanOrEqual(M,0);
            testCase.verifyLessThanOrEqual(dMdt,0);
            testCase.verifyFinite(M)
        end

    end
end

%% ========================================================================
%   DUMMY ROCKET GENERATORS
%% ========================================================================

function R = rocketNonHybrid()
    R.isHybrid = 0;
    R.propelMass = 10;
    R.burnTime = 5;
    R.emptyMass = 50;
    R.motorMass = 20;
    R.casingMass = 5;
    R.emptyCenterOfMass = 1.0;
    R.motor_length = 0.4;
    R.motor_dia = 0.12;
    R.L = 2.5;
    R.emptyInertia = 0.3;
    R.thrust2dMassRatio = 0.02;
end

function R = rocketHybrid()
    R.isHybrid = 1;
    R.propelMass = 8;
    R.burnTime = 6;
    R.emptyMass = 55;
    R.casingMass = 4;
    R.motorMass = 18;
    R.emptyCenterOfMass = 1.0;
    R.motor_length = 0.45;
    R.motorLengthPropel = 0.25;
    R.motorLengthFuel = 0.15;
    R.motor_massP = 10;
    R.motor_massF = 8;
    R.massPropel = 4;
    R.massFuel = 4;
    R.distanceInterMotors = 0.02;
    R.motor_dia = 0.11;
    R.L = 2.6;
    R.emptyInertia = 0.25;
    R.thrust2dMassRatio = 0.02;
end

%% ========================================================================
%   DUMMY THRUST FUNCTION  (used for NonLinear tests)
%% ========================================================================
function y = Thrust(t, Rocket)
    % Deterministic thrust curve:  simply T = 1000*N*sin(t)
    if t<Rocket.burnTime
        y = 1000*sin(pi*t/Rocket.burnTime);
    else
        y=0;
    end
end