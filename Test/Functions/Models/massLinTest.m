classdef massLinTest < matlab.unittest.TestCase
    % Unit tests for massProperties.m
    properties
        AbsTol = 1e-10;
    end

    methods (Test)

        %% -------------------------------------------------------------
        %   HYBRID = 0  (STANDARD MOTOR)
        %% -------------------------------------------------------------

        function test_nonHybrid_t0(testCase)
            R = rocketNonHybrid();
            t = 0;
            [M,dMdt] = massLin(t,R);

            exp_dMdt = R.propelMass / R.burnTime;
            exp_M = R.emptyMass;

            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
        end

        function test_nonHybrid_afterBurn(testCase)
            R = rocketNonHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massLin(t,R);

            exp_M = R.emptyMass + R.casingMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,0,'AbsTol',testCase.AbsTol);
        end

        %% -------------------------------------------------------------
        %   HYBRID = 1  (HYBRID MOTOR)
        %% -------------------------------------------------------------

        function test_hybrid_t0(testCase)
            R = rocketHybrid();
            t = 0;

            [M,dMdt] = massLin(t,R);

            exp_dMdt = R.propelMass / R.burnTime;
            exp_M    = R.emptyMass;

            testCase.verifyEqual(M,exp_M,'AbsTol',testCase.AbsTol);
            testCase.verifyEqual(dMdt,exp_dMdt,'AbsTol',testCase.AbsTol);
        end

        function testLinear_hybrid_afterBurn(testCase)
            R = rocketHybrid();
            t = R.burnTime + 1;

            [M,dMdt] = massProperties(t,R);

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
            [M,dMdt] = massLin(t,R);
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

