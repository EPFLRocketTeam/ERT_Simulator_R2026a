%  Tests the corrective moment c1 = 0.5 * rho * sm * v^2 * cNa * (xp - cm)

classdef correctionMomentTest < matlab.unittest.TestCase
    % Unit tests for massProperties.m

    properties
        AbsTol = 1e-10;
        AddedPath
    end

    methods (Test)
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the
            % massProperties function to the MATLAB path so the tests can find it

            % 1. Get the path of the current test file directory
            % (e.g., ...\ERT_Simulator_R2023b\Test\Functions\Models)
            testDir = fileparts(mfilename('fullpath'));

            % 2. Move up two directories to reach the root folder
            % (e.g., ...\ERT_Simulator_R2023b)
            rootPath = fileparts(fileparts(fileparts(testDir)));

            % 3. Construct the path to the function file's directory
            % (e.g., ...\ERT_Simulator_R2023b\Src\Simulator_3D)
            functionPath = fullfile(rootPath, 'Src', 'Snippets');

            % 4. Add the paths to MATLAB's search path
            addpath(functionPath);

            % 5. Store the paths so we can remove it later in TestClassTeardown
            testCase.AddedPath = {functionPath};
        end



        %--------------------------------------------------------------------------
        % Test 1 – Nominal positive stability (xp > cm)
        %--------------------------------------------------------------------------
        function test_NominalPositiveStability(testCase)
            rocket   = buildRocket();
            env      = buildEnvironnement();
            velocity = 250;    % m/s
            cNa      = 10.0;   % Normal-force slope [1/rad]
            xp       = 2.0;    % Centre of pressure (from nose) [m]
            % massProperties will return cm = 1.5 for t=0 via the real function;
            % here we test the sign and rough magnitude using a mock cm value.
            % We therefore call the function and only assert qualitative properties
            % that must hold regardless of the exact cm returned by massProperties.

            c1 = correctionMoment(0, rocket, cNa, xp, velocity, env, 0);

            % c1 must be positive when xp > CoM (statically stable rocket)
            testCase.verifyGreaterThan(c1, 0, ...
                'c1 should be positive for a stable rocket (xp > cm)');
        end

        %--------------------------------------------------------------------------
        % Test 2 – Scaling with velocity squared
        %   Doubling v must quadruple c1 (all else equal)
        %--------------------------------------------------------------------------
        function test_VelocitySquaredScaling(testCase)
            rocket = buildRocket();
            env    = buildEnvironnement();
            cNa    = 8.0;
            xp     = 1.9;

            c1Low  = correctionMoment(0, rocket, cNa, xp, 100, env, 0);
            c1High = correctionMoment(0, rocket, cNa, xp, 200, env, 0);

            ratio = c1High / c1Low;
            testCase.verifyEqual(ratio, 4.0, 'AbsTol', 1e-6, ...
                'c1 must scale as v^2: doubling v should quadruple c1');
        end

        %--------------------------------------------------------------------------
        % Test 3 – Scaling with cNa
        %   Doubling cNa must double c1
        %--------------------------------------------------------------------------
        function test_LinearScalingWithCNa(testCase)
            rocket   = buildRocket();
            env      = buildEnvironnement();
            xp       = 1.9;
            velocity = 200;

            c1Base   = correctionMoment(0, rocket, 5.0,  xp, velocity, env, 0);
            c1Double = correctionMoment(0, rocket, 10.0, xp, velocity, env, 0);

            testCase.verifyEqual(c1Double / c1Base, 2.0, 'AbsTol', 1e-6, ...
                'c1 must scale linearly with cNa');
        end

        %--------------------------------------------------------------------------
        % Test 4 – Zero cNa gives zero restoring moment
        %--------------------------------------------------------------------------
        function test_ZeroCNaGivesZeroC1(testCase)
            rocket = buildRocket();
            env    = buildEnvironnement();

            c1 = correctionMoment(0, rocket, 0.0, 1.8, 250, env, 0);

            testCase.verifyEqual(c1, 0.0, 'AbsTol', 1e-10, ...
                'c1 must be zero when cNa = 0');
        end

        %--------------------------------------------------------------------------
        % Test 5 – Output is a real scalar
        %--------------------------------------------------------------------------
        function test_OutputIsRealScalar(testCase)
            rocket = buildRocket();
            env    = buildEnvironnement();

            c1 = correctionMoment(0, rocket, 7.0, 2.0, 300, env, 0);

            testCase.verifySize(c1, [1 1], 'c1 must be a scalar');
            testCase.verifyTrue(isreal(c1), 'c1 must be real-valued');
        end

        %--------------------------------------------------------------------------
        % Test 6 – Non-zero altitude reduces c1 (lower rho)
        %--------------------------------------------------------------------------
        function test_HigherAltitudeLowerC1(testCase)
            rocket   = buildRocket();
            env      = buildEnvironnement();
            cNa      = 9.0;
            xp       = 2.0;
            velocity = 250;

            c1Sea  = correctionMoment(0, rocket, cNa, xp, velocity, env, 0);
            c1High = correctionMoment(0, rocket, cNa, xp, velocity, env, 3000);

            testCase.verifyGreaterThan(c1Sea, c1High, ...
                'c1 should decrease at higher altitude due to lower air density');
        end
    end
end

%--------------------------------------------------------------------------
        % Fixtures
        %--------------------------------------------------------------------------
        function rocket = buildRocket()
            rocket.Sm        = 0.02;   % Reference area [m^2]
            rocket.length         = 3.0;    % Total length [m]
            rocket.burnTime = 5.0;    % Burn time [s]
        end

        function env = buildEnvironnement()
            env.P0 = 101325;
            env.T0 = 288.15;
            env.L  = 0.0065;
            env.R  = 287.05;
            env.g  = 9.81;
        end