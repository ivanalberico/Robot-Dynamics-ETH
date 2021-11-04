function params = parameters()
% TECHPOD UAV MODEL PARAMETERS

% Thomas Stastny
% 2018.12.12

% geometry
params.S = 0.47;            % wing surface area [m^2]
params.b = 2.59;            % wingspan [m]
params.c = 0.180;           % mean chord length [m]
params.epsilon = 0;         % thrust incidence angle [rad] --> NOTE: epsilon is defined from body x in positive pitch (up)
params.deltamax = 0.3491;	% max control surface deflection [rad]

% mass/inertia
params.mass = 2.65;         % total mass of airplane [kg]
params.Ixx = 0.1512*1.1;	% [kg m^2]
params.Iyy = 0.2785*1.4;	% [kg m^2]
params.Izz = 0.3745*1.4;	% [kg m^2]

% environment
params.rho = 1.225;         % air density
params.g = 9.81;            % acceleration of gravity

% aerodynamic/thrust coefficients - component build-up
params.cD0 = 0.136022235375284;
params.cDa = -0.673702786581529;
params.cDa2 = 5.45456589719510;
params.cL0 = 0.212657549371211;
params.cLa = 10.8060289182568;
params.cLa2 = -46.8323561880705;
params.cLa3 = 60.6017115061355;
params.cLq = 0;
params.cLde = 0;
params.cm0 = 0.0435007528360901;
params.cma = -2.96903143325122;
params.cmq = -106.154115386179;
params.cmde = 6.13078257823941;
params.cT0 = 0;
params.cT1 = 14.7217343655508;
params.cT2 = 0;
params.clb = -0.0154186740460301;
params.clp = -0.164692484392609;
params.clr = 0.0116850531725225;
params.clda = 0.0285;
params.cYb = -0.307330834028566;
params.cnb = 0.0429867867785536;
params.cnp = -0.0838525811770174;
params.cnr = -0.0826784989984419;
params.cndr = 0.0600000000000000;

end