%%ANALYS:
% Scenariot är att tåget stannar så fort som möjligt och på så kort sträcka som möjligt från 100kmh.
%% Resultat:
% Anti-Glid inbromsning: Tid: 10.92s, Bromssträcka: 153.2759m, Medel Bromstemp: 337.5271541334959 K
% Perfekt inbromsning: Tid: 10.30s, Bromssträcka: 144.644m, Medel Bromstemp: 338.6922578057968 K  (maxbroms: 0.73)
% Tvärnit, glidning: Tid: 14.8s, Bromssträcka: 199.4272m, Medel Bromstemp: 290.1389782230344 K


% Medelvärde av bromssträckan då man ignorerar de punkter där upplösningen < 500: 154.0093334843901 meter. (Anti-Glid inbromsning)
% Noggranhetsanalys Sträcka över tid-grafen visar den beräknade optimala bromssträckan med anti-glid-system över olika upplösningar. (medel är 5 point trailing average)

%Simulation engine for ~~rockets~~trains and stuff
clear all
LOG_FREQUENCY = 1.0;
LOG_ENABLED = false;

%Simulation parameters
SIMULATION_TIME = 60.0;       %Simulation duration (s)
SIMULATION_END_ON_STOP = true;
%SIMULATION_RESOLUTION = 100;  %Steps per second (s^-1)
SIMULATION_GRAVITY = 9.80665; %Assume gravity constant (ms^-2)

%Environmental parameters
e_AtmosphericPressure = 101300.0; % 100kPa
e_AtmosphericTemperature = 10 + 274.15; % (K)
e_AtmosphericDensity = e_AtmosphericPressure / (8.31446261815324 * e_AtmosphericTemperature); % kgm^-3
e_WindVelocity = 0;
e_WindDirection = 0; % Global Wind Direction (RAD)

e_GroundNormal = [0, 0, 1]; % Normal vector of the ground plane

simulation_data_l = [];
for SIMULATION_RESOLUTION=10:1:1000
dTime = 1/SIMULATION_RESOLUTION;
%Train parameters
t_MassCargo = 0.0; % (kg) - MAX 29800
t_MassCart = 15000; % (kg)
t_Length = 15.140; % (m)
t_Width = 2.9; % (m)
t_Height = 2.765; % (m)
t_WheelRadius = 0.5; % (m)
t_NBrakes = 4; % Number of brakes

t_CFriction = 0.3044; % Coefficient of friction, Wheel
t_CFrictionS = 0.18685; % Coefficient of friction, Wheel, Slipping

t_StartHastighet = 100 / 3.6; % (km/h)

%Brake parameters
t_MassBrake = 4*50; % (kg)
t_BrakeForce = 2500.0; % (N)
b_CFriction = 0.42; % Inner brake coefficient of friction
t_BrakeSurfaceArea = 2 * (0.5*0.3 + 0.3*0.15 + 0.15*0.5); % (m^2)
ABS_ENABLED = true;

% Aerodynamics
t_CDrag = 0.98; % Drag coefficient
t_Area = t_Width * t_Height; % Frontal crossection area

t_Mass = t_MassCart + t_MassBrake + t_MassCargo;
t_MassCenter = 0.5 * ((t_MassCargo + t_MassCart) * t_Height) / t_Mass; % (m) Center of mass on the z-axis

%Simulation data
simulation_data_b = []; % Brake data
simulation_data_k = []; % Kinematic data

environment_data = [dTime, SIMULATION_GRAVITY, e_AtmosphericPressure, e_AtmosphericTemperature, e_AtmosphericTemperature, e_AtmosphericDensity, [0, 0, 0], e_WindVelocity, e_WindDirection, e_GroundNormal];
train_data = [t_Mass, t_MassCenter, t_BrakeForce, b_CFriction, t_CFriction, t_CFrictionS, t_WheelRadius, t_NBrakes, t_MassBrake, t_BrakeSurfaceArea];
brake_state = [0, e_AtmosphericTemperature, 0, 0, e_AtmosphericTemperature, 0, 0, e_AtmosphericTemperature, 0, 0, e_AtmosphericTemperature, 0]; %Vector of braking forces, temperature, slip - Top left, Top right, Bottom left, Bottom right
kinematic_state = [0, 0, 0, t_StartHastighet, 0, 0, 0, 0, 0]; % Pos: X, Y, Z  Velocity: X, Y, Z  Acceleration: X, Y, Z
brake_actuation = 0;
brake_limit = 1;
brake_slippage_prev = false;
for t=0:SIMULATION_TIME*SIMULATION_RESOLUTION
    
    position_vector = [kinematic_state(1), kinematic_state(2), kinematic_state(3)];
    velocity_vector = [kinematic_state(4), kinematic_state(5), kinematic_state(6)];
    acceleration_vector = [kinematic_state(7), kinematic_state(8), kinematic_state(9)];
    wind_vector = [environment_data(7), environment_data(8), environment_data(9)];
    air_velocity_vector = velocity_vector + wind_vector;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  Driving parameters
    BrakeRampSpeed = 2.0; % (s^-1)
    ActuationMaxSpeed = 1.0; % Max actuation speed (s^-1)

    %Shitty ABS
    if (ABS_ENABLED)
    brake_slippage = brake_state(3) || brake_state(6) || brake_state(9) || brake_state(12);
    if (brake_slippage)
        brake_target = 0.0;
        if not (brake_slippage_prev)
            brake_limit = brake_actuation - 0.05;
        end
    else
        brake_target = brake_limit;
    end
    else
        brake_target = 1;
    end
    brake_slippage_prev = brake_slippage;

    brake_actuation = brake_actuation + dTime * clamp((brake_target - brake_actuation) * BrakeRampSpeed, -ActuationMaxSpeed, ActuationMaxSpeed);
    
    %brake_actuation = t / (SIMULATION_TIME*SIMULATION_RESOLUTION); Linear actuation
    train_data(3) = t_BrakeForce * brake_actuation; % Brake actuator force

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % Update environment
    environment_data = UpdateEnvironment(position_vector(3), environment_data);

    %Brake simulations
    %moment from uneven braking is ignored, add to N? (not needed for straight travel)
    brake_state(1:3) = BrakeCalc(brake_state(1:3), train_data, environment_data, acceleration_vector, velocity_vector, [t_Length/2, -t_Width/2]);    %Front left brake
    brake_state(4:6) = BrakeCalc(brake_state(4:6), train_data, environment_data, acceleration_vector, velocity_vector, [t_Length/2, t_Width/2]);     %Front right brake
    brake_state(7:9) = BrakeCalc(brake_state(7:9), train_data, environment_data, acceleration_vector, velocity_vector, [-t_Length/2, -t_Width/2]);   %Back left brake
    brake_state(10:12) = BrakeCalc(brake_state(10:12), train_data, environment_data, acceleration_vector, velocity_vector, [-t_Length/2, t_Width/2]);%Back right brake

    brake_force = brake_state(1) + brake_state(4) + brake_state(7) + brake_state(10);
    brake_force = clamp(brake_force, 0, norm(velocity_vector) * t_Mass / dTime);

    %Log brake data
    simulation_data_b = cat(2, simulation_data_b, [t * dTime; brake_state(1); brake_state(2); brake_state(3); brake_state(4); brake_state(5); brake_state(6); brake_state(7); brake_state(8); brake_state(9); brake_state(10); brake_state(11); brake_state(12)]);



    % Kinematic calc
    %Forces
    force_vector = t_Mass * [0, 0, -SIMULATION_GRAVITY];
    force_vector = force_vector - force_vector .* [environment_data(12), environment_data(13), environment_data(14)]; % Pro tip- don't fall through the map
    
    drag_vector = -(0.5 * environment_data(6) * (air_velocity_vector / (norm(air_velocity_vector) + eps)) * norm(air_velocity_vector)^2 * t_CDrag * t_Area); % Drag
    brake_vector = -(velocity_vector / norm(velocity_vector) + eps) * brake_force; % Total braking force
    
    force_vector = force_vector + brake_vector + drag_vector;
    force_vector(isnan(force_vector))=0;


    % Apply forces
    acceleration_vector = force_vector / t_Mass;
    kinematic_state(7) = acceleration_vector(1);
    kinematic_state(8) = acceleration_vector(2);
    kinematic_state(9) = acceleration_vector(3);
    velocity_vector = velocity_vector + dTime * acceleration_vector;
    kinematic_state(4) = velocity_vector(1);
    kinematic_state(5) = velocity_vector(2);
    kinematic_state(6) = velocity_vector(3);
    position_vector = position_vector + dTime * velocity_vector;
    kinematic_state(1) = position_vector(1);
    kinematic_state(2) = position_vector(2);
    kinematic_state(3) = position_vector(3);



    %Log data
    simulation_data_k = cat(2, simulation_data_k, [t * dTime; kinematic_state(1); kinematic_state(2); kinematic_state(3); kinematic_state(4); kinematic_state(5); kinematic_state(6); kinematic_state(7); kinematic_state(8); kinematic_state(9)]);

    % Debug readout
    if (mod(t, SIMULATION_RESOLUTION * LOG_FREQUENCY) < 1) && LOG_ENABLED
        disp("--------------------------------------------")
        disp("TIME:  "+(t * dTime)+" s")
        disp("ENVIRONMENT:")
        disp("Pressure: "+(environment_data(3)/1000))
        disp("Temperature: "+(environment_data(4)-273.15))
        disp("Atm. Density: "+environment_data(6))
        disp(newline)
        disp("BRAKES:")
        disp("Braking Force: "+norm(brake_vector))
        disp(newline)
        disp("KINEMATIC:")
        disp(sprintf("Position: (%d, %d, %d)", position_vector)) %#ok<DSPSP>
        disp("Speed: "+norm(velocity_vector))
        disp(sprintf("Velocity: (%d, %d, %d)", velocity_vector)) %#ok<DSPSP>
        disp(sprintf("Acceleration: (%d, %d, %d)", acceleration_vector)) %#ok<DSPSP>
        disp("Drag Force: "+norm(drag_vector))
        disp("Total Force: "+norm(force_vector))
    end

    %Check if stopped
    if (norm(velocity_vector) == 0) && SIMULATION_END_ON_STOP
        simulation_duration = t / SIMULATION_RESOLUTION;
        disp(newline)
        disp("FINAL STATE")
        disp("Simulation Time: "+simulation_duration)
        disp("Final Speed: "+norm(velocity_vector))
        disp("Final Distance: "+norm(position_vector))
        break;
    end
end
simulation_data_l = cat(2, simulation_data_l, [SIMULATION_RESOLUTION; norm(position_vector)]);
end

%Plots
plotselector = 1;
clf
hold on
if (plotselector == 1) % Calculated distance over simulation resolution
    rolling_mean = movmean(simulation_data_l(2, :), [25 0]); % 5 point trailing average
    plot(simulation_data_l(1, :), simulation_data_l(2, :))
    plot(simulation_data_l(1, :), rolling_mean)
    title('Beräknad Sträcka');
    xlabel('Steg per Sekund [s^{-1}]');
    ylabel('Sträcka [m]');
    legend('Punktvis','Medelvärde');
end




%Functions
function [brake_state] = BrakeCalc(brake_state, train_data, environment_data, acceleration_vector, velocity_vector, centeroffset_xy) %Calculate braking force

    dTime = environment_data(1);
    Gravity = environment_data(2);
    atmospheric_temperature = environment_data(4);
    atmospheric_density = environment_data(6);
    air_velocity_vector = velocity_vector + [environment_data(7), environment_data(8), environment_data(9)];
    ground_normal = [environment_data(12), environment_data(13), environment_data(14)];

    mass = train_data(1);
    z_mass = train_data(2);
    brake_force = train_data(3);
    brake_cf = train_data(4);
    wheel_cf = train_data(5);
    wheel_cf_s = train_data(6);
    wheel_radius = train_data(7);
    brake_number = train_data(8);
    brake_mass = train_data(9);
    brake_location = [centeroffset_xy, 0]; %vector to the brake
    brake_surface_area = train_data(10);
    brake_chord_length = 0.3; % (m)

    specific_heat_capacity = 510.7896; % (J*kg^-1*K^-1)
    stefan_boltzmann = 5.67*10^-8;
    emissivity = 0.79;

    prandtl_number = 0.71;
    thermal_conductivity = 0.02572; % (W/(m^2K))
    kinematic_viscosity = 1.460 * 10^-5; % (m^2/s)

    b_Temp = brake_state(2);
    WheelSlip = brake_state(3);

    %Calculate braking moment
    M_b = 0.3 * wheel_radius * ((35 * brake_force) / (3 + 0.5 * brake_cf) + (8 * brake_force) / (1 - brake_cf / 6));
    
    %Calculate normal force
    brake_location_length2 = dot(brake_location, brake_location);
    N = mass * (Gravity * ground_normal(3) + z_mass * (Gravity * dot(ground_normal, brake_location) / brake_location_length2 - dot(acceleration_vector, brake_location) / brake_location_length2)) / brake_number;

    if (WheelSlip)
        mu_w = wheel_cf_s;
    else
        mu_w = wheel_cf;
    end
    M_w = wheel_radius * N * mu_w;

    %Slipping
    if (M_b > M_w)
        WheelSlip = true;

        b_Force = mu_w * N;
    else
        WheelSlip = false;

        M_w = clamp(M_w, 0, M_b);
        b_Force = M_w / wheel_radius;
    end


    %Thermal calculations
    if not (WheelSlip)
        Q = b_Force * dTime * norm(velocity_vector);
        dTemperature = brake_number * Q / (brake_mass * specific_heat_capacity);
        b_Temp = b_Temp + dTemperature;
    end

    %Thermodynamics brake-environment
    %Radiated
    temp_difference = b_Temp - atmospheric_temperature;
    Q = dTime * stefan_boltzmann * emissivity * brake_surface_area * sign(temp_difference) * temp_difference^4;

    %Convection
    reynolds_number = (norm(air_velocity_vector) * brake_chord_length) / kinematic_viscosity;
    if (reynolds_number > 5*10^5)
        nusselt_number = 0.0308 * reynolds_number^0.8 * prandtl_number^(1/3); % Turbulent flow
    else
        nusselt_number = 0.453 * reynolds_number^0.5 * prandtl_number^(1/3); % Laminar flow
    end
    convective_heat_transfer = (nusselt_number * thermal_conductivity) / brake_chord_length;
    Q = Q + dTime * convective_heat_transfer * brake_surface_area * temp_difference;

    b_Temp = b_Temp - brake_number * Q / (brake_mass * specific_heat_capacity);
    
    
    brake_state = [max(b_Force, 0), b_Temp, WheelSlip];
end
function [environment_data] = UpdateEnvironment(altitude, environment_data) % Calculate as functions of altitude
    % Source: https://www.grc.nasa.gov/WWW/K-12/airplane/atmosmet.html 
    air_mm = 0.0289647; % Molar mass of air (kg)
    atmospheric_temperature = 288.19 - 0.00649 * altitude;
    atmospheric_pressure = 101290 * (atmospheric_temperature / 288.08)^5.256;
    atmospheric_density = air_mm * atmospheric_pressure / (8.31446261815324 * atmospheric_temperature);

    wind_vector = environment_data(10) * [cos(environment_data(11)), sin(environment_data(11)), 0];
    
    % Write values
    environment_data(3) = atmospheric_pressure; % Pressure
    environment_data(5) = environment_data(4); % Old Temperature
    environment_data(4) = atmospheric_temperature; % Temperature
    environment_data(6) = atmospheric_density; % Density
    environment_data(7) = wind_vector(1); % Wind x
    environment_data(8) = wind_vector(2); % Wind y
    environment_data(9) = wind_vector(3); % Wind z
end


function y = clamp(x,bl,bu)
    y=min(max(x,bl),bu);
end
