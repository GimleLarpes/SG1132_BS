%Simulation engine for rockets and stuff
clear all
LOG_FREQUENCY = 0.5;

%Simulation parameters
SIMULATION_TIME = 10.0;       %Simulation duration (s)
SIMULATION_RESOLUTION = 100; %Steps per second (s^-1)
SIMULATION_GRAVITY = 9.80665; %Assume gravity constant (ms^-2)

dTime = 1/SIMULATION_RESOLUTION;

%Environmental parameters
e_AtmosphericPressure = 101300.0; % 100kPa
e_AtmosphericTemperature = 20 + 274.15; % (K)
e_AtmosphericDensity = e_AtmosphericPressure / (8.31446261815324 * e_AtmosphericTemperature); % kgm^-3
e_WindVelocity = 0;
e_WindDirection = 0; % Global Wind Direction (RAD)

e_GroundNormal = [0, 0, 1]; % Normal vector of the ground plane

%Train parameters
t_MassCargo = 1000.0; % (kg)
t_MassCart = 500; % (kg)
t_Length = 5.0; % (m)
t_Width = 2.0; % (m)
t_Height = 2.0; % (m)
t_WheelRadius = 0.5; % (m)
t_NBrakes = 4; % Number of brakes

t_CFriction = 0.18; % Coefficient of friction, Wheel
t_CFrictionG = 0.03; % Coefficient of friction, Wheel, Gliding

t_StartHastighet = 10 / 3.6; % (km/h)

%Brake parameters
t_MassBrake = 4*50; % (kg)
t_BrakeForce = 100.0; % (N)
b_CFriction = 0.3; % Inner brake coefficient of friction

% Aerodynamics
t_CDrag = 0.31; % Drag coefficient
t_Area = t_Width * t_Height; % Frontal crossection area

t_Mass = t_MassCart + t_MassBrake + t_MassCargo;
t_MassCenter = 0.5 * ((t_MassCargo + t_MassCart) * t_Height) / t_Mass; % (m) Center of mass on the z-axis

%Simulation data
simulation_data_b = []; % Brake data
simulation_data_k = []; % Kinematic data

environment_data = [dTime, SIMULATION_GRAVITY, e_AtmosphericPressure, e_AtmosphericTemperature, e_AtmosphericTemperature, e_AtmosphericDensity, [0, 0, 0], e_WindVelocity, e_WindDirection, e_GroundNormal];
train_data = [t_Mass, t_MassCenter, t_BrakeForce, b_CFriction, t_CFriction, t_CFrictionG, t_WheelRadius, t_NBrakes];
brake_state = [0, 0, 0, 0, 0, 0, 0, 0]; %Vector of braking forces, temperature - Top left, Top right, Bottom left, Bottom right
kinematic_state = [0, 0, 0, t_StartHastighet, 0, 0, 0, 0, 0]; % Pos: X, Y, Z  Velocity: X, Y, Z  Acceleration: X, Y, Z
for t=0:SIMULATION_TIME*SIMULATION_RESOLUTION
    
    position_vector = [kinematic_state(1), kinematic_state(2), kinematic_state(3)];
    velocity_vector = [kinematic_state(4), kinematic_state(5), kinematic_state(6)];
    acceleration_vector = [kinematic_state(7), kinematic_state(8), kinematic_state(9)];
    wind_vector = [environment_data(7), environment_data(8), environment_data(9)];
    air_velocity_vector = velocity_vector + wind_vector;


    % Update environment
    environment_data = UpdateEnvironment(position_vector(3), environment_data);

    %Brake simulations
    %moment from uneven braking is ignored, add to N? (not needed for straight travel)
    brake_state(1:2) = BrakeCalc(brake_state(1:2), train_data, environment_data, acceleration_vector, velocity_vector, [t_Length/2, -t_Width/2]);%Front left brake

    %Log brake data
    simulation_data_b = cat(2, simulation_data_b, [t * dTime; brake_state(1); brake_state(2); brake_state(3); brake_state(4); brake_state(5); brake_state(6); brake_state(7); brake_state(8)]);



    % Kinematic calc
    %Forces
    force_vector = t_Mass * [0, 0, -SIMULATION_GRAVITY];
    force_vector = force_vector - force_vector .* [environment_data(12), environment_data(13), environment_data(14)]; % Pro tip- don't fall through the map
    
    drag_vector = -(0.5 * environment_data(6) * (air_velocity_vector / (norm(air_velocity_vector) + eps)) * norm(air_velocity_vector)^2 * t_CDrag * t_Area); % Drag
    brake_vector = [0,0,0];%TOTAL BRAKING FORCE, opposite direction to velocity, moment from uneven braking ignored.
    
    force_vector = force_vector + brake_vector + drag_vector;


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
    if (mod(t, SIMULATION_RESOLUTION * LOG_FREQUENCY) < 1) && true
        disp("--------------------------------------------")
        disp("TIME:  "+(t * dTime)+" s")
        disp("ENVIRONMENT:")
        disp("Pressure: "+(environment_data(3)/1000))
        disp("Temperature: "+(environment_data(4)-273))
        disp("Atm. Density: "+environment_data(6))
        disp(newline)
        %%BRAKE DATA HERE
        disp("KINEMATIC:")
        disp(sprintf("Position: (%d, %d, %d)", position_vector)) %#ok<DSPSP>
        disp("Speed: "+norm(velocity_vector))
        disp(sprintf("Velocity: (%d, %d, %d)", velocity_vector)) %#ok<DSPSP>
        disp(sprintf("Acceleration: (%d, %d, %d)", acceleration_vector)) %#ok<DSPSP>
        disp("Drag Force: "+norm(drag_vector))
        disp("Total Force: "+norm(force_vector))
    end

    %Check if stopped
    if (norm(velocity_vector) == 0)
        simulation_duration = t / SIMULATION_RESOLUTION;
        disp("Simulation Time: "+simulation_duration)
        break;
    end
end

%Readout (not working)
plotselector = 1;
hold on
%PLOTS HERE



%Functions
function [brake_state] = BrakeCalc(brake_state, train_data, environment_data, acceleration_vector, velocity_vector, centeroffset_xy) %Calculate braking force

    dTime = environment_data(1);
    atmospheric_pressure = environment_data(3);
    atmospheric_temperature = environment_data(4);
    prev_atmospheric_temperature = environment_data(5);

    number_of_brakes = train_data(8);
    brake_location = [centeroffset_xy, 0]; %vector to the brake
    

    b_Force = 0;%BRAKING FORCE
    b_Temp = 0;%BRAKE TEMP(of the internal brake)?

    brake_state = [b_Force, b_Temp];
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


