package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeCal {
    public static final double INTAKE_POSITION_HOME_DEGREES = 0.0; 
    public static final double INTAKE_POSITION_EXTENDED_DEGREES = 175.0; 
    public static final double INTAKE_POSITION_SHOOTING = 100.0; // TODO calibrate this

    public static final double SLAPDOWN_MAX_ACCELERATION_RPS_SQUARED = 6000.0;
    public static final double SLAPDOWN_MAX_VELOCITY_RPS = 6000.0; 
    public static final double ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double ROLLERS_P = 8.0; 
    public static final double ROLLERS_I = 0.0;
    public static final double ROLLERS_D = 0.0;
    public static final double ROLLERS_FF = 0.0;

    public static final double SLAPDOWN_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double SLAPDOWN_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double SLAPDOWN_P = 1.5;
    public static final double SLAPDOWN_I = 0.0;
    public static final double SLAPDOWN_FF = 0.0;
    public static final double SLAPDOWN_D = 0.0;

    public static final double SLAPDOWN_MOTOR_TO_SLAPDOWN_RATIO = (75.0/1.0)*(54.0/42.0)*(14.0/42.0);

    public static final double SLAPDOWN_POSITION_MARGIN = 5.0; 
    public static final double ROLLERS_RUNNING_SPEED = 0.65; 

    public static final double INTAKE_CANCODER_MAGNET_OFFSET = 0.0; 
}
