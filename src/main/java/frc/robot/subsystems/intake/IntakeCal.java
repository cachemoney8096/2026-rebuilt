package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeCal {
    public static final double INTAKE_POSITION_HOME_DEGREES = Constants.PLACEHOLDER_DOUBLE; // TODO calibrate this
    public static final double INTAKE_POSITION_EXTENDED_DEGREES = Constants.PLACEHOLDER_DOUBLE; // TODO calibrate this

    public static final double SLAPDOWN_MAX_ACCELERATION_RPS_SQUARED = 1000;
    public static final double SLAPDOWN_MAX_VELOCITY_RPS = 1000; // TODO consider tuning these, though pid is pretty effective and I am just giving a high ceiling rn

    public static final double ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double ROLLERS_P = 0.5; // TODO tune this
    public static final double ROLLERS_I = 0.0;
    public static final double ROLLERS_D = 0.0;
    public static final double ROLLERS_FF = 0.0;

    public static final double SLAPDOWN_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double SLAPDOWN_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double SLAPDOWN_P = 0.5;
    public static final double SLAPDOWN_I = 0.0;
    public static final double SLAPDOWN_FF = 0.0;
    public static final double SLAPDOWN_D = 0.0;

    public static final double SLAPDOWN_MOTOR_TO_SLAPDOWN_RATIO = Constants.PLACEHOLDER_DOUBLE; // TODO get this

    public static final double SLAPDOWN_POSITION_MARGIN = 5.0; // im just gonna assume this is degrees since it's late
    public static final double ROLLERS_RUNNING_SPEED = 0.7; // TODO tune this

    public static final double INTAKE_CANCODER_MAGNET_OFFSET = 0.0; // TODO calibrate this
}
