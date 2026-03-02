package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterCal {    
    /* Rollers */
    public static final double ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double ROLLERS_P = 1.0;
    public static final double ROLLERS_I = 0.0;
    public static final double ROLLERS_D = 0.0;
    public static final double ROLLERS_FF = 0.0;

    public static final double ROLLERS_MAX_RPS = 5000; // TODO tune this

    /* Hood */
    public static final double HOOD_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    public static final double HOOD_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    public static final double HOOD_P = 2.6; // TODO check this
    public static final double HOOD_HOLD_P = 5.0; // TODO check this
    public static final double HOOD_I = 0.0;
    public static final double HOOD_D = 0.0;
    public static final double HOOD_FF = 0.0;

    public static final double HOOD_MAX_VELOCITY_RPS = 6000.0;
    public static final double HOOD_MAX_ACCELERATION_RPS_SQUARED = 6000.0; // TODO again, maybe should tune these sometime, but they'll work for now cause pid is good

    public static final double HOOD_HOME_DEGREES = 45.0; // TODO calibrate these based on true angle
    public static final double HOOD_MAX_DEGREES = 70.0;
    public static final double HOOD_MIN_DEGREES = 45.0;

    public static final double HOOD_MOTOR_TO_HOOD_RATIO = 340.0/28.0; // TODO get this
    
    public static final double HOOD_POSITION_MARGIN = 2.0;
}
