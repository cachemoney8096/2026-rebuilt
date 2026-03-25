package frc.robot.subsystems.turret;

import frc.robot.Constants;

public class TurretCal {
    public static final double TURRET_SUPPLY_CURRENT_LIMIT_AMPS = 80;
    public static final double TURRET_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 80;

    public static final double TURRET_P = 0.15;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.00;
    public static final double TURRET_FF = 0.2;

    public static final double TURRET_MAX_VELOCITY_RPS = 6000.0;
    public static final double TURRET_MAX_ACCELERATION_RPS_SQUARED = 6000.0;

    public static final double TURRET_HOME_DEGREES = 0.0;
    public static final double TURRET_MAX_DEGREES = 350.0;
    public static final double TURRET_MIN_DEGREES = 0.0;

    public static final double TURRET_MOTOR_TO_TURRET_RATIO = (18.0/15.0)*(78.0/34.0)*5.0;  // TODO or 2.16 apparently??
    
    public static final double TURRET_POSITION_MARGIN = 2.0;
}
