package frc.robot.subsystems.turret;

import frc.robot.Constants;

public class TurretCal {
    public static final double TURRET_SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final double TURRET_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40;

    public static final double TURRET_P = 6.0;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0;
    public static final double TURRET_FF = 1.0;

    public static final double TURRET_MAX_VELOCITY_RPS = 6000.0;
    public static final double TURRET_MAX_ACCELERATION_RPS_SQUARED = 6000.0;

    public static final double TURRET_HOME_DEGREES = 90.0;
    public static final double TURRET_MAX_DEGREES = 180.0;
    public static final double TURRET_MIN_DEGREES = 0.0;

    public static final double TURRET_MOTOR_TO_TURRET_RATIO = (18.0/12.0)*(78.0/34.0);  // TODO get this
    
    public static final double TURRET_POSITION_MARGIN = 1.0;
}
