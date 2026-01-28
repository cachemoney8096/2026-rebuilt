package frc.robot.subsystems.turret;

public class TurretCal {
    public static final double TURRET_SUPPLY_CURRENT_LIMIT_AMPS = 40;
    public static final double TURRET_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40;

    public static final double TURRET_P = 8;
    public static final double TURRET_I = 0;
    public static final double TURRET_D = 0;
    public static final double TURRET_FF = 0;

    public static final double TURRET_MAX_VELOCITY_RPS = 6000;
    public static final double TURRET_MAX_ACCELERATION_RPS_SQUARED = 6000;

    public static final double TURRET_HOME_DEGREES = 0;
    public static final double TURRET_MAX_DEGREES = 180;
    public static final double TURRET_MIN_DEGREES = 0;

    public static final double TURRET_MOTOR_TO_TURRET_RATIO = 1; 
    
    public static final double TURRET_POSITION_MARGIN = 0.1;
}
