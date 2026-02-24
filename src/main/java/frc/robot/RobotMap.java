package frc.robot;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
  // 0 and 1 reserved for rio/pdh, block 2-16 reserved for drive
  public static final int LEFT_CLIMB_MOTOR_CAN_ID = 26;
  public static final int RIGHT_CLIMB_MOTOR_CAN_ID = 27;
  
  public static final int INTAKE_SLAPDOWN_MOTOR_CAN_ID = 21;
  public static final int INTAKE_LEFT_ROLLER_MOTOR_CAN_ID = 22;
  public static final int INTAKE_RIGHT_ROLLER_MOTOR_CAN_ID = 23;
  public static final int INTAKE_CANCODER_CAN_ID = 24;
    
  public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 16;

  public static final int SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID = 15;
  public static final int SHOOTER_RIGHT_ROLLER_MOTOR_CAN_ID = 17;

  public static final int TURRET_MOTOR_CAN_ID = 18; 

  public static final int CANDLE_CAN_ID = 25;

  public static final int INDEXER_MOTOR_CAN_ID = 19;
  public static final int KICKER_MOTOR_CAN_ID = 20;

  public static final CANBus MAIN_CAN_BUS = new CANBus("Swerve");
  public static final CANBus SWERVE_CAN_BUS = new CANBus("rio"); // got a bit flip flopped here but whatever electrical works, swerve is just the canivore bus
}
