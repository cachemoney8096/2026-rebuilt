package frc.robot;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
  public static final int LEFT_CLIMB_MOTOR_CAN_ID = 0;
  public static final int RIGHT_CLIMB_MOTOR_CAN_ID = 0;
  public static final int CLIMB_CANRANGE = 0;
  
  public static final int INTAKE_SLAPDOWN_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
  public static final int INTAKE_LEFT_ROLLER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
  public static final int INTAKE_RIGHT_ROLLER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
    
  public static final int SHOOTER_HOOD_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;

  public static final int SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
  public static final int SHOOTER_RIGHT_ROLLER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;

  public static final int TURRET_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT; 

  public static final int CANDLE_CAN_ID = Constants.PLACEHOLDER_INT;

  public static final int INDEXER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
  public static final int KICKER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
  public static final int INTAKE_CANCODER_CAN_ID = Constants.PLACEHOLDER_INT;

  public static final CANBus RIO_CAN_BUS = new CANBus("rio");
}
