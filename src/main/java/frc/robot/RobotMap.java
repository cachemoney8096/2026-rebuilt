package frc.robot;

import com.ctre.phoenix6.CANBus;

public class RobotMap {
    public static final int INTAKE_SLAPDOWN_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;
    public static final int INTAKE_ROLLER_MOTOR_CAN_ID = Constants.PLACEHOLDER_INT;

    public static final CANBus RIO_CAN_BUS = new CANBus("rio");
}
