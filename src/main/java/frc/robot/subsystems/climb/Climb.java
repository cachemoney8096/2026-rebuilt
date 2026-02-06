package frc.robot.subsystems.climb;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.TreeMap;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase{
    public enum ClimbHeight {
        HOME,
        FINISHED,
        PREP;
    }

    private TreeMap<ClimbHeight, Double> climbPositions = new TreeMap<ClimbHeight, Double>();

    private ClimbHeight desiredPosition = ClimbHeight.HOME;

    private TalonFX leftMotor = new TalonFX(RobotMap.LEFT_CLIMB_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_CLIMB_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    private boolean allowClimbMovement = true; 

    public Climb() {
    climbPositions.put(ClimbHeight.HOME, ClimbCal.POSITION_HOME_INCHES);
    climbPositions.put(ClimbHeight.FINISHED, ClimbCal.POSITION_FINISHED_INCHES);
    climbPositions.put(ClimbHeight.PREP, ClimbCal.POSITION_PREP_INCHES);
  }

  private void initTalons() {
    TalonFXConfigurator cfgLeft = leftMotor.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimit = ClimbCal.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit =
        ClimbCal.CLIMB_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.Slot0.kP = ClimbCal.CLIMB_SCORE_P;
    toApply.Slot0.kI = ClimbCal.CLIMB_SCORE_I;
    toApply.Slot0.kD = ClimbCal.CLIMB_SCORE_D;
    toApply.Slot0.kV = ClimbCal.CLIMB_SCORE_FF;
    toApply.Slot0.kG = 0.25;

    cfgLeft.apply(toApply);
    Follower master = new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    rightMotor.setControl(master);

    zeroClimbToHome();

  }

  public void zeroClimbToHome() {
    leftMotor.setPosition(climbPositions.get(ClimbHeight.HOME));
    setDesiredPosition(ClimbHeight.HOME);
  }

  public void setDesiredPosition(ClimbHeight height) {
    desiredPosition = height;
  }

  private void controlPosition(double inputRotations) {
    double inches =
        inputRotations
            / ClimbCal.MOTOR_TO_GEAR_RATIO
            * ClimbCal.GEAR_CIRCUMFERENCE;

    final TrapezoidProfile trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(ClimbCal.FIRST_CONSTRAINT, ClimbCal.SECOND_CONSTRAINT));
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(inches, 0.0);
    TrapezoidProfile.State setpoint =
        new TrapezoidProfile.State(
            leftMotor.getPosition().getValueAsDouble(), leftMotor.getVelocity().getValueAsDouble());
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    setpoint = trapezoidProfile.calculate(0.020, setpoint, tGoal);
    request.Position = setpoint.position;
    request.Velocity = setpoint.velocity;
    leftMotor.setControl(request);
  }

  public boolean atDesiredPosition() {
    return Math.abs(
            getClimbHeight()
                - climbPositions.get(desiredPosition))
        < ClimbCal.CLIMB_MARGIN_INCHES;
  }

    public boolean atClimbPosition(ClimbHeight height) {
    return Math.abs(
            getClimbHeight()
                - climbPositions.get(height))
        < ClimbCal.CLIMB_MARGIN_INCHES;
  }

  public double getClimbHeight() {
    return leftMotor 
    .getPosition()
    .getValueAsDouble()
    * ClimbCal.GEAR_CIRCUMFERENCE
    / ClimbCal.MOTOR_TO_GEAR_RATIO;
  }

  public void periodic() {
    if (allowClimbMovement) {
        controlPosition(climbPositions.get(desiredPosition));
    }
  }

  public void stopClimbMovement() {
    leftMotor.setVoltage(0.0);
  }

  public void setClimbMovementAllowed(boolean allowed) {
    allowClimbMovement = allowed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addStringProperty("Climb DESIRED Pos", () -> desiredPosition.toString(), null);
    builder.addDoubleProperty(
        "Climb DESIRED Pos (in)", () -> climbPositions.get(desiredPosition), null);
    builder.addBooleanProperty("Climb at desired", () -> atDesiredPosition(), null);

    builder.addDoubleProperty(
        "Climb Left Motor RELATIVE (deg)",
        () -> leftMotor.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addDoubleProperty(
        "Climb Right Motor RELATIVE (deg)",
        () -> rightMotor.getPosition().getValueAsDouble() * 360.0,
        null);

    builder.addDoubleProperty(
        "Elevator CURRENT Pos (in)",
        () ->
            (getClimbHeight()),
        null);

    builder.addBooleanProperty("Allow Climb Movement", () -> allowClimbMovement, null);
    builder.addDoubleProperty(
        "Climb voltage commanded", () -> leftMotor.getMotorVoltage().getValueAsDouble(), null);
  }


    
}
