package frc.robot.subsystems.climb;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase{
    public enum ClimbHeight {
        HOME,
        L1_AUTO,
        L1_TELEOP;
    }

    private TreeMap<ClimbHeight, Double> climbPositions = new TreeMap<ClimbHeight, Double>();

    private ClimbHeight desiredPosition = ClimbHeight.HOME;
    private TalonFX leftMotor = new TalonFX(RobotMap.LEFT_CLIMB_MOTOR_CAN_ID, "rio");
    private TalonFX rightMotor = new TalonFX(RobotMap.RIGHT_CLIMB_MOTOR_CAN_ID, "rio");

    private boolean allowClimbMovement = true; 

    public Climb() {
    climbPositions.put(ClimbHeight.HOME, ClimbCal.POSITION_HOME_INCHES);
    climbPositions.put(ClimbHeight.L1_AUTO, ClimbCal.POSITION_L1_AUTO_INCHES);
    climbPositions.put(ClimbHeight.L1_TELEOP, ClimbCal.POSITION_LI_TELEOP_INCHES);
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
    // Slot 0 is for Scoring PID values and Slot 1 is for Shallow Climbing PID values
    toApply.Slot0.kP = ClimbCal.CLIMB_SCORE_P;
    toApply.Slot0.kI = ClimbCal.CLIMB_SCORE_I;
    toApply.Slot0.kD = ClimbCal.CLIMB_SCORE_D;
    toApply.Slot0.kV = ClimbCal.CLIMB_SCORE_FF;
    // [\]
    toApply.Slot0.kG = 0.25;

    cfgLeft.apply(toApply);
    Follower master = new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    rightMotor.setControl(master);

    zeroClimbToHome();
    // zeroElevatorUsingCanrange();

  }

  public void zeroClimbToHome() {
    leftMotor.setPosition(0.0);
  }

  public void setDesiredPosition(ClimbHeight height) {
    desiredPosition = height;
  }

  private void controlPosition(double inputPositionInch) {
    // final PositionVoltage m_request = new PositionVoltage(0.0).withSlot(currentSlotValue);
    double rotations =
        inputPositionInch
            / ClimbCal.DRUM_CIRCUMFERENCE
            * ClimbCal.MOTOR_TO_DRUM_RATIO;

    final TrapezoidProfile trapezoidProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1000, 2000));
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(rotations, 0.0);
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
    * ClimbCal.DRUM_CIRCUMFERENCE
    / ClimbCal.MOTOR_TO_DRUM_RATIO;
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

    builder.addStringProperty("Elevator DESIRED Pos", () -> desiredPosition.toString(), null);
    builder.addDoubleProperty(
        "Elevator DESIRED Pos (in)", () -> climbPositions.get(desiredPosition), null);
    builder.addBooleanProperty("Elevator at desired", () -> atDesiredPosition(), null);

    builder.addDoubleProperty(
        "Elevator Left Motor RELATIVE (deg)",
        () -> leftMotor.getPosition().getValueAsDouble() * 360.0,
        null);
    builder.addDoubleProperty(
        "Elevator Right Motor RELATIVE (deg)",
        () -> rightMotor.getPosition().getValueAsDouble() * 360.0,
        null);

    builder.addDoubleProperty(
        "Elevator CURRENT Pos (in)",
        () ->
            (leftMotor.getPosition().getValueAsDouble()
                * climbCal.DRUM_CIRCUMFERENCE
                / ElevatorConstants.MOTOR_TO_DRUM_RATIO),
        null);

    builder.addStringProperty(
        "Elevator PID Slot",
        () -> {
          return currentSlotValue == 0 ? "SCORING" : "CLIMBING";
        },
        null);
    /*builder.addBooleanProperty("Elevator Limit Switch HOME ", () -> getLimitSwitchHome(), null);
    builder.addBooleanProperty(
        "Elevator Limit Switch BELOW HOME", () -> getLimitSwitchBelowHome(), null);
    builder.addBooleanProperty("Elevator Limit Switch Switch TOP", () -> getLimitSwitchTop(), null);*/

    builder.addBooleanProperty("Allow Elevator Movement", () -> allowElevatorMovement, null);
    builder.addDoubleProperty(
        "Canrange distance INCHES",
        () -> Units.metersToInches(canrange.getDistance().getValueAsDouble()),
        null);
    builder.addDoubleProperty(
        "Elevator voltage commanded", () -> leftMotor.getMotorVoltage().getValueAsDouble(), null);
    builder.addBooleanProperty("arm movement allowed", this::armMovementAllowed, null);
  }


    
}
