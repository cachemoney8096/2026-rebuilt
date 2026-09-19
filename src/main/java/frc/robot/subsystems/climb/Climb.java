package frc.robot.subsystems.climb;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
  public enum ClimbHeight {
    HOME,
    FINISHED,
    PREP;
  }

  private TreeMap<ClimbHeight, Double> climbPositions = new TreeMap<ClimbHeight, Double>();

  private ClimbHeight desiredPosition = ClimbHeight.HOME;

  private TalonFX motor = new TalonFX(RobotMap.LEFT_CLIMB_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);
  private boolean allowClimbMovement = true;
  private Servo ratchet;

  public Climb() {
    climbPositions.put(ClimbHeight.HOME, ClimbCal.POSITION_HOME_INCHES);
    climbPositions.put(ClimbHeight.FINISHED, ClimbCal.POSITION_FINISHED_INCHES);
    climbPositions.put(ClimbHeight.PREP, ClimbCal.POSITION_PREP_INCHES);
    ratchet = new Servo(1);
    initTalons();
  }

  private void initTalons() {
    TalonFXConfigurator cfgLeft = motor.getConfigurator();
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.CurrentLimits.SupplyCurrentLimit = ClimbCal.CLIMB_MOTOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit = ClimbCal.CLIMB_MOTOR_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.Slot0.kP = ClimbCal.CLIMB_SCORE_P;
    toApply.Slot0.kI = ClimbCal.CLIMB_SCORE_I;
    toApply.Slot0.kD = ClimbCal.CLIMB_SCORE_D;
    toApply.Slot0.kV = ClimbCal.CLIMB_SCORE_FF;
    toApply.Slot0.kG = 0.0;

    cfgLeft.apply(toApply);
    zeroClimbToHome();

  }

  public void zeroClimbToHome() {
    motor.setPosition(climbPositionToMotorPosition(ClimbHeight.HOME));
    setDesiredPosition(ClimbHeight.HOME);
  }

  public void setRatchetLocked(){
    ratchet.setAngle(0.0);
  }

  public void setRatchetUnlocked(){
    ratchet.setAngle(180.0);
  }

  public void setDesiredPosition(ClimbHeight height) {
    desiredPosition = height;
  }

  private double climbPositionToMotorPosition(ClimbHeight climbPosition)  {
        return (climbPositions.get(climbPosition)) * ClimbCal.CLIMB_MOTOR_TO_CLIMB_INCHES_RATIO;
  }

  private void controlPosition() {
    final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(6000.0, 6000.0));
    TrapezoidProfile.State tGoal = new TrapezoidProfile.State(climbPositionToMotorPosition(desiredPosition), 0.0);
    TrapezoidProfile.State setpoint = new TrapezoidProfile.State(
        motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble());
    final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    setpoint = trapezoidProfile.calculate(0.020, setpoint, tGoal);
    request.Position = setpoint.position;
    request.Velocity = setpoint.velocity;
    motor.setControl(request);
  }

  public boolean atDesiredPosition() {
    return Math.abs(
        getClimbHeight()
            - climbPositions.get(desiredPosition)) < ClimbCal.CLIMB_MARGIN_INCHES;
  }

  public boolean atClimbPosition(ClimbHeight height) {
    return Math.abs(
        getClimbHeight()
            - climbPositions.get(height)) < ClimbCal.CLIMB_MARGIN_INCHES;
  }

  public double getClimbHeight() {
    return motor
        .getPosition()
        .getValueAsDouble()
        / ClimbCal.CLIMB_MOTOR_TO_CLIMB_INCHES_RATIO;
  }

  public void periodic() {
    if (allowClimbMovement) {
      controlPosition();
    }
  }

  public void stopClimbMovement() {
    motor.setVoltage(0.0);
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
        () -> motor.getPosition().getValueAsDouble() * 360.0,
        null);

    builder.addDoubleProperty(
        "Elevator CURRENT Pos (in)",
        () -> (getClimbHeight()),
        null);

    builder.addBooleanProperty("Allow Climb Movement", () -> allowClimbMovement, null);
    builder.addDoubleProperty(
        "Climb voltage commanded", () ->motor.getMotorVoltage().getValueAsDouble(), null);
      builder.addDoubleProperty("Pos in", ()->motor.getPosition().getValueAsDouble()/ClimbCal.CLIMB_MOTOR_TO_CLIMB_INCHES_RATIO, null);
      builder.addDoubleProperty("pos rot", ()->motor.getPosition().getValueAsDouble(), null);
  }

}
