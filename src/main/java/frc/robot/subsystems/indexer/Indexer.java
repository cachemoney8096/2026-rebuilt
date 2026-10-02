package frc.robot.subsystems.indexer;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {

  private final TelemetryTable indexerTelemetry =
    Telemetry.getTable("Indexer");

  private final TalonFX rotatorMotor =
      new TalonFX(RobotMap.INDEXER_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);
  private final TalonFX kickerMotor =
      new TalonFX(RobotMap.KICKER_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);

  public Indexer() {
    initTalons();
  }

  private void initTalons() {
    TalonFXConfiguration toApply = new TalonFXConfiguration();

    toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    toApply.CurrentLimits.SupplyCurrentLimit = IndexerCal.INDEXER_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimit = IndexerCal.INDEXER_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
    toApply.CurrentLimits.StatorCurrentLimitEnable = true;
    toApply.Slot0.kP = IndexerCal.INDEXER_P;
    toApply.Slot0.kI = IndexerCal.INDEXER_I;
    toApply.Slot0.kD = IndexerCal.INDEXER_D;
    toApply.Slot0.kV = IndexerCal.INDEXER_FF;

    TalonFXConfigurator indexerConfigurator = rotatorMotor.getConfigurator();
    indexerConfigurator.apply(toApply);

    TalonFXConfigurator kickerConfigurator = kickerMotor.getConfigurator();
    // adjust direction if needed (not needed, they are both counterclockwise positive)
    toApply.Slot0.kP = IndexerCal.KICKER_P;
    toApply.Slot0.kI = IndexerCal.KICKER_I;
    toApply.Slot0.kD = IndexerCal.KICKER_D;
    toApply.Slot0.kV = IndexerCal.KICKER_FF;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfigurator.apply(toApply);
  }

  public void runIndexer() {
    rotatorMotor.setThrottle(IndexerCal.INDEXER_SPEED);
  }
  
  public boolean indexerIsOn() {
    return Math.abs(rotatorMotor.getVelocity().getValueAsDouble()) > 0.0;
  }

  public boolean kickerIsOn() {
    return Math.abs(kickerMotor.getVelocity().getValueAsDouble()) > 0.0;
  }

  public void stopIndexer() {
    rotatorMotor.setThrottle(0.0);
  }

  public void runKicker() {
    kickerMotor.setThrottle(IndexerCal.KICKER_SPEED);
  }

  public void stopKicker() {
    kickerMotor.setThrottle(0.0);
  }

  public void reverseKicker(){
    kickerMotor.setThrottle(-0.5);
  }

  public void reverseIndexer(){
    rotatorMotor.setThrottle(0.5);
  }

  @Override
  public void periodic() {
    sendTelemetry();
  }

  private void sendTelemetry() {
    indexerTelemetry.log("Indexer Speed (RPM)", rotatorMotor.getVelocity().getValueAsDouble() * 60);
    indexerTelemetry.log("Indexer Current (A)", rotatorMotor.getTorqueCurrent().getValueAsDouble());
    indexerTelemetry.log("Kicker Speed (RPM)", kickerMotor.getVelocity().getValueAsDouble() * 60);
    indexerTelemetry.log("Kicker Current (A)", kickerMotor.getTorqueCurrent().getValueAsDouble());
  }
}
