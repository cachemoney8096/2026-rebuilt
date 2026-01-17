package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;


public class Indexer extends SubsystemBase {

    private final TalonFX rotatorMotor = new TalonFX(RobotMap.INDEXER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private final TalonFX kickerMotor = new TalonFX(RobotMap.KICKER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    
    public Indexer() {
        initTalons();
    }

    private void initTalons() {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        // adjust direction if needed
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
        // adjust direction if needed
        toApply.Slot0.kP = IndexerCal.KICKER_P;
        toApply.Slot0.kI = IndexerCal.KICKER_I;
        toApply.Slot0.kD = IndexerCal.KICKER_D;
        toApply.Slot0.kV = IndexerCal.KICKER_FF;
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kickerConfigurator.apply(toApply);

    }

    public void runIndexer() {
        rotatorMotor.set(IndexerCal.INDEXER_SPEED);
    }

    public void stopIndexer() {
        rotatorMotor.set(0.0);
    }

    public void runKicker(){
        kickerMotor.set(IndexerCal.KICKER_SPEED);
    }

    public void stopKicker(){
        kickerMotor.set(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        super.initSendable(builder);
        builder.addDoubleProperty("Indexer Speed (percent)", () -> rotatorMotor.get(), null);
        builder.addDoubleProperty("Indexer Amperage (amps)", () -> rotatorMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Kicker Speed (percent)", () -> kickerMotor.get(), null);
        builder.addDoubleProperty("Kicker Amperage (amps)", () -> kickerMotor.getTorqueCurrent().getValueAsDouble(), null);
    }

    
}
