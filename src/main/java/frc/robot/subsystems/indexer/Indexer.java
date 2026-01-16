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

    private final TalonFX motor = new TalonFX(RobotMap.INDEXER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    
    public Indexer() {
        initTalons();
    }

    private void initTalons() {

        /* Init rollers */
        TalonFXConfiguration rollersToApply = new TalonFXConfiguration();

        //TODO investigate Clockwise
        rollersToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollersToApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollersToApply.CurrentLimits.SupplyCurrentLimit = IndexerCal.INDEXER_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimit = IndexerCal.INDEXER_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersToApply.Slot0.kP = IndexerCal.INDEXER_P;
        rollersToApply.Slot0.kI = IndexerCal.INDEXER_I;
        rollersToApply.Slot0.kD = IndexerCal.INDEXER_D;
        rollersToApply.Slot0.kV = IndexerCal.INDEXER_FF;

        TalonFXConfigurator indexerConfigurator = motor.getConfigurator();
        indexerConfigurator.apply(rollersToApply);

    }

    public void runIndexer() {
        motor.setVoltage(IndexerCal.ROLLERS_RUNNING_VOLTAGE);
    }

    public void stopIndexer() {
        motor.setVoltage(0.0);
    }
    @Override
    public void initSendable(SendableBuilder builder) {

        super.initSendable(builder);
        builder.addDoubleProperty("Indexer Speed (percent)", () -> motor.get(), null);
        builder.addDoubleProperty("Indexer Amperage (amps)", () -> motor.getTorqueCurrent().getValueAsDouble(), null);
    }

    
}
