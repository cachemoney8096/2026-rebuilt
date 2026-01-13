package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private final TalonFX hoodMotor = new TalonFX(RobotMap.SHOOTER_HOOD_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private double hoodDesiredPosition = ShooterCal.HOOD_DEFAULT_POSITION_DEGREES; 

    private final TrapezoidProfile hoodTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        ShooterCal.HOOD_MAX_VELOCITY_RPS, 
        ShooterCal.HOOD_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX leftRollerMotor = new TalonFX(RobotMap.SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private final TalonFX rightRollerMotor = new TalonFX(RobotMap.SHOOTER_RIGHT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    public Shooter() {
        initTalons();
        zeroHood();
    }

    private void initTalons() {
        /* Init rollers */
        TalonFXConfiguration rollersToApply = new TalonFXConfiguration();
        rollersToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollersToApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollersToApply.CurrentLimits.SupplyCurrentLimit = ShooterCal.ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimit = ShooterCal.ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersToApply.Slot0.kP = ShooterCal.ROLLERS_P;
        rollersToApply.Slot0.kI = ShooterCal.ROLLERS_I;
        rollersToApply.Slot0.kD = ShooterCal.ROLLERS_D;
        rollersToApply.Slot0.kV = ShooterCal.ROLLERS_FF;

        TalonFXConfigurator leftRollerConfig = leftRollerMotor.getConfigurator();
        leftRollerConfig.apply(rollersToApply);

        Follower master = new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Aligned);
        rightRollerMotor.setControl(master); 

        /* Init hood */
        TalonFXConfiguration hoodToApply = new TalonFXConfiguration();
        hoodToApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodToApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodToApply.CurrentLimits.SupplyCurrentLimit = ShooterCal.HOOD_SUPPLY_CURRENT_LIMIT_AMPS;
        hoodToApply.CurrentLimits.StatorCurrentLimit = ShooterCal.HOOD_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        hoodToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodToApply.Slot0.kP = ShooterCal.HOOD_P;
        hoodToApply.Slot0.kI = ShooterCal.HOOD_I;
        hoodToApply.Slot0.kD = ShooterCal.HOOD_D;
        hoodToApply.Slot0.kV = ShooterCal.HOOD_FF;

        TalonFXConfigurator hoodConfig = hoodMotor.getConfigurator();
        hoodConfig.apply(hoodToApply);
    }

    public void zeroHood() {
        hoodMotor.setPosition(
            (ShooterCal.HOOD_DEFAULT_POSITION_DEGREES / 360) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO);
    }

    public void runRollers() {
        leftRollerMotor.setVoltage(ShooterCal.ROLLER_VOLTAGE);
    }

    public void stopRollers() {
        leftRollerMotor.setVoltage(0.0);
    }

    private void controlHoodPosition() {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(hoodDesiredPosition, 0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Hood Actual Position (deg.)", () -> hoodMotor.getPosition().getValueAsDouble() * 360, null);
        builder.addDoubleProperty("Hood Desired Position (deg.)", () -> hoodDesiredPosition, null);
    }
}
