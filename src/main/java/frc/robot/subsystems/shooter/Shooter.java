package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    private double hoodDesiredPosition = (ShooterCal.HOOD_DEFAULT_POSITION_DEGREES / 360.0); 

    private final TrapezoidProfile hoodTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        ShooterCal.HOOD_MAX_VELOCITY_RPS, 
        ShooterCal.HOOD_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX rollerMotor = new TalonFX(RobotMap.SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    public Shooter() {
        
        initTalons();
        zeroHood();
    }

    private void initTalons() {
        /* Init roller */
        TalonFXConfiguration rollerToApply = new TalonFXConfiguration();
        rollerToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerToApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerToApply.CurrentLimits.SupplyCurrentLimit = ShooterCal.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        rollerToApply.CurrentLimits.StatorCurrentLimit = ShooterCal.ROLLER_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollerToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerToApply.Slot0.kP = ShooterCal.ROLLER_P;
        rollerToApply.Slot0.kI = ShooterCal.ROLLER_I;
        rollerToApply.Slot0.kD = ShooterCal.ROLLER_D;
        rollerToApply.Slot0.kV = ShooterCal.ROLLER_FF;

        TalonFXConfigurator leftRollerConfig = rollerMotor.getConfigurator();
        leftRollerConfig.apply(rollerToApply);

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
            (ShooterCal.HOOD_DEFAULT_POSITION_DEGREES / 360.0) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO);
    }

    public void setDesiredHoodPosition(double newPositionDegrees) {
        hoodDesiredPosition = (newPositionDegrees / 360.0) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO;
    }
    public void stopHoodMotor() {
        hoodMotor.setVoltage(0.0);
        hoodDesiredPosition = hoodMotor.getPosition().getValueAsDouble();
        
    }

    public void runRoller() {
        rollerMotor.setVoltage(ShooterCal.ROLLER_VOLTAGE);
    }

    public void stopRoller() {
        rollerMotor.setVoltage(0.0);
    }

    public boolean atDesiredPosition() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodDesiredPosition) < ShooterCal.HOOD_POSITION_MARGIN;

    }

    private void controlHoodPosition() {

        TrapezoidProfile.State goal = new TrapezoidProfile.State(hoodDesiredPosition, 0.0);
        TrapezoidProfile.State start = 
            new TrapezoidProfile.State(hoodMotor.getPosition().getValueAsDouble(), hoodMotor.getVelocity().getValueAsDouble());
        
        PositionVoltage request = new PositionVoltage(0.0).withSlot(0);

        TrapezoidProfile.State setpoint = hoodTrapezoidProfile.calculate(0.020, start, goal);
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        hoodMotor.setControl(request);
    }

    @Override
    public void periodic() {
        controlHoodPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Hood Actual Position (deg.)", () -> hoodMotor.getPosition().getValueAsDouble() * 360, null);
        builder.addDoubleProperty("Hood Desired Position (deg.)", () -> hoodDesiredPosition, null);

        builder.addDoubleProperty("Roller current speed (percent)", () -> rollerMotor.get(), null);
        builder.addDoubleProperty("Roller Amps", () -> rollerMotor.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Hood Amps", () -> hoodMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Hood commanded voltage (volts)", () -> hoodMotor.getMotorVoltage().getValueAsDouble(), null);
    }
}
