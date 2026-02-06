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
    private double hoodDesiredPositionDeg = ShooterCal.HOOD_HOME_DEGREES; 

    private final TrapezoidProfile hoodTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        ShooterCal.HOOD_MAX_VELOCITY_RPS, 
        ShooterCal.HOOD_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX leftRollerMotor = new TalonFX(RobotMap.SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private final TalonFX rightRollerMotor = new TalonFX(RobotMap.SHOOTER_RIGHT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    private double currentRollerSpeedRPS = 0.05;

    public Shooter() {
        initTalons();
        relativeZeroHood();
    }

    private void initTalons() {
        /* Init rollers */
        TalonFXConfiguration rollersToApply = new TalonFXConfiguration();
        rollersToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollersToApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollersToApply.CurrentLimits.SupplyCurrentLimit = ShooterCal.ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimit = ShooterCal.ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersToApply.Slot0.kP = ShooterCal.ROLLERS_P;
        rollersToApply.Slot0.kI = ShooterCal.ROLLERS_I;
        rollersToApply.Slot0.kD = ShooterCal.ROLLERS_D;
        rollersToApply.Slot0.kV = ShooterCal.ROLLERS_FF;

        TalonFXConfigurator leftRollersConfig = leftRollerMotor.getConfigurator();
        leftRollersConfig.apply(rollersToApply);

        Follower master = new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
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

    public void relativeZeroHood() {
        hoodMotor.setPosition(
            (ShooterCal.HOOD_HOME_DEGREES / 360.0) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO);
        hoodDesiredPositionDeg = ShooterCal.HOOD_HOME_DEGREES;
    }

    public void setDesiredHoodPosition(double newPositionDegrees) {
        hoodDesiredPositionDeg = Math.max(ShooterCal.HOOD_MIN_DEGREES, Math.min(ShooterCal.HOOD_MAX_DEGREES, newPositionDegrees));
    }

    public void runRollers() {
        leftRollerMotor.set(currentRollerSpeedRPS / ShooterCal.ROLLERS_MAX_RPS);
    }

    public void stopRollers() {
        leftRollerMotor.set(0.0);
    }

    public void setRollerSpeedRPS(double speedRPS) {
        currentRollerSpeedRPS = Math.min(Math.max(speedRPS, 0.0), ShooterCal.ROLLERS_MAX_RPS);
    }
    public double getDesiredPositionDeg() {
        return hoodDesiredPositionDeg;
    }
    public double getRollerSpeedRPS() {
        return currentRollerSpeedRPS;
    }

    public boolean atDesiredHoodPosition() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodPositionToMotorPosition(hoodDesiredPositionDeg)) < ShooterCal.HOOD_POSITION_MARGIN;
    }

    private double hoodPositionToMotorPosition(double hoodPositionDeg)  {
        return (hoodPositionDeg / 360.0) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO;
    }

    public boolean atDesiredSpeed() {

        //TODO change 1 and add tolernce
        return leftRollerMotor.getVelocity().getValueAsDouble() == 1.0 * ShooterCal.ROLLERS_MAX_RPS;

    }

    private void controlHoodPosition() {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(
            hoodPositionToMotorPosition(hoodDesiredPositionDeg), 0.0);
        TrapezoidProfile.State start = new TrapezoidProfile.State(
            hoodMotor.getPosition().getValueAsDouble(), hoodMotor.getVelocity().getValueAsDouble());
        
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

        /* Hood */
        builder.addDoubleProperty("Hood Actual Position (deg.)", () -> (hoodMotor.getPosition().getValueAsDouble() / 360.0) * ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO, null);
        builder.addDoubleProperty("Hood Desired Position (deg.)", () -> hoodDesiredPositionDeg, null);

        builder.addBooleanProperty("Hood at Desired Position", this::atDesiredHoodPosition, null);

        builder.addDoubleProperty("Hood Amperage (amps)", () -> hoodMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Hood Commanded Voltage (volts)", () -> hoodMotor.getMotorVoltage().getValueAsDouble(), null);

        /* Rollers */
        builder.addDoubleProperty("Rollers Speed (RPS)", () -> leftRollerMotor.getVelocity().getValueAsDouble(), null);
        
        builder.addDoubleProperty("Left Roller Amperage (amps)", () -> leftRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right Roller Amperage (amps)", () -> rightRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
    }
}
