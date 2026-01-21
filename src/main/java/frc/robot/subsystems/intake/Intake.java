package frc.robot.subsystems.intake;
import java.util.TreeMap;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private final TalonFX slapdownMotor = new TalonFX(RobotMap.INTAKE_SLAPDOWN_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    private final TrapezoidProfile slapdownTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        IntakeCal.SLAPDOWN_MAX_VELOCITY_RPS, 
        IntakeCal.SLAPDOWN_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX leftRollerMotor = new TalonFX(RobotMap.INTAKE_LEFT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private final TalonFX rightRollerMotor = new TalonFX(RobotMap.INTAKE_RIGHT_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    private final CANcoder absoluteEncoder = new CANcoder(RobotMap.INTAKE_CANCODER_CAN_ID, RobotMap.RIO_CAN_BUS);

    public enum IntakePosition {
        HOME,
        EXTENDED
    }

    public final TreeMap<IntakePosition, Double> intakePositions = new TreeMap<IntakePosition, Double>();

    private IntakePosition slapdownDesiredPosition = IntakePosition.HOME;

    public Intake(){
        initPositions();
        initTalons();
        zeroSlapdown();
    }

    private void initPositions() {
        intakePositions.put(IntakePosition.HOME, IntakeCal.INTAKE_POSITION_HOME_DEGREES);
        intakePositions.put(IntakePosition.EXTENDED, IntakeCal.INTAKE_POSITION_EXTENDED_DEGREES);
    }

    private void initTalons() {
        /* Init rollers */
        TalonFXConfiguration rollersToApply = new TalonFXConfiguration();
        rollersToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollersToApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollersToApply.CurrentLimits.SupplyCurrentLimit = IntakeCal.ROLLERS_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimit = IntakeCal.ROLLERS_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollersToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollersToApply.Slot0.kP = IntakeCal.ROLLERS_P;
        rollersToApply.Slot0.kI = IntakeCal.ROLLERS_I;
        rollersToApply.Slot0.kD = IntakeCal.ROLLERS_D;
        rollersToApply.Slot0.kV = IntakeCal.ROLLERS_FF;

        TalonFXConfigurator leftRollerConfig = leftRollerMotor.getConfigurator();
        leftRollerConfig.apply(rollersToApply);

        Follower master = new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        rightRollerMotor.setControl(master);

        /* Init slapdown */
        TalonFXConfiguration slapdownToApply = new TalonFXConfiguration();
        slapdownToApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        slapdownToApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        slapdownToApply.CurrentLimits.SupplyCurrentLimit = IntakeCal.SLAPDOWN_SUPPLY_CURRENT_LIMIT_AMPS;
        slapdownToApply.CurrentLimits.StatorCurrentLimit = IntakeCal.SLAPDOWN_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        slapdownToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        slapdownToApply.Slot0.kP = IntakeCal.SLAPDOWN_P;
        slapdownToApply.Slot0.kI = IntakeCal.SLAPDOWN_I;
        slapdownToApply.Slot0.kD = IntakeCal.SLAPDOWN_D;
        slapdownToApply.Slot0.kV = IntakeCal.SLAPDOWN_FF;

        TalonFXConfigurator slapdownConfig = slapdownMotor.getConfigurator();
        slapdownConfig.apply(slapdownToApply);

        CANcoderConfiguration canCoderToApply = new CANcoderConfiguration();
        canCoderToApply.MagnetSensor.MagnetOffset = IntakeCal.INTAKE_CANCODER_MAGNET_OFFSET; 
        canCoderToApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

        CANcoderConfigurator canCoderConfig = absoluteEncoder.getConfigurator();
        canCoderConfig.apply(canCoderToApply);
    }

    public void zeroSlapdown() {
        slapdownMotor.setPosition(
            (IntakeCal.SLAPDOWN_HOME_DEGREES / 360.0) * IntakeCal.SLAPDOWN_MOTOR_TO_SLAPDOWN_RATIO);
        slapdownDesiredPosition = IntakePosition.HOME;
    }

    public void setDesiredSlapdownPosition(IntakePosition newPosition) {
        slapdownDesiredPosition = newPosition;
    }

    public double getRealPositionRotations() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void runRollers() {
        leftRollerMotor.set(IntakeCal.ROLLERS_RUNNING_SPEED);
    }

    public void stopRollers() {
        leftRollerMotor.set(0.0);
    }

    public boolean atDesiredSlapdownPosition() {
        return Math.abs(slapdownMotor.getPosition().getValueAsDouble() - slapdownPositionToMotorPosition(slapdownDesiredPosition)) < IntakeCal.SLAPDOWN_POSITION_MARGIN;
    }

    private double slapdownPositionToMotorPosition(IntakePosition slapdownPosition)  {
        return (intakePositions.get(slapdownPosition) / 360.0) * IntakeCal.SLAPDOWN_MOTOR_TO_SLAPDOWN_RATIO;
    }

    private void controlSlapdownPosition() {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(
            slapdownPositionToMotorPosition(slapdownDesiredPosition), 0.0);
        TrapezoidProfile.State start = new TrapezoidProfile.State(
            slapdownMotor.getPosition().getValueAsDouble(), slapdownMotor.getVelocity().getValueAsDouble());
        
        PositionVoltage request = new PositionVoltage(0.0).withSlot(0);
        TrapezoidProfile.State setpoint = slapdownTrapezoidProfile.calculate(0.020, start, goal);
        
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        
        slapdownMotor.setControl(request);
    }

    @Override
    public void periodic() {
        controlSlapdownPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        /* Slapdown */
        builder.addDoubleProperty("Position (deg.)", () -> (slapdownMotor.getPosition().getValueAsDouble() / 360.0) * IntakeCal.SLAPDOWN_MOTOR_TO_SLAPDOWN_RATIO, null);
        builder.addDoubleProperty("Real Position (rot.)", this::getRealPositionRotations, null);
        builder.addDoubleProperty("Slapdown Desired Position (deg.)", () -> intakePositions.get(slapdownDesiredPosition), null);
        builder.addStringProperty("Slapdown Desired Position", () -> slapdownDesiredPosition.toString(), null);

        builder.addBooleanProperty("Slapdown at Desired Position", this::atDesiredSlapdownPosition, null);

        builder.addDoubleProperty("Slapdown Amperage (amps)", () -> slapdownMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Slapdown Commanded Voltage (volts)", () -> slapdownMotor.getMotorVoltage().getValueAsDouble(), null);

        /* Rollers */
        builder.addDoubleProperty("Rollers Speed (percent)", () -> leftRollerMotor.get(), null);
        
        builder.addDoubleProperty("Left roller Amperage (amps)", () -> leftRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right roller Amperage (amps)", () -> rightRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
    }

}
