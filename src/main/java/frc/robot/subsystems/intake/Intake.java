package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.shooter.ShooterCal;

public class Intake extends SubsystemBase{
    private final TalonFX slapdownMotor = new TalonFX(RobotMap.INTAKE_SLAPDOWN_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private double slapdownDesiredPositionDeg = IntakeCal.HOME_SLAPDOWN_DEGREES;

    private final TrapezoidProfile slapdownTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        IntakeCal.SLAPDOWN_MAX_VELOCITY_RPS, 
        IntakeCal.SLAPDOWN_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX rollerMotor = new TalonFX(RobotMap.INTAKE_ROLLER_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);

    public Intake(){
        initTalons();
        zeroSlapdown();
    }

    private void initTalons(){
    /* Init roller */
        TalonFXConfiguration rollerToApply = new TalonFXConfiguration();
        rollerToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerToApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerToApply.CurrentLimits.SupplyCurrentLimit = IntakeCal.ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        rollerToApply.CurrentLimits.StatorCurrentLimit = IntakeCal.ROLLER_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        rollerToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerToApply.Slot0.kP = IntakeCal.ROLLER_P;
        rollerToApply.Slot0.kI = IntakeCal.ROLLER_I;
        rollerToApply.Slot0.kD = IntakeCal.ROLLER_D;
        rollerToApply.Slot0.kV = IntakeCal.ROLLER_FF;

        TalonFXConfigurator rollerConfig = rollerMotor.getConfigurator();
        rollerConfig.apply(rollerToApply);

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
    }

    public void zeroSlapdown() {
        slapdownMotor.setPosition(
            (IntakeCal.SLAPDOWN_HOME_DEGREES / 360.0) * IntakeCal.SLAPDOWN_MOTOR_TO_Slapdown_RATIO);
    }

    public void setDesiredSlapdownPosition(double newPositionDegrees) {
        slapdownDesiredPositionDeg = Math.max(IntakeCal.Slapdown_MIN_DEGREES, Math.min(IntakeCal.Slapdown_MAX_DEGREES, newPositionDegrees));
    }

    public void runRoller() {
        rollerMotor.setVoltage(IntakeCal.ROLLER_RUNNING_VOLTAGE);
    }

    public void stopRoller() {
        rollerMotor.setVoltage(0.0);
    }

    public boolean atDesiredSlapdownosition() {
        return Math.abs(slapdownMotor.getPosition().getValueAsDouble() - SlapdownPositionToMotorPosition(slapdownDesiredPositionDeg)) < IntakeCal.Slapdown_POSITION_MARGIN;
    }

    private double SlapdownPositionToMotorPosition(double SlapdownPositionDeg)  {
        return (SlapdownPositionDeg / 360.0) / IntakeCal.Slapdown_MOTOR_TO_Slapdown_RATIO;
    }

    private void controlSlapdownPosition() {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(
            SlapdownPositionToMotorPosition(slapdownDesiredPositionDeg), 0.0);
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
        builder.addDoubleProperty(" Actual Position (deg.)", () -> (SlapdownMotor.getPosition().getValueAsDouble() / 360.0) * ShooterCal.Slapdown_MOTOR_TO_Slapdown_RATIO, null);
        builder.addDoubleProperty("Slapdown Desired Position (deg.)", () -> SlapdownDesiredPositionDeg, null);

        builder.addBooleanProperty("Slapdown at Desired Position", this::atDesiredSlapdownPosition, null);

        builder.addDoubleProperty("Slapdown Amperage (amps)", () -> SlapdownMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Slapdown Commanded Voltage (volts)", () -> SlapdownMotor.getMotorVoltage().getValueAsDouble(), null);

        /* Rollers */
        builder.addDoubleProperty("Rollers Speed (percent)", () -> leftRollerMotor.get(), null);
        
        builder.addDoubleProperty("Left Roller Amperage (amps)", () -> leftRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right Roller Amperage (amps)", () -> rightRollerMotor.getTorqueCurrent().getValueAsDouble(), null);
    }

}
