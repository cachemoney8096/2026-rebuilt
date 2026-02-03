package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(RobotMap.TURRET_MOTOR_CAN_ID, RobotMap.RIO_CAN_BUS);
    private double turretDesiredPositionDeg = TurretCal.TURRET_HOME_DEGREES; 

    private final TrapezoidProfile turretTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        TurretCal.TURRET_MAX_VELOCITY_RPS, 
        TurretCal.TURRET_MAX_ACCELERATION_RPS_SQUARED));

    public Turret() {
        initTalons();
        relativeZeroTurret();
    }

    private void initTalons() {
        TalonFXConfiguration turretToApply = new TalonFXConfiguration();
        turretToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretToApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretToApply.CurrentLimits.SupplyCurrentLimit = TurretCal.TURRET_SUPPLY_CURRENT_LIMIT_AMPS;
        turretToApply.CurrentLimits.StatorCurrentLimit = TurretCal.TURRET_STATOR_SUPPLY_CURRENT_LIMIT_AMPS;
        turretToApply.CurrentLimits.StatorCurrentLimitEnable = true;
        turretToApply.Slot0.kP = TurretCal.TURRET_P;
        turretToApply.Slot0.kI = TurretCal.TURRET_I;
        turretToApply.Slot0.kD = TurretCal.TURRET_D;
        turretToApply.Slot0.kV = TurretCal.TURRET_FF;

        TalonFXConfigurator turretConfig = turretMotor.getConfigurator();
        turretConfig.apply(turretToApply);
    }

    public void relativeZeroTurret() {
        turretMotor.setPosition(
            (TurretCal.TURRET_HOME_DEGREES / 360.0) * TurretCal.TURRET_MOTOR_TO_TURRET_RATIO);
        turretDesiredPositionDeg = TurretCal.TURRET_HOME_DEGREES;
    }

    public void setDesiredTurretPosition(double newPositionDegrees) {
        turretDesiredPositionDeg = Math.max(TurretCal.TURRET_MIN_DEGREES, Math.min(TurretCal.TURRET_MAX_DEGREES, newPositionDegrees));
    }

    public double getDesiredPositionDeg() {
        return turretDesiredPositionDeg;
    }

    public boolean atDesiredTurretPosition() {
        return Math.abs(turretMotor.getPosition().getValueAsDouble() - turretPositionToMotorPosition(turretDesiredPositionDeg)) < TurretCal.TURRET_POSITION_MARGIN;
    }

    private double turretPositionToMotorPosition(double turretPositionDeg)  {
        return (turretPositionDeg / 360.0) * TurretCal.TURRET_MOTOR_TO_TURRET_RATIO;
    }

    private void controlTurretPosition() {
        TrapezoidProfile.State goal = new TrapezoidProfile.State(
            turretPositionToMotorPosition(turretDesiredPositionDeg), 0.0);
        TrapezoidProfile.State start = new TrapezoidProfile.State(
            turretMotor.getPosition().getValueAsDouble(), turretMotor.getVelocity().getValueAsDouble());
        
        PositionVoltage request = new PositionVoltage(0.0).withSlot(0);
        TrapezoidProfile.State setpoint = turretTrapezoidProfile.calculate(0.020, start, goal);
        
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        
        turretMotor.setControl(request);
    }

    @Override
    public void periodic() {
        controlTurretPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Turret Actual Position (deg.)", () -> (turretMotor.getPosition().getValueAsDouble() / 360.0) * TurretCal.TURRET_MOTOR_TO_TURRET_RATIO, null);
        builder.addDoubleProperty("Turret Desired Position (deg.)", () -> turretDesiredPositionDeg, null);

        builder.addBooleanProperty("Turret at Desired Position", this::atDesiredTurretPosition, null);

        builder.addDoubleProperty("Turret Amperage (amps)", () -> turretMotor.getTorqueCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Turret Commanded Voltage (volts)", () -> turretMotor.getMotorVoltage().getValueAsDouble(), null);
    }
}
