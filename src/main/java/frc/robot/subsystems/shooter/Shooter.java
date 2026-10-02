package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private final TelemetryTable shooterTelemetry = 
        Telemetry.getTable("Shooter");

    private final TalonFX hoodMotor = new TalonFX(RobotMap.SHOOTER_HOOD_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);

    public TunableDouble hoodDesiredPositionDeg = 
        Tunables.addDouble("Hood Desired Postion (deg)", ShooterCal.HOOD_HOME_DEGREES); 

    private final TrapezoidProfile hoodTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        ShooterCal.HOOD_MAX_VELOCITY_RPS, 
        ShooterCal.HOOD_MAX_ACCELERATION_RPS_SQUARED));

    private final TalonFX leftRollerMotor = new TalonFX(RobotMap.SHOOTER_LEFT_ROLLER_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);
    private final TalonFX rightRollerMotor = new TalonFX(RobotMap.SHOOTER_RIGHT_ROLLER_MOTOR_CAN_ID, RobotMap.MAIN_CAN_BUS);

    private TunableDouble currentRollerSpeedRPM =
        Tunables.addDouble("Current Roller Speed (RPM)", 0.0); 

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
        hoodToApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
            hoodPositionToMotorPosition(ShooterCal.HOOD_HOME_DEGREES)); 
        hoodDesiredPositionDeg.set(ShooterCal.HOOD_HOME_DEGREES);
    }

    public void setDesiredHoodPosition(DoubleSupplier newPositionDegrees) {
        hoodDesiredPositionDeg.set(Math.min(Math.max(45+(70-newPositionDegrees.getAsDouble()), ShooterCal.HOOD_MIN_DEGREES), ShooterCal.HOOD_MAX_DEGREES));
    }

    public void setDesiredHoodPositionAbsolute(DoubleSupplier newPositionDegrees){
        hoodDesiredPositionDeg.set(Math.min(70.0, Math.max(45.0, newPositionDegrees.getAsDouble())));
    }

    public void addHoodOneDeg(){
        hoodDesiredPositionDeg.set(hoodDesiredPositionDeg.get() + 1);
    }

    public void subtractHoodOneDeg(){
        hoodDesiredPositionDeg.set(hoodDesiredPositionDeg.get() - 1);
    }

    public void runRollers() {
        leftRollerMotor.setThrottle(currentRollerSpeedRPM.get() / ShooterCal.ROLLERS_MAX_RPS);
    }

    public void stopRollers() {
        leftRollerMotor.setThrottle(0.0);
    }

    public void setRollerSpeedRPS(DoubleSupplier speedRPS) {
        currentRollerSpeedRPM.set(Math.min(Math.max(speedRPS.getAsDouble(), 0.0), ShooterCal.ROLLERS_MAX_RPS));
    }

    public double getRollerSpeedRPS() {
        return currentRollerSpeedRPM.get();
    }

    public boolean atDesiredHoodPosition() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodPositionToMotorPosition(hoodDesiredPositionDeg.get())) < ShooterCal.HOOD_POSITION_MARGIN;
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
            hoodPositionToMotorPosition(hoodDesiredPositionDeg.get()), 0.0);
        TrapezoidProfile.State start = new TrapezoidProfile.State(
            hoodMotor.getPosition().getValueAsDouble(), hoodMotor.getVelocity().getValueAsDouble());
        
        PositionVoltage request = new PositionVoltage(0.0).withSlot(0);
        TrapezoidProfile.State setpoint = hoodTrapezoidProfile.calculate(0.020, start, goal);
        
        request.Position = setpoint.position;
        request.Velocity = setpoint.velocity;
        
        hoodMotor.setControl(request);
    }

    public void setSpeedShuffleboard(double d){
        this.setRollerSpeedRPS(()->d);
    }

    @Override
    public void periodic() {
        controlHoodPosition();

        sendTelemetry();       
    }

    private void sendTelemetry() {
        shooterTelemetry.log("Hood Postion (deg)", ((hoodMotor.getPosition().getValueAsDouble()) / ShooterCal.HOOD_MOTOR_TO_HOOD_RATIO) * 360);
        shooterTelemetry.log("Hood Desired Postion (deg)", hoodDesiredPositionDeg);
        shooterTelemetry.log("Hood Current (A)", hoodMotor.getTorqueCurrent());
        shooterTelemetry.log("Hood Commanded Voltage (V)", hoodMotor.getMotorVoltage());

        shooterTelemetry.log("Rollers Speed (RPM)", leftRollerMotor.getVelocity().getValueAsDouble() * 60);
        shooterTelemetry.log("Left Roller Current (A)", leftRollerMotor.getTorqueCurrent().getValueAsDouble());
        shooterTelemetry.log("Right Roller Current (A)", rightRollerMotor.getTorqueCurrent().getValueAsDouble());
    }
}
