// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {
  /* Controllers */
  private final CommandXboxController driverController = new CommandXboxController(
      Constants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(
      Constants.OPERATOR_CONTROLLER_PORT);

  private final double joystickDeadband = 0.05;

  private Supplier<Boolean> rotationalJoystickInput = () -> {
    return Math.abs(MathUtil.applyDeadband(driverController.getRightX(), joystickDeadband)) > 0.0;
  };

  private Supplier<Boolean> positionalJoystickInput = () -> {
    return Math.abs(MathUtil.applyDeadband(driverController.getLeftX(), joystickDeadband)) > 0.0
        || Math.abs(MathUtil.applyDeadband(driverController.getLeftY(), joystickDeadband)) > 0.0;
  };

  private Supplier<Boolean> joystickInput = () -> {
    return rotationalJoystickInput.get() || positionalJoystickInput.get();
  };

  /* Auto chooser */
  private final SendableChooser<Command> autoChooser;

  /* Drive control */
  private Supplier<SwerveRequest> driveController = this::driveCommand;
  public double desiredHeadingDeg = 0.0;
  private double visionBasedX = 0.0;
  private double visionBasedY = 0.0;

  private Consumer<Double> headingSetter = (Double d) -> {
    this.desiredHeadingDeg = d;
  };

  private BiConsumer<Double, Double> visionVelocitySetter = (Double x, Double y) -> {
    this.visionBasedX = x.doubleValue();
    this.visionBasedY = y.doubleValue();
  };

  /* Drivetrain config */
  private final double driveDeadband = 0.1;

  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * driveDeadband)
      .withRotationalDeadband(MaxAngularRate * driveDeadband)
      .withDriveRequestType(
          DriveRequestType.OpenLoopVoltage); /* Use open-loop control for drive motors */

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * driveDeadband)
      .withRotationalDeadband(MaxAngularRate * driveDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * driveDeadband)
      .withRotationalDeadband(MaxAngularRate * driveDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Vision variables */
  private double visionOffsetX = 0.0;
  private double visionOffsetY = 0.0;

  private PIDController visionXController = new PIDController(1.0, 0.0, 0.0); 
  private PIDController visionYController = new PIDController(1.0, 0.0, 0.0); 
  private Pose3d tagPoseRobotSpaceInstance;
  private Pose3d tagPoseRobotSpaceCurrent;

  /* Robot centric controller */
  private boolean isManualRobotCentric = false;

  /* Team color */
  public boolean isBlue = true;

  /* Subsystems */
  

  public String autoPathCmd = "";

  /* Prep states */
  

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    /* Warmup PathPlanner to avoid Java pauses */
    FollowPathCommand.warmupCommand().schedule();

    /* Init subsystems */
    

    /* Named commands must be registered immediately */
    

    /* Auto chooser */
    autoChooser = AutoBuilder.buildAutoChooser(""); //TODO default auto name
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /* Field centric heading controller */
    fieldCentricFacingAngle.HeadingController.setPID(6.7, 0.0001, 0.02); //TODO update drive pid

    isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue; //TODO robot.java stuff

    zeroRobot();

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();
    // configureDebugBindings(); 

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    /* Shuffleboard */
    Shuffleboard.getTab("Subsystems").add("RobotContainer", this);

    SmartDashboard.putData(autoChooser);
  }

  private void zeroRobot() {
    drivetrain.seedFieldCentric();

    this.desiredHeadingDeg = isBlue ? 0.0 : 180.0;

    drivetrain.resetPose(new Pose2d(
        drivetrain.getState().Pose.getX(),
        drivetrain.getState().Pose.getY(),
        Rotation2d.fromDegrees(isBlue ? 0.0 : 180)));
  }

  private SwerveRequest driveCommand() {
    double visionX = MathUtil.applyDeadband(visionBasedX, joystickDeadband);
    double visionY = MathUtil.applyDeadband(visionBasedY, joystickDeadband);

    double xVelocity;
    double yVelocity;

    if (Math.abs(visionX) > 0.0 || Math.abs(visionY) > 0.0) {
      /* If vision is present, set velocities to vision */
      xVelocity = visionX;
      yVelocity = visionY;
    } else {
      /* Else set velocity based on left stick */
      xVelocity = -driverController.getLeftY() * MaxSpeed;
      yVelocity = -driverController.getLeftX() * MaxSpeed;
    }

    /* Rotational veloity based on right stick */
    double rotationVelocity = -driverController.getRightX() * MaxAngularRate;

    if (isManualRobotCentric) {
      /* Is robot centric */
      return robotCentric
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withRotationalRate(rotationVelocity);
    } else if (rotationalJoystickInput.get()) {
      /* If rotation stick is being used */
      desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();

      return drive
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withRotationalRate(rotationVelocity);
    } else {
      return fieldCentricFacingAngle
          .withVelocityX(xVelocity)
          .withVelocityY(yVelocity)
          .withTargetDirection(
              Rotation2d.fromDegrees(isBlue ? desiredHeadingDeg : (desiredHeadingDeg + 180)));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    /* Set drivetrain control command */
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(driveController));

    Command rumbleBriefly = new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            }),
        new WaitCommand(0.25),
        new InstantCommand(
            () -> {
              driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }));

    /* Cardinals */
    driverController
        .a()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 180.0 : 0.0));

    driverController
        .b()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 270.0 : 90.0));

    driverController
        .x()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 90.0 : 270.0));

    driverController
        .y()
        .onTrue(new InstantCommand(() -> this.desiredHeadingDeg = isBlue ? 0.0 : 180.0));
  }

  private void configureOperatorBindings() {
    /* Toggle robot centric */
    operatorController.b().onTrue(new InstantCommand(() -> isManualRobotCentric = !isManualRobotCentric));
  }

  private void configureDebugBindings() {
    /* Tests vision */
    driverController.povUp().onTrue(
        /* Vision command */
        new SequentialCommandGroup(
            new InstantCommand(() -> {
              visionXController.reset();
              visionYController.reset();

              /* According to the Limelight, Y rotation is yaw */
              final Rotation3d tagRot = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME)
                  .getRotation();
              this.desiredHeadingDeg -= Math.toDegrees(tagRot.getY());
            }),
            new WaitUntilCommand(
                () -> Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - desiredHeadingDeg) < 10.0),
            new InstantCommand(() ->
            /* According to the Limelight, XZ plane is floor */
            tagPoseRobotSpaceInstance = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME)),
            new WaitUntilCommand(() -> {
              if (tagPoseRobotSpaceInstance.getZ() == 0.0 && tagPoseRobotSpaceInstance.getX() == 0.0) {
                /* If no inital April Tag is seen, cancel command */
                return true;
              }

              final Pose3d tagPoseRobotSpace = LimelightHelpers
                  .getTargetPose3d_RobotSpace(Constants.LIMELIGHT_FRONT_NAME);

              if (tagPoseRobotSpaceCurrent.getZ() != 0.0 && tagPoseRobotSpace.getY() != 0.0) {
                /* If April Tag is still in sight, update instance pose */
                tagPoseRobotSpaceInstance = tagPoseRobotSpaceCurrent;
              }

              /* Converts from Limelight Pose3d to WPI conventional Pose2d */
              Pose2d tagPoseRobotSpaceWPIConvention = new Pose2d(
                  tagPoseRobotSpaceInstance.getZ() - this.visionOffsetX,
                  -tagPoseRobotSpaceInstance.getX() + this.visionOffsetY,
                  Rotation2d.fromDegrees(tagPoseRobotSpaceInstance.getRotation().getY()));

              /* Get fieldspace poses */
              final Pose2d robotPoseFieldSpace = drivetrain.getState().Pose;
              final Pose2d targetPoseFieldSpace = robotPoseFieldSpace
                  .plus(new Transform2d(new Pose2d(), tagPoseRobotSpaceWPIConvention));

              double xOutput = visionXController.calculate(
                  robotPoseFieldSpace.getX(), targetPoseFieldSpace.getX());
              double yOutput = visionYController.calculate(
                  robotPoseFieldSpace.getY(), targetPoseFieldSpace.getY());

              xOutput = MathUtil.clamp(xOutput, -1.5, 1.5);
              yOutput = MathUtil.clamp(yOutput, -1.5, 1.5);

              if (this.isBlue) {
                visionVelocitySetter.accept(xOutput, yOutput);
              } else {
                visionVelocitySetter.accept(-xOutput, -yOutput);
              }

              return (Math.abs(visionXController.getPositionError()) < 0.01
                  && Math.abs(visionYController.getPositionError()) < 0.01);

            })).until(
                /* Break vision if joystick input */
                () -> joystickInput.get())
            .finallyDo(() -> visionVelocitySetter.accept(0.0, 0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("robot centric enabled", () -> isManualRobotCentric, null);
    builder.addDoubleProperty("pose heading", () -> drivetrain.getState().Pose.getRotation().getDegrees(), null);
    builder.addStringProperty("path CMD", () -> autoPathCmd, null);
    builder.addDoubleProperty("odometry X", () -> drivetrain.getState().Pose.getX(), null);
    builder.addDoubleProperty("odometry Y", () -> drivetrain.getState().Pose.getY(), null);
    builder.addDoubleProperty(
        "odometry rotation deg", () -> drivetrain.getState().Pose.getRotation().getDegrees(), null);
    builder.addDoubleProperty("desired heading deg", () -> this.desiredHeadingDeg, null);
    builder.addDoubleProperty(
        "gyro rotation deg", () -> drivetrain.getPigeon2().getRotation2d().getDegrees() % 360, null);
    builder.addStringProperty(
        "Current selected auto", () -> this.getAutonomousCommand().getName(), null);
    builder.addBooleanProperty("is blue", () -> isBlue, null);
  }
}