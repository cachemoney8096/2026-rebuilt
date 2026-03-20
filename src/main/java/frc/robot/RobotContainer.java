// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightCode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShootOnMoveUtil;
import frc.robot.utils.ShooterPitchPower;
import frc.robot.commands.*;

import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.TargetCorner;

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

  private boolean lookForNote = false;

  private PIDController visionXController = new PIDController(1.0, 0.0, 0.0);
  private PIDController visionYController = new PIDController(1.0, 0.0, 0.0);
  private Pose3d tagPoseRobotSpaceInstance;
  private Pose3d tagPoseRobotSpaceCurrent;

  /* Robot centric controller */
  private boolean isManualRobotCentric = false;

  /* Team color */
  public boolean isBlue = true;

  /* Subsystems */
  public Climb climb;
  public Indexer indexer;
  public Intake intake;
  public Lights lights;
  public Shooter shooter;
  public Turret turret;

  public String autoPathCmd = "";

  private boolean turretActive = false;
  private boolean isAiming = false;

  // photonvision testing
  PhotonCamera camera = new PhotonCamera("Up");
  PhotonCamera lowAI = new PhotonCamera("Down");
  PhotonCamera highAI = new PhotonCamera("Middle");

  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
      new Rotation3d(0, 0, 0));

  public static PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    /* Warmup PathPlanner to avoid Java pauses */
    FollowPathCommand.warmupCommand().schedule();

    /* Init subsystems */
    climb = new Climb();
    indexer = new Indexer();
    intake = new Intake();
    lights = new Lights();
    shooter = new Shooter();
    turret = new Turret();
    DoubleSupplier headingSupplier = () -> desiredHeadingDeg;
    BooleanSupplier isBluBooleanSupplier = () -> isBlue;
    Supplier<Pose2d> robotPoseSupplier = () -> drivetrain.getState().Pose;
    Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> drivetrain.getState().Speeds;
    RunCommand aimTurret = new RunCommand(() -> {
      double heading = headingSupplier.getAsDouble();
      heading = MathUtil.inputModulus(heading, 0, 360);

      Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(
          isBluBooleanSupplier.getAsBoolean(),
          robotPoseSupplier.get(),
          chassisSpeedsSupplier.get(),
          heading);

      turret.setDesiredTurretPosition(results.getSecond());

      if (shooter.atDesiredHoodPosition() && turret.atDesiredTurretPosition()) {
        lights.setLEDColor(LightCode.ALIGNED);
      } else {
        lights.setLEDColor(LightCode.ALIGNING);
      }

    }, turret, shooter);

    /* Named commands must be registered immediately */
    NamedCommands.registerCommand("DEPLOY INTAKE",
        new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.EXTENDED)));
    NamedCommands.registerCommand("RUN INTAKE", new InstantCommand(() -> intake.runRollers()));
    NamedCommands.registerCommand("STOP INTAKE", new InstantCommand(() -> intake.stopRollers()));
    NamedCommands.registerCommand("STOW INTAKE",
        new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.HOME)));
    NamedCommands.registerCommand("SHOOT INTAKE",
        new InstantCommand(() -> intake.setDesiredSlapdownPosition(IntakePosition.SHOOTING)));
    NamedCommands.registerCommand("PREP SHOOT SEQUENCE",
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setRollerSpeedRPS(this::getShooterPowerAuto)),
            new InstantCommand(() -> shooter.setDesiredHoodPositionAbsolute(this::getShooterPitch)),
            new InstantCommand(() -> shooter.runRollers())));
    NamedCommands.registerCommand("SHOOT SEQUENCE", new SequentialCommandGroup(
        new InstantCommand(() -> indexer.runKicker()),
        new InstantCommand(() -> indexer.runIndexer())));
    NamedCommands.registerCommand("STOP SHOOT SEQUENCE", new InstantCommand(() -> {
      turret.setDesiredTurretPosition(90);
      indexer.stopIndexer();
      indexer.stopKicker();
    }));
    NamedCommands.registerCommand("STOP SHOOT PREP SEQUENCE", new InstantCommand(() -> {
      shooter.stopRollers();
    }));
    NamedCommands.registerCommand("AIM TURRET", new InstantCommand(() -> {
      CommandScheduler.getInstance().schedule(aimTurret);
    }));
    NamedCommands.registerCommand("STOP TURRET", new InstantCommand(() -> {
      CommandScheduler.getInstance().cancel(aimTurret);
    }));

    ShooterPitchPower.init();

    /* Auto chooser */
    autoChooser = AutoBuilder.buildAutoChooser("LEFT NT");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /* Field centric heading controller */
    fieldCentricFacingAngle.HeadingController.setPID(6.0, 0.0001, 0.02);

    isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    zeroRobot();

    /* Configure controller bindings */
    configureDriverBindings();
    configureOperatorBindings();
    // configureDebugBindings();

    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    /* Shuffleboard */
    Shuffleboard.getTab("Subsystems").add("RobotContainer", this);
    Shuffleboard.getTab("Subsystems").add("Turret", turret);
    Shuffleboard.getTab("Subsystems").add("Intake", intake);
    Shuffleboard.getTab("Subsystems").add("Climb", climb);
    Shuffleboard.getTab("Subsystems").add("Spindexer/Kicker", indexer);
    Shuffleboard.getTab("Subsystems").add("Shooter", shooter);
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
    if (lookForNote) {
      double targetYaw = 0.0;
      var results = camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
          targetYaw = result.getTargets().get(0).getYaw();
        }
      }
      double targetYawRad = Math.toRadians(targetYaw);
      double kP = 7;
      double output = -kP * targetYawRad;

      output = MathUtil.clamp(output, -MaxAngularRate, MaxAngularRate);
      desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
      if (Math.abs(targetYawRad) > Math.toRadians(1.0)) {
        return drive
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(output);
      } else {
        return drive
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationVelocity);
      }
    } else if (isManualRobotCentric) {
      /* Is robot centric */
      return robotCentric
          .withVelocityX(xVelocity);
          // .withVelocityY(yVelocity)
          // .withRotationalRate(rotationVelocity);
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

    /* Cardinals */ // TODO make these correct
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

    driverController.start().onTrue(new InstantCommand(() -> zeroRobot()));

    driverController.rightTrigger().whileTrue(
        new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setRollerSpeedRPS(this::getShooterPower)),
                new InstantCommand(() -> shooter.setDesiredHoodPositionAbsolute(this::getShooterPitch)),
                new InstantCommand(() -> shooter.runRollers())),
            new WaitCommand(0.5),
            new RepeatCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> shooter.setRollerSpeedRPS(this::getShooterPower)),
                    new InstantCommand(() -> shooter.setDesiredHoodPositionAbsolute(this::getShooterPitch)),
                    new InstantCommand(() -> shooter.runRollers()),
                    new InstantCommand(() -> indexer.runKicker()),
                    new InstantCommand(() -> indexer.runIndexer())))
                .finallyDo(
                    (b) -> {
                      shooter.stopRollers();
                      indexer.stopIndexer();
                      indexer.stopKicker();
                      intake.stopRollers();
                    })));

    driverController.leftBumper().onTrue(
        new InstantCommand(() -> {
          if (intake.slapdownDesiredPosition == IntakePosition.HOME) {
            intake.setDesiredSlapdownPosition(IntakePosition.EXTENDED);
          } else {
            intake.setDesiredSlapdownPosition(IntakePosition.HOME);
          }
        }));

    driverController.leftTrigger().onTrue(
        new InstantCommand(() -> intake.runRollers()));

    driverController.leftTrigger().onFalse(
        new InstantCommand(() -> intake.stopRollers()));

    DoubleSupplier headingSupplier = () -> desiredHeadingDeg; // yes i know this is duplicate code i don't care for now
    BooleanSupplier isBlueBooleanSupplier = () -> isBlue;
    Supplier<Pose2d> robotPoseSupplier = () -> drivetrain.getState().Pose;
    Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> drivetrain.getState().Speeds;
    RunCommand aimTurret = new RunCommand(() -> {
      double heading = headingSupplier.getAsDouble();
      heading = MathUtil.inputModulus(heading, 0, 360);

      Pair<Double, Double> results = ShootOnMoveUtil.calcTurret(
          isBlueBooleanSupplier.getAsBoolean(),
          robotPoseSupplier.get(),
          chassisSpeedsSupplier.get(),
          heading);

      turret.setDesiredTurretPosition(results.getSecond());

      if (shooter.atDesiredHoodPosition() && turret.atDesiredTurretPosition()) {
        lights.setLEDColor(LightCode.ALIGNED);
      } else {
        lights.setLEDColor(LightCode.ALIGNING);
      }

    }, turret, shooter);

    InstantCommand aim = new InstantCommand(() -> {
      Translation2d target = new Translation2d();
      if (isBlue) {
        target = new Translation2d(5.0, 4.0); // TODO this may be wrong
      } else {
        target = new Translation2d(14.4, 4.0);
      }
      Translation2d botPose = drivetrain.getState().Pose.getTranslation();
      Translation2d difference = target.minus(botPose);
      double angle = Math.atan2(difference.getY(), difference.getX());
      desiredHeadingDeg = MathUtil.inputModulus(Math.toDegrees(angle), 0.0, 360.0);
    });

    driverController.rightBumper().onTrue(
        aim);

    // driverController.povRight().onTrue(
    // new GoHomeSequence(turret, intake, climb, shooter, indexer, lights));

    // driverController.povLeft().onTrue(
    // new FeedSequence(turret, () ->
    // drivetrain.getState().Pose.getRotation().getDegrees(), isBlue,
    // driverController.povLeft()));
    driverController.povLeft().onTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> {
      
        var lowResults = lowAI.getAllUnreadResults();
        if (!lowResults.isEmpty()) {
          var result = lowResults.get(lowResults.size() - 1);
          if (result.hasTargets()) {
            desiredHeadingDeg -= result.getTargets().get(0).getYaw();
          }
        } else {
          var highResults = highAI.getAllUnreadResults();
          if (!highResults.isEmpty()) {
            var result = highResults.get(highResults.size() - 1);
            if (result.hasTargets()) {
              desiredHeadingDeg -= result.getTargets().get(0).getYaw();
              List<TargetCorner> l = result.getTargets().get(0).detectedCorners;
            }
          }
        }

    }),
    new WaitCommand(0.25),
    new InstantCommand(()->{
      if (isManualRobotCentric) {
        isManualRobotCentric = false;
      } else {
        isManualRobotCentric = true;
      }
    })
      )
    );
  }

  private void configureOperatorBindings() {
    // operatorController.a().onTrue(new InstantCommand(() -> isManualRobotCentric =
    // !isManualRobotCentric));

    // operatorController.start().onTrue(
    //     new InstantCommand(() -> {
    //       var driveState = drivetrain.getState();
    //       double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    //       var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-turret"); // TODO megatag1
    //                                                                                                     // check
    //       // var llMeasurement =
    //       // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");
    //       if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
    //         // drivetrain.addVisionMeasurement(llMeasurement.pose,
    //         // llMeasurement.timestampSeconds);
    //         drivetrain.resetPose(new Pose2d(llMeasurement.pose.getTranslation(),
    //             Rotation2d.fromDegrees(isBlue ? 0.0
    //                 : 180.0)));
    //         desiredHeadingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
    //       }
    //     }));

    operatorController.b().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> {
              shooter.stopRollers();
              indexer.reverseIndexer();
              indexer.stopKicker();
            }),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
              indexer.stopIndexer();
            })));

    // operatorController.povUp().onTrue(
    // new InstantCommand(() -> shooter.setRollerSpeedRPS(() ->
    // shooter.currentRollerSpeedRPM + 100)));

    // operatorController.povDown().onTrue(
    // new InstantCommand(() -> shooter.setRollerSpeedRPS(() ->
    // shooter.currentRollerSpeedRPM - 100)));

    operatorController.a().onTrue(
        new InstantCommand(() -> shooter.addHoodOneDeg()));

    operatorController.y().onTrue(
        new InstantCommand(() -> shooter.subtractHoodOneDeg()));

    operatorController.leftTrigger().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.reverseRollers()),
            new WaitCommand(0.5),
            new InstantCommand(() -> intake.stopRollers())));

    // operatorController.rightBumper().whileTrue(
    //     new RepeatCommand(
    //         new SequentialCommandGroup(
    //             new InstantCommand(() -> shooter.setRollerSpeedRPS(this::getShooterPowerRelativeDist)),
    //             new InstantCommand(() -> shooter.setDesiredHoodPositionAbsolute(this::getShooterPitchRelativeDist)),
    //             new InstantCommand(() -> shooter.runRollers()),
    //             new InstantCommand(() -> indexer.runKicker()),
    //             new InstantCommand(() -> indexer.runIndexer())))
    //         .finallyDo(
    //             (b) -> {
    //               shooter.stopRollers();
    //               indexer.stopIndexer();
    //               indexer.stopKicker();
    //               intake.stopRollers();
    //             }));

    // operatorController.rightBumper().onTrue(
    //     new InstantCommand(() -> {
    //       desiredHeadingDeg -= LimelightHelpers.getTX("limelight-turret");
    //     }));

    operatorController.rightTrigger().whileTrue(
        new RepeatCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setRollerSpeedRPS(() -> 5000)),
                new InstantCommand(() -> shooter.setDesiredHoodPosition(() -> 45)),
                new InstantCommand(() -> shooter.runRollers()),
                new InstantCommand(() -> indexer.runKicker()),
                new WaitCommand(0.5),
                new InstantCommand(() -> indexer.runIndexer())))
            .finallyDo(
                (b) -> {
                  shooter.stopRollers();
                  indexer.stopIndexer();
                  indexer.stopKicker();
                  shooter.setDesiredHoodPosition(() -> 70);
                }));

    operatorController.leftBumper().whileTrue(
        new RepeatCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setRollerSpeedRPS(() -> 4800)),
                new InstantCommand(() -> shooter.setDesiredHoodPosition(() -> 57)),
                new InstantCommand(() -> shooter.runRollers()),
                new InstantCommand(() -> indexer.runKicker()),
                new WaitCommand(0.5),
                new InstantCommand(() -> indexer.runIndexer())))
            .finallyDo(
                (b) -> {
                  shooter.stopRollers();
                  indexer.stopIndexer();
                  indexer.stopKicker();
                  shooter.setDesiredHoodPosition(() -> 70);
                }));

    operatorController.povDown().onTrue(
        new InstantCommand(() -> {
          var result = camera.getLatestResult();
          if (result.hasTargets()) {
            try {
              AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
                  Filesystem.getDeployDirectory().toPath().resolve("2026-rebuilt-welded.json"));
              Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                  result.getBestTarget().getBestCameraToTarget(),
                  fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get(),
                  new Transform3d(0.5, 0.0, 0.5, new Rotation3d(0, Math.toRadians(20), 0)));
              Pose2d rp = robotPose.toPose2d();
              drivetrain.resetPose(rp);
              desiredHeadingDeg = rp.getRotation().getDegrees();
            } catch (Exception e) {

            }
          }
        }));
    operatorController.povUp().onTrue(
        // new InstantCommand(() -> {
        // var results = camera.getAllUnreadResults();
        // if (!results.isEmpty()) {
        // var result = results.get(results.size() - 1);
        // if (result.hasTargets()) {
        // desiredHeadingDeg -= result.getTargets().get(0).getYaw();
        // //
        // System.out.println(result.getTargets().get(0).bestCameraToTarget.getTranslation().getNorm());
        // }
        // }
        // })
        new InstantCommand(() -> {
          var lowResults = lowAI.getAllUnreadResults();
          if (!lowResults.isEmpty()) {
            var result = lowResults.get(lowResults.size() - 1);
            if (result.hasTargets()) {
              desiredHeadingDeg -= result.getTargets().get(0).getYaw();
              // System.out.println(result.getTargets().get(0).bestCameraToTarget.getTranslation().getNorm());
            }
          } else {
            var highResults = highAI.getAllUnreadResults();
            if (!highResults.isEmpty()) {
              var result = highResults.get(highResults.size() - 1);
              if (result.hasTargets()) {
                desiredHeadingDeg -= result.getTargets().get(0).getYaw();
                // System.out.println(result.getTargets().get(0).bestCameraToTarget.getTranslation().getNorm());
              }
            }
          }
        }));
  }

  public Translation2d getTarget() {
    Translation2d target = new Translation2d();
    if (isBlue) {
      target = new Translation2d(4.0, 4.0); // TODO this may be wrong
    } else {
      target = new Translation2d(12.4, 4.0);
    }
    return target;
  }

  public double getShooterPower() {
    Translation2d target = getTarget();
    double dist = drivetrain.getState().Pose.getTranslation().getDistance(target);
    return ShooterPitchPower.getPower(dist);
  }

  public double getShooterPowerAuto() {
    Translation2d target = getTarget();
    double dist = drivetrain.getState().Pose.getTranslation().getDistance(target);
    return ShooterPitchPower.getPower(dist);
  }

  public double getShooterPitch() {
    Translation2d target = getTarget();
    double dist = drivetrain.getState().Pose.getTranslation().getDistance(target);
    return ShooterPitchPower.getPitch(dist);
  }

  // private double getRelativeDistanceToTarget() {
  //   double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight-turret");
  //   double x = pose[0];
  //   double z = pose[2];
  //   return Math.sqrt(x * x + z * z);
  // }

  // public double getShooterPowerRelativeDist() {
  //   return ShooterPitchPower.getPower(getRelativeDistanceToTarget());
  // }

  // public double getShooterPitchRelativeDist() {
  //   return ShooterPitchPower.getPitch(getRelativeDistanceToTarget());
  // }

  public double getDistance() {
    Translation2d target = getTarget();
    double dist = drivetrain.getState().Pose.getTranslation().getDistance(target);
    return dist;
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
    // builder.addDoubleProperty("limelight tx", () -> LimelightHelpers.getTX("limelight-turret"), null);
    builder.addDoubleProperty("turret calc heading",
        () -> ShootOnMoveUtil
            .calcTurret(true, drivetrain.getState().Pose, drivetrain.getState().Speeds, desiredHeadingDeg).getSecond(),
        null);
    builder.addDoubleProperty("distance to target", this::getDistance, null);
    // builder.addDoubleProperty("relative distance to target", this::getRelativeDistanceToTarget, null);
    // builder.addIntegerProperty("pv detections",
    // ()->camera.getAllUnreadResults().size(), null);
  }
}