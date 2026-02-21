// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OdometryConstants;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.TeleopDriveConstants;
import frc.robot.commands.ClimbToL1Command;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commands.TuneShootingCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_TELEOP_VELOCITY.times(0.01))
      .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.01))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();

  // Create the drivetrain subsystem here instead of using TunerConstants.createDrivetrain() to set standard deviations
  // without editing generated TunerConstants file
  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      TunerConstants.DrivetrainConstants,
      0,
      OdometryConstants.STATE_STD_DEVS,
      QuestNavConstants.QUESTNAV_STD_DEVS,
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight);
  @Logged
  private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(
      drivetrain::addVisionMeasurement,
      drivetrain::resetPose,
      () -> Radians.of(drivetrain.getState().Pose.getRotation().getRadians()),
      drivetrain::getIMUYawVelocity);
  @Logged
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  @Logged
  private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  @Logged
  private final IntakeSubsytem intakeSubsystem = new IntakeSubsytem();
  @Logged
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  @Logged
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private final ControlBindings controlBindings;

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Configure and populate the auto command chooser with autos from PathPlanner
    configurePathPlannerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    autoChooser.onChange(this::setStartingPose);

    configureBindings();

    // Warmup PathPlanner to avoid Java pauses
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    // Run the boot animation
    var bootAnimation = new LEDBootAnimationCommand(ledSubsystem);
    CommandScheduler.getInstance().schedule(bootAnimation);

    // Set up default commmands
    ledSubsystem.setDefaultCommand(new DefaultLEDCommand(ledSubsystem));
  }

  private void configureBindings() {
    // Default drivetrain command for teleop control
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive.withVelocityX(controlBindings.translationX().get())
                .withVelocityY(controlBindings.translationY().get())
                .withRotationalRate(controlBindings.omega().get())));

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.seedFieldCentric().ifPresent(trigger -> trigger.onTrue(Commands.runOnce(() -> {
      Pose2d robotCurrentPose = drivetrain.getState().Pose;
      Pose2d robotNewPose = new Pose2d(robotCurrentPose.getTranslation(), drivetrain.getOperatorForwardDirection());
      localizationSubsystem.setQuestNavPose(robotNewPose);
      drivetrain.resetPose(robotNewPose);
    })));

    // Intake controls
    controlBindings.runIntake().ifPresent(trigger -> trigger.onTrue(new IntakeCommand(intakeSubsystem)));

    controlBindings.stopIntake().ifPresent(trigger -> trigger.onTrue(Commands.runOnce(() -> {
      intakeSubsystem.stop();
      spindexerSubsystem.stop();
    }, intakeSubsystem, spindexerSubsystem)));

    controlBindings.eject().ifPresent(trigger -> trigger.whileTrue(Commands.run(() -> {
      intakeSubsystem.reverseIntake();
      spindexerSubsystem.agitate();
    }, intakeSubsystem, spindexerSubsystem).finallyDo(() -> {
      intakeSubsystem.stop();
      spindexerSubsystem.stop();
    })));

    controlBindings.deployIntake().ifPresent(trigger -> trigger.onTrue(new DeployIntakeCommand(intakeSubsystem)));

    controlBindings.retractIntake().ifPresent(trigger -> trigger.onTrue(new RetractIntakeCommand(intakeSubsystem)));

    // Shooting controls
    controlBindings.manualShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(new ShootCommand(spindexerSubsystem, feederSubsystem, shooterSubsystem)));

    controlBindings.tuneShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new TuneShootingCommand(
                    spindexerSubsystem,
                    feederSubsystem,
                    shooterSubsystem,
                    ledSubsystem,
                    () -> drivetrain.getState().Pose)));

    controlBindings.autoShoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new TeleopShootCommand(
                    drivetrain,
                    shooterSubsystem,
                    feederSubsystem,
                    spindexerSubsystem,
                    ledSubsystem,
                    () -> controlBindings.translationX().get(),
                    () -> controlBindings.translationY().get(),
                    () -> drivetrain.getState().Pose,
                    ShootingConstants.TARGET_RED,
                    ShootingConstants.TARGET_BLUE,
                    ShootingConstants.SHOOTER_TARGETS_BY_DISTANCE_METERS,
                    TeleopDriveConstants.SHOOT_VELOCITY_MULTIPLIER)));

    // Climb controls
    controlBindings.climbForward()
        .ifPresent(
            trigger -> trigger
                .whileTrue(Commands.runEnd(climbSubsystem::forward, climbSubsystem::stop, climbSubsystem)));
    controlBindings.climbReverse()
        .ifPresent(
            trigger -> trigger
                .whileTrue(Commands.runEnd(climbSubsystem::reverse, climbSubsystem::stop, climbSubsystem)));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final SwerveRequest idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);
  }

  public void setStartingPose(Command auto) {
    if (auto instanceof PathPlannerAuto ppAuto) {
      localizationSubsystem.setInitialPose(ppAuto.getStartingPose());
    } else {
      localizationSubsystem.setInitialPose(new Pose2d(new Translation2d(), new Rotation2d(0)));
    }
  }

  private void configurePathPlannerCommands() {
    NamedCommands.registerCommand("ClimbToL1", new ClimbToL1Command(climbSubsystem));
    NamedCommands.registerCommand("DeployIntake", new DeployIntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("Intake", new IntakeCommand(intakeSubsystem));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  /** Populate the SysID dashboard controls with commands for system identification */
  public void populateTestModeDashboard() {
    // Drive
    SmartDashboard.putData("Drive Quasi Fwd", drivetrain.sysIdTranslationQuasiCommand(kForward));
    SmartDashboard.putData("Drive Quasi Rev", drivetrain.sysIdTranslationQuasiCommand(kReverse));
    SmartDashboard.putData("Drive Dynam Fwd", drivetrain.sysIdTranslationDynamCommand(kForward));
    SmartDashboard.putData("Drive Dynam Rev", drivetrain.sysIdTranslationDynamCommand(kReverse));

    // Drive TorqueFOC
    SmartDashboard.putData("Drive Torque Quasi Fwd", drivetrain.sysIdTranslationQuasiTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Quasi Rev", drivetrain.sysIdTranslationQuasiTorqueCommand(kReverse));
    SmartDashboard.putData("Drive Torque Dynam Fwd", drivetrain.sysIdTranslationDynamTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Dynam Rev", drivetrain.sysIdTranslationDynamTorqueCommand(kReverse));

    // Steer
    SmartDashboard.putData("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward));
    SmartDashboard.putData("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse));
    SmartDashboard.putData("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward));
    SmartDashboard.putData("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse));

    // Rotation
    SmartDashboard.putData("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward));
    SmartDashboard.putData("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse));
    SmartDashboard.putData("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward));
    SmartDashboard.putData("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse));

    // Spindexer
    SmartDashboard.putData("Spindexer Quasi Fwd", spindexerSubsystem.sysIdSpindexerQuasistaticCommand(kForward));
    SmartDashboard.putData("Spindexer Quasi Rev", spindexerSubsystem.sysIdSpindexerQuasistaticCommand(kReverse));
    SmartDashboard.putData("Spindexer Dynam Fwd", spindexerSubsystem.sysIdSpindexerDynamicCommand(kForward));
    SmartDashboard.putData("Spindexer Dynam Rev", spindexerSubsystem.sysIdSpindexerDynamicCommand(kReverse));

    // Feeder
    SmartDashboard.putData("Feeder Quasi Fwd", feederSubsystem.sysIdFeederQuasistaticCommand(kForward));
    SmartDashboard.putData("Feeder Quasi Rev", feederSubsystem.sysIdFeederQuasistaticCommand(kReverse));
    SmartDashboard.putData("Feeder Dynam Fwd", feederSubsystem.sysIdFeederDynamicCommand(kForward));
    SmartDashboard.putData("Feeder Dynam Rev", feederSubsystem.sysIdFeederDynamicCommand(kReverse));

    // Intake
    SmartDashboard.putData("Intake Deploy Quasi Fwd", intakeSubsystem.sysIdDeployQuasistaticCommand(kForward));
    SmartDashboard.putData("Intake Deploy Quasi Rev", intakeSubsystem.sysIdDeployQuasistaticCommand(kReverse));
    SmartDashboard.putData("Intake Deploy Dynam Fwd", intakeSubsystem.sysIdDeployDynamicCommand(kForward));
    SmartDashboard.putData("Intake Deploy Dynam Rev", intakeSubsystem.sysIdDeployDynamicCommand(kReverse));

    SmartDashboard.putData("Intake Roller Quasi Fwd", intakeSubsystem.sysIdRollerQuasistaticCommand(kForward));
    SmartDashboard.putData("Intake Roller Quasi Rev", intakeSubsystem.sysIdRollerQuasistaticCommand(kReverse));
    SmartDashboard.putData("Intake Roller Dynam Fwd", intakeSubsystem.sysIdRollerDynamicCommand(kForward));
    SmartDashboard.putData("Intake Roller Dynam Rev", intakeSubsystem.sysIdRollerDynamicCommand(kReverse));

    // Shooter
    SmartDashboard.putData("Shooter Flywheel Quasi Fwd", shooterSubsystem.sysIdFlywheelQuasistaticCommand(kForward));
    SmartDashboard.putData("Shooter Flywheel Quasi Rev", shooterSubsystem.sysIdFlywheelQuasistaticCommand(kReverse));
    SmartDashboard.putData("Shooter Flywheel Dynam Fwd", shooterSubsystem.sysIdFlywheelDynamicCommand(kForward));
    SmartDashboard.putData("Shooter Flywheel Dynam Rev", shooterSubsystem.sysIdFlywheelDynamicCommand(kReverse));

    SmartDashboard.putData("Shooter Yaw Quasi Fwd", shooterSubsystem.sysIdYawQuasistaticCommand(kForward));
    SmartDashboard.putData("Shooter Yaw Quasi Rev", shooterSubsystem.sysIdYawQuasistaticCommand(kReverse));
    SmartDashboard.putData("Shooter Yaw Dynam Fwd", shooterSubsystem.sysIdYawDynamicCommand(kForward));
    SmartDashboard.putData("Shooter Yaw Dynam Rev", shooterSubsystem.sysIdYawDynamicCommand(kReverse));

    SmartDashboard.putData("Shooter Pitch Quasi Fwd", shooterSubsystem.sysIdPitchQuasistaticCommand(kForward));
    SmartDashboard.putData("Shooter Pitch Quasi Rev", shooterSubsystem.sysIdPitchQuasistaticCommand(kReverse));
    SmartDashboard.putData("Shooter Pitch Dynam Fwd", shooterSubsystem.sysIdPitchDynamicCommand(kForward));
    SmartDashboard.putData("Shooter Pitch Dynam Rev", shooterSubsystem.sysIdPitchDynamicCommand(kReverse));
  }
}