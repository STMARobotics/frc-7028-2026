// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_TELEOP_VELOCITY.times(0.05))
      .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.05))
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
      drivetrain::getIMUYaw,
      drivetrain::getIMUYawVelocity);
  @Logged
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  @Logged
  private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();
  @Logged
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  @Logged
  private final IntakeSubsytem intakeSubsystem = new IntakeSubsytem();
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

    populateSysIdDashboard();

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

    controlBindings.shoot()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new TeleopShootCommand(
                    drivetrain,
                    shooterSubsystem,
                    transferSubsystem,
                    spindexerSubsystem,
                    ledSubsystem,
                    controlBindings.translationX(),
                    controlBindings.translationY(),
                    () -> drivetrain.getState().Pose,
                    ShootingConstants.TARGET_RED,
                    ShootingConstants.TARGET_BLUE,
                    ShootingConstants.SHOOTER_TARGETS_BY_DISTANCE_METERS,
                    TeleopDriveConstants.SHOOT_VELOCITY_MULTIPLIER)));

    controlBindings.expandClimb()
        .ifPresent(
            trigger -> trigger
                .whileTrue(Commands.runEnd(climbSubsystem::expand, climbSubsystem::stop, climbSubsystem)));
    controlBindings.contractClimb()
        .ifPresent(
            trigger -> trigger
                .whileTrue(Commands.runEnd(climbSubsystem::contract, climbSubsystem::stop, climbSubsystem)));

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
    NamedCommands.registerCommand("DeployInstake", new DeployIntakeCommand(intakeSubsystem));
    NamedCommands.registerCommand("Intake", new IntakeCommand(intakeSubsystem, spindexerSubsystem));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  /** Populate the SysID dashboard controls with commands for system identification */
  public void populateSysIdDashboard() {
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

    // Transfer
    SmartDashboard.putData("Transfer Quasi Fwd", transferSubsystem.sysIdTransferQuasistaticCommand(kForward));
    SmartDashboard.putData("Transfer Quasi Rev", transferSubsystem.sysIdTransferQuasistaticCommand(kReverse));
    SmartDashboard.putData("Transfer Dynam Fwd", transferSubsystem.sysIdTransferDynamicCommand(kForward));
    SmartDashboard.putData("Transfer Dynam Rev", transferSubsystem.sysIdTransferDynamicCommand(kReverse));
  }
}