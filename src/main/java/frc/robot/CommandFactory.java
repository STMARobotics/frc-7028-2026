package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootAtTargetCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetpoints;
import frc.robot.subsystems.SpindexerSubsystem;
import java.util.function.Supplier;

/**
 * Factory class for creating commands with the necessary subsystem dependencies.
 */
public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final ClimbSubsystem climbSubsystem;

  /**
   * Constructor for Commandfactory, takes in all subsystems as parameters to use in command creation methods.
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param shooterSubsystem shooter subsystem
   * @param spindexerSubsystem spindexer subsystem
   * @param feederSubsystem feeder subsystem
   * @param intakeSubsystem intake subsystem
   * @param ledSubsystem led subsystem
   * @param climbSubsystem climb subsystem
   */
  public CommandFactory(
      CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsytem intakeSubsystem,
      LEDSubsystem ledSubsystem,
      ClimbSubsystem climbSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.climbSubsystem = climbSubsystem;
  }

  /**
   * Creates a new ShootAtTargetCommand
   * 
   * @param redTarget red target to shoot at when on the red alliance
   * @param blueTarget blue target to shoot at when on the blue alliance
   * @param lookupTable lookup table to use for determining shooter setpoints based on distance to target
   * @return a new ShootAtTargetCommand
   */
  public Command createShootAtTargetCommand(
      Translation2d redTarget,
      Translation2d blueTarget,
      InterpolatingTreeMap<Double, ShooterSetpoints> lookupTable) {
    return new ShootAtTargetCommand(
        shooterSubsystem,
        feederSubsystem,
        spindexerSubsystem,
        ledSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds,
        redTarget,
        blueTarget,
        lookupTable);
  }

  /**
   * Creates a new ShootAtTargetCommand that also drives the robot using the passed translation and rotation suppliers.
   * The velocities from the suppliers are reduced by the specified velocity reduction factor to allow for more accurate
   * shooting while moving.
   * 
   * @param redTarget red target to shoot at when on the red alliance
   * @param blueTarget blue target to shoot at when on the blue alliance
   * @param lookupTable lookup table to use for determining shooter setpoints based on distance to target
   * @param translationXSupplier supplier for the robot's x translation velocity
   * @param translationYSupplier supplier for the robot's y translation velocity
   * @param omegaSupplier supplier for the robot's rotational velocity
   * @param velocityReductionFactor factor by which to reduce the velocities from the suppliers
   * @return a new ShootAtTargetCommand that also drives the robot
   */
  public Command createShootAtTargetWhileDrivingCommand(
      Translation2d redTarget,
      Translation2d blueTarget,
      InterpolatingTreeMap<Double, ShooterSetpoints> lookupTable,
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Supplier<AngularVelocity> omegaSupplier) {
    return new ShootAtTargetCommand(
        shooterSubsystem,
        feederSubsystem,
        spindexerSubsystem,
        ledSubsystem,
        () -> drivetrainSubsystem.getState().Pose,
        drivetrainSubsystem::getCurrentFieldChassisSpeeds,
        redTarget,
        blueTarget,
        lookupTable).alongWith(createDriveCommand(translationXSupplier, translationYSupplier, omegaSupplier));
  }

  /**
   * Creates a new Command to control the drivetrain using the provided translation and rotation suppliers.
   *
   * @param translationXSupplier supplier for the robot's x translation velocity
   * @param translationYSupplier supplier for the robot's y translation velocity
   * @param omegaSupplier supplier for the robot's rotational velocity
   * @return a new Command to control the drivetrain
   */
  public Command createDriveCommand(
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Supplier<AngularVelocity> omegaSupplier) {
    /* Setting up bindings for necessary control of the swerve drive platform */
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_TELEOP_VELOCITY.times(0.01))
        .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.01))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    return drivetrainSubsystem.applyRequest(
        () -> drive.withVelocityX(translationXSupplier.get())
            .withVelocityY(translationYSupplier.get())
            .withRotationalRate(omegaSupplier.get()));
  }

}
