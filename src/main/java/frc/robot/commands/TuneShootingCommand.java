package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterConstants.HUB_POSE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import java.util.function.Supplier;

/**
 * Testing command for tuning shots. This is not intended to be used in-game. This command reads
 * from the NetworkTables to get shooter pitch, velocity, and yaw.
 */
public class TuneShootingCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final Supplier<Pose2d> poseSupplier;

  private final DoubleEntry pitchSubscriber;
  private final DoubleEntry velocitySubscriber;
  private final DoubleEntry yawSubscriber;
  private final DoublePublisher distancePublisher;

  private boolean shooting = false;
  private Translation2d hubTranslation;

  private MutAngle pitchMeasure = Radian.mutable(0);
  private MutAngle yawMeasure = Radian.mutable(0);
  private MutAngularVelocity velocityMeasure = RotationsPerSecond.mutable(0);

  public TuneShootingCommand(
      ShooterSubsystem shooterSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      FeederSubsystem feederSubsystem,
      Supplier<Pose2d> poseSupplier) {

    this.shooterSubsystem = shooterSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.poseSupplier = poseSupplier;

    var nt = NetworkTableInstance.getDefault();
    var table = nt.getTable("Tune Shoot");
    distancePublisher = table.getDoubleTopic("Speaker Distance").publish();
    pitchSubscriber = table.getDoubleTopic("Pitch (degrees)").getEntry(0.0);
    pitchSubscriber.set(0.0);
    velocitySubscriber = table.getDoubleTopic("Velocity Top (RPS)").getEntry(0.0);
    velocitySubscriber.set(0.0);
    yawSubscriber = table.getDoubleTopic("Yaw (Degrees)").getEntry(180.0);
    yawSubscriber.set(180.0);

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooting = false;
    hubTranslation = HUB_POSE.getTranslation();
  }

  @Override
  public void execute() {
    var turretDistanceToHub = ShooterSubsystem.getTurretTranslation(poseSupplier.get()).getDistance(hubTranslation);
    distancePublisher.accept(turretDistanceToHub);

    shooterSubsystem.setPitchAngle(pitchMeasure.mut_replace(pitchSubscriber.get(0.0), Degrees));
    shooterSubsystem.setYawAngle(yawMeasure.mut_replace(yawSubscriber.get(180.0), Degrees));
    shooterSubsystem.setFlywheelSpeed(velocityMeasure.mut_replace(velocitySubscriber.get(0.0), RotationsPerSecond));
    if (shooting || shooterSubsystem.isReadyToShoot()) {
      spindexerSubsystem.feedShooter();
      feederSubsystem.feedShooter();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
  }
}