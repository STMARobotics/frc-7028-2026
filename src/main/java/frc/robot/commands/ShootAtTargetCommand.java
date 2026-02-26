package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSetpoints;
import frc.robot.subsystems.SpindexerSubsystem;
import java.util.function.Supplier;

/**
 * This command automatically shoots at a target until interrupted. It takes current robot velocity into account to
 * "shoot on the move". This command must be interrupted to stop, most likely by the driver releasing a button or by an
 * autonomous routine setting an end condition.
 */
public class ShootAtTargetCommand extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final LEDSubsystem ledSubsystem;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedSupplier;
  private final Translation2d targetRed;
  private final Translation2d targetBlue;
  private final InterpolatingTreeMap<Double, ShooterSetpoints> lookupTable;

  // Reusable object to prevent reallocation (to reduce memory pressure)
  private final MutAngle turretYawTarget = Rotations.mutable(0);

  private Translation2d targetTranslation;
  private boolean isShooting;

  /**
   * Constructor
   * 
   * @param shooterSubsystem shooter subsystem
   * @param feederSubsystem feeder subsystem
   * @param spindexerSubsystem spindexer subsystem
   * @param ledSubsystem led subsystem
   * @param robotPoseSupplier robot pose supplier
   * @param robotSpeedSupplier robot speed supplier
   * @param targetRed target red to shoot at if on red alliance
   * @param targetBlue target blue to shoot at if on blue alliance
   * @param lookupTable lookup table mapping distance to shooter setpoints
   */
  public ShootAtTargetCommand(
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotSpeedSupplier,
      Translation2d targetRed,
      Translation2d targetBlue,
      InterpolatingTreeMap<Double, ShooterSetpoints> lookupTable) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedSupplier = robotSpeedSupplier;
    this.targetRed = targetRed;
    this.targetBlue = targetBlue;
    this.lookupTable = lookupTable;

    addRequirements(shooterSubsystem, feederSubsystem, spindexerSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    // Set the target based on the current alliance color, since it's different for each match
    var alliance = DriverStation.getAlliance();
    targetTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? targetBlue : targetRed;
    // Reset states
    ledSubsystem.off();
    isShooting = false;
  }

  @Override
  public void execute() {
    var robotPose = robotPoseSupplier.get();
    var currentChassisSpeeds = robotSpeedSupplier.get();

    // Translation of the shooter on the field (used for distance/angle calculations).
    var shooterTranslation = ShooterSubsystem.getShooterTranslation(robotPose);

    // 1. Compute the velocity of the fuel at the shooter's location on the field.
    //
    // - When the robot (or turret) is moving, the fuel inherits that motion at the moment it leaves the flywheel. To
    // predict where the fuel will land we must account for that initial velocity.
    // - There are two contributions:
    // A) vRobot: the robot's linear translation velocity
    // B) vTanTotal: tangential velocity at the shooter from angular rotation (robot yaw + turret yaw velocity)
    // acting on the shooter
    //
    // The sum of these gives the initial velocity of the fuel relative to the field, which we use to compute how far
    // the projectile will shift during its flight.
    var vRobot = new Translation2d(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);

    // Total angular rate that affects the shooter (robot yaw + turret yaw rate).
    var omegaRobot = currentChassisSpeeds.omegaRadiansPerSecond;
    var turretVelocity = shooterSubsystem.getYawVelocity().in(RadiansPerSecond);
    var totalOmega = omegaRobot + turretVelocity;

    // Convert rotation about the robot center into a tangential linear velocity at the shooter.
    // This creates a tangential velocity at the shooter mount because the shooter is offset from the robot center.
    var robotToShooter = shooterTranslation.minus(robotPose.getTranslation());
    var vTanTotal = new Translation2d(-totalOmega * robotToShooter.getY(), totalOmega * robotToShooter.getX());

    // Effective shooter velocity = robot linear + tangential from rotation.
    var effectiveShooterVelocity = vRobot.plus(vTanTotal);

    // 2. Iteratively solve for the aim point that compensates for time-of-flight.
    //
    // Rationale:
    // - The flight time depends on the flywheel speed and pitch, which are chosen based on the distance to the target.
    // But the distance depends on the offset caused by the robot's motion during flight — a circular dependency.
    // - We therefore perform a few iterations: estimate distance -> choose shooter setpoints -> compute flight time ->
    // predict where the target will appear after that flight -> repeat.
    var predictedTargetTranslation = targetTranslation;
    Translation2d targetPredictedOffset = null;

    var actualTargetDistance = shooterTranslation.getDistance(targetTranslation);
    ShooterSetpoints shootingSettings;
    for (int i = 0; i < 4; i++) {
      // Distance from the shooter to the currently predicted target point.
      // This distance drives the lookup table which selects pitch and flywheel speed.
      var predictedTargetDistance = shooterTranslation.getDistance(predictedTargetTranslation);
      shootingSettings = lookupTable.get(predictedTargetDistance);

      var timeUntilScored = 0.0;
      var rps = shootingSettings.targetFlywheelSpeed().in(RotationsPerSecond);
      var pitch = shootingSettings.targetPitch();

      if (Math.abs(rps) > 1e-3) { // Avoid divide-by-zero if flywheel is stopped.
        // Approximate time-of-flight using the horizontal component of the fuel's exit velocity. Use actual distance to
        // target (not predicted) for flight time to avoid divergence when moving away from the target.
        //
        // - For compensating lateral translation caused by robot motion, the horizontal (field-parallel) component of
        // the fuel velocity determines how far the fuel travels horizontally during flight. How high the ball arcs
        // doesn't matter, it only matters how fast it moves horizontally.
        // - We approximate flight time as: time = horizontal_distance / (exit_speed * cos(pitch)).
        double fuelExitVelocity = FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER * rps;
        timeUntilScored = actualTargetDistance
            / (fuelExitVelocity * Math.cos(ShooterSubsystem.getFuelPitch(pitch).in(Radians)));
      }

      // Compute how far the fuel initial velocity (from robot motion) will shift the target point.
      // We subtract this offset from the real target position to produce a "lead" point to aim at.
      //
      // - If the robot is moving forward, the fuel will land further forward relative to a stationary shot. To hit the
      // desired (static) target we need to aim backwards relative to the instantaneous target position by the amount
      // the fuel will be carried during flight.
      targetPredictedOffset = effectiveShooterVelocity.times(timeUntilScored);
      predictedTargetTranslation = targetTranslation.minus(targetPredictedOffset);
    }

    SmartDashboard.putNumber("Shooting offset X", targetPredictedOffset.getX());
    SmartDashboard.putNumber("Shooting offset Y", targetPredictedOffset.getY());

    // After iterating, resolve final shooter setpoints for the converged predicted target.
    shootingSettings = lookupTable.get(predictedTargetTranslation.getDistance(shooterTranslation));

    // Compute the turret yaw required to point at the compensated aim point.
    var angleToTarget = predictedTargetTranslation.minus(shooterTranslation).getAngle();
    turretYawTarget.mut_replace(angleToTarget.minus(robotPose.getRotation()).getRotations(), Rotations);

    // Command the shooter pitch, yaw, and flywheel speed from the lookup table.
    // shooterSubsystem.setYawAngle(turretYawTarget); // TODO restore when yaw is implemented
    shooterSubsystem.setPitchAngle(shootingSettings.targetPitch());
    shooterSubsystem.setFlywheelSpeed(shootingSettings.targetFlywheelSpeed());

    // Check if all of the conditions are met to be ready to shoot
    var isFlywheelReady = shooterSubsystem.isFlywheelAtSpeed();
    var isPitchReady = shooterSubsystem.isPitchAtSetpoint();
    var isYawReady = true; // shooterSubsystem.isYawAtSetpoint(); TODO restore when yaw is implemented
    if (isShooting || isFlywheelReady && isPitchReady && isYawReady) {
      // All conditions met. Continuously feeding until the command is interrupted.
      spindexerSubsystem.runSpindexer(shootingSettings.spindexerVelocity());
      feederSubsystem.runFeeder(shootingSettings.feederVelocity());
      ledSubsystem.runPattern(LEDPattern.solid(kGreen));
      isShooting = true;
    } else {
      // Not ready. Don't feed and display readiness using segmented LED lights.
      ledSubsystem
          .runPattern(LEDSubsystem.ledSegments(kBlue, () -> isFlywheelReady, () -> isPitchReady, () -> isYawReady));
      spindexerSubsystem.stop();
      feederSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
    feederSubsystem.stop();
    spindexerSubsystem.stop();
    ledSubsystem.off();
  }
}
