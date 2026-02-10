package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreen;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterTarget;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import java.util.function.Supplier;

/**
 * This command automatically shoots at a target while a supplier (the driver) is translating the
 * robot. This command will allow a slowed translation speed and disallow rotation.
 */
public class TeleopShootCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterSubsystem shooterSubsystem;
  private final TransferSubsystem transferSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final LEDSubsystem ledSubsystem;

  private final Supplier<LinearVelocity> xSupplier;
  private final Supplier<LinearVelocity> ySupplier;

  private final Supplier<Pose2d> robotPoseSupplier;

  // Reusable object to prevent reallocation (to reduce memory pressure)
  private final MutAngle turretYawTarget = Rotations.mutable(0);

  private final Translation2d targetRed;
  private final Translation2d targetBlue;
  private final InterpolatingTreeMap<Double, ShooterTarget> lookupTable;
  private final double velocityMultiplier;
  private final double maxVelocity;

  private Translation2d targetTranslation;

  private final SwerveRequest.FieldCentric swerveRequestRotation = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public TeleopShootCommand(
      CommandSwerveDrivetrain drivetrain,
      ShooterSubsystem shooter,
      TransferSubsystem transferSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      LEDSubsystem ledSubsystem,
      Supplier<LinearVelocity> xSupplier,
      Supplier<LinearVelocity> ySupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Translation2d targetRed,
      Translation2d targetBlue,
      InterpolatingTreeMap<Double, ShooterTarget> lookupTable,
      double velocityMultiplier) {
    this.drivetrain = drivetrain;
    this.shooterSubsystem = shooter;
    this.transferSubsystem = transferSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetRed = targetRed;
    this.targetBlue = targetBlue;
    this.lookupTable = lookupTable;
    this.velocityMultiplier = velocityMultiplier;
    this.maxVelocity = MAX_TELEOP_VELOCITY.in(MetersPerSecond) * velocityMultiplier;

    addRequirements(drivetrain, shooter, transferSubsystem, spindexerSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    targetTranslation = (alliance.isEmpty() || alliance.get() == Blue) ? targetBlue : targetRed;
    ledSubsystem.off();
    transferSubsystem.unjam();
  }

  @Override
  public void execute() {
    var robotPose = robotPoseSupplier.get();
    var currentChassisSpeeds = drivetrain.getCurrentFieldChassisSpeeds();
    var currentTurretYaw = shooterSubsystem.getYawAngle();

    // Translation to the muzzle (exit point of the fuel)
    var muzzleTranslation = ShooterSubsystem.getMuzzleTranslation(robotPose, currentTurretYaw);

    // --- PHYSICS-BASED PREDICTION START ---

    // 1. Calculate the TOTAL effective velocity vector that gets imparted to the fuel.

    // A. Robot Linear Velocity (field-relative)
    var vRobot = new Translation2d(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);

    // B. Total Angular Velocity (field-relative)
    // The muzzle rotates due to both robot rotation AND turret rotation
    var omegaRobot = currentChassisSpeeds.omegaRadiansPerSecond;
    var turretVelocity = shooterSubsystem.getYawVelocity().in(RadiansPerSecond);
    var totalOmega = omegaRobot + turretVelocity;

    // C. Tangential Velocity at Muzzle due to Total Rotation
    // The muzzle is offset from the robot center, so rotation imparts tangential velocity
    var robotToMuzzle = muzzleTranslation.minus(robotPose.getTranslation());
    var vTanTotal = new Translation2d(-totalOmega * robotToMuzzle.getY(), totalOmega * robotToMuzzle.getX());

    // Sum all velocity vectors to get the "Effective Injection Velocity"
    var effectiveShooterVelocity = vRobot.plus(vTanTotal);

    // 2. Iteratively solve for the correct aim point (Fixes circular dependency)
    var predictedTargetTranslation = targetTranslation;

    // Iterate 4 times to converge on the intersection of trajectory and target
    ShooterTarget shootingSettings = null;
    for (int i = 0; i < 4; i++) {
      var dist = predictedTargetTranslation.getDistance(muzzleTranslation);
      shootingSettings = lookupTable.get(dist);

      var timeUntilScored = 0.0;
      var rps = shootingSettings.targetFlywheelSpeed().in(RotationsPerSecond);
      var pitchRads = shootingSettings.targetPitch().in(Radians);

      if (Math.abs(rps) > 1e-3) { // Prevents division by zero in flight time calc
        // Calculate flight time using horizontal velocity component:
        // Horizontal Velocity = Total Velocity * cos(pitch)
        // Time = Distance / Horizontal Velocity
        double fuelExitVelocity = FLYWHEEL_TO_FUEL_VELOCITY_MULTIPLIER * rps;
        timeUntilScored = dist / (fuelExitVelocity * Math.cos(pitchRads));
      }

      // The offset is how far the robot's velocity vector moves the fuel relative to a static shot
      var targetPredictedOffset = effectiveShooterVelocity.times(timeUntilScored);

      // Shift the virtual target opposite to the motion so we shoot "ahead"
      predictedTargetTranslation = targetTranslation.minus(targetPredictedOffset);
    }

    // Calculate the angle to the target
    var angleToTarget = predictedTargetTranslation.minus(muzzleTranslation).getAngle();

    // Calculate required turret angle, accounting for the robot heading
    turretYawTarget.mut_replace(angleToTarget.minus(robotPose.getRotation()).getRotations(), Rotations);

    // Prepare shooter
    shooterSubsystem.setYawAngle(turretYawTarget);
    shooterSubsystem.setPitchAngle(shootingSettings.targetPitch());
    shooterSubsystem.setFlywheelSpeed(shootingSettings.targetFlywheelSpeed());

    // Let the driver translate
    drivetrain.setControl(
        swerveRequestRotation.withVelocityX(xSupplier.get().in(MetersPerSecond) * velocityMultiplier)
            .withVelocityY(ySupplier.get().in(MetersPerSecond) * velocityMultiplier)
            .withRotationalRate(0.0));

    // We might want to debounce some of the ready states, we'll see
    // Calculate ready state
    var isShooterReady = shooterSubsystem.isReadyToShoot();
    var magnitude = Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
    var isDrivetrainReady = magnitude <= maxVelocity;
    var isPitchReady = shooterSubsystem.isPitchAtSetpoint();
    var isYawReady = shooterSubsystem.isYawAtSetpoint();
    if (isShooterReady && isPitchReady && isYawReady && isDrivetrainReady) {
      // Shooter is spun up, robot is slowed, and the turret is aimed - shoot and start timer
      spindexerSubsystem.feedShooter();
      transferSubsystem.feedShooter();
      ledSubsystem.runPattern(LEDPattern.solid(kGreen));
    } else {
      ledSubsystem.runPattern(
          LEDSubsystem
              .ledSegments(kBlue, () -> isShooterReady, () -> isPitchReady, () -> isYawReady, () -> isDrivetrainReady));
      spindexerSubsystem.stop();
      transferSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopAll();
    transferSubsystem.stop();
    spindexerSubsystem.stop();
    drivetrain.setControl(new SwerveRequest.Idle());
    ledSubsystem.off();
  }
}
