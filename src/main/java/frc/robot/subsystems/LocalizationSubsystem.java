package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.isValidFieldTranslation;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_FAILURE_THRESHOLD;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;
import static frc.robot.Constants.VisionConstants.ANGULAR_VELOCITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.APRILTAG_STD_DEVS;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_BLUE_PIPELINE;
import static frc.robot.Constants.VisionConstants.LIMELIGHT_RED_PIPELINE;
import static frc.robot.Constants.VisionConstants.QUESTNAV_ACTIVE_APRILTAG_STD_DEVS;
import static frc.robot.Constants.VisionConstants.QUESTNAV_APRILTAG_ERROR_THRESHOLD;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;
import static frc.robot.Constants.VisionConstants.STARTING_DISTANCE_THRESHOLD;
import static frc.robot.Constants.VisionConstants.TAG_DISTANCE_THRESHOLD;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.VisionMeasurementConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Subsystem for the localization system.
 * <p>
 * This subsystem integrates Limelight and QuestNav vision systems to provide pose estimation and field localization.
 * It manages camera pose configuration, vision measurement consumption, and field position validation.
 * Vision measurements are fused from multiple sources and published for use by other subsystems.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class LocalizationSubsystem extends SubsystemBase {

  private final QuestNav questNav = new QuestNav();
  private final VisionMeasurementConsumer visionMeasurementConsumer;
  private final Consumer<Pose2d> poseResetConsumer;

  private final NetworkTable localizationTable = NetworkTableInstance.getDefault().getTable("Localization-Robot");
  private final StructPublisher<Pose3d> questPublisher = localizationTable
      .getStructTopic("Quest Robot Pose", Pose3d.struct)
      .publish();
  private final BooleanPublisher trackingPublisher = localizationTable.getBooleanTopic("Quest Tracking").publish();
  private final BooleanPublisher questHealthPublisher = localizationTable.getBooleanTopic("Quest Healthy").publish();
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<AngularVelocity> robotAngularVelocitySupplier;
  @Logged
  private Pose2d startingPose = new Pose2d();
  @Logged
  private double questNavFaultCounter = 0.0;

  /**
   * Constructs a new LocalizationSubsystem.
   *
   * @param addVisionMeasurement the consumer for vision-based pose measurements
   * @param poseResetConsumer the consumer for resetting the robot's pose when the robot is disabled
   * @param poseSupplier supplier for the robot's current pose estimate from the fused estimator
   * @param angularVelocitySupplier supplier for the robot's current angular velocity, NOT from the fused estimator but
   *          directly from the IMU
   */
  public LocalizationSubsystem(
      VisionMeasurementConsumer addVisionMeasurement,
      Consumer<Pose2d> poseResetConsumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<AngularVelocity> angularVelocitySupplier) {
    this.visionMeasurementConsumer = addVisionMeasurement;
    this.poseResetConsumer = poseResetConsumer;
    this.poseSupplier = poseSupplier;
    this.robotAngularVelocitySupplier = angularVelocitySupplier;

    for (int i = 0; i < APRILTAG_CAMERA_NAMES.length; i++) {
      LimelightHelpers.setCameraPose_RobotSpace(
          APRILTAG_CAMERA_NAMES[i],
            ROBOT_TO_CAMERA_TRANSFORMS[i].getX(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getY(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getZ(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureX().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureY().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureZ().in(Degrees));
    }
  }

  /**
   * Sets the starting pose for the robot at the beginning of a match.
   * <p>
   * This pose is used to initialize robot orientation and serves as a fallback
   * for translation if vision-based localization is unavailable.
   * 
   * @param pose starting pose ALWAYS on the blue side
   */
  public void setInitialPose(Pose2d pose) {
    startingPose = pose;
  }

  /**
   * Periodically updates the localization system with vision and pose data.
   * <p>
   * This method is called automatically by the scheduler. It retrieves and validates vision-based pose estimates from
   * Limelight and QuestNav, and provides vision measurements to the consumer.
   */
  @Override
  public void periodic() {
    PoseEstimate bestEstimate = null;
    if (RobotState.isDisabled()) {
      periodicDisabled();
    } else {
      bestEstimate = periodicEnabled();
    }
    periodicQuestNav(bestEstimate);
  }

  /**
   * Validates a pose estimate based on common criteria.
   * <p>
   * Checks if the pose has valid tag count, is within field boundaries, and is within the tag distance threshold.
   *
   * @param poseEstimate the pose estimate to validate
   * @return true if the pose estimate meets basic validation criteria
   */
  private boolean isValidPoseEstimate(PoseEstimate poseEstimate) {
    return poseEstimate != null && poseEstimate.tagCount > 0
        && isValidFieldTranslation(poseEstimate.pose.getTranslation())
        && poseEstimate.avgTagDist < TAG_DISTANCE_THRESHOLD.in(Meters);
  }

  /**
   * Handles periodic updates when the robot is disabled.
   * <p>
   * Validates AprilTag poses from the camera, and resets the pose estimator and QuestNav
   */
  private void periodicDisabled() {
    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      boolean isBlueAlliance = alliance.isEmpty() || alliance.get() == Alliance.Blue;
      // Set the pipeline based on alliance color
      if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_BLUE_PIPELINE);
      } else {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_RED_PIPELINE);
      }

      if (RobotState.isAutonomous() || DriverStation.isFMSAttached()) {
        // Preparing to run auto
        Pose2d allianceStartingPose = isBlueAlliance ? startingPose : FlippingUtil.flipFieldPose(startingPose);
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

        if (isValidPoseEstimate(poseEstimate) && poseEstimate.pose.getTranslation()
            .getDistance(allianceStartingPose.getTranslation()) < STARTING_DISTANCE_THRESHOLD.in(Meters)) {
          // The pose is valid and close to the starting position, use its translation but the auto's rotation
          Pose2d poseToUse = new Pose2d(poseEstimate.pose.getTranslation(), allianceStartingPose.getRotation());
          setQuestNavPose(poseToUse);
          poseResetConsumer.accept(poseToUse);
        } else {
          // The pose is valid but far from the starting position, likely a bad reading - use the alliance's starting
          // pose
          setQuestNavPose(allianceStartingPose);
          poseResetConsumer.accept(allianceStartingPose);
        }
      } else {
        // Not prepping for auto, so add the vision measurement from limelight
        PoseEstimate limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        visionMeasurementConsumer.addVisionMeasurement(
            limelightPose.pose,
              limelightPose.timestampSeconds,
              VecBuilder.fill(APRILTAG_STD_DEVS, APRILTAG_STD_DEVS, 0.01));
        // Get the most recent fused pose estimate and give it to QuestNav
        Pose2d currentPoseEstimate = poseSupplier.get();
        setQuestNavPose(currentPoseEstimate);
      }
    }
  }

  /**
   * Handles periodic updates when the robot is enabled.
   * <p>
   * Processes all AprilTag cameras, validates poses, adds vision measurements with adjusted standard deviations,
   * and returns the best pose estimate for comparison with QuestNav.
   *
   * @return the best validated pose estimate from all cameras, or null if no valid estimates
   */
  private PoseEstimate periodicEnabled() {
    PoseEstimate bestEstimate = null;
    double bestDeviation = Double.MAX_VALUE;

    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

      if (isValidPoseEstimate(poseEstimate)
          && robotAngularVelocitySupplier.get().gt(ANGULAR_VELOCITY_THRESHOLD.unaryMinus())
          && robotAngularVelocitySupplier.get().lt(ANGULAR_VELOCITY_THRESHOLD)) {

        double baseStandardDeviations = (questNavFaultCounter > QUESTNAV_FAILURE_THRESHOLD) ? APRILTAG_STD_DEVS
            : QUESTNAV_ACTIVE_APRILTAG_STD_DEVS;
        double adjustedXYDeviation = baseStandardDeviations + (0.01 * Math.pow(poseEstimate.avgTagDist, 2));
        Matrix<N3, N1> adjustedDeviations = VecBuilder.fill(adjustedXYDeviation, adjustedXYDeviation, Double.MAX_VALUE);

        visionMeasurementConsumer
            .addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, adjustedDeviations);

        // Track the best estimate for QuestNav comparison
        if (bestDeviation > adjustedXYDeviation) {
          bestEstimate = poseEstimate;
          bestDeviation = adjustedXYDeviation;
        }
      }
    }

    return bestEstimate;
  }

  /**
   * Handles periodic updates for QuestNav pose estimation.
   * <p>
   * Processes unread QuestNav pose frames, validates poses against AprilTag estimates,
   * manages fault detection, and adds valid vision measurements.
   *
   * @param bestVisionEstimate the best AprilTag pose estimate for comparison, or null if none available
   */
  private void periodicQuestNav(PoseEstimate bestVisionEstimate) {
    questNav.commandPeriodic();
    boolean isTracking = questNav.isTracking();
    trackingPublisher.set(isTracking);
    if (!isTracking) {
      questNavFaultCounter++;
    }
    PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
    // Iterate backwards through frames to find the most recent valid frame
    for (int i = frames.length - 1; i >= 0; i--) {
      PoseFrame frame = frames[i];
      if (frame.isTracking()) {
        Pose3d robotPose = frame.questPose3d().transformBy(ROBOT_TO_QUEST.inverse());
        questPublisher.set(robotPose);

        double bestVisionEstimateDistance = (bestVisionEstimate != null)
            ? robotPose.toPose2d().getTranslation().getDistance(bestVisionEstimate.pose.getTranslation())
            : 0;

        if (bestVisionEstimateDistance > QUESTNAV_APRILTAG_ERROR_THRESHOLD.in(Meters)) {
          // QuestNav disagrees with AprilTag vision - increment fault counter
          questNavFaultCounter += Math.pow(bestVisionEstimateDistance, 2);
        } else {
          // QuestNav agrees with AprilTag vision - decrement fault counter to allow recovery
          if (questNavFaultCounter > 0.0) {
            questNavFaultCounter -= 1.0;
          }
        }

        questHealthPublisher.set(questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD);

        // Only use QuestNav measurements when fault counter is below threshold and pose is valid
        if (questNavFaultCounter < QUESTNAV_FAILURE_THRESHOLD && isValidFieldTranslation(robotPose.getTranslation())) {
          visionMeasurementConsumer
              .addVisionMeasurement(robotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
        }
        break; // Found the most recent tracking frame, exit loop
      }
    }
  }

  /**
   * Sets the QuestNav pose from the given robot pose.
   * 
   * @param robotPose robot pose
   */
  public void setQuestNavPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  /**
   * Sets the QuestNav pose from the given 2D robot pose.
   * <p>
   * This sets the 3D pose with a Z of 0 (on the floor) and no pitch or roll.
   *
   * @param pose2d the robot's 2D pose
   */
  public void setQuestNavPose(Pose2d pose2d) {
    setQuestNavPose(new Pose3d(pose2d));
  }

}
