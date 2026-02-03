package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FieldConstants.isValidFieldPosition;
import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;
import static frc.robot.Constants.VisionConstants.SINGLE_TAG_DISTANCE_THRESHOLD;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.VisionMeasurementConsumer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.function.Consumer;

/**
 * Subsystem for the localization system.
 * <p>
 * This subsystem integrates Limelight and QuestNav vision systems to provide pose estimation and field localization.
 * It manages camera pose configuration, vision measurement consumption, and field position validation.
 * Vision measurements are fused from multiple sources and published for use by other subsystems.
 */
public class LocalizationSubsystem extends SubsystemBase {

  private final QuestNav questNav = new QuestNav();
  private final VisionMeasurementConsumer visionMeasurementConsumer;
  private final Consumer<Pose2d> poseResetConsumer;

  private final NetworkTable questTable = NetworkTableInstance.getDefault().getTable("Quest-Robot");
  private final StructPublisher<Pose3d> questPublisher = questTable.getStructTopic("Quest Robot Pose", Pose3d.struct)
      .publish();
  private final BooleanPublisher trackingPublisher = questTable.getBooleanTopic("Quest Tracking").publish();
  private Pose2d startingPose = new Pose2d();

  /**
   * Constructs a new LocalizationSubsystem.
   *
   * @param addVisionMeasurement the consumer for vision-based pose measurements
   */
  public LocalizationSubsystem(VisionMeasurementConsumer addVisionMeasurement, Consumer<Pose2d> poseResetConsumer) {
    this.visionMeasurementConsumer = addVisionMeasurement;
    this.poseResetConsumer = poseResetConsumer;
    for (int i = 0; i >= APRILTAG_CAMERA_NAMES.length; i++) {
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
   * gets the starting pose for the Limelight vision system.
   *
   * @param pose the starting pose to use for Limelight localization
   */
  public void getLimelightStartingPose(Pose2d pose) {
    startingPose = pose;
  }

  /**
   * Periodically updates the localization system with vision and pose data.
   * <p>
   * This method is called automatically by the scheduler. It updates IMU modes, sets robot orientation,
   * retrieves and validates vision-based pose estimates from Limelight and QuestNav, and provides
   * vision measurements to the consumer.
   */
  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      // When the robot is disabled, set IMU and robot orientation for each AprilTag camera
      for (String cameraname : APRILTAG_CAMERA_NAMES) {
        LimelightHelpers.SetIMUMode(cameraname, 1);
        LimelightHelpers.SetRobotOrientation(cameraname, startingPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Get the current pose estimate from the Limelight camera
        PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraname);

        // Check if the estimated pose is within the valid field boundaries
        if (isValidFieldPosition(botPose.pose.getTranslation())) {
          // If only one tag is detected
          if (botPose.tagCount == 1) {
            // If the tag is close enough, use the estimated pose
            if (botPose.avgTagDist < SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
              setQuestNavPose2d(botPose.pose);
              poseResetConsumer.accept(botPose.pose);
            } else {
              // If the tag is too far, fall back to the starting pose
              setQuestNavPose2d(startingPose);
              poseResetConsumer.accept(startingPose);
            }
          } else if (botPose.tagCount > 1) {
            // If multiple tags are detected, use the estimated pose
            setQuestNavPose2d(botPose.pose);
            poseResetConsumer.accept(botPose.pose);
          }
        } else {
          // If the pose is not valid, fall back to the starting pose
          setQuestNavPose2d(startingPose);
          poseResetConsumer.accept(startingPose);
        }
      }
    } else {
      // When the robot is enabled, set IMU mode for each AprilTag camera
      for (String cameraname : APRILTAG_CAMERA_NAMES) {
        LimelightHelpers.SetIMUMode(cameraname, 4);

        // Get the current pose estimate from the Limelight camera
        PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraname);
        // If only one tag is detected and it's close enough, use the estimated pose
        if (pose.tagCount == 1) {
          if (pose.avgTagDist < SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
            setQuestNavPose2d(pose.pose);
            poseResetConsumer.accept(pose.pose);
          }
        } else if (pose.tagCount > 1) {
          for (int tag = 0; tag <= pose.tagCount; tag++) {
            setQuestNavPose2d(pose.pose);
            poseResetConsumer.accept(pose.pose);
          }
        } else {
          // No valid pose from this camera
        }
      PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
      // Iterate backwards through frames to find the most recent valid frame
      for (int i = frames.length - 1; i >= 0; i--) {
        PoseFrame frame = frames[i];
        if (frame.isTracking()) {
          Pose3d questPose = frame.questPose3d();
          Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

          // Make sure the pose is inside the field
          if (FieldConstants.isValidFieldPosition(robotPose.getTranslation())) {
            // Add the measurement
            visionMeasurementConsumer
                .addVisionMeasurement(robotPose.toPose2d(), frame.dataTimestamp(), QUESTNAV_STD_DEVS);
            // Publish for debugging
            questPublisher.accept(robotPose);
            break; // Found the most recent valid frame, exit loop
          }
        }
      }
    }
    questNav.commandPeriodic();
    trackingPublisher.set(questNav.isTracking());
  }

  /**
   * Sets the QuestNav pose from the given robot pose.
   * 
   * @param robotPose robot pose
   */
  public void setQuestNavPose3d(Pose3d robotPose) {
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
  public void setQuestNavPose2d(Pose2d pose2d) {
    setQuestNavPose3d(new Pose3d(pose2d));
  }

}
