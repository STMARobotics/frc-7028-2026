package frc.robot.subsystems;

import static frc.robot.Constants.QuestNavConstants.QUESTNAV_STD_DEVS;
import static frc.robot.Constants.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import gg.questnav.questnav.QuestNav;

/** Subsystem for the QuestNav vision system */
public class QuestNavSubsystem extends SubsystemBase {

  private final QuestNav questNav = new QuestNav();
  private final VisionMeasurementConsumer visionMeasurementConsumer;

  private final NetworkTable questTable = NetworkTableInstance.getDefault().getTable("Quest-Robot");
  private final StructPublisher<Pose3d> questPublisher = questTable.getStructTopic("Quest Robot Pose", Pose3d.struct)
      .publish();
  private final BooleanPublisher trackingPublisher = questTable.getBooleanTopic("Quest Tracking").publish();

  public QuestNavSubsystem(VisionMeasurementConsumer addVisionMeasurement) {
    this.visionMeasurementConsumer = addVisionMeasurement;
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    trackingPublisher.set(questNav.isTracking());
    var frames = questNav.getAllUnreadPoseFrames();
    // Iterate backwards through frames to find the most recent valid frame
    for (int i = frames.length - 1; i >= 0; i--) {
      var frame = frames[i];
      if (frame.isTracking()) {
        var questPose = frame.questPose3d();
        var robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

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

  /**
   * Sets the QuestNav pose from the given robot pose.
   * 
   * @param robotPose robot pose
   */
  public void setPose(Pose3d robotPose) {
    var questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  /**
   * Sets the QuestNav pose from the given robot pose. This sets the 3D pose with a Z of 0 (on the floor) and no pitch
   * or roll.
   * 
   * @param pose2d robot pose
   */
  public void setPose(Pose2d pose2d) {
    setPose(new Pose3d(pose2d));
  }

  /**
   * Functional interface for consuming vision measurements.
   */
  @FunctionalInterface
  public static interface VisionMeasurementConsumer {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

}
