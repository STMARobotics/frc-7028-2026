package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class Constants {

  public static final String CANIVORE_BUS_NAME = "canivore";
  public static final CANBus CANIVORE_BUS = new CANBus(CANIVORE_BUS_NAME);

  public static class FieldConstants {
    // Dimensions for the WELDED field
    public static final Distance FIELD_LENGTH = Meters.of(17.548);
    public static final Distance FIELD_WIDTH = Meters.of(8.052);

    /**
     * Checks if a pose is within the field boundaries, and less than 3 meters high.
     * 
     * @param pose The pose to check
     * @return True if the pose is within the field boundaries
     */
    public static boolean isValidFieldPosition(Translation3d pose) {
      return pose.getX() >= 0.0 && pose.getX() <= FIELD_LENGTH.in(Meters) && pose.getY() >= 0.0
          && pose.getY() <= FIELD_WIDTH.in(Meters) && pose.getZ() >= 0.0 && pose.getZ() <= 3.0;
    }

    /**
     * Checks if a 2D pose is within the field boundaries.
     * 
     * @param pose The pose to check
     * @return True if the pose is within the field boundaries
     */
    public static boolean isValidFieldPosition(Translation2d pose) {
      return isValidFieldPosition(new Translation3d(pose));
    }
  }

  /**
   * Constants for teleoperated driver control
   */
  public static class TeleopDriveConstants {
    /** Max velocity the driver can request */
    public static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts;
    /** Max angular velicity the driver can request */
    public static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.25);
  }

  /**
   * Constants for odometry state estimation
   */
  public static class OdometryContants {
    // Trust the physics/encoders moderately
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(
        0.1, // X: 10cm error per meter (Trust wheels moderately)
          0.1, // Y: 10cm error per meter
          0.05 // Theta: 0.05 radians to (Trust the Pigeon heavily)
    );
  }

  public static class QuestNavConstants {
    // TODO - Set this once the robot is designed
    public static Transform3d ROBOT_TO_QUEST = new Transform3d(new Translation3d(), new Rotation3d());

    public static Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.03, // X: Trust Quest to within 3cm (Trust more than odometry)
          0.03, // Y: Trust Quest to within 3cm
          0.5 // Theta: Trust Quest rotation LESS than Gyro (Trust Pigeon more)
    );
  }

  /**
   * Constants for vision processing
   */
  public static class VisionConstants {
    public static final String[] APRILTAG_CAMERA_NAMES = { "figure", "these", "out" };
    // TODO - Set this once the robot is designed
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
        new Transform3d(
            new Translation3d(inchesToMeters(0), inchesToMeters(0), inchesToMeters(0)),
            new Rotation3d(0, 0, degreesToRadians(0))),
        new Transform3d(
            new Translation3d(inchesToMeters(0), inchesToMeters(0), inchesToMeters(0)),
            new Rotation3d(0, 0, degreesToRadians(0))),
        new Transform3d(
            new Translation3d(inchesToMeters(0), inchesToMeters(0), inchesToMeters(0)),
            new Rotation3d(0, degreesToRadians(0), degreesToRadians(0))) };

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 99999999);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 99999999);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when
     * multiple tags are in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.5);
  }

}
