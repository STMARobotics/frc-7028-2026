package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.generated.TunerConstants;

public class Constants {

  public static final CANBus CANIVORE_BUS = new CANBus("canivore");

  /**
   * Constants for the field dimensions of the WELDED field.
   */
  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(651.2);
    public static final Distance FIELD_WIDTH = Inches.of(317.7);

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
  public static class OdometryConstants {
    // Trust the physics/encoders moderately
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(
        0.1, // X: 10cm error per meter (Trust wheels moderately)
          0.1, // Y: 10cm error per meter
          0.05 // Theta: 0.05 radians to (Trust the Pigeon heavily)
    );
  }

  public static class QuestNavConstants {
    // TODO - Set this once the robot is designed
    public static final Transform3d ROBOT_TO_QUEST = new Transform3d(new Translation3d(), new Rotation3d());

    public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.03, // X: Trust Quest to within 3cm (Trust more than odometry)
          0.03, // Y: Trust Quest to within 3cm
          0.5 // Theta: Trust Quest rotation LESS than Gyro (Trust Pigeon more)
    );
  }

  public static class IntakeConstants {
    public static final int DEVICE_ID_POSITION = 10;
    public static final int DEVICE_ID_ROLLER = 11;
    public static final int DEVICE_ID_CANCODER = 10;
    public static final Current PEAK_FORWARD_ROLLER_CURRENT = Amps.of(80);
    public static final Current PEAK_REVERSE_ROLLER_CURRENT = PEAK_FORWARD_ROLLER_CURRENT.unaryMinus();

    public static final Current ROLLER_SUPPLY_LIMIT = Amps.of(40);

    public static final Current DEPLOY_PEAK_FORWORD_CURRENT = Amps.of(80);
    public static final Current DEPLOY_PEAK_REVERSE_CURRENT = Amps.of(80);

    public static final Current DEPLOY_SUPPLY_LIMIT = Amps.of(40);

    public static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs().withKP(0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    public static final SlotConfigs DEPLOY_SLOT_CONFIGS = new SlotConfigs().withKP(0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0);

    public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
    public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(0.0);
    public static final AngularVelocity REVERSE_VELOCITY = RotationsPerSecond.of(0.0);
    public static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs();

    public static final Measure<AngleUnit> DEPLOY_POSITION_DEPLOYED = Rotations.of(1.0);
    public static final Measure<AngleUnit> DEPLOY_POSITIONS_RETRACTED = Rotations.of(1.0);
    public static final Measure<AngleUnit> DEPLOY_TOLERANCE = Rotations.of(1.0);

    public static final double ROLLER_ROTOR_TO_SENSOR_RATIO = 1;
    public static final Measure<AngularVelocityUnit> ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(0.0);
    public static final Measure<AngularVelocityUnit> ROLLER_REVERSE_VELOCITY = RotationsPerSecond.of(0.0);

    public static final double DEPLOY_ROTOR_TO_SENSOR_RATIO = 1d;
  }

  public static class SpindexerConstants {
    public static final int DEVICE_ID_SPINDEXER_MOTOR = 15;
    public static final String HOPPER_CAMERA_NAME = "HopperCam";

    public static final Current SPINDEXER_TORQUE_CURRENT_LIMIT = Amps.of(80);
    public static final Current SPINDEXER_STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SPINDEXER_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final SlotConfigs SPINDEXER_SLOT_CONFIGS = new SlotConfigs().withKP(10).withKS(0);

    public static final AngularVelocity SPINDEXER_FEED_VELOCITY = RotationsPerSecond.of(20);
    public static final AngularVelocity SPINDEXER_INTAKE_VELOCITY = RotationsPerSecond.of(-10);
    public static final AngularVelocity SPINDEXER_AGITATE_FORWARD_VELOCITY = RotationsPerSecond.of(5);
    public static final AngularVelocity SPINDEXER_AGITATE_BACKWARD_VELOCITY = RotationsPerSecond.of(-5);

    public static final double HOPPER_FULL_THRESHOLD = 85.0; // percent
    public static final Time PIPELINE_RESULT_TTL = Seconds.of(0.25);
  }

  /**
   * Constants for the Transfer Subsystem
   */
  public static class TransferConstants {
    public static final int DEVICE_ID_TRANSFER_MOTOR = 20;
    public static final int DEVICE_ID_TRANSFER_CANRANGE = 20;

    public static final Current TRANSFER_TORQUE_CURRENT_LIMIT = Amps.of(80);
    public static final Current TRANSFER_STATOR_CURRENT_LIMIT = Amps.of(90);
    public static final Current TRANSFER_SUPPLY_CURRENT_LIMIT = Amps.of(40);
    public static final SlotConfigs TRANSFER_SLOT_CONFIGS = new SlotConfigs().withKP(10).withKS(0);

    public static final AngularVelocity TRANSFER_FEED_VELOCITY = RotationsPerSecond.of(60);
    public static final AngularVelocity TRANSFER_UNJAM_VELOCITY = RotationsPerSecond.of(-10);
  }

  /**
   * Constants for the LEDs
   */
  public static class LEDConstants {
    public static final int DEVICE_ID_LEDS = 9;

    public static final int LED_STRIP_LENGTH = 49;

    public static final int TOTAL_LEDS = 2 * LED_STRIP_LENGTH;
  }

}
