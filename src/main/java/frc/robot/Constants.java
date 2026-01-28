package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {

  private Constants() {
  } // prevent instantiation

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

    public static Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.03, // X: Trust Quest to within 3cm (Trust more than odometry)
          0.03, // Y: Trust Quest to within 3cm
          0.5 // Theta: Trust Quest rotation LESS than Gyro (Trust Pigeon more)
    );
  }

  /**
   * Constants for the turret subsystem
   */
  public static class TurretConstants {
    // TODO: Confirm naming schemes and replace placeholders
    public static final int YAW_MOTOR_ID = 25;
    public static final int PITCH_MOTOR_ID = 26;
    public static final int PITCH_ENCODER_ID = 30;

    public static final Current PITCH_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final Current PITCH_SUPPLY_CURRENT_LIMIT = Amps.of(30);

    public static final Current YAW_STATOR_CURRENT_LIMIT = Amps.of(100);
    public static final Current YAW_SUPPLY_CURRENT_LIMIT = Amps.of(40);

    public static final double YAW_ROTOR_TO_SENSOR_RATIO = 1.0; // placeholder
    public static final double PITCH_ROTOR_TO_SENSOR_RATIO = 1.0; // placeholder

    public static final Angle PITCH_MAGNETIC_OFFSET = Rotations.of(0.0); // placeholder

    public static final Angle YAW_LIMIT_FORWARD = Rotations.of(0.5); // placeholder (~180deg)
    public static final Angle YAW_LIMIT_REVERSE = Rotations.of(-0.5); // placeholder

    // TODO: Tune PID values
    public static final SlotConfigs YAW_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.5)
        .withKA(0.0);

    public static final MotionMagicConfigs YAW_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1.0) // placeholder
        .withMotionMagicCruiseVelocity(1.0); // placeholder

    public static final Angle PITCH_LIMIT_FORWARD = Rotations.of(0.05); // placeholder
    public static final Angle PITCH_LIMIT_REVERSE = Rotations.of(-0.02); // placeholder

    // TODO: Tune PID values
    public static final SlotConfigs PITCH_SLOT_CONFIGS = new SlotConfigs().withKP(0.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKS(0.0)
        .withKV(0.0)
        .withKA(0.0)
        .withKG(0.0)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final MotionMagicConfigs PITCH_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(1.0) // placeholder
        .withMotionMagicCruiseVelocity(1.0); // placeholder
  }
}
