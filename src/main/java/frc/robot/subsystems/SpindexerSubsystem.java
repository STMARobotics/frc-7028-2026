package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS;
import static frc.robot.Constants.SpindexerConstants.DEVICE_ID_SPINDEXER_MOTOR;
import static frc.robot.Constants.SpindexerConstants.HOPPER_CAMERA_NAME;
import static frc.robot.Constants.SpindexerConstants.HOPPER_FULL_THRESHOLD;
import static frc.robot.Constants.SpindexerConstants.PIPELINE_RESULT_TTL;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_AGITATE_BACKWARD_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_AGITATE_FORWARD_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_FEED_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_INTAKE_VELOCITY;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_PEAK_TORQUE_CURRENT_FORWARD;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_PEAK_TORQUE_CURRENT_REVERSE;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_SLOT_CONFIGS;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.SpindexerConstants.SPINDEXER_SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.List;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Subsystem for the Spindexer.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class SpindexerSubsystem extends SubsystemBase {

  private final TalonFX spindexerMotor = new TalonFX(DEVICE_ID_SPINDEXER_MOTOR, CANIVORE_BUS);

  private final PhotonCamera hopperCam = new PhotonCamera(HOPPER_CAMERA_NAME);

  private final VelocityTorqueCurrentFOC spindexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC spindexerTorqueControl = new TorqueCurrentFOC(0.0);

  private static final PhotonPipelineResult EMPTY_PHOTON_PIPELINE_RESULT = new PhotonPipelineResult();

  private PhotonPipelineResult photonPipelineResult = EMPTY_PHOTON_PIPELINE_RESULT;

  // NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine spindexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("Spindexer SysId", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> spindexerMotor.setControl(spindexerTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new Subsystem for the Spindexer
   */
  public SpindexerSubsystem() {
    var spinTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(SPINDEXER_SLOT_CONFIGS))
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(SPINDEXER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(SPINDEXER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(SPINDEXER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SPINDEXER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));

    spindexerMotor.getConfigurator().apply(spinTalonconfig);
  }

  public Command sysIdSpindexerDynamicCommand(Direction direction) {
    return spindexerSysIdRoutine.dynamic(direction)
        .withName("SysId spindexer dynamic " + direction)
        .finallyDo(this::stop);
  }

  public Command sysIdSpindexerQuasistaticCommand(Direction direction) {
    return spindexerSysIdRoutine.quasistatic(direction)
        .withName("SysId spindexer quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Spins the spindexer forward to feed the shooter
   */
  public void feedShooter() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_FEED_VELOCITY));
  }

  /**
   * Spins the spindexer backward well intakeing fuel
   */
  public void intake() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_INTAKE_VELOCITY));
  }

  /**
   * Agitates the spindexer back and forth to prevent jams
   */
  public Command agitate() {
    return run(this::spinForward).withTimeout(0.5)
        .andThen(run(this::spinBackward).withTimeout(0.5))
        .repeatedly()
        .finallyDo(this::stop);
  }

  /**
   * Run the spindexer at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the spindexer
   */
  public void runSpindexer(AngularVelocity velocity) {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(velocity));
  }

  /**
   * Stops the spindexer
   */
  public void stop() {
    spindexerMotor.stopMotor();
  }

  /**
   * Returns true if the hopper is empty
   */
  @Logged
  public boolean isEmpty() {
    return getLatestTarget().isEmpty();
  }

  /**
   * Returns true if the hopper is full based on the area of the detected targets
   */
  @Logged
  public boolean isFull() {
    if (isEmpty()) {
      return false;
    }
    double accumulatedArea = 0;
    for (var target : getLatestTarget()) {
      accumulatedArea += target.getArea();
      if (accumulatedArea >= HOPPER_FULL_THRESHOLD) {
        return true;
      }
    }

    return false;
  }

  /**
   * Spins the spindexer forward
   */
  private void spinForward() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_AGITATE_FORWARD_VELOCITY));
  }

  /**
   * Spins the spindexer backward
   */
  private void spinBackward() {
    spindexerMotor.setControl(spindexerVelocityTorque.withVelocity(SPINDEXER_AGITATE_BACKWARD_VELOCITY));
  }

  private List<PhotonTrackedTarget> getLatestTarget() {
    List<PhotonPipelineResult> photonResults = hopperCam.getAllUnreadResults();

    /**
     * If there are results this will grab the last item on the list
     */
    if (photonResults.size() >= 1) {
      photonPipelineResult = photonResults.get(photonResults.size() - 1);
    }

    double currentTime = Timer.getFPGATimestamp();
    double resultTime = photonPipelineResult.getTimestampSeconds();

    /**
     * since the results are cached, the value is checked against the Time To Live (TTL) to
     * ensure that the value isn't to old
     */
    if (currentTime - resultTime >= PIPELINE_RESULT_TTL.in(Second)) {
      photonPipelineResult = EMPTY_PHOTON_PIPELINE_RESULT;
    }

    return photonPipelineResult.targets;
  }
}