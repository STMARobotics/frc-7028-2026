package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The is the Subsystem for the Spindexer.
 * 
 */

public class SpindexerSubsystem extends SubsystemBase {
  private final TalonFX spindexerMotor = new TalonFX(Constants.SpindexerConstants.DEVICE_ID_SPINDEXER_MOTOR);
  private final VelocityTorqueCurrentFOC spindexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC spindexerTorqueControl = new TorqueCurrentFOC(0.0);
  private final PhotonCamera hopperCam = new PhotonCamera(Constants.SpindexerConstants.HOPPER_CAMERA_NAME);
  private static final double PIPELINE_RESULT_TTL = 0.25;
  private static final PhotonPipelineResult EMPTY_PHOTON_PIPELINE_RESULT = new PhotonPipelineResult();
  private PhotonPipelineResult photonPipelineResult = EMPTY_PHOTON_PIPELINE_RESULT;

  /*
   * Directions for spindexer agitation
   */
  private enum SpindexerDirection {
    Forward,
    Backward
  }

  /*
   * NOTE: the output type is amps, NOT volts (even though it says volts)
   * https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
   */
  private final SysIdRoutine spindexerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("SysIdSpindexer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> spindexerMotor.setControl(spindexerTorqueControl.withOutput(amps.in(Volts))),
          null,
          this));

  /**
   * Creates a new Subsystem for the Spindexer
   */
  public SpindexerSubsystem() {
    var spinTalonconfig = new TalonFXConfiguration();
    spinTalonconfig.withSlot0(Slot0Configs.from(Constants.SpindexerConstants.SPINDEXER_SLOT_CONFIGS));
    spinTalonconfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    spinTalonconfig.TorqueCurrent
        .withPeakForwardTorqueCurrent(Constants.SpindexerConstants.SPINDEXER_TORQUE_CURRENT_LIMIT);
    spinTalonconfig.CurrentLimits.withStatorCurrentLimit(Constants.SpindexerConstants.SPINDEXER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.SpindexerConstants.SPINDEXER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    spinTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    spindexerMotor.getConfigurator().apply(spinTalonconfig);
  }

  public Command sysIdSpindexerCommand(Direction direction) {
    return spindexerSysIdRoutine.dynamic(direction).withName("Spindexer dynam " + direction).finallyDo(this::stop);
  }

  public Command sysIdSpindexerQuasistaticCommand(Direction direction) {
    return spindexerSysIdRoutine.quasistatic(direction).withName("Spindexer quasi " + direction).finallyDo(this::stop);
  }

  /*
   * Spins the spindexer forward to feed the shooter
   */
  public void feedShooter() {
    spindexerMotor
        .setControl(spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_FEED_VELOCITY));
  }

  /*
   * Spins the spindexer backward well intakeing fuel
   */
  public void intake() {
    spindexerMotor
        .setControl(spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_INTAKE_VELOCITY));
  }

  /*
   * Agitates the spindexer back and forth to prevent jams
   */
  public Command agitate() {
    return run(() -> this.spin(SpindexerDirection.Forward)).withTimeout(0.5)
        .andThen(run(() -> this.spin(SpindexerDirection.Backward)).withTimeout(0.5))
        .repeatedly()
        .finallyDo(this::stop);
  }

  /*
   * Stops the spindexer
   */
  public void stop() {
    spindexerMotor.stopMotor();
  }

  /*
   * Returns true if the hopper is empty
   */
  public boolean isEmpty() {
    return getLatestTarget().isEmpty();
  }

  /*
   * Returns true if the hopper is full based on the area of the detected targets
   */
  public boolean isFull() {
    if (isEmpty()) {
      return false;
    }
    double accumulatedArea = 0;
    for (var target : getLatestTarget()) {
      accumulatedArea += target.getArea();
      if (accumulatedArea >= Constants.SpindexerConstants.HOPPER_FULL_THRESHOLD) {
        return true;
      }
    }

    return false;
  }

  /*
   * spins the spindexer back and forth to unjam the fuel
   */
  private void spin(SpindexerDirection direction) {
    if (direction == SpindexerDirection.Backward) {
      spindexerMotor.setControl(
          spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_AGITATE_BACKWARDS_VELOCITY));
    } else {
      spindexerMotor.setControl(
          spindexerVelocityTorque.withVelocity(Constants.SpindexerConstants.SPINDEXER_AGITATE_FORWARDS_VELOCITY));
    }
  }

  private List<PhotonTrackedTarget> getLatestTarget() {
    List<PhotonPipelineResult> photonResults = hopperCam.getAllUnreadResults();

    /*
     * If there are results this will grab the last item on the list
     */
    if (photonResults.size() >= 1) {
      photonPipelineResult = photonResults.get(photonResults.size() - 1);
    }

    double currentTime = Timer.getFPGATimestamp();
    double resultTime = photonPipelineResult.getTimestampSeconds();

    /*
     * since the results are cached, the value is checked against the Time To Live (TTL) to
     * ensure that the value isn't to old
     */
    if (currentTime - resultTime >= PIPELINE_RESULT_TTL) {
      photonPipelineResult = EMPTY_PHOTON_PIPELINE_RESULT;
    }

    return photonPipelineResult.targets;
  }
}