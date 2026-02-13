package frc.robot.controls;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Abstract class that defines the available bindings for driver controls. It can be extended to provide a "control
 * scheme"
 */
public abstract class ControlBindings {

  /**
   * Supplier for the driver desired X speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationX();

  /**
   * Supplier for the driver desired Y speed
   *
   * @return velocity supplier
   */
  public abstract Supplier<LinearVelocity> translationY();

  /**
   * Supplier for the drive desired omega rotation
   *
   * @return velocity supplier
   */
  public abstract Supplier<AngularVelocity> omega();

  /**
   * Triggers putting the wheels into an X configuration
   *
   * @return optional trigger
   */
  public Optional<Trigger> wheelsToX() {
    return Optional.empty();
  }

  /**
   * Trigger to reset reset the heading to zero.
   * <p>
   * Because of global pose estimation, <b>there is no reason this should be used in competition bindings.<b>
   * </p>
   *
   * @return optional trigger
   */
  public Optional<Trigger> seedFieldCentric() {
    return Optional.empty();
  }

  /**
   * Trigger to shoot fuel into the hub.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> shoot() {
    return Optional.empty();
  }

  /**
   * Trigger for expanding the climb
   * 
   * @return optional trigger
   */
  public Optional<Trigger> expandClimb() {
    return Optional.empty();
  }

  /**
   * Trigger for contracting the climb
   * 
   * @return optional trigger
   */
  public Optional<Trigger> contractClimb() {
    return Optional.empty();
  }

  /**
   * Trigger to intake fuel from the floor. This includes deploying the intake.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> intake() {
    return Optional.empty();
  }

  /**
   * Trigger to stop the intake rollers.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> stopIntake() {
    return Optional.empty();
  }

  /**
   * Trigger to retract the intake.
   * 
   * @return optional trigger
   */
  public Optional<Trigger> retractIntake() {
    return Optional.empty();
  }

}