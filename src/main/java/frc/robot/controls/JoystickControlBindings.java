package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

/** Control bindings for driving with joysticks */
public class JoystickControlBindings extends ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getY()), MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getX()), MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_TELEOP_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(rightJoystick.getX()),
          RadiansPerSecond);
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  @Override
  public Optional<Trigger> autoShoot() {
    return Optional.of(rightJoystick.trigger());
  }

}