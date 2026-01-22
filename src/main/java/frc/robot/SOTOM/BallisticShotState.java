package frc.robot.SOTOM;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;

public class BallisticShotState {
  BallisticCalculator calculator;
  BallisticShooter shooter;
  BallisticRobotState robot;

  Translation2d exitVelocity;
  Translation2d exitPosition;

  // stateful object properties which I think will be nice :)
  Translation2d projectedPosition;
  Time projectedShotTime;
  int iterations = 0;

  boolean active;
  boolean mayProceed;
  boolean successfullyConverged;
  Time deltaTime;

  // calc is short for calculator
  public BallisticShotState(BallisticCalculator calculator, BallisticShooter shooter) {
    this.calculator = calculator;
    this.shooter = shooter;
    this.robot = shooter.robotState;
    this.exitVelocity = this.robot.getFieldVelocity()
        .plus(this.calculator.calculateMuzzelTangentialVelocity(robot, shooter));
    this.exitPosition = this.shooter.getMuzzlePositionWTRField().toTranslation2d();
    this.active = true;
    this.mayProceed = true;
    this.iterations = 0;
  };

  public void step(boolean isInstantiating) {
    Time timePrior = this.projectedShotTime;
    this.projectedShotTime = Seconds.of(
        Meters.of(
            this.exitPosition.getDistance(isInstantiating ? this.calculator.targetPosition : this.projectedPosition))
            .div(this.exitVelocity.getSquaredNorm())
            .baseUnitMagnitude());
    this.projectedPosition = this.exitPosition
        .plus(this.exitVelocity.times(this.projectedShotTime.baseUnitMagnitude()));
    if (!isInstantiating)
      this.iterations++;
    this.mayProceed = this.iterations <= this.calculator.maxConvergenceIterationCycleCount;
    this.deltaTime = this.projectedShotTime.minus(timePrior);
    this.successfullyConverged = (this.deltaTime.abs(Seconds) <= this.calculator.deltaTimeConvergencThreshold
        .baseUnitMagnitude());
  }

  public void attempt() {
    while (this.mayProceed) {
      this.step(false);
      if (this.successfullyConverged) {
        break;
      }
    }
    this.active = false;
  };
}
