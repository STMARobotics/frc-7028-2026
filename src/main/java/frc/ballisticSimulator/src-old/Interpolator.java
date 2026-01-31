package frc.ballisticSimulator;

import static java.lang.Math.floor;
import static java.lang.Math.signum;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.function.Function;

public abstract class Interpolator {
  private double shooterHeight;

  private double minPitch;
  private double maxPitch;

  private double targetX;
  private double targetY;
  private double targetZ;

  private double fieldMinX;
  private double fieldMaxX;
  private double fieldMinY;
  private double fieldMaxY;

  private double velocityRange;

  private double maxShotSpeed;

  private double gravity = 9.81;

  private double velocityIncrement;
  private double positionIncrement;
  private double pitchIncrement;

  public InterpolatorGenerator(
    double shooterHeight, double minPitch, double maxPitch, double fieldMinX, double fieldMaxX, double fieldMinY,
     double fieldMaxY, double velocityRange, double maxShotSpeed, double velocityIncrement, 
     double positionIncrement, double pitchIncrement, double targetX, double targetY, double targetZ
  ) {

    this.shooterHeight = shooterHeight;
    this.targetX = targetX;
    this.targetY = targetY;
    this.targetZ = targetZ;
    this.fieldMaxX = fieldMaxX;
    this.fieldMaxY = fieldMaxY;
    this.fieldMinX = fieldMinX;
    this.fieldMinY = fieldMinY;
    this.velocityRange = velocityRange;
    this.maxShotSpeed = maxShotSpeed;
    this.velocityIncrement = velocityIncrement;
    this.positionIncrement = positionIncrement;
    this.pitchIncrement = pitchIncrement;
  };

  private class solutionSpace<Result> {
    public int size;

    private double min;
    private double max;
    private double delta;
    private ArrayList solutions = new ArrayList<Solution>();

    private Comparator comparator;

    private Function<Record, Record> solve;
    private Function<Result, Double> score;

    private record Solution<T>(T result, double score) {
    }

    public class SolutionComparator implements Comparator<Solution> {
      public int compare(Interpolator.solutionSpace.Solution solutionA, Interpolator.solutionSpace.Solution solutionB) {
        return signum(solutionA.score() - solutionB.score());
      }
    }

    public solutionSpace(
        double delta,
        double min,
        double max,
        Function<Double, Result> solutionSolver,
        Function<Result, Double> scoreSolution) {
      this.size = (int) floor((max - min) / delta);
      this.min = min;
      this.max = max;
      this.delta = delta;

      this.comparator = new SolutionComparator();

      this.solve = solutionSolver;
      this.score = scoreSolution;
    }

    public populate() {
      for (int index = 0; index <= this.size; index++) {
        double input = index * this.delta;
        Result result = this.solve(input);
        double score = this.score(solution);
        this.solutions.add(new Solution(result, score));
      }
    }

    public Solution<T> selectSolution() {
      this.solutions.sort(this.comparator);
      return this.solutions.get(0);
    }
  }

  public void simulate() {
    for (double px = fieldMinX; px <= fieldMaxX; px += positionIncrement) {
      for (double py = fieldMinY; py <= fieldMaxY; py += positionIncrement) {
        double tx = this.targetX;
        double ty = this.targetY;
        double tz = this.targetZ;

        // distance from robot on field
        double rxy = Math.sqrt(Math.pow(px - tx, 2) + Math.pow(py - ty, 2));

      }
    }
  }
}
