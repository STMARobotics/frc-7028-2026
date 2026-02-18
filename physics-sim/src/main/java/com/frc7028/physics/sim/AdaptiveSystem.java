package com.frc7028.physics.sim;

import java.util.function.BiFunction;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveSystem<context> {

  public record AdaptiveOutput(int iterations, boolean converged, SingularOutput output) {
  }

  public record SingularOutput(double error, SimpleMatrix inputs, SimpleMatrix errors) {
  }

  BiFunction<SimpleMatrix, context, SimpleMatrix> computeError;
  Function<SimpleMatrix, Double> errorQuantifier;

  double errorTolerance;

  SimpleMatrix inputDeltas;

  private int maxIterations;
  int N;
  int M;

  public AdaptiveSystem(
      BiFunction<SimpleMatrix, context, SimpleMatrix> computeErrorFunction,
      Function<SimpleMatrix, Double> errorQuantifier,
      SimpleMatrix inputDeltas,
      double tolerance,
      int maxIterations,
      int inputs,
      int outputs) {

    this.computeError = computeErrorFunction;
    this.errorQuantifier = errorQuantifier;
    this.errorTolerance = tolerance;
    this.maxIterations = maxIterations;
    this.inputDeltas = inputDeltas;
    this.N = inputs;
    this.M = outputs;
  }

  public SingularOutput computeCorrection(SimpleMatrix baselineInputs, SimpleMatrix baselineErrors, context Context) {
    // using central diff, but now im doing 3 to

    SimpleMatrix Jacobian = new SimpleMatrix(this.M, this.N);

    for (int inputIndex = 0; inputIndex < this.N; inputIndex++) {
      SimpleMatrix shiftedInputs = baselineInputs.copy();
      double delta = inputDeltas.get(inputIndex, 0);
      shiftedInputs.set(inputIndex, 0, shiftedInputs.get(inputIndex, 0) - delta);

      SimpleMatrix shiftErrorA = this.computeError.apply(shiftedInputs, Context);

      shiftedInputs.set(inputIndex, 0, shiftedInputs.get(inputIndex, 0) + 2 * delta);

      SimpleMatrix shiftErrorB = this.computeError.apply(shiftedInputs, Context);
      SimpleMatrix epsilon = shiftErrorB.minus(shiftErrorA);

      for (int outputIndex = 0; outputIndex < this.M; outputIndex++) {
        Jacobian.set(outputIndex, inputIndex, epsilon.get(outputIndex, 0) / (2 * delta));
      }
    }

    SimpleMatrix correction = Jacobian.pseudoInverse().mult(baselineErrors).scale(-1.0);
    SimpleMatrix correctedInput = baselineInputs.plus(correction);
    SimpleMatrix correctedOutput = this.computeError.apply(correctedInput, Context);
    double error = this.errorQuantifier.apply(correctedOutput);

    return new SingularOutput(error, correctedInput, correctedOutput);
  }

  public AdaptiveOutput simpleConverge(SimpleMatrix inputGuess, context Context) {
    int iterations = 0;

    SimpleMatrix baselineErrors = this.computeError.apply(inputGuess, Context);

    SingularOutput currentOutput = this.computeCorrection(inputGuess, baselineErrors, Context);
    SingularOutput bestOutput = currentOutput;

    while (iterations++ < this.maxIterations) {
      currentOutput = this.computeCorrection(currentOutput.inputs(), currentOutput.errors(), Context);
      System.out.println(currentOutput.error);
      if (currentOutput.error < bestOutput.error) {
        bestOutput = currentOutput;
      }

      if (this.errorTolerance > bestOutput.error) {
        break;
      }
    }

    return new AdaptiveOutput(iterations, bestOutput.error < this.errorTolerance, bestOutput);
  }
}
