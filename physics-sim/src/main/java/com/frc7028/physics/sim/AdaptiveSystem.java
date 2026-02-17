package com.frc7028.physics.sim;

import java.util.ArrayList;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveSystem {

  public record AdaptiveOutput(int iterations, boolean converged, SingularOutput output) {
  }

  public record SingularOutput(double error, SimpleMatrix inputs, SimpleMatrix errors) {
  }

  Function<SimpleMatrix, SimpleMatrix> computeError;
  Function<SimpleMatrix, Double> errorQuantifier;

  double errorTolerance;

  SimpleMatrix inputDeltas;
  SimpleMatrix deltaMatrix;

  private int maxIterations;
  int N;
  int M;

  public AdaptiveSystem(
      Function<SimpleMatrix, SimpleMatrix> computeErrorFunction,
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
    this.N = inputs;
    this.M = outputs;
    this.deltaMatrix = new SimpleMatrix(M, M);
    for (int inputIndex = 0; inputIndex < M; inputIndex++) {
      this.deltaMatrix.set(inputIndex, inputIndex, inputDeltas.get(inputIndex, 0));
    }
  }

  public SingularOutput computeCorrection(SimpleMatrix baselineInputs, SimpleMatrix baselineErrors) {
    // SimpleMatrix baselineErrors = this.computeError.apply(baselineInputs);

    SimpleMatrix inputMatrix = new SimpleMatrix(this.M, this.M);
    SimpleMatrix outputMatrix = new SimpleMatrix(this.N, this.M);

    SimpleMatrix baselineReferenceMatrix = new SimpleMatrix(this.N, this.M);

    for (int inputIndex = 0; inputIndex < this.M; inputIndex++) {
      inputMatrix.setRow(inputIndex, baselineInputs.transpose());
      baselineReferenceMatrix.setColumn(inputIndex, baselineErrors);
    }

    SimpleMatrix perturbedInputMatrix = inputMatrix.plus(this.deltaMatrix);

    for (int inputIndex = 0; inputIndex < this.M; inputIndex++) {
      outputMatrix.setColumn(inputIndex, this.computeError.apply(perturbedInputMatrix.getRow(inputIndex).transpose()));
    }

    outputMatrix = outputMatrix.minus(baselineReferenceMatrix);

    for (int inputIndex = 0; inputIndex < this.M; inputIndex++) {
      outputMatrix.setColumn(inputIndex, outputMatrix.getColumn(inputIndex).divide(inputDeltas.get(inputIndex, 0)));
    }

    SimpleMatrix newInput = outputMatrix.pseudoInverse().mult(baselineErrors);
    SimpleMatrix newError = this.computeError.apply(newInput);
    double error = this.errorQuantifier.apply(newError);

    return new SingularOutput(error, newInput, newError);
  }

  public AdaptiveOutput simpleConverge(SimpleMatrix inputGuess) {
    int iterations = 0;

    SingularOutput currentOutput = this.computeCorrection(inputGuess, this.computeError.apply(inputGuess));
    SingularOutput bestOutput = currentOutput;

    ArrayList<SingularOutput> outputs = new ArrayList<SingularOutput>();

    while (iterations++ < this.maxIterations) {
      currentOutput = this.computeCorrection(currentOutput.inputs(), currentOutput.errors());
      if (currentOutput.error < bestOutput.error) {
        bestOutput = currentOutput;
      }
    }

    return new AdaptiveOutput(iterations, iterations < this.maxIterations, bestOutput);
  }
}
