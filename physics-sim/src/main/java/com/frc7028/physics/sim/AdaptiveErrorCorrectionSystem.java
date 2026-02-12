package com.frc7028.physics.sim;

import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveErrorCorrectionSystem {

  Function<SimpleMatrix, SimpleMatrix> computeError;
  SimpleMatrix inputDeltas;
  SimpleMatrix deltaMatrix;
  // error(n) x input(m)
  // [ ]
  // [ ]
  // [ ]

  int N;
  int M;

  public AdaptiveErrorCorrectionSystem(
      Function<SimpleMatrix, SimpleMatrix> computeErrorFunction,
      SimpleMatrix inputDeltas,
      int N,
      int M) {

    this.computeError = computeErrorFunction;
    this.N = N;
    this.M = M;
    this.deltaMatrix = new SimpleMatrix(M, M);
    for (int inputIndex = 0; inputIndex < M; inputIndex++) {
      this.deltaMatrix.set(inputIndex, inputIndex, inputDeltas.get(inputIndex, 0));
    }
  }

  public SimpleMatrix computeCorrection(SimpleMatrix baselineInputs) {
    SimpleMatrix baselineErrors = this.computeError.apply(baselineInputs);

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

    return outputMatrix.pseudoInverse().mult(baselineErrors);
  }
}
