package com.frc7028.physics.sim;

import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveErrorCorrectionSystem<inputState, N, M> {

  Function<inputState, SimpleMatrix> getInputs;
  Function<SimpleMatrix, SimpleMatrix> computeSystem;
  SimpleMatrix inputDeltas;

  int N;
  int M;

  public AdaptiveErrorCorrectionSystem(
      Function<inputState, SimpleMatrix> getInputs,
      Function<SimpleMatrix, SimpleMatrix> computeSystem,
      SimpleMatrix inputDeltas,
      int N,
      int M) {
    this.getInputs = getInputs;
    this.computeSystem = computeSystem;
  }

  public SimpleMatrix computeCorrection(inputState input) {
    SimpleMatrix baselineInputs = this.getInputs.apply(input);
    SimpleMatrix baselineErrors = this.computeSystem.apply(baselineInputs);

    SimpleMatrix inputMatrix = new SimpleMatrix(this.M, this.M);
    SimpleMatrix outputMatrix = new SimpleMatrix(this.N, this.M);
    SimpleMatrix differenceMatrix = new SimpleMatrix(this.N, this.M);
    SimpleMatrix jacobian = new SimpleMatrix(this.N, this.M);

    for (int inputIndex = 0; inputIndex < this.N; inputIndex++) {
      inputMatrix.setRow(inputIndex, baselineErrors);
      inputMatrix.set(inputIndex, inputIndex, baselineErrors.get(inputIndex) + this.inputDeltas.get(inputIndex));
      outputMatrix.setRow(inputIndex, this.computeSystem.apply(inputMatrix.getRow(inputIndex)));
      differenceMatrix.setRow(inputIndex, baselineErrors.minus(outputMatrix.getRow(inputIndex)));
      jacobian.setRow(inputIndex, differenceMatrix.getRow(inputIndex).elementDiv(this.inputDeltas));
    }

    return jacobian.pseudoInverse().mult(baselineErrors);
  }
}
