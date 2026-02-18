package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveSystem<context> {

  public enum DebugType {
    Event,
    Display
  }

  public enum DebugEventType {
    CorrectionStart,
    CorrectionEnd,
    IterationStart,
    IterationStop,
    IterationStepStart,
    IterationStepStop,
    IterationStepBreak,
    AdaptiveStart,
    AdaptiveEnd,
    ComputeStart,
    ComputeEnd,
    NA
  }

  public enum DebugDisplayEvent {
    NA,
    CurrentOutput
  }

  private SimpleMatrix defaultMatrix = new SimpleMatrix(0, 0);

  public record AdaptiveOutput(int iterations, boolean converged, SingularOutput output) {
  }

  public record SingularOutput(double error, SimpleMatrix inputs, SimpleMatrix errors) {
  }

  public record AdaptiveDebugObject(
      DebugType type,
      DebugEventType event,
      DebugDisplayEvent displayEvent,
      String stringOutput,
      Double doubleOutput,
      Translation3d T3Doutput,
      SimpleMatrix matrixOutput) {
  }

  BiFunction<SimpleMatrix, context, SimpleMatrix> computeError;
  Consumer<AdaptiveDebugObject> debugger;
  Function<SimpleMatrix, Double> errorQuantifier;

  double errorTolerance;

  SimpleMatrix inputDeltas;

  private int maxIterations;
  int N;
  int M;

  public AdaptiveSystem(
      BiFunction<SimpleMatrix, context, SimpleMatrix> computeErrorFunction,
      Function<SimpleMatrix, Double> errorQuantifier,
      Consumer<AdaptiveDebugObject> debugger,
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

    this.debugger = debugger;
  }

  private void debug(DebugType debugType, DebugEventType eventType) {
    this.debugger.accept(
        new AdaptiveDebugObject(
            debugType,
            eventType,
            DebugDisplayEvent.NA,
            "",
            0d,
            Translation3d.kZero,
            defaultMatrix));
  }

  private void debug(DebugType debugType, DebugDisplayEvent debugDisplayEvent, double output) {
    this.debugger.accept(
        new AdaptiveDebugObject(
            debugType,
            DebugEventType.NA,
            debugDisplayEvent,
            "",
            output,
            Translation3d.kZero,
            defaultMatrix));
  }

  private void debug(DebugType debugType, DebugDisplayEvent debugDisplayEvent, String output) {
    this.debugger.accept(
        new AdaptiveDebugObject(
            debugType,
            DebugEventType.NA,
            debugDisplayEvent,
            output,
            0d,
            Translation3d.kZero,
            defaultMatrix));
  }

  private void debug(DebugType debugType, DebugDisplayEvent debugDisplayEvent, Translation3d output) {
    this.debugger.accept(
        new AdaptiveDebugObject(debugType, DebugEventType.NA, debugDisplayEvent, "", 0d, output, defaultMatrix));
  }

  private void debug(DebugType debugType, DebugDisplayEvent debugDisplayEvent, SimpleMatrix output) {
    this.debugger.accept(
        new AdaptiveDebugObject(debugType, DebugEventType.NA, debugDisplayEvent, "", 0d, Translation3d.kZero, output));
  }

  public SingularOutput computeCorrection(SimpleMatrix baselineInputs, SimpleMatrix baselineErrors, context Context) {
    // using central diff

    debug(DebugType.Event, DebugEventType.CorrectionStart);

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
    System.out.println(Jacobian);
    SimpleMatrix correction = Jacobian.pseudoInverse().mult(baselineErrors).scale(-1.0);
    SimpleMatrix correctedInput = baselineInputs.plus(correction);
    SimpleMatrix correctedOutput = this.computeError.apply(correctedInput, Context);
    double error = this.errorQuantifier.apply(correctedOutput);

    debug(DebugType.Event, DebugEventType.CorrectionEnd);

    return new SingularOutput(error, correctedInput, correctedOutput);
  }

  public AdaptiveOutput simpleConverge(SimpleMatrix inputGuess, context Context) {
    debug(DebugType.Event, DebugEventType.AdaptiveStart);
    int iterations = 0;
    System.out.println(this.maxIterations);
    SimpleMatrix baselineErrors = this.computeError.apply(inputGuess, Context);

    SingularOutput currentOutput = this.computeCorrection(inputGuess, baselineErrors, Context);
    SingularOutput bestOutput = currentOutput;

    debug(DebugType.Event, DebugEventType.IterationStart);

    while (iterations++ < this.maxIterations) {
      debug(DebugType.Event, DebugEventType.IterationStepStart);
      currentOutput = this.computeCorrection(currentOutput.inputs(), currentOutput.errors(), Context);
      debug(DebugType.Display, DebugDisplayEvent.CurrentOutput, currentOutput.error);
      if (currentOutput.error < bestOutput.error) {
        bestOutput = currentOutput;
      }
      if (this.errorTolerance > bestOutput.error) {
        debug(DebugType.Event, DebugEventType.IterationStepBreak);
        break;
      }

      debug(DebugType.Event, DebugEventType.IterationStepStop);
    }

    debug(DebugType.Event, DebugEventType.IterationStop);

    return new AdaptiveOutput(iterations, bestOutput.error < this.errorTolerance, bestOutput);
  }
}
