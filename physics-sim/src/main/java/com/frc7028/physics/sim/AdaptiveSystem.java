package com.frc7028.physics.sim;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import org.ejml.simple.SimpleMatrix;

public class AdaptiveSystem<context> {

  // Source - https://stackoverflow.com/a/5762502
  // Posted by WhiteFang34, modified by community. See post 'Timeline' for change history
  // Retrieved 2026-02-18, License - CC BY-SA 3.0

  public static final String ANSI_RESET = "\u001B[0m";
  public static final String ANSI_BLACK = "\u001B[30m";
  public static final String ANSI_RED = "\u001B[31m";
  public static final String ANSI_GREEN = "\u001B[32m";
  public static final String ANSI_YELLOW = "\u001B[33m";
  public static final String ANSI_BLUE = "\u001B[34m";
  public static final String ANSI_PURPLE = "\u001B[35m";
  public static final String ANSI_CYAN = "\u001B[36m";
  public static final String ANSI_WHITE = "\u001B[37m";

  // ^^^ use colorful outputs

  public enum DebugType {
    Event,
    Display
  }

  public enum DebugEventType {
    CorrectionStart(ANSI_BLUE),
    CorrectionEnd(ANSI_BLUE),
    IterationStart(ANSI_GREEN),
    IterationStop(ANSI_GREEN),
    IterationStepStart(ANSI_PURPLE),
    IterationStepStop(ANSI_PURPLE),
    IterationStepBreak(ANSI_RED),
    AdaptiveStart(ANSI_WHITE),
    AdaptiveEnd(ANSI_WHITE),
    ComputeStart(ANSI_YELLOW),
    ComputeEnd(ANSI_YELLOW),
    NA(ANSI_WHITE);

    private final String color;

    public String getColor() {
      return this.color;
    }

    DebugEventType(String color) {
      this.color = color;
    }
  }

  public enum DebugDisplayEvent {
    NA,
    CurrentOutput,
    Jacobian
  }

  public SimpleMatrix defaultMatrix = new SimpleMatrix(0, 0);

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
    debug(DebugType.Display, DebugDisplayEvent.Jacobian, Jacobian);
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
