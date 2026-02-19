package com.frc7028.physics.sim;

public class Debugger {
  public final class DisplayColors {
    public static final String ANSI_RESET = "\u001B[0m";
    public static final String ANSI_BLACK = "\u001B[30m";
    public static final String ANSI_RED = "\u001B[31m";
    public static final String ANSI_GREEN = "\u001B[32m";
    public static final String ANSI_YELLOW = "\u001B[33m";
    public static final String ANSI_BLUE = "\u001B[34m";
    public static final String ANSI_PURPLE = "\u001B[35m";
    public static final String ANSI_CYAN = "\u001B[36m";
    public static final String ANSI_WHITE = "\u001B[37m";
  }

  // ^^^ use colorful outputs

  public enum DebugType {
    Event,
    Display
  }

  public enum DebugEventType {
    CorrectionStart(DisplayColors.ANSI_BLUE),
    CorrectionEnd(DisplayColors.ANSI_BLUE),
    IterationStart(DisplayColors.ANSI_GREEN),
    IterationStop(DisplayColors.ANSI_GREEN),
    IterationStepStart(DisplayColors.ANSI_PURPLE),
    IterationStepStop(DisplayColors.ANSI_PURPLE),
    IterationStepBreak(DisplayColors.ANSI_RED),
    AdaptiveStart(DisplayColors.ANSI_WHITE),
    AdaptiveEnd(DisplayColors.ANSI_WHITE),
    ComputeStart(DisplayColors.ANSI_YELLOW),
    ComputeEnd(DisplayColors.ANSI_YELLOW),
    NA(DisplayColors.ANSI_WHITE);

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
}
