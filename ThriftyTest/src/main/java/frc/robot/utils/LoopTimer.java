package frc.robot.utils;

public class LoopTimer {
  private long m_lastUpdateTime;

  private final OnboardLogger m_ologger;

  public LoopTimer(String identifier) {
    m_ologger = new OnboardLogger(identifier);
    m_ologger.registerDouble("Loop Time (ms)", this::getElapsedTime);
  }

  /**
   * Resets the timer
   */
  public void reset() {
    m_lastUpdateTime = System.nanoTime();
  }

  private double getElapsedTime() {
    long end = System.nanoTime();
    long elapsed = end - m_lastUpdateTime;
    double elapsedMillis = (double) elapsed * 1e-6;
    return elapsedMillis;
  }

  /**
   * Logs the time since last timer reset
   *
   * This does not reset the timer. A call to reset() is necessary to reset the timer.
   */
  public void log() {
    m_ologger.log();
  }

  /**
   * Runs the given runnable, and tracks how long it takes.
   */
  public void time(Runnable runnable) {
    reset();
    runnable.run();
    log();
  }

  /**
   * Returns a runnable that runs and tracks the given runnable.
   */
  public Runnable timed(Runnable runnable) {
    return () -> {
      time(runnable);
    };
  }
}
