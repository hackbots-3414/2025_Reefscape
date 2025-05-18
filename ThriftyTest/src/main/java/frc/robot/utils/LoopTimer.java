package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoopTimer {
  private final String m_ident;

  private long m_lastUpdateTime;

  public LoopTimer(String identifier) {
    m_ident = identifier;
  }

  /**
   * Resets the timer
   */
  public void reset() {
    m_lastUpdateTime = System.nanoTime();
  }

  /**
   * Logs the time since last timer reset
   *
   * This does not reset the timer. A call to reset() is necessary to reset the timer.
   */
  public void log() {
    long end = System.nanoTime();
    long elapsed = end - m_lastUpdateTime;
    double elapsedMillis = (double) elapsed * 1e-6;
    SmartDashboard.putNumber(m_ident + "/Loop time (ms)", elapsedMillis);
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
