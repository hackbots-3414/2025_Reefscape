package frc.robot.subsystems.pivot;

public enum PivotState {
  /** The pivot angle for intaking algae off the ground */
  Ground(0.0669),
  /** The angle for scoring in the processor */
  Processor(0.085),
  /** The angle for intaking algae off the reef */
  ReefIntake(0.2),
  /** The angle for removing algae off the reef after intake */
  ReefExtract(0.281),
  /** The angle to score at the net */
  Net(0.342),
  /** The "home" angle for the pivot */
  Stow(0.343);

  private double m_position;

  private PivotState(double position) {
    m_position = position;
  }

  protected double position() {
    return m_position;
  }
}
