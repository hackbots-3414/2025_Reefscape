package frc.robot.subsystems.elevator;

public enum ElevatorState {
  /** Elevator at lowest position */
  Zero(0),
  /** Elevator at ground algae intake height*/
  Ground(0),
  /** Height for ground algae intake */
  HighGround(0.60),
  /** Regular "home" position - also intake position */
  Stow(0.31),
  /** A little higher than stow to eject a coral */
  Eject(ElevatorState.Stow.position() + 2 * ElevatorConstants.kInch),
  /** Height to score processor */
  Processor(0),
  /** L1 height */
  L1(2.63),
  /** Secondary L1 height for when a coral is already present */
  SecondaryL1(ElevatorState.L1.position() + 8 * ElevatorConstants.kInch),
  /** L2 height */
  L2(4.016 + 3 * ElevatorConstants.kInch),
  /** L3 height */
  L3(7.257 - 4 * ElevatorConstants.kInch),
  /** L4 height */
  L4(9.757 + 0.3 * ElevatorConstants.kInch),
  /** Height to score net */
  Net(9.31 + 4 * ElevatorConstants.kInch),
  /** Height to intake algae from lower reef */
  LowerReef(2.0),
  /** Height to intake algae from upper reef */
  UpperReef(4.5);

  private final double m_position;

  private ElevatorState(double position) {
    m_position = position;
  }

  protected double position() {
    return m_position;
  }
}

