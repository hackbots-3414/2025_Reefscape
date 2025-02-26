package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeRollers.AlgaeRollerSpeeds;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;
import frc.robot.subsystems.Elevator.ElevatorSetpoints;
import frc.robot.subsystems.Pivot.PivotSetpoints;

public class Superstructure extends SubsystemBase {
    public enum Goal {STOW, INTAKE, CORAL_L1, CORAL_L2, CORAL_L3, CORAL_L4, ALGAE_L2, ALGAE_L3, PROCESSOR, NET, CLIMB, GROUND_PICKUP}

    private final Elevator elevator;
    private final Pivot pivot;
    private final CoralRollers coralRollers;
    private final AlgaeRollers algaeRollers;

    private final Goal currState = Goal.STOW;

    public Superstructure(Elevator elevator, Pivot pivot, CoralRollers coralRollers, AlgaeRollers algaeRollers) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.coralRollers = coralRollers;
        this.algaeRollers = algaeRollers;
    }

    public void setState(Goal state) {
        switch (state) {
            case STOW -> {
                elevator.stow();;
                pivot.stow();
                coralRollers.stop();
                algaeRollers.smartStop();
            }
            case INTAKE -> {
                elevator.stow();
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.INTAKE);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L1 -> {
                elevator.setLevel(1);
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.L1);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L2 -> {
                elevator.setLevel(2);
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.L2);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L3 -> {
                elevator.setLevel(3);
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.L3);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L4 -> {
                elevator.setLevel(4);
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.L4);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
            case ALGAE_L2 -> {
                elevator.set(ElevatorSetpoints.ALGAE_L2);
                pivot.set(PivotSetpoints.REEF_PICKUP);
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.INTAKE);
            }
            case ALGAE_L3 -> {
                elevator.set(ElevatorSetpoints.ALGAE_L3);
                pivot.set(PivotSetpoints.REEF_PICKUP);
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.INTAKE);
            }
            case GROUND_PICKUP -> {
                elevator.set(ElevatorSetpoints.GROUND);
                pivot.set(PivotSetpoints.GROUND);
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.INTAKE);
            }
            case PROCESSOR -> {
                elevator.stow();
                pivot.set(PivotSetpoints.PROCESSOR);
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.EJECT);
            }
            case NET -> {
                elevator.set(ElevatorSetpoints.NET);
                pivot.set(PivotSetpoints.NET);
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.EJECT);
            }
            case CLIMB -> {
                elevator.stow();
                pivot.stow();
                coralRollers.set(CoralRollerSpeeds.STOP);
                algaeRollers.set(AlgaeRollerSpeeds.STOP);
            }
        }
    }

    public void stow() {setState(Goal.STOW);}
    public void intake() {setState(Goal.INTAKE);}
    public void coralL1() {setState(Goal.CORAL_L1);}
    public void coralL2() {setState(Goal.CORAL_L2);}
    public void coralL3() {setState(Goal.CORAL_L3);}
    public void coralL4() {setState(Goal.CORAL_L4);}
    public void algaeLow() {setState(Goal.ALGAE_L2);}
    public void algaeHigh() {setState(Goal.ALGAE_L3);}
    public void processor() {setState(Goal.PROCESSOR);}
    public void net() {setState(Goal.NET);}
    public void climb() {setState(Goal.CLIMB);}
    public void ground() {setState(Goal.GROUND_PICKUP);}

    public boolean hasCoral() {return coralRollers.holdingPiece();}
    public boolean hasAlgae() {return algaeRollers.hasObject();}

    private void log() {
        SmartDashboard.putString("Superstructure State", currState.toString());
    }

    @Override
    public void periodic() {
        log();
    }
}
