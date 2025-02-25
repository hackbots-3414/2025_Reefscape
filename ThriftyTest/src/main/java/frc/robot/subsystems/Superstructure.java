package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AlgaeRollers.AlgaeRollerSpeeds;
import frc.robot.subsystems.CoralRollers.CoralRollerSpeeds;
import frc.robot.utils.RunOnChange;

public class Superstructure extends SubsystemBase {
    public enum Goal {STOW, INTAKE, CORAL_L1, CORAL_L2, CORAL_L3, CORAL_L4, ALGAE_L2, ALGAE_L3, PROCESSOR, NET, CLIMB, GROUND_PICKUP}

    private final Elevator elevator;
    private final Pivot pivot;
    private final CoralRollers coralRollers;
    private final AlgaeRollers algaeRollers;

    private final RunOnChange<CoralRollerSpeeds> runCoral;
    private final RunOnChange<AlgaeRollerSpeeds> runAlgae;

    private final Goal currState = Goal.STOW;

    public Superstructure(Elevator elevator, Pivot pivot, CoralRollers coralRollers, AlgaeRollers algaeRollers) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.coralRollers = coralRollers;
        this.algaeRollers = algaeRollers;

        this.runCoral = new RunOnChange<CoralRollers.CoralRollerSpeeds>(this.coralRollers::set, CoralRollerSpeeds.STOP);
        this.runAlgae = new RunOnChange<AlgaeRollers.AlgaeRollerSpeeds>(this.algaeRollers::set, AlgaeRollerSpeeds.STOP);
    }

    public void setState(Goal state) {
        switch (state) {
            case STOW -> {
                elevator.setStow();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case INTAKE -> {
                elevator.setStow();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.INTAKE);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L1 -> {
                elevator.setL1();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.L1);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L2 -> {
                elevator.setL2();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.L2);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L3 -> {
                elevator.setL3();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.L3);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case CORAL_L4 -> {
                elevator.setL4();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.L4);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
            }
            case ALGAE_L2 -> {
                elevator.setReefLower();
                pivot.setReefPickup();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.INTAKE);
            }
            case ALGAE_L3 -> {
                elevator.setReefUpper();
                pivot.setReefPickup();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.INTAKE);
            }
            case GROUND_PICKUP -> {
                elevator.setGroundIntake();
                pivot.setGroundPickup();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.INTAKE);
            }
            case PROCESSOR -> {
                elevator.setStow();
                pivot.setProcessor();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.EJECT);
            }
            case NET -> {
                elevator.setNet();
                pivot.setNet();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.EJECT);
            }
            case CLIMB -> {
                elevator.setStow();
                pivot.setStow();
                runCoral.accept(CoralRollerSpeeds.STOP);
                runAlgae.accept(AlgaeRollerSpeeds.STOP);
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
