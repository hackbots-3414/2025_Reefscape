package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    public enum Goal {STOW, INTAKE, CORAL_L1, CORAL_L2, CORAL_L3, CORAL_L4, ALGAE_L2, ALGAE_L3, PROCESSOR, NET, CLIMB, GROUND_PICKUP}

    private final Elevator elevator;
    private final Pivot pivot;

    private final Goal currState = Goal.STOW;

    public Superstructure(Elevator elevator, Pivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }

    public void setState(Goal state) {
        switch (state) {
            case STOW -> {
                elevator.setStow();
                pivot.setStow();
            }
            case INTAKE -> {
                elevator.setStow();
                pivot.setStow();
            }
            case CORAL_L1 -> {
                elevator.setL1();
                pivot.setStow();
            }
            case CORAL_L2 -> {
                elevator.setL2();
                pivot.setStow();
            }
            case CORAL_L3 -> {
                elevator.setL3();
                pivot.setStow();
            }
            case CORAL_L4 -> {
                elevator.setL4();
                pivot.setStow();
            }
            case ALGAE_L2 -> {
                elevator.setReefLower();
                pivot.setReefPickup();
            }
            case ALGAE_L3 -> {
                elevator.setReefUpper();
                pivot.setReefPickup();
            }
            case PROCESSOR -> {
                elevator.setStow();
                pivot.setProcessor();
            }
            case NET -> {
                elevator.setNet();
                pivot.setNet();
            }
            case CLIMB -> {
                elevator.setStow();
                pivot.setStow();
            }
            case GROUND_PICKUP -> {
                elevator.setGroundIntake();
                pivot.setGroundPickup();
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

    private void log() {
        SmartDashboard.putString("Superstructure State", currState.toString());
    }

    @Override
    public void periodic() {
        log();
    }
}
