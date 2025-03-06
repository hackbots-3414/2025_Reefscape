package frc.robot.utils;

import java.util.function.Consumer;

public class RunOnChange<T> {
    private Consumer<T> task;
    private T currValue;
    private boolean changed;

    /**
     * Instantiates new RunOnChange object, which can handle any consumer
     * Generally used when you only want to write in periodic
     * @param task consumer to be run on call
     * @param defaultValue initial setpoint
     */
    public RunOnChange(Consumer<T> task, T defaultValue) {
        this.task = task;
        this.changed = false;
        accept(defaultValue);
        resolve();
    }

    /**
     * Stores value to be resolved later 
     * Generally used in "set" methods of subsystems, to be resolved in periodic
     * @param newValue setpoint
     */
    public void accept(T newValue) {
        changed = currValue != newValue;
        currValue = newValue;
    }

    /**
     * Runs code at previously set value
     */
    private void resolve() {
        task.accept(currValue);
        changed = false;
    }

    /**
     * If value has changed, run command with the specified value
     * Does not wait for periodic (runs instantly)
     * @param newValue setpoint
     */
    public void run(T newValue) {
        accept(newValue);
        resolveIfChange();
    }

    /**
     * Runs no matter what at a specified value (no wait)
     * @param newValue setpoint
     */
    public void instantRun(T newValue) {
        accept(newValue);
        resolve();
    }

    /**
     * Resolves task if value has changed
     */
    public void resolveIfChange() {
        if (changed) resolve();
    }

    public T getValue() {
        return currValue;
    }
}
