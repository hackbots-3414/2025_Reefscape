// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateSpace;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.StateSpaceConstants;

public class StateSpaceController<States extends Num, Inputs extends Num, Outputs extends Num> implements AutoCloseable {
    private final LinearSystemLoop<States, Inputs, Outputs> m_loop;

    private final Supplier<Vector<Outputs>> m_outputSupplier;
    private final Consumer<Vector<Inputs>> m_inputConsumer;

    private final double m_tolerance;

    private final String m_name;

    private boolean isAtSetpoint = false;

    private Notifier m_notifier;

    /** Creates a new StateSpaceController. */
    public StateSpaceController(
            StateSpaceConfig<States, Inputs, Outputs> config,
            Supplier<Vector<Outputs>> outputSupplier,
            Consumer<Vector<Inputs>> inputConsumer,
            Vector<States> initialState) {

        m_outputSupplier = outputSupplier;
        m_inputConsumer = inputConsumer;

        m_tolerance = config.getTolerance();

        m_name = config.getName();

        m_loop = config.buildLoop();
        m_loop.reset(initialState);
        
        // setup the notifier!
        m_notifier = new Notifier(this::periodic);
        m_notifier.startPeriodic(StateSpaceConstants.k_dt);
    }

    private void periodic() {
        // Correct our Kalman Filter's state vector estimate with the latest data
        Vector<Outputs> lastMeasurement = m_outputSupplier.get();
        m_loop.correct(lastMeasurement);

        // Update the LQR to generate new control inputs
        // and use the generated values to predict the next state.
        m_loop.predict(StateSpaceConstants.k_dt);

        // Send the next calculated value to the consumer
        Vector<Inputs> inputs = new Vector<>(m_loop.getU());
        m_inputConsumer.accept(inputs);

        // Send new readings to SmartDashboard for logging purposes
        for (int row = 0; row < lastMeasurement.getNumRows(); row++) {
            double measurement = lastMeasurement.get(row);
            SmartDashboard.putNumber(m_name + " (" + row + ")", measurement);
        }

        isAtSetpoint = Math.abs(lastMeasurement.get(0) - getReference()) < m_tolerance;
    }

    /**
     * Sets the new reference state
     * 
     * @param ref The reference for each state
     */
    public void setReference(Vector<States> ref) {
        m_loop.setNextR(ref);
    }

    public double getReference() {
        return m_loop.getNextR(0);
    }

    public boolean isAtSetpoint() {
        return isAtSetpoint;
    }

    public void close() {
        m_notifier.close();
    }
}
