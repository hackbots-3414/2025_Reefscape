// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateSpace;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import frc.robot.Constants.StateSpaceConstants;

/**
 * A reusable container for the configuration of a state space system.
 * 
 * This is meant to be used with the State Space Controller class.
 */
public class StateSpaceConfig<States extends Num, Inputs extends Num, Outputs extends Num> {
    private LinearSystem<States, Inputs, Outputs> m_plant;
    private Vector<States> m_stateStdDevs;
    private Vector<Outputs> m_outputStdDevs;

    private Vector<States> m_qelms;
    private Vector<Inputs> m_relms;

    private Nat<States> m_states;
    private Nat<Outputs> m_outputs;

    private double m_tolerance;

    private String m_name;

    /**
     * Initializes a reusable state space controller configuration
     * 
     * @param plant         The <code>LinearSystem</code> that represents the actual
     *                      model. This can be generated from
     *                      <code>LinearSystemId</code>.
     * @param stateStdDevs  Standard deviations of model states
     * @param outputStdDevs Standard deviations of measurements
     * @param qelms         The maximum tolerated error for each state
     * @param relms         The maximum value for each control input
     * @param states        A Nat representing the number of states
     * @param outputs       A Nat representing the number of outputs
     * @param name          The name of the system to be used in logging.
     */
    public StateSpaceConfig(
            LinearSystem<States, Inputs, Outputs> plant,
            Vector<States> stateStdDevs,
            Vector<Outputs> outputStdDevs,
            Vector<States> qelms,
            Vector<Inputs> relms,
            Nat<States> states,
            Nat<Outputs> outputs,
            double tolerance,
            String name) {

        m_plant = plant;

        m_stateStdDevs = stateStdDevs;
        m_outputStdDevs = outputStdDevs;

        m_qelms = qelms;
        m_relms = relms;

        m_states = states;
        m_outputs = outputs;

        m_tolerance = tolerance;

        m_name = name;
    }

    /**
     * Builds a Kalman Filter to clean up noise from the measuremens.
     */
    public KalmanFilter<States, Inputs, Outputs> buildObserver() {
        return new KalmanFilter<States, Inputs, Outputs>(
                m_states,
                m_outputs,
                m_plant,
                m_stateStdDevs,
                m_outputStdDevs,
                StateSpaceConstants.k_dt);
    }

    /**
     * Builds the LQR controller.
     */
    public LinearQuadraticRegulator<States, Inputs, Outputs> buildController() {
        return new LinearQuadraticRegulator<States, Inputs, Outputs>(
                m_plant,
                m_qelms,
                m_relms,
                StateSpaceConstants.k_dt);
    }

    /**
     * Constructs the Linear System Loop with all of the parameters of the
     * configuration
     */
    public LinearSystemLoop<States, Inputs, Outputs> buildLoop() {
        return new LinearSystemLoop<States, Inputs, Outputs>(
                m_plant,
                buildController(),
                buildObserver(),
                StateSpaceConstants.k_maxVoltage,
                StateSpaceConstants.k_dt);
    }

    /**
     * Returns the system name, for logging purposes
     */
    public String getName() {
        return m_name;
    }

    public double getTolerance() {
        return m_tolerance;
    }
}