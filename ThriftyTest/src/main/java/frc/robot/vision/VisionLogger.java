package frc.robot.vision;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.vision.LogBuilder.VisionLog;

public class VisionLogger implements AutoCloseable {
    private static VisionLogger instance;

    private static VisionLogger getInstance() {
        if (instance == null) {
            instance = new VisionLogger();
        }
        return instance;
    }

    private Logger logger = LoggerFactory.getLogger(VisionLogger.class);

    private StringBuilder m_builder;

    private FileWriter m_writer;
    private BufferedWriter m_buffer;

    private VisionLogger() {
        String filepath = VisionConstants.k_logPath + Long.toHexString(System.currentTimeMillis() / 1000) + ".log";
        if (Robot.isSimulation()) {
            logger.trace("Simulation detected, using local logging path");
            filepath = VisionConstants.k_simLogPath + Long.toHexString(System.currentTimeMillis() / 1000) + ".log";
        }
        try {
            m_writer = new FileWriter(filepath);
        } catch (IOException e) {
            logger.error("failed to open vision log file : {}", e.toString());
            return;
        }
        m_buffer = new BufferedWriter(m_writer);

        try {
            m_buffer.write("time, source, x, y, err\n");
        } catch (IOException e) {
            logger.error("failed to write to vision log: {}", e.toString());
        }
        m_builder = new StringBuilder();
    }

    private synchronized void recordLogs(List<VisionLog > logs) {
        if (m_buffer == null) {
            for (VisionLog log : logs) {
                SmartDashboard.putNumber(log.estimate().source(), log.error());
            }
            return;
        }

        m_builder.setLength(0);

        for (VisionLog log : logs) {
            m_builder.append(log.estimate().timestamp());
            m_builder.append(",");
            m_builder.append(log.estimate().source());
            m_builder.append(",");
            m_builder.append(log.robot().getX());
            m_builder.append(",");
            m_builder.append(log.robot().getY());
            m_builder.append(",");
            m_builder.append(log.error());
            m_builder.append(",");
            m_builder.append(log.estimate().algorithm().toString());
            m_builder.append("\n");
        }

        try {
            m_buffer.write(m_builder.toString());
            m_buffer.flush();
        } catch (IOException e) {
            logger.error("Failed writing to vision log file: {}", e.toString());
        }

    }

    public synchronized static void record(List<VisionLog> logs) {
        getInstance().recordLogs(logs);
    }

    @Override
    public void close() {
        try {
            m_buffer.close();
        } catch (IOException e) {
            logger.error("failed closing file: {}", e.toString());
        }
    }
}
