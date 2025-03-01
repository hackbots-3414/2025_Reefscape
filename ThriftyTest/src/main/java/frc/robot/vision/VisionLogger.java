package frc.robot.vision;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LogBuilder.VisionLog;
import jdk.internal.org.commonmark.node.BulletList;

public class VisionLogger implements AutoCloseable {
    private static VisionLogger instance;

    private static VisionLogger getInstance() {
        if (instance == null) {
            instance = new VisionLogger();
        }
        return instance;
    }

    private Logger logger = LoggerFactory.getLogger(VisionLogger.class);

    private FileWriter m_writer;
    private BufferedWriter m_buffer;

    private VisionLogger() {
        try {
            m_writer = new FileWriter(VisionConstants.k_logPath);
        } catch (IOException e) {
            logger.error("failed to open vision log file : {}", e.toString());
            return;
        }
        m_buffer = new BufferedWriter(m_writer);
    }

    private synchronized void recordLogs(List<VisionLog> logs) {
        if (m_buffer == null) {
            for (VisionLog log : logs) {
                SmartDashboard.putNumber(log.estimate().source(), log.error());
            }
            return;
        }

        StringBuilder builder = new StringBuilder();

        for (VisionLog log : logs) {
            builder.append(log.estimate().source());
            builder.append(" : ");
            builder.append(log.error());
            builder.append("\n");
        }

        try {
            m_buffer.write(builder.toString());
        } catch (IOException e) {
            logger.error("Failed writing to vision log file: {}", e.toString());
        }
    }

    public synchronized static void record(List<VisionLog> logs) {
        getInstance().recordLogs(logs);
    }

    public void close() {

    }
}
