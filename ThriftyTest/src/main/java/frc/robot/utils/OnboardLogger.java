package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * A utility class to help log structured data to a DataLog without worrying about extra writes or
 * other nasty situations.
 */
public class OnboardLogger {
  private static final DataLog dataLog = DataLogManager.getLog();

  private final String m_name;

  private final List<Pair<Supplier<Double>, DoubleLogEntry>> m_doubleEntries;
  private final List<Pair<Supplier<Boolean>, BooleanLogEntry>> m_booleanEntries;
  private final List<Pair<Supplier<String>, StringLogEntry>> m_stringEntries;
  private final List<Pair<Supplier<Pose2d>, StructLogEntry<Pose2d>>> m_poseEntries;
  private final List<Pair<Supplier<Pose2d[]>, StructArrayLogEntry<Pose2d>>> m_poseListEntries;

  public OnboardLogger(String name) {
    m_name = name;
    m_doubleEntries = new ArrayList<>();
    m_booleanEntries = new ArrayList<>();
    m_stringEntries = new ArrayList<>();
    m_poseEntries = new ArrayList<>();
    m_poseListEntries = new ArrayList<>();
  }

  public void registerDouble(String name, DoubleSupplier supplier) {
    DoubleLogEntry entry = new DoubleLogEntry(dataLog, m_name + "/" + name);
    m_doubleEntries.add(new Pair<>(supplier::getAsDouble, entry));
  }

  public void registerBoolean(String name, BooleanSupplier supplier) {
    BooleanLogEntry entry = new BooleanLogEntry(dataLog, m_name + "/" + name);
    m_booleanEntries.add(new Pair<>(supplier::getAsBoolean, entry));
  }

  public void registerString(String name, Supplier<String> supplier) {
    StringLogEntry entry = new StringLogEntry(dataLog, m_name + "/" + name);
    m_stringEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPose(String name, Supplier<Pose2d> supplier) {
    StructLogEntry<Pose2d> entry =
        StructLogEntry.create(dataLog, m_name + "/" + name, Pose2d.struct);
    m_poseEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPoses(String name, Supplier<Pose2d[]> supplier) {
    StructArrayLogEntry<Pose2d> entry =
        StructArrayLogEntry.create(dataLog, m_name + "/" + name, Pose2d.struct);
    m_poseListEntries.add(new Pair<>(supplier, entry));
  }

  public void log() {
    for (Pair<Supplier<Double>, DoubleLogEntry> pair : m_doubleEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Boolean>, BooleanLogEntry> pair : m_booleanEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<String>, StringLogEntry> pair : m_stringEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose2d>, StructLogEntry<Pose2d>> pair : m_poseEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose2d[]>, StructArrayLogEntry<Pose2d>> pair : m_poseListEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
  }
}
