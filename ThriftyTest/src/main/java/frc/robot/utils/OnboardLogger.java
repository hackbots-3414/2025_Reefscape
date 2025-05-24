package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * A utility class to help log structured data to a DataLog without worrying about extra writes or
 * other nasty situations.
 */
public class OnboardLogger {
  private static final DataLog dataLog = DataLogManager.getLog();

  private final String m_name;

  private final ArrayList<Pair<MonitoredSupplier<Double>, DoubleLogEntry>> m_doubleEntries;
  private final ArrayList<Pair<MonitoredSupplier<Boolean>, BooleanLogEntry>> m_booleanEntries;
  private final ArrayList<Pair<MonitoredSupplier<String>, StringLogEntry>> m_stringEntries;
  private final ArrayList<Pair<MonitoredSupplier<Pose2d>, StructLogEntry<Pose2d>>> m_poseEntries;

  public OnboardLogger(String name) {
    m_name = name;
    m_doubleEntries = new ArrayList<>();
    m_booleanEntries = new ArrayList<>();
    m_stringEntries = new ArrayList<>();
    m_poseEntries = new ArrayList<>();
  }

  public void registerDouble(String name, DoubleSupplier supplier) {
    DoubleLogEntry entry = new DoubleLogEntry(dataLog, m_name + "/" + name);
    m_doubleEntries.add(new Pair<>(MonitoredSupplier.fromDoubleSuplier(supplier), entry));
  }

  public void registerBoolean(String name, BooleanSupplier supplier) {
    BooleanLogEntry entry = new BooleanLogEntry(dataLog, m_name + "/" + name);
    m_booleanEntries.add(new Pair<>(MonitoredSupplier.fromBooleanSupplier(supplier), entry));
  }

  public void registerString(String name, Supplier<String> supplier) {
    StringLogEntry entry = new StringLogEntry(dataLog, m_name + "/" + name);
    m_stringEntries.add(new Pair<>(new MonitoredSupplier<>(supplier), entry));
  }

  public void registerPose(String name, Supplier<Pose2d> supplier) {
    StructLogEntry<Pose2d> entry =
        StructLogEntry.create(dataLog, m_name + "/" + name, Pose2d.struct);
    m_poseEntries.add(new Pair<>(new MonitoredSupplier<>(supplier), entry));
  }

  public void log() {
    for (Pair<MonitoredSupplier<Double>, DoubleLogEntry> pair : m_doubleEntries) {
      pair.getFirst().ifChanged(d -> pair.getSecond().append(d));
    }
    for (Pair<MonitoredSupplier<Boolean>, BooleanLogEntry> pair : m_booleanEntries) {
      pair.getFirst().ifChanged(b -> pair.getSecond().append(b));
    }
    for (Pair<MonitoredSupplier<String>, StringLogEntry> pair : m_stringEntries) {
      pair.getFirst().ifChanged(s -> pair.getSecond().append(s));
    }
    for (Pair<MonitoredSupplier<Pose2d>, StructLogEntry<Pose2d>> pair : m_poseEntries) {
      pair.getFirst().ifChanged(p -> pair.getSecond().append(p));
    }
  }
}
