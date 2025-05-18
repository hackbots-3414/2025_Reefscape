package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

@SuppressWarnings("rawtypes")
public class StatusSignalUtil {
  private static StatusSignal[] m_rioSignals = new StatusSignal[0];
  private static StatusSignal[] m_canivoreSignals = new StatusSignal[0];

  public static void registerRioSignals(StatusSignal... signals) {
    StatusSignal[] newSignals = new StatusSignal[m_rioSignals.length + signals.length];
    System.arraycopy(m_rioSignals, 0, newSignals, 0, m_rioSignals.length);
    System.arraycopy(signals, 0, newSignals, m_rioSignals.length, signals.length);
    m_rioSignals = newSignals;
  }

  public static void registerCANivoreSignals(StatusSignal... signals) {
    StatusSignal[] newSignals = new StatusSignal[m_canivoreSignals.length + signals.length];
    System.arraycopy(m_canivoreSignals, 0, newSignals, 0, m_canivoreSignals.length);
    System.arraycopy(signals, 0, newSignals, m_canivoreSignals.length, signals.length);
    m_canivoreSignals = newSignals;
  }

  public static void refreshAll() {
    BaseStatusSignal.refreshAll(m_rioSignals);
    BaseStatusSignal.refreshAll(m_canivoreSignals);
  }
}
