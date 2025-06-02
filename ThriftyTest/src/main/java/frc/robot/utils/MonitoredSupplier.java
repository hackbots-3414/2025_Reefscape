package frc.robot.utils;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class MonitoredSupplier<T> implements Supplier<T> {
  private T m_prev;
  
  private final Supplier<T> m_supplier;

  public MonitoredSupplier(Supplier<T> supplier) {
    m_supplier = supplier;
  }

  public void ifChanged(Consumer<T> action) {
    T curr = m_supplier.get();
    if (!curr.equals(m_prev)) {
      m_prev = curr;
      action.accept(curr);
    }
  }

  public static MonitoredSupplier<Double> fromDoubleSuplier(DoubleSupplier supplier) {
    return new MonitoredSupplier<>(supplier::getAsDouble);
  }

  public static MonitoredSupplier<Boolean> fromBooleanSupplier(BooleanSupplier supplier) {
    return new MonitoredSupplier<Boolean>(supplier::getAsBoolean);
  }

  public T get() {
    return m_supplier.get();
  }
}
