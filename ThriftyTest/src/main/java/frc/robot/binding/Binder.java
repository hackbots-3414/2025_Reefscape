package frc.robot.binding;

import frc.robot.superstructure.Superstructure;

/**
 * To bind robot actions to event, group trigger bindings to a grouping inside of a Binder
 */
public interface Binder {
  /**
   * Binds all associated bindings for this binder
   */
  void bind(Superstructure superstructure);
}
