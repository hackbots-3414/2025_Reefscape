package frc.robot.driveassist;

import frc.robot.Constants.FFConstants;

public class ForceField {
  public static double calculate(double velo, double disp) {
    if (disp * velo <= 0) {
      // we're going in the same direction as displacement, i.e. away from barge
      return velo;
    }
    double maxVelocity = Math.sqrt(
        2 * FFConstants.k_decceleration * Math.max(0, Math.abs(disp) - FFConstants.k_radius));
    return Math.min(maxVelocity, Math.abs(velo)) * Math.signum(velo);
  }
}
