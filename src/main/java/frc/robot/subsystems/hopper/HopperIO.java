/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  public static class HopperIOInputs {
    public boolean hopperConnected = false;
    public Angle hopperPosition = Rotations.of(0);
    public AngularVelocity hopperVelocity = RotationsPerSecond.of(0);
    public double hopperAppliedVolts = 0;
    public double hopperCurrentAmps = 0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setHopperSpeed(double speed) {}

  public default void runMotors(double voltage) {}

  public default void stopMotors() {}

  public default void updatePID(double P, double I, double D, double kV, double maxAccel) {}

  public default void runDutyCycle(double speed) {}

  public default double getSetpoint() {
    return 0.0d;
  }
}
