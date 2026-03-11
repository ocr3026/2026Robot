/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean climberConnected = false;
    public double climberPosition = (0);
    public AngularVelocity climberVelocity = DegreesPerSecond.of(0);
    public double climberAppliedVolts = 0;
    public double climberCurrentAmps = 0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberSpeed(double speed) {}

  public default void runMotors(double voltage) {}

  public default void updatePID(double p, double i, double d, double v) {}

  public default double getSetpoint() {
    return 0.0d;
  }

  public default double getClimberPosition() {
    return 0.0d;
  }

  public default void setClimberPos(double pos) {}

  public default void zeroClimber() {}
}
