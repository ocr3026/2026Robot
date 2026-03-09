/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeConnected = false;
    public Angle intakePosition = Rotations.of(0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0);
    public double intakeAppliedVolts = 0;
    public double intakeCurrentAmps = 0;

    public boolean intakeLiftConnected = false;
    public Angle intakeLiftPosition = Rotations.of(0);
    public AngularVelocity intakeLiftVelocity = RotationsPerSecond.of(0);
    public double intakeLiftAppliedVolts = 0;
    public double intakeLiftCurrentAmps = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeLiftSpeed(double speed) {}

  public default void setIntakeSpeed(double speed) {}

  public default void stopMotors() {}

  public default double getIntakePosition() {
    return 0.0d;
  }

  public default void updatePID(
      double iP,
      double iI,
      double iD,
      double iV,
      double iA,
      double lP,
      double lI,
      double lD,
      double lV,
      double lA) {}

  public default void setIntakeLiftPos(double pos) {}

  public default void zeroIntakeLift() {}
}
