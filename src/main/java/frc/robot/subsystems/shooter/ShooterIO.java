/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterConnected = false;
    public Angle shooterPosition = Degrees.of(0);
    public AngularVelocity shooterVelocity = DegreesPerSecond.of(0);
    public double shooterAppliedVolts = 0;
    public double shooterCurrentAmps = 0;

    public boolean shooter2Connected = false;
    public Angle shooter2Position = Degrees.of(0);
    public AngularVelocity shooter2Velocity = DegreesPerSecond.of(0);
    public double shooter2AppliedVolts = 0;
    public double shooter2CurrentAmps = 0;

    public boolean shooterKickupConnected = false;
    public Angle shooterKickupPosition = Degrees.of(0);
    public AngularVelocity shooterKickupVelocity = DegreesPerSecond.of(0);
    public double shooterKickupAppliedVolts = 0;
    public double shooterKickupCurrentAmps = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterSpeed(double speed) {}

  public default void setShooter2Speed(double speed) {}

  public default void setKickupSpeed(double speed) {}

  public default void updatePID(
      double sP,
      double sI,
      double sD,
      double sV,
      double sMaxAccel,
      double s2P,
      double s2I,
      double s2D,
      double s2V,
      double s2MaxAccel,
      double kickP,
      double kickI,
      double kickD,
      double kickV,
      double kickAccel) {}

  public default void stopMotors() {}

  public default double getSetpoint() {
    return 0.0d;
  }
}
