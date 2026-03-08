package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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

	public default void stopMotors() {}
	public default double getSetpoint() {return 0.0d;}
}