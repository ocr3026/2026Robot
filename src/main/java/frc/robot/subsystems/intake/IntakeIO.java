package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
	@AutoLog
	public static class IntakeIOInputs {
		public boolean intakeConnected = false;
		public Angle intakePosition = Degrees.of(0);
		public AngularVelocity intakeVelocity = DegreesPerSecond.of(0);
		public double intakeAppliedVolts = 0;
		public double intakeCurrentAmps = 0;
	}

	public default void updateInputs(IntakeIOInputs inputs) {}

	public default void setAngularSpeed(double speed) {}

	public default void runMotors(double voltage) {}

	public default void stopMotors() {}
}