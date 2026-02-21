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

		public boolean intakeLiftConnected = false;
		public Angle intakeLiftPosition = Degrees.of(0);
		public AngularVelocity intakeLiftVelocity = DegreesPerSecond.of(0);
		public double intakeLiftAppliedVolts = 0;
		public double intakeLiftCurrentAmps = 0;
	}

	public default void updateInputs(IntakeIOInputs inputs) {}

	public default void setIntakeLiftSpeed(double speed) {}

	public default void setIntakeSpeed(double speed) {}

	public default void stopMotors() {}

	public default double getIntakePosition() {return 0.0d;}

	public default void setIntakeLiftPos(double pos) {}

	public default void zeroIntakeLift() {}
}