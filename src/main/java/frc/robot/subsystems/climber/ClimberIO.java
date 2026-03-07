package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ClimberIO {
    @AutoLog
	public static class ClimberIOInputs {
		public boolean climberConnected = false;
		public Angle climberPosition = Degrees.of(0);
		public AngularVelocity climberVelocity = DegreesPerSecond.of(0);
		public double climberAppliedVolts = 0;
		public double climberCurrentAmps = 0;
		
	}

	public default void updateInputs(ClimberIOInputs inputs) {}

	public default void setClimberSpeed(double speed) {}

	public default void runMotors(double voltage) {}

	public default void stopMotors() {}

	public default double getSetpoint() {return 0.0d;}

	public default double getClimberPosition() {return 0.0d;}

	public default void setClimberPos(double pos) {}

	public default void zeroClimber() {}
}
