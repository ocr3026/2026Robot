package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final Frequency odometryFrequency = Hertz.of(100);




    //TODO: GET THESE THINGS FINALIZED WHEN POSSIBLE 
    // public static final LinearVelocity maxSpeed = MetersPerSecond.of(5.3);
	// public static final LinearVelocity maxSpeedCoral = MetersPerSecond.of(2);
	// public static final LinearAcceleration maxAccel = FeetPerSecondPerSecond.of(3);
	// public static final LinearAcceleration maxAccelAuto = MetersPerSecondPerSecond.of(1.5);
	// public static final Distance trackWidth = Inches.of(24.75);
	// public static final Distance wheelBase = Inches.of(24.75);
	// public static final Distance driveBaseRadius =
	// 		Meters.of(Math.hypot(trackWidth.div(2).in(Meters), wheelBase.div(2).in(Meters)));
	// public static final Translation2d[] moduleTranslations = new Translation2d[] {
	// 	new Translation2d(trackWidth.div(2).in(Meters), wheelBase.div(2).in(Meters)),
	// 	new Translation2d(
	// 			trackWidth.div(2).in(Meters), wheelBase.div(2).unaryMinus().in(Meters)),
	// 	new Translation2d(
	// 			trackWidth.div(2).unaryMinus().in(Meters), wheelBase.div(2).in(Meters)),
	// 	new Translation2d(
	// 			trackWidth.div(2).unaryMinus().in(Meters),
	// 			wheelBase.div(2).unaryMinus().in(Meters))
	// };
}
