/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class VisionConstants {

  // TODO : UPDATE THE VALUES WHEN THE ROBOT IS BUILT TS
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static String camera0Name = "AprilTagWebcam";
  public static String camer1Name = "AprilWebcam2";

  // public static Transform3d robotToCamera0 = new Transform3d(
  //     new Translation3d(0.322175, -0.046373, 0.222987),
  //     new Rotation3d(0.009869, 0.022058, -0.011891));

  // Positive Positive cammera - april 5 2026
  public static Transform3d robotToCamera0 = new Transform3d(
      new Translation3d(0.253182, 0.182888, 0.381300),
      new Rotation3d(0.128744, -0.392545, 0.267920));

  // Positive Negative cammera - april 5 2026

  public static Transform3d robotToCamera1 = new Transform3d(
      new Translation3d(0.157125, -0.479281, 0.498840),
      new Rotation3d(0.034032, -0.097710, -0.180796));

  //   new Transform3d(
  //       new Translation3d(0.239132, -0.498375, 0.492634),
  //       new Rotation3d(0.035506, -0.101815, -0.188851));

  //   new Transform3d(
  //       new Translation3d(0.366303, -0.284608, 0.271689),
  //       new Rotation3d(-0.000084, 0.027998, -0.791399));

  public static double maxAmbiguity = 0.3;

  public static double maxZError = 0.75;

  public static Distance linearStdDevBaseline = Meters.of(0.02);
  public static Angle angularStdDevBaseline = Radians.of(0.06);

  public static double[] cameraStdDevFactors = new double[] {1.0, 1.0};

  // public static double linearStdDevMegatag2Factor = 0.5;
  // public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
