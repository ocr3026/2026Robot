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

  public static Transform3d robotToCamera1 = new Transform3d(
      new Translation3d(0.320790, 0.496752, 0.398105),
      new Rotation3d(-0.006354, 0.049945, -0.963226));

  //   new Transform3d(
  //       new Translation3d(0.284608, 0.366303, 0.271689),
  //       new Rotation3d(-0.000084, 0.027998, 0.779397));

  // new Transform3d(
  //     new Translation3d(Inches.of(-13).in(Meters), Inches.of(-13.5).in(Meters), 0.3),
  //     new Rotation3d(0.009869, 0.022058, -Math.PI / 4));
  // public static Transform3d robotToCamera1 = new Transform3d(new Translation3d(0.284608,
  // 0.366303, 0.271689), new Rotation3d(-0.000084, 0.027998, 0.779397));
  public static Transform3d robotToCamera0 = new Transform3d(
      new Translation3d(0.278442, -0.359510, 1.208001),
      new Rotation3d(-0.200164, 0.053835, 0.948172));

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
