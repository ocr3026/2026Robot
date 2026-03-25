/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Test extends AutoBase {

  public Test(DriveSubsystem drive) {
    // addCommands(setStartPose(Paths.driveBackSimple));
    addCommands(pathFindToPoseLocked(drive, DriveConstants.hubPose, Paths.leftShoot));
    // addCommands(runHopperAndShooter(hopper, shooter));
  }
}
