/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ClimbAutoTest extends AutoBase {

  public ClimbAutoTest(ClimberSubsystem climber, DriveSubsystem drive) {
    // addCommands(pathFindToStartPose(Paths.leftClimb));
    // addCommands(
    //     followPath(Paths.leftClimb).alongWith(ClimberCommands.runClimberAuto(climber, -1.0)));
    // addCommands(pathFindToPoseOneRotation(
    //     drive, Paths.leftClimbSlow.getStartingHolonomicPose().get(), Paths.leftClimbSlow));
    // addCommands(followPathYOnly(Paths.leftClimbSlow));

    addCommands(pathFindToStartPoseNoRotation(Paths.leftShoot, drive));
  }
}
