/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class HoardBalls extends AutoBase {

  public HoardBalls(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    super(hopper, shooter, intake, drive);
    addCommands(delayStartTime());
    // addCommands(setStartPose(Paths.hoardBalls));
    addCommands(pathFindToStartPose(Paths.hoardBalls));
    addCommands(followPath(Paths.hoardBalls));
  }
}
