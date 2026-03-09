/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RightMidPickup extends AutoBase {

  public RightMidPickup(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    super(hopper, shooter, intake, drive);
    addCommands(setStartPose(Paths.driveBackSimple));

    addCommands(delayStartTime());
    addCommands(lowerIntake(intake));
    addCommands(pathFindToStartPose(Paths.midRightPickup));
    addCommands(followPathAndIntake(Paths.midRightPickup, intake));
    addCommands(pathFindToPoseLocked(drive, Paths.aimTurret, Paths.rightShoot));
    addCommands(runHopperAndShooter(hopper, shooter));
  }
}
