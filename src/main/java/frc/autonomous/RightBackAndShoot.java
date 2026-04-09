/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RightBackAndShoot extends AutoBase {

  public RightBackAndShoot(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    super(hopper, shooter, intake, drive);
    addCommands(delayStartTime());
    // addCommands(setStartPose(Paths.driveBackSimple));
    addCommands(lowerIntake(intake));
    addCommands(pathFindToPoseLocked(drive, DriveConstants.hubPose, Paths.rightShoot));
    // addCommands(followPath(Paths.driveBackSimple));

    // addCommands(shootFuel(hopper, shooter));
    // addCommands(HopperCommands.runHopper(hopper, -200));
    addCommands(runHopperAndShooter(hopper, shooter));
  }
}
