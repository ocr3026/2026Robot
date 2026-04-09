/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RightTrenchShootThenMove extends AutoBase {

  public RightTrenchShootThenMove(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    super(hopper, shooter, intake, drive);

    addCommands(runHopperAndShooterForTime(hopper, shooter, 5));
    addCommands(pathFindToStartPose(Paths.rightShoot));
    addCommands(lowerIntake(intake));
  }
}
