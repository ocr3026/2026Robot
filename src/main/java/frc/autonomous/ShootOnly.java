/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootOnly extends AutoBase {
  public ShootOnly(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    addCommands(delayStartTime());
    addCommands(runHopperAndShooter(hopper, shooter));
  }
}
