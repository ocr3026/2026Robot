/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ZRobotContainerAbstract.RobotContainerAbstract;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeDepotShoot extends AutoBase {

  public IntakeDepotShoot(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {
    super(hopper, shooter, intake, drive);
    addCommands(delayStartTime());
    // addCommands(lowerIntake(intake));
    addCommands(pathFindToStartPoseSlow(Paths.depotStart));
    addCommands(new ParallelCommandGroup(
        IntakeCommands.intakeFuel(intake, RobotContainerAbstract.intakeSpeed),
        followPath(Paths.depotIntakePath)));
    addCommands(pathFindToPoseLocked(drive, DriveConstants.hubPose, Paths.depotShootPose));
    addCommands(runHopperAndShooterForTime(hopper, shooter, 10.0));
  }
}
