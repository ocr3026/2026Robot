package frc.robot.autonomous;

import frc.robot.commands.HopperCommands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class DriveBackAndShoot extends AutoBase {

    public DriveBackAndShoot(HopperSubsystem hopper, ShooterSubsystem shooter, IntakeSubsystem intake) {
        super(hopper, shooter, intake);
        addCommands(delayStartTime());
        addCommands(setStartPose(Paths.driveBackSimple));
        addCommands(lowerIntake(intake));
        addCommands(followPath(Paths.driveBackSimple));
        addCommands(shootFuel(hopper, shooter));
    }
}
