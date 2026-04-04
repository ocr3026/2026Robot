// /* Generated and Formatted by yours truly <3*/
// package frc.autonomous;

// import frc.robot.subsystems.drive.DriveConstants;
// import frc.robot.subsystems.drive.DriveSubsystem;
// import frc.robot.subsystems.hopper.txt.HopperSubsystem;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem;

// public class DriveBackAndShoot extends AutoBase {

//   public DriveBackAndShoot(
//       HopperSubsystem hopper,
//       ShooterSubsystem shooter,
//       IntakeSubsystem intake,
//       DriveSubsystem drive) {
//     addCommands(delayStartTime());
//     // addCommands(setStartPose(Paths.driveBackSimple));
//     addCommands(lowerIntake(intake));
//     addCommands(pathFindToPoseLocked(drive, DriveConstants.hubPose, Paths.leftShoot));
//     // addCommands(shootFuel(hopper, shooter));
//     // addCommands(HopperCommands.runHopper(hopper, -200));
//     addCommands(runHopperAndShooter(hopper, shooter));
//   }
// }
