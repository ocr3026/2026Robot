/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutoBase extends SequentialCommandGroup {
  static Timer timer = new Timer();

  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final AngularVelocity ANGLE_MAX_VELOCITY = RadiansPerSecond.of(8.0);
  private static final AngularAcceleration ANGLE_MAX_ACCELERATION =
      RadiansPerSecondPerSecond.of(20.0);

  static ProfiledPIDController angleController = new ProfiledPIDController(
      ANGLE_KP,
      0.0,
      ANGLE_KD,
      new TrapezoidProfile.Constraints(
          ANGLE_MAX_VELOCITY.in(RadiansPerSecond),
          ANGLE_MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

  public AutoBase(
      HopperSubsystem hopper,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      DriveSubsystem drive) {}
  /**
   * @param name
   * @return PathPlannerPath
   */
  public static PathPlannerPath getPathFromFile(String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      return path;
    } catch (Exception e) {
      DriverStation.reportError("Cant Find Path : " + e.getMessage(), e.getStackTrace());
      SmartDashboard.putString("PathErrors", "Cant Find Path : " + name);
      return null;
    }
  }

  /**
   * @param time
   * @return Command
   */
  public static final Command wait(double time) {
    return new WaitCommand(time);
  }

  public static final Command delayStartTime() {
    return new FunctionalCommand(
        () -> {
          timer.reset();
          timer.start();
        },
        () -> {},
        (interupted) -> {
          timer.stop();
          timer.reset();
        },
        () -> {
          return timer.hasElapsed(SmartDashboard.getNumber("delayStartTime", 0));
        });
  }

  public static final Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public static final Command pathFindToStartPose(PathPlannerPath path) {
    return AutoBuilder.pathfindToPoseFlipped(
        path.getStartingHolonomicPose().get(), DriveConstants.PATH_CONSTRAINTS);
  }

  public static final ParallelCommandGroup runHopperAndShooter(
      HopperSubsystem hopper, ShooterSubsystem shooter) {
    return new ParallelCommandGroup(
        HopperCommands.runHopper(hopper, RobotContainer.hopperSpeed),
        ShooterCommands.shootFuel(
            shooter,
            RobotContainer.shooterSpeed,
            RobotContainer.shooter2Speed,
            RobotContainer.shooterKickupSpeed));
  }

  public static final ParallelRaceGroup followPathAndIntake(
      PathPlannerPath path, IntakeSubsystem intake) {
    return new ParallelRaceGroup(
        followPath(path), IntakeCommands.intakeFuel(intake, RobotContainer.intakeSpeed));
  }

  public static final FunctionalCommand shootFuel(
      HopperSubsystem hopper, ShooterSubsystem shooter) {
    return new FunctionalCommand(
        () -> {
          timer.reset();
          timer.start();
        },
        () -> {
          hopper.runHopper(-200);
          shooter.runShooter(-2500);
          shooter.runShooter2(2500);
          shooter.runShooterKickup(-2500);
          SmartDashboard.putString("status", "larping that diddy foid on five");
        },
        (interrupted) -> {
          hopper.runHopper(0);
          shooter.runShooter(0);
          shooter.runShooter2(0);
          shooter.runShooterKickup(0);
        },
        () -> {
          return timer.hasElapsed(5);
        });
  }

  public static final Command runHopper(HopperSubsystem hopper) {
    return HopperCommands.runHopper(hopper, -200);
  }

  public static final Command setStartPose(PathPlannerPath path) {
    Pose2d holoPose = path.getStartingHolonomicPose().get();
    return AutoBuilder.resetOdom(holoPose);
  }

  public static final Command lowerIntake(IntakeSubsystem intake) {
    // placeholder pos value, we need to measure real (in rotations)
    Logger.recordOutput("IntakeLowerStatus", "Lowering intake!");
    return IntakeCommands.intakeLiftPos(intake, -23);
  }

  public static final Command followPathLocked(
      DriveSubsystem drive, PathPlannerPath posePath, PathPlannerPath path) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return AutoBuilder.followPath(path)
        .beforeStarting(() -> {
          PPHolonomicDriveController.overrideRotationFeedback(() -> angleController.calculate(
              0, drive.getDeltaRotation(posePath.getStartingHolonomicPose().get())));
        })
        .finallyDo(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
        });
  }

  public static final Command pathFindToPoseLocked(
      DriveSubsystem drive, PathPlannerPath posePath, PathPlannerPath path) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    Pose2d pose = new Pose2d(
        path.getStartingHolonomicPose().get().getX(),
        path.getStartingHolonomicPose().get().getY(),
        drive.getPose().getRotation());
    return AutoBuilder.pathfindToPoseFlipped(
            new Pose2d(
                path.getStartingHolonomicPose().get().getX(),
                path.getStartingHolonomicPose().get().getY(),
                new Rotation2d(drive.getTargetRotation(
                    posePath.getStartingHolonomicPose().get(),
                    path.getStartingHolonomicPose().get()))),
            DriveConstants.PATH_CONSTRAINTS)
        .beforeStarting(() -> {
          PPHolonomicDriveController.overrideRotationFeedback(() -> angleController.calculate(
              0, drive.getDeltaRotation(posePath.getStartingHolonomicPose().get())));
        })
        .finallyDo(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
        });
  }

  public static final Command pathFindToContinousPoseLocked(
      DriveSubsystem drive, PathPlannerPath posePath, PathPlannerPath path) {
    return Commands.startRun(
        () -> {
          angleController.enableContinuousInput(-Math.PI, Math.PI);
        },
        () -> {
          AutoBuilder.pathfindToPoseFlipped(
              new Pose2d(
                  path.getStartingHolonomicPose().get().getX(),
                  path.getStartingHolonomicPose().get().getY(),
                  drive.getPose().getRotation()),
              DriveConstants.PATH_CONSTRAINTS);
        });
  }

  public static final class Paths {
    public static final PathPlannerPath driveBackSimple = getPathFromFile("Simple Drive Back");
    public static final PathPlannerPath hoardBalls = getPathFromFile("Push Balls");
    public static final PathPlannerPath rightShoot = getPathFromFile("RightShoot");
    public static final PathPlannerPath midRightPickup = getPathFromFile("RightMidPickup");
    public static final PathPlannerPath aimTurret = getPathFromFile("Turret Aim Point");
  }
}
