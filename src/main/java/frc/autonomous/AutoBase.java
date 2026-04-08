/* Generated and Formatted by yours truly <3*/
package frc.autonomous;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ZRobotContainerAbstract.RobotContainerAbstract;
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

  static PIDController turretController = new PIDController(5.8, 0.0, ANGLE_KD);

  static PPHolonomicDriveController controller = new PPHolonomicDriveController(
      new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(0.5, 0.0, 0.0));

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

  public static final Command followPathYOnly(PathPlannerPath path) {
    return AutoBuilder.followPath(path).beforeStarting(() -> {
      PPHolonomicDriveController.overrideRotationFeedback(() -> 0.0);
    });
  }

  public static final Command pathFindToStartPose(PathPlannerPath path) {
    return AutoBuilder.pathfindToPoseFlipped(
        path.getStartingHolonomicPose().get(), DriveConstants.PATH_CONSTRAINTS);
  }

  public static final Command pathFindToStartPoseSlow(PathPlannerPath path) {
    return AutoBuilder.pathfindToPoseFlipped(
            path.getStartingHolonomicPose().get(), DriveConstants.PATH_CONSTRAINTS_SLOW)
        .beforeStarting(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
        });
  }

  public static PathPlannerTrajectory currentTrajectory;
  public static boolean isFlipped;

  public static final Command pathFindToStartPoseNoRotation(
      PathPlannerPath path, DriveSubsystem drive) {

    return Commands.run(
            () -> {
              PathPlannerTrajectoryState state = currentTrajectory.sample(timer.get());
              ChassisSpeeds speeds =
                  controller.calculateRobotRelativeSpeeds(drive.getPose(), state);

              Logger.recordOutput("PPState", state.pose);
              Logger.recordOutput(
                  "ChassisSpeeds",
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              Logger.recordOutput("Whatwe think the drive pose is: ", drive.getPose());
              speeds.omegaRadiansPerSecond = 0;
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
            },
            drive)
        .beforeStarting(() -> {
          PathPlannerPath newpath = new PathPlannerPath(
              PathPlannerPath.waypointsFromPoses(
                  drive.getPose(), path.getStartingHolonomicPose().get()),
              DriveConstants.PATH_CONSTRAINTS_SLOW,
              new IdealStartingState(0, drive.getRotation()),
              new GoalEndState(0, path.getStartingHolonomicPose().get().getRotation()));
          isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          timer.stop();
          timer.reset();
          controller.setEnabled(true);

          currentTrajectory = newpath.generateTrajectory(
              new ChassisSpeeds(), drive.getRotation(), DriveConstants.PP_CONFIG);
          timer.start();

          for (PathPlannerTrajectoryState state : currentTrajectory.getStates()) {
            state.fieldSpeeds.omegaRadiansPerSecond = 0;
          }
        });
    // return new FunctionalCommand(
    //     () -> {
    //       // for (PathPlannerTrajectoryState state : traj.getStates()) {
    //       //   state.fieldSpeeds.omegaRadiansPerSecond = 0;
    //       // }
    //       timer.stop();
    //       timer.reset();
    //       timer.start();
    //     },
    //     () -> {
    //       PathPlannerTrajectoryState state = traj.sample(timer.get());
    //       ChassisSpeeds robotRel = controller.calculateRobotRelativeSpeeds(drive.getPose(),
    // state);
    //       ChassisSpeeds fieldRel =
    //           ChassisSpeeds.fromRobotRelativeSpeeds(robotRel, drive.getRotation());

    //       Logger.recordOutput("PPState", state.pose);
    //       Logger.recordOutput("ChassisSpeeds", fieldRel);
    //       drive.runVelocity(fieldRel);
    //     },
    //     (interrupted) -> {},
    //     () -> {
    //       return false;
    //     });
  }

  public static final ParallelCommandGroup runHopperAndShooter(
      HopperSubsystem hopper, ShooterSubsystem shooter) {
    return new ParallelCommandGroup(
        HopperCommands.runHopper(hopper, RobotContainerAbstract.hopperSpeed),
        ShooterCommands.shootFuel(
            shooter,
            () -> RobotContainerAbstract.shooterSpeed,
            () -> RobotContainerAbstract.shooter2Speed,
            RobotContainerAbstract.shooterKickupSpeed));
  }

  public static final ParallelCommandGroup stopHopperAndShooter(
      HopperSubsystem hopper, ShooterSubsystem shooter) {
    return new ParallelCommandGroup(
        HopperCommands.runHopper(hopper, 0.0),
        ShooterCommands.shootFuel(shooter, () -> 0.0, () -> 0.0, 0.0));
  }

  public static final FunctionalCommand lowerIntake(IntakeSubsystem intake) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          intake.runIntakeLiftUntil(RobotContainerAbstract.intakeLiftPos, -0.1);
        },
        (interrupted) -> {
          intake.intakeLift(0.0);
        },
        () -> {
          return (intake.getIntakeLiftPos() >= RobotContainerAbstract.intakeLiftPos);
        });
  }

  // public static final Command raiseIntake(IntakeSubsystem intake) {
  //   return new FunctionalCommand(() -> {}, () -> {
  //     intake.runIntake
  //   }, null, null, null);
  // }

  public static final ParallelCommandGroup shootAndIntakeUp(
      ShooterSubsystem shooter, IntakeSubsystem intake, HopperSubsystem hopper) {
    return new ParallelCommandGroup(
        IntakeCommands.intakeRunLess(intake, -5, 0.1), runHopperAndShooter(hopper, shooter));
  }

  public static final ParallelRaceGroup followPathAndIntake(
      PathPlannerPath path, IntakeSubsystem intake) {
    return new ParallelRaceGroup(
        followPath(path), IntakeCommands.intakeFuel(intake, RobotContainerAbstract.intakeSpeed));
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

  public static final Command followPathLocked(
      DriveSubsystem drive, PathPlannerPath posePath, PathPlannerPath path) {
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return AutoBuilder.followPath(path)
        .beforeStarting(() -> {
          PPHolonomicDriveController.overrideRotationFeedback(() -> angleController.calculate(
              0,
              drive.getDeltaRotation(posePath.getStartingHolonomicPose().get(), drive.getPose())));
        })
        .finallyDo(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
        });
  }

  /** @param drive DriveSubsystem
   * @param poseToLockOnTo The starting pose of the path that you want the robot to lock on to
   * @param poseToPathfindTo The starting pose of the path that you want to end up at */
  public static final Command pathFindToPoseLocked(
      DriveSubsystem drive, Pose2d poseToLockOnTo, PathPlannerPath poseToPathfindTo) {
    turretController.enableContinuousInput(-Math.PI, Math.PI);
    turretController.setTolerance(0.007);

    return AutoBuilder.pathfindToPoseFlipped(
            poseToPathfindTo.getStartingHolonomicPose().get(), DriveConstants.PATH_CONSTRAINTS)
        .alongWith(Commands.waitUntil(() -> turretController.atSetpoint()))
        .beforeStarting(() -> {
          PPHolonomicDriveController.overrideRotationFeedback(() -> -turretController.calculate(
              drive.getPose().getRotation().minus(new Rotation2d(Math.PI)).getRadians(),
              drive.getTargetRotation(poseToLockOnTo, drive.getPose())));
        })
        .finallyDo(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
          turretController.close();
        });
  }

  public static final Command pathFindToPoseOneRotation(
      DriveSubsystem drive, Pose2d poseWithDesiredRotation, PathPlannerPath poseToPathfindTo) {
    turretController.enableContinuousInput(-Math.PI, Math.PI);
    turretController.setTolerance(0.007);

    return AutoBuilder.pathfindToPoseFlipped(
            poseToPathfindTo.getStartingHolonomicPose().get(), DriveConstants.PATH_CONSTRAINTS)
        .alongWith(Commands.waitUntil(() -> turretController.atSetpoint()))
        .beforeStarting(() -> {
          PPHolonomicDriveController.overrideRotationFeedback(() -> -turretController.calculate(
              drive.getPose().getRotation().minus(new Rotation2d(Math.PI)).getRadians(),
              poseWithDesiredRotation
                  .getRotation()
                  .minus(new Rotation2d(Math.PI))
                  .getRadians()));
        })
        .finallyDo(() -> {
          PPHolonomicDriveController.clearRotationFeedbackOverride();
          turretController.close();
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
    public static final PathPlannerPath leftShoot = getPathFromFile("LeftShoot");
    public static final PathPlannerPath leftClimb = getPathFromFile("LeftClimb");
    public static final PathPlannerPath leftClimbSlow = getPathFromFile("LeftClimbSlow");
    public static final PathPlannerPath leftLadderPose = getPathFromFile("LeftLadderPose");
  }
}
