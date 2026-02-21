package frc.robot.autonomous;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoBase extends ParallelCommandGroup{
    static Timer timer = new Timer();
    public AutoBase(HopperSubsystem hopper, ShooterSubsystem shooter, IntakeSubsystem intake) {}
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

    public static final ParallelCommandGroup runHopperAndShooter(HopperSubsystem hopper, ShooterSubsystem shooter, double hopperSpeed, double shooterSpeed) {
        return new ParallelCommandGroup(HopperCommands.runHopper(hopper, hopperSpeed), ShooterCommands.shootFuel(shooter, shooterSpeed));
    }

    public static final FunctionalCommand shootFuel(HopperSubsystem hopper, ShooterSubsystem shooter) {
        return new FunctionalCommand(() -> {
            timer.reset();
            timer.start();
        }, () -> {
            runHopperAndShooter(hopper, shooter, -0.05, 0.7);
        }, (interrupted) -> {
            runHopperAndShooter(hopper, shooter, 0, 0);
        }, () -> {
            return timer.hasElapsed(5);
        });
    }

    public static final Command setStartPose(PathPlannerPath path) {
		Pose2d holoPose = path.getStartingHolonomicPose().get();
		return AutoBuilder.resetOdom(holoPose);
	}

    public static final Command lowerIntake(IntakeSubsystem intake) {
        //placeholder pos value, we need to measure real (in rotations)
        return IntakeCommands.intakeLiftPos(intake, -23);
    }

    public static final class Paths {
        public static final PathPlannerPath driveBackSimple = getPathFromFile("Simple Drive Back");
    }

}
