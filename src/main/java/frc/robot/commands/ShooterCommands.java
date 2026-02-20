package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommands{
 ShooterSubsystem shooterSubsystem;

  public static Command shootFuel(ShooterSubsystem shooterSubsystem, double speed) {
    return Commands.runEnd(() -> {
      shooterSubsystem.runShooter(speed);
    }, () -> {
      shooterSubsystem.runShooter(0.0);
    });
  }
}