/* Generated and Formatted by yours truly <3*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  ShooterSubsystem shooterSubsystem;

  public static Command shootFuel(
      ShooterSubsystem shooterSubsystem,
      DoubleSupplier shooterSpeed,
      DoubleSupplier shooter2Speed,
      DoubleSupplier shooterReduction,
      double kickupSpeed) {
    return Commands.runEnd(
        () -> {
          shooterSubsystem.runShooter(shooterSpeed.getAsDouble() * shooterReduction.getAsDouble());
          shooterSubsystem.runShooter2(
              shooter2Speed.getAsDouble() * shooterReduction.getAsDouble());
          shooterSubsystem.runShooterKickup(kickupSpeed);
          System.out.println(
              "SHooter speed: " + shooter2Speed.getAsDouble() * shooterReduction.getAsDouble());
        },
        () -> {
          shooterSubsystem.runShooter(0.0);
          shooterSubsystem.runShooter2(0.0);
          shooterSubsystem.runShooterKickup(0.0);
        });
  }

  public static Command runShooterKickup(ShooterSubsystem shooterSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          shooterSubsystem.runShooterKickup(speed);
        },
        () -> {
          shooterSubsystem.runShooterKickup(0.0);
        });
  }

  public static Command runShooter(
      ShooterSubsystem shooterSubsystem, DoubleSupplier speed, DoubleSupplier speed2) {
    return Commands.runEnd(
        () -> {
          shooterSubsystem.runShooter(speed.getAsDouble());
          shooterSubsystem.runShooter2(speed2.getAsDouble());
          System.out.println("SHooter speed: " + speed2.getAsDouble());
        },
        () -> {
          shooterSubsystem.runShooter(0.0);
          shooterSubsystem.runShooter2(0.0);
        });
  }
}
