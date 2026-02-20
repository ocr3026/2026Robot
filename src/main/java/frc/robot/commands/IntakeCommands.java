package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommands{
 IntakeSubsystem intakeSubsystem;

  public static Command intakeFuel(IntakeSubsystem intakeSubsystem, double speed) {
    return Commands.runEnd(() -> {
      intakeSubsystem.intakeFuel(speed);
    }, () -> {
      intakeSubsystem.intakeFuel(0.0);
    });
  }

  public static Command intakeLift(IntakeSubsystem intakeSubsystem, double speed) {
    return Commands.runEnd(() -> {
      intakeSubsystem.intakeLift(speed);
    }, () -> {
      intakeSubsystem.intakeLift(0.0);
    });
  }
}