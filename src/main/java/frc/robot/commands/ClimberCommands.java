package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands{
  ClimberSubsystem climberSubsystem;

  public static Command climberUp(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(() -> {
      climberSubsystem.runClimber(speed);
      SmartDashboard.putNumber("CurrentRanspeed", speed);
    }, () -> {
      climberSubsystem.runClimber(0.0);
    });
  }

  public static Command cimberDown(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(() -> {
      climberSubsystem.runClimber(-speed);
    }, () -> {
      climberSubsystem.runClimber(-0.0);
    });
  }
}