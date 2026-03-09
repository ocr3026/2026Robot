/* Generated and Formatted by yours truly <3*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands {
  ClimberSubsystem climberSubsystem;

  public static Command runClimber(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          climberSubsystem.climberUp(speed);
          SmartDashboard.putNumber("CurrentRanspeed", speed);
        },
        () -> {
          climberSubsystem.climberUp(0.0);
        });
  }

  public static Command setClimberPos(ClimberSubsystem subsystem, double pos) {
    return Commands.runOnce(() -> {
      subsystem.climberUp(pos);
    });
  }

  public static Command reverseCLimber(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          climberSubsystem.climberDown(speed);
        },
        () -> {
          climberSubsystem.climberDown(0.0);
        });
  }
}
