/* Generated and Formatted by yours truly <3*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands {
  ClimberSubsystem climberSubsystem;

  public static Command runClimber(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          double adjSpeed = speed * (ClimberConstants.climberClockwise ? -1 : 1);
          ClimberConstants.upNegative = (adjSpeed < 0);
          climberSubsystem.climberUp(adjSpeed);
          SmartDashboard.putNumber("CurrentRanspeed", speed);
        },
        () -> {
          climberSubsystem.stopMotor();
        });
  }

  public static Command setClimberPos(ClimberSubsystem subsystem, double pos) {
    return Commands.runOnce(() -> {
      subsystem.setClimberPos(pos);
    });
  }

  public static Command reverseCLimber(ClimberSubsystem climberSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          double adjSpeed = speed * (ClimberConstants.climberClockwise ? -1 : 1);
          ClimberConstants.downNegative = (adjSpeed < 0);
          climberSubsystem.climberDown(adjSpeed);
        },
        () -> {
          climberSubsystem.stopMotor();
        });
  }
}
