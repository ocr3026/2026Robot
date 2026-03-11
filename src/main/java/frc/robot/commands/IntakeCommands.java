/* Generated and Formatted by yours truly <3*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {
  IntakeSubsystem intakeSubsystem;

  public static Command intakeFuel(IntakeSubsystem intakeSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          intakeSubsystem.intakeFuel(speed);
        },
        () -> {
          intakeSubsystem.intakeFuel(0.0);
        });
  }

  public static Command intakeLift(IntakeSubsystem intakeSubsystem, double speed) {
    return Commands.runEnd(
        () -> {
          intakeSubsystem.intakeLift(speed);
        },
        () -> {
          intakeSubsystem.intakeLift(0.0);
        });
  }

  public static Command intakeLiftPos(IntakeSubsystem intake, double pos) {
    return Commands.runOnce(() -> {
      intake.intakeLiftPos(pos);
    });
  }

  public static Command intakeRunGreater(IntakeSubsystem intake, double pos, double speed) {
    Logger.recordOutput("wow", pos);
    return Commands.runEnd(
        () -> {
          if (intake.getIntakeLiftPos() > pos) {
            intake.runIntakeLiftUntil(pos, speed);
          } else {
            intake.runIntakeLiftUntil(pos, 0.0);
          }
        },
        () -> {
          intake.runIntakeLiftUntil(pos, 0.0);
        });
  }
    public static Command intakeRunLess(IntakeSubsystem intake, double pos, double speed) {
    Logger.recordOutput("wow", pos);
    return Commands.runEnd(
        () -> {
          if (intake.getIntakeLiftPos() < pos) {
            intake.runIntakeLiftUntil(pos, speed);
          } else {
            intake.runIntakeLiftUntil(pos, 0.0);
          }
        },
        () -> {
          intake.runIntakeLiftUntil(pos, 0.0);
        });
  }
}
