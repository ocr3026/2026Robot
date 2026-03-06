package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class HopperCommands{
  HopperSubsystem hopperSubsystem;

  public static Command runHopper(HopperSubsystem hopperSubsystem, double speed) {
    return Commands.runEnd(() -> {
      hopperSubsystem.runHopper(speed);
      SmartDashboard.putNumber("CurrentRanspeed", speed);
    }, () -> {
      hopperSubsystem.stopHopper();
    });
  }

  public static Command runHopperLight(HopperSubsystem subsystem, double speed) {
    return Commands.runEnd(() -> {
      subsystem.setHopperDuty(speed);
    }, () -> {
      subsystem.setHopperDuty(0.0);
    });
  }

  public static Command reverseHopper(HopperSubsystem hopperSubsystem, double speed) {
    return Commands.runEnd(() -> {
      hopperSubsystem.reverseHopper(speed);
    }, () -> {
      hopperSubsystem.stopHopper();
    });
  }
}
