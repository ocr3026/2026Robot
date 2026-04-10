/* Generated and Formatted by yours truly <3*/
package frc.robot.ZRobotContainerAbstract;

import com.orangefrc.annotation.GSON;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Keybinds;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalon;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TopRobotContainer extends RobotContainerAbstract {

  @Override
  public void configureBindings() {
    Keybinds.intakeFuel.whileTrue(IntakeCommands.intakeFuel(intake, intakeSpeed));
    Keybinds.intakeLiftUp.whileTrue(IntakeCommands.intakeLift(intake, intakeLiftSpeed));
    // Keybinds.intakeLiftDown.whileTrue(IntakeCommands.intakeLift(intake, -intakeLiftSpeed));
    Keybinds.intakeLiftDown.whileTrue(IntakeCommands.intakeRunGreater(intake, intakeLiftPos, -0.1));
    // Keybinds.intakeLiftUp.whileTrue(IntakeCommands.intakeRunGreater(intake, -5, 0.1));
    Keybinds.reverseIntake.whileTrue(IntakeCommands.intakeFuel(intake, -intakeSpeed));

    Keybinds.shootFuel.whileTrue(new ParallelCommandGroup(
        ShooterCommands.shootFuel(
            shooter,
            () -> -shooterSpeed,
            () -> shooterSpeed,
            () -> (-rotationJoystick.getZ() + shooterReductionOffset) * 0.2 + 1.0,
            shooterKickupSpeed),
        HopperCommands.runHopper(hopper, hopperSpeed)));
    // Keybinds.shooterFlywheel.whileTrue(
    //     ShooterCommands.runShooter(shooter, () -> -shooter2Speed, () -> shooter2Speed));

    Keybinds.shooterFlywheel.whileTrue(ShooterCommands.shootFuel(
        shooter,
        () -> -shooterSpeed,
        () -> shooterSpeed,
        () -> (-rotationJoystick.getZ() + shooterReductionOffset) * 0.2 + 1.0,
        1000));

    Keybinds.runHopper.whileTrue(HopperCommands.runHopper(hopper, hopperSpeed));
    Keybinds.reverseHopper.whileTrue(HopperCommands.reverseHopper(hopper, -hopperSpeed));

    Keybinds.climberUp.whileTrue(ClimberCommands.runClimber(climber, -1.0));
    Keybinds.climberDown.whileTrue(ClimberCommands.reverseCLimber(climber, 1.0));

    Keybinds.climberPosUp.whileTrue(ClimberCommands.runClimber(climber, -0.1));
    Keybinds.climberPosDown.whileTrue(ClimberCommands.reverseCLimber(climber, 0.1));

    Keybinds.zeroShooterReduction.onTrue(Commands.runOnce(() -> {
      double curZPos = rotationJoystick.getZ();
      shooterReductionOffset = curZPos;
      System.out.println("Current OFfset: " + shooterReductionOffset);
    }));
  }

  @Override
  public void init() {
    switch (Constants.currentMode) {
      case REAL:
        hopper = new HopperSubsystem(new HopperIOSpark());
        shooter = new ShooterSubsystem(new ShooterIOSpark());
        intake = new IntakeSubsystem(new IntakeIOSpark());
        climber = new ClimberSubsystem(new ClimberIOTalon());
        GSON.createDir();
        break;
      case SIM:
        hopper = new HopperSubsystem(new HopperIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        intake = new IntakeSubsystem(new IntakeIOSim());
        climber = new ClimberSubsystem(new ClimberIOSim());

        break;
      default:
        hopper = new HopperSubsystem(new HopperIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        climber = new ClimberSubsystem(new ClimberIOSim() {});

        break;
    }
  }

  @Override
  public void initAutos() {
    System.out.println("Does it change: " + test);
    compileAutos();
  }
}
