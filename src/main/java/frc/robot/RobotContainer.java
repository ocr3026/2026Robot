// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutoBase;
import frc.robot.autonomous.DriveBackAndShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalon;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperIOSpark;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;

public class RobotContainer {
    public static double hopperSpeed = -200;


  public static final CommandJoystick translationJoystick = new CommandJoystick(0);
	public static final CommandJoystick rotationJoystick = new CommandJoystick(1);
  public static final CommandXboxController m_driverController = new CommandXboxController(2);

  private final DriveSubsystem drive;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;

  private SwerveDriveSimulation driveSimulation = null;

  
	public final LoggedDashboardChooser<Command> autoChooser;
	public static Command currentSelectedCommand = null;

  public RobotContainer() {
        SmartDashboard.putNumber("HopperSpeed", hopperSpeed);

        shooter = new ShooterSubsystem(new ShooterIOSpark());
        hopper = new HopperSubsystem(new HopperIOSpark());
        intake = new IntakeSubsystem(new IntakeIOSpark());
    switch(Constants.currentMode) {
      case REAL:
        drive = new DriveSubsystem(new GyroIONavX(), new ModuleIOTalon(TunerConstants.FrontLeft), new ModuleIOTalon(TunerConstants.FrontRight), new ModuleIOTalon(TunerConstants.BackLeft), new ModuleIOTalon(TunerConstants.BackRight), (pose) -> {});
        this.vision = new Vision(drive, new VisionIOPhotonvision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        
        break;
      case SIM:
          driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3,3, new Rotation2d()));
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            drive = new DriveSubsystem(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]), driveSimulation::setSimulationWorldPose);

                  vision = new Vision(drive,new VisionIOPhotonvisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));


                                break;
      default:
        drive = new DriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
    }
    SmartDashboard.putNumber("delayStartTime", 0);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    
    autoChooser.addOption("Drive Back and Shoot", new DriveBackAndShoot(hopper, shooter, intake));


    configureBindings();
  }


  private void configureBindings() {
    		drive.setDefaultCommand(DriveCommands.joystickDrive(
				drive,
				() -> -translationJoystick.getY(),
				() -> -translationJoystick.getX(),
				() -> -rotationJoystick.getX()));

        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
				? () -> drive.setPose(
						driveSimulation
								.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
				: () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

        //Keybinds.playSong.onTrue(new InstantCommand(() -> DriveConstants.m_orchestra.play()));
                //Keybinds.playSong.onFalse(new InstantCommand(() -> DriveConstants.m_orchestra.stop()));


		Keybinds.resetGyroTrigger.onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    Keybinds.intakeFuel.whileTrue(IntakeCommands.intakeFuel(intake,-0.5d));
    Keybinds.intakeLiftUp.whileTrue(IntakeCommands.intakeLift(intake,0.1d));
    Keybinds.intakeLiftDown.whileTrue(IntakeCommands.intakeLift(intake, -0.1d));
    Keybinds.shootFuel.whileTrue(new ParallelCommandGroup(ShooterCommands.shootFuel(shooter,0.65d), HopperCommands.runHopper(hopper, hopperSpeed)));
    //Keybinds.shootFuel.whileTrue(AutoBase.shootFuel(hopper, shooter));
    Keybinds.runHopper.whileTrue(HopperCommands.runHopper(hopper, hopperSpeed));
    Keybinds.reverseHopper.whileTrue(HopperCommands.reverseHopper(hopper, -hopperSpeed));
    Keybinds.reverseIntake.whileTrue(IntakeCommands.intakeFuel(intake, 0.5d));

  }


  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		drive.setPose(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();
		Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
		Logger.recordOutput(
				"FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
		Logger.recordOutput(
				"FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
	}

  public void updateHopperSpeed() {
  
    hopperSpeed = SmartDashboard.getNumber("HopperSpeed", 0);
    SmartDashboard.putString("Updated", "Updated!" + hopperSpeed);
  }
}
