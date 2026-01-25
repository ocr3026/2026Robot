// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;

public class RobotContainer {

  private final DriveSubsystem drive;
  private final Vision vision;

  private SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {

    switch(Constants.currentMode) {
      case REAL:
        drive = new DriveSubsystem(new GyroIONavX(), new ModuleIOTalonReal(TunerConstants.FrontLeft), new ModuleIOTalonReal(TunerConstants.FrontRight), new ModuleIOTalonReal(TunerConstants.BackLeft), new ModuleIOTalonReal(TunerConstants.BackRight), (pose) -> {});
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

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
}
