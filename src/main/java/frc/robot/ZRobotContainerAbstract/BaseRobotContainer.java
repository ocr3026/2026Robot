/* Generated and Formatted by yours truly <3*/
package frc.robot.ZRobotContainerAbstract;

import com.orangefrc.annotation.GSON;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Keybinds;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalon;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class BaseRobotContainer extends RobotContainerAbstract {

  @Override
  public void init() {
    System.out.println("We are initializing");
    switch (Constants.currentMode) {
      case REAL:
        GSON.createDir();

        drive = new DriveSubsystem(
            new GyroIONavX(),
            new ModuleIOTalon(TunerConstants.FrontLeft),
            new ModuleIOTalon(TunerConstants.FrontRight),
            new ModuleIOTalon(TunerConstants.BackLeft),
            new ModuleIOTalon(TunerConstants.BackRight),
            (pose) -> {});
        vision = new Vision(
            drive,
            new VisionIOPhotonvision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
            new VisionIOPhotonvision(VisionConstants.camer1Name, VisionConstants.robotToCamera1));
        // climber = new ClimberSubsystem(new ClimberIOTalon());
        break;
      case SIM:
        test = 10;
        driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig,
            DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? new Pose2d(3, 10, new Rotation2d())
                : new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive = new DriveSubsystem(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);

        System.out.println(
            "Initializing drive.. " + drive.getName() + "at : " + System.currentTimeMillis());
        vision = new Vision(
            drive,
            new VisionIOPhotonvisionSim(
                VisionConstants.camera0Name,
                VisionConstants.robotToCamera0,
                driveSimulation::getSimulatedDriveTrainPose));

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
  }

  @Override
  public void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(
        drive,
        () -> -translationJoystick.getY(),
        () -> -translationJoystick.getX(),
        () -> rotationJoystick.getX() * 0.8));

    Logger.recordOutput("THe pose we get rotation from", DriveConstants.hubPose);

    // 3.3meters
    Keybinds.lockOnHubDrive
        .whileTrue(DriveCommands.turretDrive(
            drive,
            () -> -translationJoystick.getY(),
            () -> -translationJoystick.getX(),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? drive.getTargetRotation(DriveConstants.hubPose, drive.getPose())
                : drive.getTargetRotation(DriveConstants.hubPose, drive.getPose())))
        .whileFalse(DriveCommands.joystickDrive(
            drive,
            () -> -translationJoystick.getY(),
            () -> -translationJoystick.getX(),
            () -> rotationJoystick.getX() * 0.8));

    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Keybinds.playSong.onTrue(new InstantCommand(() -> DriveConstants.m_orchestra.play()));
    // Keybinds.playSong.onFalse(new InstantCommand(() -> DriveConstants.m_orchestra.stop()));

    Keybinds.resetGyroTrigger.onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  @Override
  public void initAutos() {

    compileAutos();

    // RobotContainer.autoChooser.addOption("Test", new Test(drive));
    // RobotContainer.autoChooser.addOption("ClimbTest", new ClimbAutoTest(climber, drive));
  }
}
