/* Generated and Formatted by yours truly <3*/
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.AutoBase;
import frc.autonomous.AutoBase.Paths;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HopperCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalon;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalon;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;

public class RobotContainer {
    public static double hopperSpeed = -200;
    public static double intakeSpeed = 3392;
    public static double intakeLiftSpeed = 0.1;
    public static double shooterSpeed = 4750;
    public static double shooter2Speed = 4750;
    public static double shooterKickupSpeed = 4750;
    public static double climberSpeed = 5;
    public static double climberPos = 5;


  public static final CommandJoystick translationJoystick = new CommandJoystick(0);
  public static final CommandJoystick rotationJoystick = new CommandJoystick(1);
  public static final CommandXboxController m_driverController = new CommandXboxController(2);

  private final DriveSubsystem drive;
  private final Vision vision;
  private final ShooterSubsystem shooter;
  private final HopperSubsystem hopper;
  private final IntakeSubsystem intake;
  private final ClimberSubsystem climber;

  private SwerveDriveSimulation driveSimulation = null;

  public final LoggedDashboardChooser<Command> autoChooser;
  public static Command currentSelectedCommand = null;

  public RobotContainer() {
    SmartDashboard.putNumber("HopperSpeed", hopperSpeed);

    switch (Constants.currentMode) {
      case REAL:
        drive = new DriveSubsystem(
            new GyroIONavX(),
            new ModuleIOTalon(TunerConstants.FrontLeft),
            new ModuleIOTalon(TunerConstants.FrontRight),
            new ModuleIOTalon(TunerConstants.BackLeft),
            new ModuleIOTalon(TunerConstants.BackRight),
            (pose) -> {});
        this.vision = new Vision(
            drive,
            new VisionIOPhotonvision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

        hopper = new HopperSubsystem(new HopperIOSpark());
        shooter = new ShooterSubsystem(new ShooterIOSpark());
        intake = new IntakeSubsystem(new IntakeIOSpark());
        climber = new ClimberSubsystem(new ClimberIOTalon());

        break;
      case SIM:
        driveSimulation = new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive = new DriveSubsystem(
            new GyroIOSim(driveSimulation.getGyroSimulation()),
            new ModuleIOSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
            new ModuleIOSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
            new ModuleIOSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
            new ModuleIOSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
            driveSimulation::setSimulationWorldPose);

        hopper = new HopperSubsystem(new HopperIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        intake = new IntakeSubsystem(new IntakeIOSim());
        climber = new ClimberSubsystem(new ClimberIOSim());

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
        hopper = new HopperSubsystem(new HopperIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        climber = new ClimberSubsystem(new ClimberIOSim() {});

        break;
    }
    SmartDashboard.putNumber("delayStartTime", 0);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    compileAutos();

    configureBindings();
  }

  private void configureBindings() {
    // drive.setDefaultCommand(DriveCommands.joystickDrive(
    //     drive,
    //     () -> -translationJoystick.getY(),
    //     () -> -translationJoystick.getX(),
    //     () -> -rotationJoystick.getX()));
    Logger.recordOutput(
        "THe pose we get rotation from",
        Paths.aimTurret.getStartingHolonomicPose().get());

    drive.setDefaultCommand(DriveCommands.turretDrive(
        drive,
        () -> -translationJoystick.getY(),
        () -> -translationJoystick.getX(),
        () -> drive.getDeltaRotation(Paths.aimTurret.getStartingHolonomicPose().get())));

    final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
        ? () -> drive.setPose(
            driveSimulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
        // simulation
        : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Keybinds.playSong.onTrue(new InstantCommand(() -> DriveConstants.m_orchestra.play()));
    // Keybinds.playSong.onFalse(new InstantCommand(() -> DriveConstants.m_orchestra.stop()));

    Keybinds.resetGyroTrigger.onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    Keybinds.intakeFuel.whileTrue(IntakeCommands.intakeFuel(intake, intakeSpeed));
    Keybinds.intakeLiftUp.whileTrue(IntakeCommands.intakeLift(intake, intakeLiftSpeed));
    Keybinds.intakeLiftDown.whileTrue(IntakeCommands.intakeLift(intake, -intakeLiftSpeed));
    Keybinds.reverseIntake.whileTrue(IntakeCommands.intakeFuel(intake, -intakeSpeed));

    Keybinds.shootFuel.whileTrue(new ParallelCommandGroup(
        ShooterCommands.shootFuel(shooter, shooterSpeed, shooter2Speed, shooterKickupSpeed),
        HopperCommands.runHopper(hopper, hopperSpeed)));
    Keybinds.shooterFlywheel.whileTrue(
        ShooterCommands.runShooterKickup(shooter, shooterKickupSpeed));

    // Keybinds.shootFuel.whileTrue(AutoBase.shootFuel(hopper, shooter));

    Keybinds.runHopper.whileTrue(HopperCommands.runHopper(hopper, hopperSpeed));
    Keybinds.reverseHopper.whileTrue(HopperCommands.reverseHopper(hopper, -hopperSpeed));
    Keybinds.climberUp.whileTrue(ClimberCommands.setClimberPos(climber, climberPos));
    Keybinds.climberDown.whileTrue(ClimberCommands.setClimberPos(climber, 0.0));
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
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public void updateHopperSpeed() {

    hopperSpeed = SmartDashboard.getNumber("HopperSpeed", 0);
    SmartDashboard.putString("Updated", "Updated!" + hopperSpeed);
  }

  public void compileAutos() {
    Reflections reflection = new Reflections(new ConfigurationBuilder()
        .setUrls(ClasspathHelper.forPackage("frc.autonomous"))
        .setScanners(Scanners.SubTypes));
    Set<Class<?>> autoClasses =
        reflection.get(Scanners.SubTypes.of(AutoBase.class).asClass());

    SmartDashboard.putString("CompilerError", "autoclassessize: " + autoClasses.size());
    for (Class<?> autoClass : autoClasses) {
      try {
        SmartDashboard.putString("CompilerError", "ITS WORKING TTPYPEEE");

        SequentialCommandGroup command;
        command = (SequentialCommandGroup) autoClass
            .getDeclaredConstructor(
                HopperSubsystem.class,
                ShooterSubsystem.class,
                IntakeSubsystem.class,
                DriveSubsystem.class)
            .newInstance(hopper, shooter, intake, drive);
        autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);

      } catch (NoSuchMethodException
          | SecurityException
          | InstantiationException
          | IllegalAccessException
          | IllegalArgumentException
          | InvocationTargetException e) {
        SmartDashboard.putString(
            "CompilerError",
            Arrays.stream(e.getStackTrace())
                .map(StackTraceElement::toString)
                .collect(Collectors.joining(System.lineSeparator() + "\tat")));
      }
    }
  }
}
