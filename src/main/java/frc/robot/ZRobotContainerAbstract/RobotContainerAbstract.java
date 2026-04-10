/* Generated and Formatted by yours truly <3*/
package frc.robot.ZRobotContainerAbstract;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.AutoBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;

public abstract class RobotContainerAbstract {

  public static final CommandJoystick translationJoystick = new CommandJoystick(0);
  public static final CommandJoystick rotationJoystick = new CommandJoystick(1);
  public static final CommandXboxController m_driverController = new CommandXboxController(2);
  protected SwerveDriveSimulation driveSimulation = null;

  public static DriveSubsystem drive;
  public static Vision vision;
  public static ShooterSubsystem shooter;
  public static HopperSubsystem hopper;
  public static IntakeSubsystem intake;
  public static ClimberSubsystem climber;

  public static int test = 0;
  public static double hopperSpeed = -400;
  public static double intakeSpeed = -3300;
  public static double intakeLiftSpeed = 50;
  public static double shooterSpeed = -4000;
  public static double shooter2Speed = 4000;
  public static double shooterKickupSpeed = -3000;
  public static double climberSpeed = 5;
  public static double climberPos = 50;
  public static double intakeLiftPos = -20;

  public static double shooterReductionOffset = 0;

  protected void compileAutos() {
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
        RobotContainer.autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);

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

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public static void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    // Logger.recordOutput(
    //     "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public abstract void init();

  public abstract void configureBindings();

  public void initBase() {}

  public void initTop() {}

  public void configureTopBindings() {}

  public void configureBaseBindings() {}

  public abstract void initAutos();
}
