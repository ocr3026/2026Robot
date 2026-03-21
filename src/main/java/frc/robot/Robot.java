/* Generated and Formatted by yours truly <3*/
package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ZRobotContainerAbstract.RobotContainer;
import frc.robot.ZRobotContainerAbstract.RobotContainerAbstract;
import java.lang.reflect.InvocationTargetException;
import java.util.Set;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.reflections.Reflections;
import org.reflections.scanners.Scanners;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    robotContainer = new RobotContainer(findAllSubClasses());
  }

  @Override
  public void robotPeriodic() {
    // Sets thread to high priority
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();
    // robotContainer.updateHopperSpeed();

    // returns to a low priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {
    robotContainer.resetSimulationField();
    // CommandScheduler.getInstance().cancelAll();
    // robotContainer.compileAutos();
  }

  @Override
  public void disabledPeriodic() {
    // robotContainer.updateHopperSpeed();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }

  private RobotContainerAbstract[] findAllSubClasses() {
    RobotContainerAbstract[] arr;
    Reflections reflection = new Reflections(new ConfigurationBuilder()
        .setUrls(ClasspathHelper.forPackage("frc.robot.ZRobotContainerAbstract"))
        .setScanners(Scanners.SubTypes));
    Set<Class<?>> subClasses =
        reflection.get(Scanners.SubTypes.of(RobotContainerAbstract.class).asClass());
    arr = new RobotContainerAbstract[subClasses.size()];
    int index = 0;
    for (Class<?> subClass : subClasses) {
      System.out.println(subClass.getSimpleName());
      try {
        arr[index] = (RobotContainerAbstract) (subClass.getDeclaredConstructor().newInstance());
        index++;
      } catch (InstantiationException
          | IllegalAccessException
          | IllegalArgumentException
          | InvocationTargetException
          | NoSuchMethodException
          | SecurityException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    return arr;
  }
}
