/* Generated and Formatted by yours truly <3*/
package frc.robot.ZRobotContainerAbstract;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  RobotContainerAbstract abstractBasic;
  public static LoggedDashboardChooser<Command> autoChooser;

  public static Command currentSelectedCommand = null;

  public RobotContainer(RobotContainerAbstract... abstracts) {
    // Initialize the Subsystems
    for (RobotContainerAbstract abstract1 : abstracts) {
      System.out.println(abstract1.getClass().getSimpleName() + "In da rc");
      abstract1.init();
    }

    SmartDashboard.putNumber("delayStartTime", 0);

    // Try deleting this, maybe only call in disabled init? Calling twice may bog down start
    // times...
    // compileAutos();

    configureBindings(abstracts);

    if (Constants.hasConfiguredAutobuilder) {
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    }

    for (RobotContainerAbstract abstract1 : abstracts) {
      abstract1.initAutos();
      System.out.println("Init autos for: " + abstract1.getClass().getSimpleName());
    }
  }

  private void configureBindings(RobotContainerAbstract... abstracts) {
    for (RobotContainerAbstract abstract1 : abstracts) {
      abstract1.configureBindings();
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField(RobotContainerAbstract... abstracts) {
    for (RobotContainerAbstract abstract1 : abstracts) {
      abstract1.resetSimulationField();
    }
  }

  public void updateSimulation(RobotContainerAbstract... abstracts) {
    RobotContainerAbstract.updateSimulation();
  }

  public static double calculateShooterSpeed(double dist) {
    // Calculate with formula from Desmos.
    return 0.0;
  }
}
