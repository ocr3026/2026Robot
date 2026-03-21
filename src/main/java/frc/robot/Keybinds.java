/* Generated and Formatted by yours truly <3*/
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ZRobotContainerAbstract.RobotContainerAbstract;

public class Keybinds {
  public static final Trigger resetGyroTrigger =
      RobotContainerAbstract.translationJoystick.button(12);

  public static final Trigger lockOnHubDrive = RobotContainerAbstract.translationJoystick.button(1);
  // public static final Trigger playSong = RobotContainer.translationJoystick.button(1);
  public static final Trigger shootFuel = RobotContainerAbstract.m_driverController.rightTrigger();
  public static final Trigger shooterFlywheel = RobotContainerAbstract.m_driverController.start();

  public static final Trigger climberUp = RobotContainerAbstract.m_driverController.povUp();
  public static final Trigger climberDown = RobotContainerAbstract.m_driverController.povDown();

  public static final Trigger intakeFuel = RobotContainerAbstract.m_driverController.leftTrigger();
  public static final Trigger intakeLiftUp =
      RobotContainerAbstract.m_driverController.rightBumper();
  public static final Trigger intakeLiftDown =
      RobotContainerAbstract.m_driverController.leftBumper();

  public static final Trigger runHopper = RobotContainerAbstract.m_driverController.x();
  public static final Trigger reverseHopper = RobotContainerAbstract.m_driverController.y();
  public static final Trigger reverseIntake = RobotContainerAbstract.m_driverController.a();

  public static final Trigger zeroClimber = RobotContainerAbstract.m_driverController.back();

  public static final Trigger climberPosUp = RobotContainerAbstract.m_driverController.povLeft();
  public static final Trigger climberPosDown = RobotContainerAbstract.m_driverController.povRight();
}
