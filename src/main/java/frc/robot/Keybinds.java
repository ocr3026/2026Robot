package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Keybinds {
    	public static final Trigger resetGyroTrigger = RobotContainer.translationJoystick.button(12);
		    	//public static final Trigger playSong = RobotContainer.translationJoystick.button(1);
		public static final Trigger shootFuel = RobotContainer.m_driverController.rightTrigger();
		public static final Trigger intakeFuel = RobotContainer.m_driverController.leftTrigger();
		public static final Trigger intakeLiftUp = RobotContainer.m_driverController.rightBumper();
		public static final Trigger intakeLiftDown = RobotContainer.m_driverController.leftBumper();
		public static final Trigger runHopper = RobotContainer.m_driverController.x();



}
