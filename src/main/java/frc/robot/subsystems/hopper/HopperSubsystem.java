package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;

public class HopperSubsystem extends SubsystemBase {
  private final HopperIO io;
 //private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();



  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }
    public void runHopper(double speed){
      io.setHopperSpeed(speed);
  }
    public void reverseHopper(double speed){
      io.setHopperSpeed(speed);
  }


  @Override
  public void periodic() {
    //io.updateInputs(inputs);
    //Logger.processInputs("Hopper", inputs);
    // RobotContainer.hopperSpeed = SmartDashboard.getNumber("HopperSpeed", 0.123);
    // SmartDashboard.putNumber("HopperSetpoint", io.getSetpoint());
    // SmartDashboard.putNumber("HopperError", (io.getSetpoint() - (60 * inputs.hopperVelocity.in(RotationsPerSecond))));
  }

  @Override
  public void simulationPeriodic() {
  }
}
