package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hopper.HopperIO.HopperIOInputs;

public class HopperSubsystem extends SubsystemBase {
  private final HopperIO io;
 private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
    Debouncer debouncer = new Debouncer(0.4);
    boolean debounceInput = false;



  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }
    public void runHopper(double speed){

        io.setHopperSpeed(speed);
  }
    public void reverseHopper(double speed){
      io.setHopperSpeed(speed);
  }

  public void setHopperDuty(double speed) {
            io.setHopperDuty(speed);

  }

  public void stopHopper(){

        io.setHopperSpeed(0.0);

  }

  


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    RobotContainer.hopperSpeed = SmartDashboard.getNumber("HopperSpeed", 0.123);
    SmartDashboard.putNumber("HopperSetpoint", io.getSetpoint());
    SmartDashboard.putNumber("HopperError", (io.getSetpoint() - (60 * inputs.hopperVelocity.in(RotationsPerSecond))));

    if(inputs.hopperCurrentAmps >= 10) {
      debounceInput = true;
    }
    else if(inputs.hopperCurrentAmps <= 10) {
      debounceInput = false;
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
