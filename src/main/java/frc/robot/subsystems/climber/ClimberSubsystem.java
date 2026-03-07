package frc.robot.subsystems.climber;

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

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();



  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }
    public void runClimber(double speed){
      io.setClimberSpeed(speed);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    SmartDashboard.putNumber("ClimberSetpoint", io.getSetpoint());
    SmartDashboard.putNumber("ClimberError", (io.getSetpoint() - (60 * inputs.climberVelocity.in(RotationsPerSecond))));
  }

  @Override
  public void simulationPeriodic() {
  }
}

