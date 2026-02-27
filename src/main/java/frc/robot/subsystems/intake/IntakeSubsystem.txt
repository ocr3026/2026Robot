package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();



  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }
    public void intakeFuel(double speed){
      io.setIntakeSpeed(speed);
  }
    public void intakeLift(double speed){
      io.setIntakeLiftSpeed(speed);
  }

  public double getIntakeLiftPos() {
    return io.getIntakePosition();
  }

  public void intakeLiftPos(double pos) {
    io.setIntakeLiftPos(pos);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake Lift Pos Raw", io.getIntakePosition());
  }

  @Override
  public void simulationPeriodic() {
  }
}