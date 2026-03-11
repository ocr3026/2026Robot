/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSim extends ClimberIOTalon {
  TalonFXSimState climbMotor;
  DCMotor motor;
  DCMotorSim motorSim;

  public ClimberIOSim() {
    climbMotor = climberMotor.getSimState();
    motor = DCMotor.getKrakenX60(1);
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.001, 1), motor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    super.updateInputs(inputs);
    Logger.recordOutput("VOLTAGE TYPE HIT", climbMotor.getMotorVoltageMeasure());
    motorSim.setInputVoltage(climbMotor.getMotorVoltageMeasure().in(Volts));
    motorSim.update(0.02);

    climbMotor.setRawRotorPosition(motorSim.getAngularPosition());
    climbMotor.setRotorVelocity(motorSim.getAngularVelocity());

    inputs.climberPosition = (motorSim.getAngularPosition().in(Rotations) / (2 * Math.PI));
  }
}
