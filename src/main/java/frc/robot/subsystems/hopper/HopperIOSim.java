/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class HopperIOSim implements HopperIO {
  SparkFlexSim hopperSim;
  SparkFlex hopperMotor;
  DCMotor motorSim;

  public HopperIOSim() {
    motorSim = DCMotor.getNeoVortex(1);
    hopperMotor = new SparkFlex(HopperConstants.hopperMotorID, MotorType.kBrushless);
    hopperSim = new SparkFlexSim(hopperMotor, motorSim);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.hopperConnected = true;
    inputs.hopperAppliedVolts = hopperSim.getAppliedOutput();
    inputs.hopperCurrentAmps = hopperSim.getMotorCurrent();
    inputs.hopperPosition = Rotations.of(hopperSim.getPosition());
    inputs.hopperVelocity = RotationsPerSecond.of(hopperSim.getVelocity());
  }

  @Override
  public void setHopperSpeed(double speed) {
    hopperSim.setVelocity(speed);
  }

  @Override
  public double getSetpoint() {
    return hopperSim.getSetpoint();
  }
}
