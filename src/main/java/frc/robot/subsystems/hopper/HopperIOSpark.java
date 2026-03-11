/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class HopperIOSpark implements HopperIO {
  private final SparkFlex hopperMotor;
  private final SparkClosedLoopController hopperPID;
  private double setpoint;
  SparkFlexConfig hopperConfig;

  public HopperIOSpark() {
    hopperMotor = new SparkFlex(HopperConstants.hopperMotorID, MotorType.kBrushless);
    hopperPID = hopperMotor.getClosedLoopController();
    // 0.008;
    FeedForwardConfig ffConf = new FeedForwardConfig();
    ffConf.kV(0.00245);

    setpoint = 0;
    hopperConfig = new SparkFlexConfig();

    hopperConfig.smartCurrentLimit(70);

    hopperConfig.idleMode(IdleMode.kBrake);
    hopperConfig.closedLoop.p(0.0005).i(0.000001).d(0.01);
    hopperConfig.closedLoop.maxMotion.maxAcceleration(2500);
    hopperConfig.closedLoop.apply(ffConf);

    hopperMotor.configure(
        hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.hopperAppliedVolts = hopperMotor.getAppliedOutput();
    inputs.hopperConnected = true;
    inputs.hopperCurrentAmps = hopperMotor.getOutputCurrent();
    inputs.hopperPosition = Degrees.of(hopperMotor.getEncoder().getPosition());
    inputs.hopperVelocity = RotationsPerSecond.of(hopperMotor.getEncoder().getVelocity() / 60);
  }

  @Override
  public void setHopperSpeed(double speed) {
    setpoint = speed;
    hopperPID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void updatePID(double P, double I, double D, double kV, double maxAccel) {
    FeedForwardConfig ff = new FeedForwardConfig();
    ff.kV(kV);
    hopperConfig.closedLoop.p(P).i(I).d(D);
    hopperConfig.closedLoop.apply(ff);
    hopperMotor.configure(
        hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }
}
