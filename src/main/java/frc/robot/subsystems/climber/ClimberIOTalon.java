/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

public class ClimberIOTalon implements ClimberIO {
  protected final TalonFX climberMotor;
  private double setpoint;

  public ClimberIOTalon() {
    climberMotor = new TalonFX(ClimberConstants.climberMotorID);
    // 0.008;

    setpoint = 0;

    TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.0003;
    Slot0Configs.kI = 0.0;
    Slot0Configs.kD = 0.0;
    Slot0Configs.kV = 0.0005;
    ClimberConfig.Slot0 = Slot0Configs;

    // TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();
    // var Slot0Configs = new Slot0Configs();
    // Slot0Configs = ClimberConfig.Slot0;
    // Slot0Configs.kP = 0.0;
    // Slot0Configs.kI = 0.0;
    // Slot0Configs.kD = 0.0;

    // climberMotor.getConfigurator().apply(ClimberConfig);
    zeroClimber();
    climberMotor.getConfigurator().apply(ClimberConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberAppliedVolts = climberMotor.getMotorVoltage().getValueAsDouble();
    inputs.climberConnected = true;
    inputs.climberCurrentAmps = climberMotor.getSupplyCurrent().getValueAsDouble();
    inputs.climberPosition = (climberMotor.getPosition().getValueAsDouble());
    inputs.climberVelocity = RotationsPerSecond.of(climberMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void zeroClimber() {
    climberMotor.setPosition(Rotations.of(0.0));
  }

  @Override
  public void setClimberSpeed(double speed) {
    setpoint = speed;
    Logger.recordOutput("Setpoinit for climber", setpoint);
    // climberMotor.setControl(new PositionVoltage(setpoint));
    climberMotor.setControl(new DutyCycleOut(setpoint));
  }

  @Override
  public void setClimberPos(double pos) {
    climberMotor.setControl(new PositionDutyCycle(pos));
    // Think rotation is 90?
  }

  @Override
  public double getSetpoint() {
    return setpoint;
  }

  @Override
  public void updatePID(double p, double i, double d, double v) {
    Slot0Configs c = new Slot0Configs().withKP(p).withKI(i).withKD(d).withKV(v);
    climberMotor.getConfigurator().apply(c);
  }
}
