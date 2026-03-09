/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSim implements IntakeIO {
  SparkFlexSim intakeSim;
  SparkMaxSim liftSim;

  SparkFlex intake;
  SparkMax lift;

  DCMotor intakeM;
  DCMotor liftM;

  double setpoint;

  public IntakeIOSim() {
    setpoint = 0.0;
    FeedForwardConfig intakeConf = new FeedForwardConfig();
    intakeConf.kV(0.00245);

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop.p(0.0006).i(0).d(0.00192);
    intakeConfig.closedLoop.apply(intakeConf);

    FeedForwardConfig intakeLiftConf = new FeedForwardConfig();
    intakeLiftConf.kV(0.00245);

    SparkFlexConfig intakeLiftConfig = new SparkFlexConfig();
    intakeLiftConfig.idleMode(IdleMode.kBrake);
    intakeLiftConfig.closedLoop.p(0.0006).i(0).d(0.00192);
    intakeLiftConfig.closedLoop.apply(intakeLiftConf);

    intake = new SparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    lift = new SparkMax(IntakeConstants.intakeLiftID, MotorType.kBrushless);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lift.configure(
        intakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeM = DCMotor.getNeoVortex(1);
    liftM = DCMotor.getNeo550(1);

    intakeSim = new SparkFlexSim(intake, intakeM);
    liftSim = new SparkMaxSim(lift, liftM);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = intakeSim.getAppliedOutput();
    inputs.intakeConnected = true;
    inputs.intakeCurrentAmps = intakeSim.getMotorCurrent();
    inputs.intakePosition = Rotations.of(intakeSim.getPosition());
    inputs.intakeVelocity = RotationsPerSecond.of(intakeSim.getVelocity());

    inputs.intakeLiftConnected = true;
    inputs.intakeLiftAppliedVolts = liftSim.getAppliedOutput();
    inputs.intakeLiftCurrentAmps = liftSim.getMotorCurrent();
    inputs.intakeLiftPosition = Rotations.of(liftSim.getPosition());
    inputs.intakeLiftVelocity = RotationsPerSecond.of(liftSim.getVelocity());
  }

  @Override
  public void setIntakeLiftSpeed(double speed) {
    liftSim.setVelocity(speed);
  }

  @Override
  public void setIntakeLiftPos(double pos) {
    SmartDashboard.putNumber("IntakeLiftSetpoint", pos);
    setpoint = pos;
    liftSim.setPosition(pos);
  }

  @Override
  public void zeroIntakeLift() {
    liftSim.setPosition(0);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSim.setVelocity(speed);
  }

  @Override
  public double getIntakePosition() {
    return Rotations.of(liftSim.getPosition()).in(Rotations);
  }
}
