/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex intakeMotor;
  private final SparkClosedLoopController intakePID;
  private final SparkMax intakeLift;
  private final SparkClosedLoopController intakeLiftPID;
  SparkFlexConfig intakeConfig;
  SparkFlexConfig intakeLiftConfig;

  public IntakeIOSpark() {
    intakeMotor = new SparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
    intakeLift = new SparkMax(IntakeConstants.intakeLiftID, MotorType.kBrushless);
    intakePID = intakeMotor.getClosedLoopController();
    intakeLiftPID = intakeLift.getClosedLoopController();

    FeedForwardConfig intakeConf = new FeedForwardConfig();
    intakeConf.kV(0.00245);

    intakeConfig = new SparkFlexConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop.p(0.00015).i(0).d(0.0);
    intakeConfig.closedLoop.apply(intakeConf);

    FeedForwardConfig intakeLiftConf = new FeedForwardConfig();
    intakeLiftConf.kV(0.05);

    intakeLiftConfig = new SparkFlexConfig();
    intakeLiftConfig.idleMode(IdleMode.kBrake);
    intakeLiftConfig.closedLoop.p(0.000005).i(0).d(0);
    intakeLiftConfig.closedLoop.apply(intakeLiftConf);

    intakeConfig.smartCurrentLimit(70);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeLift.configure(
        intakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    zeroIntakeLift();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput();
    inputs.intakeLiftAppliedVolts = intakeLift.getAppliedOutput();
    inputs.intakeConnected = true;
    inputs.intakeLiftConnected = true;
    inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
    inputs.intakeLiftCurrentAmps = intakeLift.getOutputCurrent();
    inputs.intakePosition = Rotations.of(intakeMotor.getEncoder().getPosition());
    inputs.intakeLiftPosition = Rotations.of(intakeLift.getEncoder().getPosition());
    inputs.intakeVelocity = RotationsPerSecond.of(intakeMotor.getEncoder().getVelocity());
    inputs.intakeLiftVelocity = RotationsPerSecond.of(intakeLift.getEncoder().getVelocity());
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakePID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void setIntakeLiftSpeed(double speed) {
    intakeLiftPID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void setIntakeLiftPos(double pos) {
    intakeLiftPID.setSetpoint(pos, ControlType.kPosition);
  }

  @Override
  public void runWhileGreater(double speed, double pos) {
    intakeLiftPID.setSetpoint(speed, ControlType.kDutyCycle);
    Logger.recordOutput("Typedihh", pos);
  }

  @Override
  public void zeroIntakeLift() {
    intakeLift.getEncoder().setPosition(0.0);
  }

  @Override
  public double getIntakePosition() {
    return intakeLift.getEncoder().getPosition();
  }

  @Override
  public void updatePID(
      double iP,
      double iI,
      double iD,
      double iV,
      double iA,
      double lP,
      double lI,
      double lD,
      double lV,
      double lA) {
    FeedForwardConfig ffI = new FeedForwardConfig().kV(iV);
    FeedForwardConfig ffL = new FeedForwardConfig().kV(lV);

    intakeConfig.closedLoop.p(iP).i(iI).d(iD).apply(ffI).maxMotion.maxAcceleration(iA);
    intakeLiftConfig.closedLoop.p(lP).i(lI).d(lD).apply(ffL).maxMotion.maxAcceleration(lA);
    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeLift.configure(
        intakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
