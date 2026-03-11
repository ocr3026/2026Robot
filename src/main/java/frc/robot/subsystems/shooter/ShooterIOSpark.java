/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.shooter;

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

public class ShooterIOSpark implements ShooterIO {
  private final SparkFlex shooterMotor;
  private final SparkFlex shooterMotor2;
  private final SparkFlex shooterKickupMotor;
  private final SparkClosedLoopController shooterPID;
  private final SparkClosedLoopController shooter2PID;
  private final SparkClosedLoopController shooterKickupPID;
  SparkFlexConfig shooterConfig;
  SparkFlexConfig shooter2Config;
  SparkFlexConfig shooterKickupConfig;
  double setpoint;

  public ShooterIOSpark() {
    shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
    shooterMotor2 = new SparkFlex(ShooterConstants.shooterMotorID2, MotorType.kBrushless);
    shooterKickupMotor = new SparkFlex(ShooterConstants.shooterMotorKickupID, MotorType.kBrushless);
    shooterPID = shooterMotor.getClosedLoopController();
    shooter2PID = shooterMotor2.getClosedLoopController();
    shooterKickupPID = shooterKickupMotor.getClosedLoopController();

    setpoint = 0;

    FeedForwardConfig shooterConf = new FeedForwardConfig();
    shooterConf.kV(0.001775);

    shooterConfig = new SparkFlexConfig();
    shooterConfig.idleMode(IdleMode.kBrake);
    shooterConfig.closedLoop.p(0.00015).i(0).d(0.00559);
    shooterConfig.closedLoop.apply(shooterConf);

    FeedForwardConfig shooter2Conf = new FeedForwardConfig();
    shooter2Conf.kV(0.001829);

    shooter2Config = new SparkFlexConfig();
    shooter2Config.idleMode(IdleMode.kBrake);
    shooter2Config.closedLoop.p(0.00025).i(0).d(0.0065);
    shooter2Config.closedLoop.apply(shooter2Conf);

    FeedForwardConfig shooterKickupConf = new FeedForwardConfig();
    shooterKickupConf.kV(0.001825);

    shooterKickupConfig = new SparkFlexConfig();
    shooterKickupConfig.idleMode(IdleMode.kBrake);
    shooterKickupConfig.closedLoop.p(0.00025).i(0).d(0.001825);
    shooterKickupConfig.closedLoop.apply(shooterKickupConf);

    shooterConfig.smartCurrentLimit(70);
    shooter2Config.smartCurrentLimit(70);
    shooterKickupConfig.smartCurrentLimit(70);
    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterKickupMotor.configure(
        shooterKickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
    inputs.shooterConnected = true;

    inputs.shooter2AppliedVolts = shooterMotor2.getAppliedOutput();
    inputs.shooter2Connected = true;

    inputs.shooterKickupAppliedVolts = shooterKickupMotor.getAppliedOutput();
    inputs.shooterKickupConnected = true;

    inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
    inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
    inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());

    inputs.shooter2CurrentAmps = shooterMotor2.getOutputCurrent();
    inputs.shooter2Position = Degrees.of(shooterMotor2.getEncoder().getPosition());
    inputs.shooter2Velocity = DegreesPerSecond.of(shooterMotor2.getEncoder().getVelocity());

    inputs.shooterKickupPosition = Degrees.of(shooterKickupMotor.getEncoder().getPosition());
    inputs.shooterKickupCurrentAmps = shooterKickupMotor.getOutputCurrent();
    inputs.shooterKickupVelocity =
        DegreesPerSecond.of(shooterKickupMotor.getEncoder().getVelocity());
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterPID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void setShooter2Speed(double speed) {
    shooter2PID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void setKickupSpeed(double speed) {
    shooterKickupPID.setSetpoint(speed, ControlType.kVelocity);
  }

  @Override
  public void updatePID(
      double sP,
      double sI,
      double sD,
      double sV,
      double sMaxAccel,
      double s2p,
      double s2i,
      double s2d,
      double s2v,
      double s2MaxAccel,
      double kickP,
      double kickI,
      double kickD,
      double kickV,
      double kickAccel) {

    FeedForwardConfig ffShooter1 = new FeedForwardConfig().kV(sV);
    FeedForwardConfig ffShooter2 = new FeedForwardConfig().kV(s2v);
    FeedForwardConfig ffKickup = new FeedForwardConfig().kV(kickV);

    shooterConfig.closedLoop.maxMotion.maxAcceleration(sMaxAccel);
    shooter2Config.closedLoop.maxMotion.maxAcceleration(s2MaxAccel);
    shooterKickupConfig.closedLoop.maxMotion.maxAcceleration(kickAccel);

    shooterConfig.closedLoop.p(sP).i(sI).d(sD).apply(ffShooter1);
    shooter2Config.closedLoop.p(s2p).i(s2i).d(s2d).apply(ffShooter2);
    shooterKickupConfig.closedLoop.p(kickP).i(kickI).d(kickD).apply(ffKickup);

    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterKickupMotor.configure(
        shooterKickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
