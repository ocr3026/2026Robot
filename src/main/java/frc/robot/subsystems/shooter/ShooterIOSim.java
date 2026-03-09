/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim implements ShooterIO {
  private SparkFlexSim shooter1;
  private SparkFlexSim shooter2;
  private SparkFlexSim shooterKick;

  SparkFlex shooter1Flex;
  SparkFlex shooter2Flex;
  SparkFlex shooterKickFlex;

  DCMotor shooter1Motor;
  DCMotor shooter2Motor;
  DCMotor shooterKickMotor;

  public ShooterIOSim() {
    shooter1Flex = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
    shooter2Flex = new SparkFlex(ShooterConstants.shooterMotorID2, MotorType.kBrushless);
    shooterKickFlex = new SparkFlex(ShooterConstants.shooterMotorKickupID, MotorType.kBrushless);

    shooter1Motor = DCMotor.getNeoVortex(1);
    shooter2Motor = DCMotor.getNeoVortex(1);
    shooterKickMotor = DCMotor.getNeoVortex(1);

    shooter1 = new SparkFlexSim(shooter1Flex, shooter1Motor);
    shooter2 = new SparkFlexSim(shooter2Flex, shooter2Motor);
    shooterKick = new SparkFlexSim(shooterKickFlex, shooterKickMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterAppliedVolts = shooter1.getAppliedOutput();
    inputs.shooterConnected = true;

    inputs.shooter2AppliedVolts = shooter2.getAppliedOutput();
    inputs.shooter2Connected = true;

    inputs.shooterKickupAppliedVolts = shooterKick.getAppliedOutput();
    inputs.shooterKickupConnected = true;

    inputs.shooterCurrentAmps = shooter1.getMotorCurrent();
    inputs.shooterPosition = Rotations.of(shooter1.getPosition());
    inputs.shooterVelocity = RotationsPerSecond.of(shooter1.getVelocity());

    inputs.shooter2CurrentAmps = shooter2.getMotorCurrent();
    inputs.shooter2Position = Rotations.of(shooter2.getPosition());
    inputs.shooter2Velocity = RotationsPerSecond.of(shooter2.getVelocity());

    inputs.shooterKickupPosition = Rotations.of(shooterKick.getPosition());
    inputs.shooterKickupCurrentAmps = shooterKick.getMotorCurrent();
    inputs.shooterKickupVelocity = RotationsPerSecond.of(shooterKick.getVelocity());
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooter1.setVelocity(speed);
  }

  @Override
  public void setShooter2Speed(double speed) {
    shooter2.setVelocity(speed);
  }

  @Override
  public void setKickupSpeed(double speed) {
    shooterKick.setVelocity(speed);
  }
}
