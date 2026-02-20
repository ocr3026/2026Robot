package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterIOSpark implements ShooterIO {
    private final SparkFlex shooterMotor;
    private final SparkFlex shooterKickbackMotor;
    private final SparkClosedLoopController shooterPID;
    private final SparkClosedLoopController shooterKickbackPID;

    public ShooterIOSpark() {
        shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
        shooterKickbackMotor = new SparkFlex(ShooterConstants.shooterMotorKickbackID, MotorType.kBrushless);
        shooterPID = shooterMotor.getClosedLoopController();
        shooterKickbackPID = shooterKickbackMotor.getClosedLoopController();
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
        inputs.shooterAppliedVolts = shooterKickbackMotor.getAppliedOutput();
        inputs.shooterConnected = true;
        inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.shooterCurrentAmps = shooterKickbackMotor.getOutputCurrent();
        inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
        inputs.shooterPosition = Degrees.of(shooterKickbackMotor.getEncoder().getPosition());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterKickbackMotor.getEncoder().getVelocity());
    }

    @Override
    public void setShooterSpeed(double speed) {
        shooterPID.setSetpoint(speed, ControlType.kVoltage);
        shooterKickbackPID.setSetpoint(speed, ControlType.kVoltage);
    }
}
