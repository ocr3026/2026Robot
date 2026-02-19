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
    private final SparkClosedLoopController shooterPID;

    public ShooterIOSpark() {
        shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
        shooterPID = shooterMotor.getClosedLoopController();
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
        inputs.shooterConnected = true;
        inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());
    }

    @Override
    public void setAngularSpeed(double speed) {
        shooterPID.setSetpoint(speed, ControlType.kVoltage);
    }
}
