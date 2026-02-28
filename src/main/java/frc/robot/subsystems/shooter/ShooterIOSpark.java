package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterIOSpark implements ShooterIO {
    private final SparkFlex shooterMotor;
    private final SparkFlex shooterKickupMotor;
    private final SparkClosedLoopController shooterPID;
    private final SparkClosedLoopController shooterKickupPID;

    public ShooterIOSpark() {
        shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
        shooterKickupMotor = new SparkFlex(ShooterConstants.shooterMotorKickupID, MotorType.kBrushless);
        shooterPID = shooterMotor.getClosedLoopController();
        shooterKickupPID = shooterKickupMotor.getClosedLoopController();

        FeedForwardConfig shooterConf = new FeedForwardConfig();
        shooterConf.kV(0.00245);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
        shooterConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        shooterConfig.closedLoop.apply(shooterConf);

        FeedForwardConfig shooterKickupConf = new FeedForwardConfig();
        shooterKickupConf.kV(0.00245);

        SparkFlexConfig shooterKickupConfig = new SparkFlexConfig();
        shooterKickupConfig.idleMode(IdleMode.kBrake);
        shooterKickupConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        shooterKickupConfig.closedLoop.apply(shooterKickupConf);

        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterKickupMotor.configure(shooterKickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
        inputs.shooterKickupAppliedVolts = shooterKickupMotor.getAppliedOutput();
        inputs.shooterConnected = true;
        inputs.shooterKickupConnected = true;

        inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.shooterKickupCurrentAmps = shooterKickupMotor.getOutputCurrent();
        inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
        inputs.shooterKickupPosition = Degrees.of(shooterKickupMotor.getEncoder().getPosition());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());
        inputs.shooterKickupVelocity = DegreesPerSecond.of(shooterKickupMotor.getEncoder().getVelocity());
    }

    @Override
    public void setShooterSpeed(double speed) {
        shooterPID.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setKickupSpeed(double speed) {
        shooterKickupPID.setSetpoint(speed, ControlType.kVelocity);
    }
}
