package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleIOTalonReal extends ModuleIOTalon {
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOTalonReal(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        super(constants);

        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        inputs.turnPositionsRad = turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }
}
