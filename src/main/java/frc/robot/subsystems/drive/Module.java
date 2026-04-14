/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {

  // Initialize many variables
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  // Method for instantiating each module object
  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert = new Alert(
        "Disconnected drive motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnDisconnectedAlert = new Alert(
        "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert = new Alert(
        "Disconnected turn encoder on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  // Periodic function for each module, runs once every period (20ms)
  public void periodic() {
    // Update inputs for logging
    io.updateInputs(inputs);

    // Put all the inputs on the dashboard
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Get how many timestamps were recorded - how many "samples" for odometry there are in this
    // period
    int sampleCount = inputs.odometryTimestamps.length;
    // Initialze the array for all swerve module positions for how many samples there were in this
    // period
    odometryPositions = new SwerveModulePosition[sampleCount];

    // Go through each sample
    for (int i = 0; i < sampleCount; i++) {
      // Get the position in meters based off of the current drive position in radians times the
      // wheel radius, dividing by the gear reduction
      double positionMeters = (inputs.odometryDrivePositionsRad[i] * constants.WheelRadius)
          / constants.DriveMotorGearRatio;

      // Get the angle from the encoder in radians
      Rotation2d angle = inputs.turnPositionsRad[i];
      // Make a new swerve module position object to hold these values easily (the object is almost
      // just a container for these values)
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /**
   *
   * @return the current turn angle
   */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /**
   * @deprecated Call this function to update the PID on the motors for this module
   */
  public void updatePID() {
    io.updateMotorConfigs();
  }

  /**
   * Runs the module to a certain "state" - drive velocity and turn angle
   * @param state The swerve module state desired
   */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint - read documentation for these functions for more information
    state.optimize(getAngle());
    state.cosineScale(inputs.turnAbsolutePosition);

    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. (Straight line) */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
