/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.orangefrc.annotation.GenerateJson;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  static double updatetimes = 0;

  @GenerateJson
  public class Hopper {
    double P;
    double I;
    double D;

    double kV;
    double maxAccel;
  }

  HopperJson json = new HopperJson();

  public HopperSubsystem(HopperIO io) {
    this.io = io;
  }

  public void runHopper(double speed) {
    io.setHopperSpeed(speed);
  }

  public void reverseHopper(double speed) {
    io.setHopperSpeed(speed);
  }

  public void runDutyCycle(double speed) {
    io.runDutyCycle(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
    // json.updateVals();
    // if (json.hasUpdated()) {
    //   updatetimes++;
    //   Logger.recordOutput("jsonUpdates/Hopper", "Hopper Updated: " + updatetimes);
    // }
    // if (json.hasUpdated()) {
    //   io.updatePID(json.getP(), json.getI(), json.getD(), json.getkV(), json.getmaxAccel());
    // }
    RobotContainer.hopperSpeed = SmartDashboard.getNumber("HopperSpeed", 0.123);
    SmartDashboard.putNumber("HopperSetpoint", io.getSetpoint());
    SmartDashboard.putNumber(
        "HopperError", (io.getSetpoint() - (60 * inputs.hopperVelocity.in(RotationsPerSecond))));
  }

  @Override
  public void simulationPeriodic() {}
}
