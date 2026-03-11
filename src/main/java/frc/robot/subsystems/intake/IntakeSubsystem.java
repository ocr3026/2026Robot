/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.intake;

import com.orangefrc.annotation.GenerateJson;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  IntakeJson json = new IntakeJson();

  @GenerateJson
  public class Intake {
    double iP = 0;
    double iI = 0;
    double iD = 0;
    double iV = 0;
    double iA = 0;

    double lP = 0;
    double lI = 0;
    double lD = 0;
    double lV = 0;
    double lA = 0;
  }

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void intakeFuel(double speed) {
    io.setIntakeSpeed(speed);
  }

  public void intakeLift(double speed) {
    io.setIntakeLiftSpeed(speed);
  }

  public double getIntakeLiftPos() {
    return io.getIntakePosition();
  }

  public void intakeLiftPos(double pos) {
    io.setIntakeLiftPos(pos);
  }

  public void runIntakeLiftUntil(double pos, double speed) {
    SmartDashboard.putNumber("intake lift shit.", getIntakeLiftPos());

    if (io.getIntakePosition() >= pos) {
      io.runWhileGreater(speed, pos);
    } else {
      io.runWhileGreater(0.0, pos);
    }
  }

  @Override
  public void periodic() {
    // json.updateVals();
    // if (json.hasUpdated()) {
    //   io.updatePID(
    //       json.getiP(),
    //       json.getiI(),
    //       json.getiD(),
    //       json.getiV(),
    //       json.getiA(),
    //       json.getlP(),
    //       json.getlI(),
    //       json.getlD(),
    //       json.getlV(),
    //       json.getlA());
    // }
    io.updateInputs(inputs);
    io.getIntakePosition();
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake Input Lift Pos", inputs.intakeLiftPosition);
    Logger.recordOutput("Intake Lift Pos Raw", io.getIntakePosition());
  }

  @Override
  public void simulationPeriodic() {}
}
