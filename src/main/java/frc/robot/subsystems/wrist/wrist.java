package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class wrist extends SubsystemBase {
  private final wristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public wrist(wristIO parameter_Io) {
    this.io = parameter_Io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("wrist", inputs);
  }
}
