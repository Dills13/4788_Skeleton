package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO parameter_io) {
    System.out.println("Intake created with IO type: " + parameter_io.getClass().getSimpleName());
    this.io = parameter_io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command stop() {
    return run(() -> io.stopIntake()).withName("Stop");
  }

  public Command runIntake(double volts) {
    return run(() -> io.setVoltage(volts)).withName("Intake " + volts);
  }

  public Command intake() {
    return run(() -> io.setVoltage(4));
  }
}
