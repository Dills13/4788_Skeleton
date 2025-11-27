package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor intakeMotor = DCMotor.getNEO(2);
  private final DCMotorSim intakeMotorSim;
  // private final SimDevice sensorImpl;
  // private final SimBoolean sensor;

  private double volts = 0;
  private double appliedVolts = 0.0;

  public IntakeIOSim() {
    intakeMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(intakeMotor, 3, 1), intakeMotor);

    // sensorImpl = SimDevice.create("IntakeSensorFront", 3);
    // sensor = sensorImpl.createBoolean("IsTriggered", Direction.kInput, false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorSim.update(0.02);
    inputs.currentAmps = intakeMotorSim.getCurrentDrawAmps();
    inputs.appliedVolts = intakeMotorSim.getInputVoltage();
    inputs.angularVelocity = intakeMotorSim.getAngularVelocityRPM();

    System.out.println(
        "IntakeIOSim.updateInputs running, RPM = " + intakeMotorSim.getAngularVelocityRPM());
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = MathUtil.clamp(voltage, -12, 12);
    intakeMotorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stopIntake() {
    intakeMotorSim.setInputVoltage(0);
  }
}
