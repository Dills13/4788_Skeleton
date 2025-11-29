package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final DCMotorSim armSim;
  private final SingleJointedArmSim visualArmSim;

  private final DCMotor armMotor = DCMotor.getNEO(1);
  private double voltage = 0.0;

  public ArmIOSim() {
    armSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(armMotor, 0.02, 100), armMotor);

    visualArmSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1), 100.0, 0.02, 0.8, -Math.PI / 4, 2 * Math.PI, true, 0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);
    visualArmSim.update(0.02);

    visualArmSim.getOutput(0);
    visualArmSim.getAngleRads();
    visualArmSim.getVelocityRadPerSec();

    inputs.arm_appliedVolts = voltage;
    inputs.arm_currentAmp = armSim.getCurrentDrawAmps();
    inputs.arm_position = armSim.getAngularPositionRad();
    inputs.arm_angularVelocity = armSim.getAngularVelocityRPM();
    inputs.arm_goalOne = Math.toRadians(45);
    inputs.arm_goalTwo = Math.toRadians(90);
    inputs.arm_goalThree = Math.toRadians(360);
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = MathUtil.clamp(voltage, -12, 12);
    armSim.setInputVoltage(voltage);
    visualArmSim.setInputVoltage(voltage);
  }
}
