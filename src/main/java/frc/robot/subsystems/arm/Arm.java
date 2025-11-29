package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final PIDController pid = new PIDController(3, 0, 0.2);
  Mechanism2d mechanism = new Mechanism2d(3, 3);
  Mechanism2d mechanism2 = new Mechanism2d(3, 3);

  private final Mechanism2d mech = new Mechanism2d(2, 2);
  private final Mechanism2d mech2 = new Mechanism2d(2, 2);
  private final MechanismRoot2d root = mech.getRoot("ArmRoot", 1, 0);
  private final MechanismLigament2d armLigament =
      new MechanismLigament2d(
          "Arm", 1.0, 0, 6, new Color8Bit(Color.kMagenta)); // length 1m, angle 0°
  private final MechanismLigament2d armLigament2 =
      new MechanismLigament2d("Arm2", 1.0, 0, 3, new Color8Bit(Color.kGold));

  public Arm(ArmIO parameter_io) {
    this.io = parameter_io;
    pid.setTolerance(0.2);
    root.append(armLigament);
    armLigament.append(armLigament2);
    SmartDashboard.putData("Arm", mech);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // set variable to check setpoint to a method
    // which uses the pid object to see if the arm is actually within the setpoint range

    // Update visualization

    armLigament2.setAngle(Math.toDegrees(inputs.arm_position * 2));
    Logger.recordOutput("Arm/Mechanism", inputs.arm_position);
    armLigament.setAngle(Math.toDegrees(inputs.arm_position));
    Logger.recordOutput("Arm/Mechanism", inputs.arm_position);

    inputs.arm_atSetpoint = pid.atSetpoint();
    Logger.processInputs("Arm", inputs);
  }

  // create commands which we will schedule later

  public Command RawControl(double volts) {
    return run(() -> io.setVoltage(volts));
  }

  public Command goToSetpoint(double target) {
    return run(
        () -> {
          var out = pid.calculate(inputs.arm_position, target);
          Logger.recordOutput("Arm/Voltage", out);
          Logger.recordOutput("Arm/Error", pid.getError());
          io.setVoltage(out);
          Logger.recordOutput("Arm/Target", target);
        });
  }

  public Command stop() {
    return runOnce(() -> io.setVoltage(0.0));
  }
}
