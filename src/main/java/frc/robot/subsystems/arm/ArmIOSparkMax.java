package frc.robot.subsystems.arm;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;

public class ArmIOSparkMax implements ArmIO {
  private final SparkMax motor = new SparkMax(13, MotorType.kBrushless); // TODO TS: CAN ID
  private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();
  private final SparkClosedLoopController pid = motor.getClosedLoopController();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public ArmIOSparkMax() {
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(50, 30)
        .secondaryCurrentLimit(60.0);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());
    inputs.appliedVolts = new double[] {motor.getAppliedOutput() * motor.getBusVoltage()};
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setPosition(Rotation2d angle, double ffVolts) {
    pid.setSetpoint(
        angle.getRotations(),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
