// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class IntakeIOSparkFlex implements IntakeIO {
  private final SparkFlex motor = new SparkFlex(1, MotorType.kBrushless); // TODO TS: CAN ID
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController pid = motor.getClosedLoopController();
  private final SparkMaxConfig config = new SparkMaxConfig();

  public IntakeIOSparkFlex() {
    config
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(30, 20)
        .secondaryCurrentLimit(50.0);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionsRadians =
        new double[] {
          Units.rotationsToRadians(encoder.getPosition()) / Intake.Constants.GEAR_RATIO,
        };
    inputs.velocitiesRadPerSec =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(
              encoder.getVelocity() / Intake.Constants.GEAR_RATIO),
        };
    inputs.appliedVolts =
        new double[] {
          motor.getAppliedOutput() * motor.getBusVoltage(),
        };
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * Intake.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
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
