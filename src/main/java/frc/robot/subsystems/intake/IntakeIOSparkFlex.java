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
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;

public class IntakeIOSparkFlex implements IntakeIO {
  private final SparkFlex primaryMotor = new SparkFlex(12, MotorType.kBrushless);
  private final RelativeEncoder primaryEncoder = primaryMotor.getEncoder();
  private final SparkClosedLoopController primaryPID = primaryMotor.getClosedLoopController();
  private final SparkFlexConfig primaryConfig = new SparkFlexConfig();
  private final SparkFlex secondaryMotor = new SparkFlex(11, MotorType.kBrushless);
  private final RelativeEncoder secondaryEncoder = secondaryMotor.getEncoder();
  private final SparkFlexConfig secondaryConfig = new SparkFlexConfig();

  public IntakeIOSparkFlex() {
    primaryConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(40, 20)
        .secondaryCurrentLimit(50.0);
    SparkUtil.tryUntilOk(
        primaryMotor,
        5,
        () ->
            primaryMotor.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    secondaryConfig.apply(primaryConfig);
    secondaryConfig.follow(12, true);
    SparkUtil.tryUntilOk(
        secondaryMotor,
        5,
        () ->
            secondaryMotor.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionsRadians =
        new double[] {
          Units.rotationsToRadians(primaryEncoder.getPosition()) / Intake.Constants.GEAR_RATIO,
          Units.rotationsToRadians(secondaryEncoder.getPosition()) / Intake.Constants.GEAR_RATIO
        };
    inputs.velocitiesRadPerSec =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(
              primaryEncoder.getVelocity() / Intake.Constants.GEAR_RATIO),
          Units.rotationsPerMinuteToRadiansPerSecond(
              secondaryEncoder.getVelocity() / Intake.Constants.GEAR_RATIO)
        };
    inputs.appliedVolts =
        new double[] {
          primaryMotor.getAppliedOutput() * primaryMotor.getBusVoltage(),
          secondaryMotor.getAppliedOutput() * secondaryMotor.getBusVoltage()
        };
    inputs.currentAmps =
        new double[] {primaryMotor.getOutputCurrent(), secondaryMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    primaryMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    primaryPID.setSetpoint(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * Intake.Constants.GEAR_RATIO,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    primaryMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    primaryConfig.closedLoop.pid(kP, kI, kD);
    SparkUtil.tryUntilOk(
        primaryMotor,
        5,
        () ->
            primaryMotor.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }
}
