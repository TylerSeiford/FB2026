package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final TargetSupplier targetSupplier;
  private final ShooterSupplier shooterSupplier;
  private final ShotCalculator shotCalculator;

  @AutoLogOutput(key = "Superstructure/AngleToTarget")
  private Rotation2d angleToTarget = Rotation2d.fromDegrees(0.0);

  @AutoLogOutput(key = "Superstructure/ShooterRPM")
  private double shooterRPM = 0.0;

  public Superstructure(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.targetSupplier = new TargetSupplier(robotPoseSupplier);
    this.shooterSupplier = new ShooterSupplier(robotPoseSupplier, robotSpeedsSupplier);
    this.shotCalculator = new ShotCalculator();
  }

  public void periodic() {
    shooterSupplier.periodic();

    TargetSupplier.Modes mode = targetSupplier.mode();
    Translation2d target = targetSupplier.target();
    Pose2d shooter = shooterSupplier.shooterPose();
    Translation2d shooterVelocity = shooterSupplier.shooterVelocity();

    SotMData sotMData = iterateSotM(target, shooter.getTranslation(), shooterVelocity);
    Logger.recordOutput("Superstructure/PredictedTarget", sotMData.predictedTarget);
    Logger.recordOutput("Superstructure/DistanceMeters", sotMData.shotData.distanceMeters);
    Logger.recordOutput("Superstructure/HubRPM", sotMData.shotData.hubRPM);
    Logger.recordOutput("Superstructure/RelayRPM", sotMData.shotData.relayRPM);
    Logger.recordOutput("Superstructure/TimeOfFlight", sotMData.shotData.timeOfFlight);
    Logger.recordOutput(
        "Superstructure/ShotLine",
        new Translation2d[] {shooter.getTranslation(), sotMData.predictedTarget});

    angleToTarget =
        sotMData
            .predictedTarget
            .minus(shooter.getTranslation())
            .getAngle()
            .minus(ShooterSupplier.shooterOffset.getRotation());
    shooterRPM =
        switch (mode) {
          case HUB -> sotMData.shotData.hubRPM;
          case RELAY -> sotMData.shotData.relayRPM;
        };
  }

  private static class SotMData {
    public final Translation2d predictedTarget;
    public final ShotCalculator.ShotData shotData;

    public SotMData(Translation2d predictedTarget, ShotCalculator.ShotData shotData) {
      this.predictedTarget = predictedTarget;
      this.shotData = shotData;
    }
  }

  private SotMData iterateSotM(Translation2d target, Translation2d pose, Translation2d velocity) {
    final double tofConvergenceToleranceSeconds = 0.02;

    int i = 0;
    SotMData sotMData =
        new SotMData(target, shotCalculator.calculateShot(target.getDistance(pose)));

    while (i < 20) {
      final double lastTof = sotMData.shotData.timeOfFlight;

      Translation2d predictedTarget = target.plus(velocity.times(sotMData.shotData.timeOfFlight));
      sotMData =
          new SotMData(
              predictedTarget, shotCalculator.calculateShot(predictedTarget.getDistance(pose)));
      if (Math.abs(sotMData.shotData.timeOfFlight - lastTof) < tofConvergenceToleranceSeconds) {
        break;
      }
      i++;
    }
    Logger.recordOutput("Superstructure/SotMIterations", i);
    return sotMData;
  }

  public Rotation2d angleToTarget() {
    return angleToTarget;
  }

  public double shooterRPM() {
    return shooterRPM;
  }
}
