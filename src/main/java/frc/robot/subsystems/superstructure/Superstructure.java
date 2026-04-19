package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public Superstructure(Supplier<Pose2d> robotPoseSupplier) {
    this.targetSupplier = new TargetSupplier(robotPoseSupplier);
    this.shooterSupplier = new ShooterSupplier(robotPoseSupplier);
    this.shotCalculator = new ShotCalculator();
  }

  public void periodic() {
    TargetSupplier.Modes mode = targetSupplier.mode();

    // Calculate angle to target
    Translation2d target = targetSupplier.target();
    Pose2d shooter = shooterSupplier.shooter();
    Logger.recordOutput(
        "Superstructure/ShotLine", new Translation2d[] {shooter.getTranslation(), target});
    angleToTarget =
        target
            .minus(shooter.getTranslation())
            .getAngle()
            .minus(
                ShooterSupplier.shooterOffset
                    .getRotation()); // TODO why is this last angle correction needed?

    // Calculate shooter RPM
    double distanceMeters = target.minus(shooter.getTranslation()).getNorm();
    ShotCalculator.ShotData shotData = shotCalculator.calculateShot(distanceMeters);
    shooterRPM =
        switch (mode) {
          case HUB -> shotData.hubRPM;
          case RELAY -> shotData.relayRPM;
        };
  }

  public Rotation2d angleToTarget() {
    return angleToTarget;
  }

  public double shooterRPM() {
    return shooterRPM;
  }
}
