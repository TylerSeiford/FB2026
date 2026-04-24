package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSupplier {
  public static final Transform2d shooterOffset =
      new Transform2d(
          Units.inchesToMeters(-8.0), Units.inchesToMeters(12.0), Rotation2d.fromDegrees(-93.0));

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> robotSpeedsSupplier;

  @AutoLogOutput(key = "Superstructure/ShooterSupplier/ShooterPose")
  private Pose2d shooterPose = new Pose2d();

  @AutoLogOutput(key = "Superstructure/ShooterSupplier/ShooterVelocity")
  private Translation2d shooterVelocity = new Translation2d();

  public ShooterSupplier(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.robotSpeedsSupplier = robotSpeedsSupplier;
  }

  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();
    shooterPose = robotPose.transformBy(shooterOffset);

    ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();
    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());
    Logger.recordOutput("Superstructure/ShooterSupplier/RobotVelocity", robotVelocity);
    Logger.recordOutput(
        "Superstructure/ShooterSupplier/RobotVelocityLine",
        new Translation2d[] {
          robotPose.getTranslation(), robotPose.getTranslation().plus(robotVelocity)
        });

    Translation2d shooterRotationalVelocity =
        new Translation2d(
            robotSpeeds.omegaRadiansPerSecond * shooterOffset.getTranslation().getNorm(),
            shooterPose.getRotation());
    shooterVelocity = robotVelocity.plus(shooterRotationalVelocity);
    Logger.recordOutput(
        "Superstructure/ShooterSupplier/ShooterVelocityLine",
        new Translation2d[] {
          shooterPose.getTranslation(), shooterPose.getTranslation().plus(shooterVelocity)
        });
  }

  public Pose2d shooterPose() {
    return shooterPose;
  }

  public Translation2d shooterVelocity() {
    return shooterVelocity;
  }
}
