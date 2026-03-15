// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;

public class VisionConstants {
  public static class CameraConfig {
    public static class SimCameraProperties {
      public final int pixelWidth;
      public final int pixelHeight;
      public final Rotation2d fov;

      public SimCameraProperties(int pixelWidth, int pixelHeight, Rotation2d fov) {
        this.pixelWidth = pixelWidth;
        this.pixelHeight = pixelHeight;
        this.fov = fov;
      }
    }

    public final String name;
    public final Transform3d robotToCamera;
    public final Optional<SimCameraProperties> simCameraProperties;

    public CameraConfig(String name, Transform3d robotToCamera) {
      this.name = name;
      this.robotToCamera = robotToCamera;
      this.simCameraProperties = Optional.empty();
    }

    public CameraConfig(
        String name, Transform3d robotToCamera, SimCameraProperties simCameraProperties) {
      this.name = name;
      this.robotToCamera = robotToCamera;
      this.simCameraProperties = Optional.of(simCameraProperties);
    }
  }

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static CameraConfig[] cameraConfigs = {
    new CameraConfig(
        "Camera0",
        new Transform3d(
            Units.inchesToMeters(8.5),
            Units.inchesToMeters(13.6275),
            Units.inchesToMeters(13.5625),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90))),
        new CameraConfig.SimCameraProperties(1280, 720, Rotation2d.fromDegrees(100))),
    new CameraConfig(
        "Camera1",
        new Transform3d(
            Units.inchesToMeters(-11.25),
            Units.inchesToMeters(12.9375),
            Units.inchesToMeters(16.625),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-45),
                Units.degreesToRadians(135))),
        new CameraConfig.SimCameraProperties(1280, 720, Rotation2d.fromDegrees(70))),
    new CameraConfig(
        "Camera2",
        new Transform3d(
            Units.inchesToMeters(11),
            Units.inchesToMeters(-12.375),
            Units.inchesToMeters(14.375),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-15),
                Units.degreesToRadians(345))),
        new CameraConfig.SimCameraProperties(1280, 720, Rotation2d.fromDegrees(90))),
    new CameraConfig(
        "Camera3",
        new Transform3d(
            Units.inchesToMeters(-11.5),
            Units.inchesToMeters(-12.75),
            Units.inchesToMeters(16.625),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-45),
                Units.degreesToRadians(225))),
        new CameraConfig.SimCameraProperties(1280, 720, Rotation2d.fromDegrees(80)))
  };

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
