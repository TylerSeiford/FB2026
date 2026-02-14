// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOCanandGyro;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkFlex;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkFlex;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOSparkFlex;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  private final Spindexer spindexer;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOCanandGyro(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        shooter = new Shooter(new ShooterIOSparkFlex(), () -> 0.0);
        intake = new Intake(new IntakeIOSparkFlex());
        arm = new Arm(new ArmIOSparkMax());
        spindexer = new Spindexer(new SpindexerIOSparkFlex());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        shooter = new Shooter(new ShooterIOSim(), () -> 0.0);
        intake = new Intake(new IntakeIOSim());
        arm = new Arm(new ArmIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {}, () -> 0.0);
        intake = new Intake(new IntakeIO() {});
        arm = new Arm(new ArmIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to cardinal 90° angles when pov buttons are held
    controller
        .povUp()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));
    controller
        .povRight()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kCW_90deg));
    controller
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kPi));
    controller
        .povLeft()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kCCW_90deg));

    // Toggle X pattern when left stick is pressed
    controller.leftStick().toggleOnTrue(Commands.startEnd(drive::stopWithX, drive::stop, drive));

    // Reset gyro to 0° when right stick is pressed
    controller
        .rightStick()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Toggle slow drive when start is pressed
    controller
        .start()
        .toggleOnTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * 0.5,
                () -> -controller.getLeftX() * 0.5,
                () -> -controller.getRightX() * 0.5)); // TODO: Improve speed adjustment

    // TODO TS: Temporary controls, replace with better systems once implemented
    controller
        .leftBumper()
        .onTrue(Commands.parallel(shooter.hub(), intake.intake(), spindexer.shoot()));
    controller
        .rightBumper()
        .onTrue(Commands.parallel(shooter.stop(), intake.stop(), spindexer.stop()));
    controller.povUp().onTrue(shooter.auto());
    controller.povLeft().onTrue(shooter.relay());

    // Eject all the things while back is held, otherwise stop
    controller
        .back()
        .onTrue(Commands.parallel(shooter.eject(), intake.eject(), spindexer.eject()))
        .onFalse(Commands.parallel(shooter.stop(), intake.stop(), spindexer.stop()));

    // ***** Prototype final controls *****

    // Shoot while left trigger is held
    controller
        .leftTrigger()
        .onTrue(Commands.sequence(intake.stop(), shooter.hub(), spindexer.shoot(), arm.stow()))
        .whileTrue(
            // TODO: Replace with orbit drive
            DriveCommands.joystickDriveAtTarget(
                drive,
                Constants.shooterOffset,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(),
                () -> FieldConstants.Hub.topCenterPoint.toTranslation2d()))
        .onFalse(Commands.sequence(spindexer.stop(), shooter.stop()));
    // Relay while left bumper is held
    controller
        .leftBumper()
        .onTrue(Commands.sequence(intake.stop(), shooter.relay(), spindexer.shoot(), arm.stow()))
        .whileTrue(
            DriveCommands.joystickDriveAtTarget(
                drive,
                Constants.shooterOffset,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> FieldConstants.LeftBump.oppFarLeftCorner, // TODO
                () -> FieldConstants.LeftBump.farLeftCorner)) // TODO
        .onFalse(Commands.sequence(spindexer.stop(), shooter.stop()));
    // Intake while right trigger is held
    controller
        .rightTrigger()
        .onTrue(Commands.sequence(shooter.stop(), spindexer.stop(), arm.intake(), intake.intake()))
        .onFalse(Commands.sequence(intake.stop(), arm.stow()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
