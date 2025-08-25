package frc.robot;
import frc.robot.utils.*;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class Robot extends LoggedRobot {
  boolean BABYMODE = true;
  // Initialize Xbox Controller and IMU (Gyro)
  private final XboxController gamepad = new XboxController(0);
  // private final Pigeon2 pigeon = new Pigeon2(10);
  // private Pose3d llTranslationData;
  // private double[] llRotationData;

  public Robot() {
    // Initialize AdvantageKit (Logger)
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // Start the camera stream
    // UsbCamera camera = CameraServer.startAutomaticCapture();

    // Configure The Camera
    // camera.setFPS(30);
    // camera.setBrightness(20);
  }

  // Create Kinematics Object w/ Module Offsets from Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.34, 0.3),   // Front Left
    new Translation2d(0.34, -0.3),  // Front Right
    new Translation2d(-0.34, 0.3),  // Rear Left
    new Translation2d(-0.34, -0.3)  // Rear Right
  );

  // Package Initialized Swerve Modules in an Array for Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
    new CustomSwerveModule(1, "Front Left"),
    new CustomSwerveModule(2, "Front Right"),
    new CustomSwerveModule(3, "Rear Left"),
    new CustomSwerveModule(4, "Rear Right")
  };

  @Override
  public void robotPeriodic() {
    // Send Limelight Diagnostics to Logger
    // llTranslationData = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    // llRotationData = LimelightHelpers.getCameraPose_TargetSpace("limelight");
    // Logger.recordOutput("LimelightData", llTranslationData);
    // Logger.recordOutput("LimelightRotationData", llRotationData);
  }

  @Override
  public void teleopPeriodic() {
    // Get Joystick Values for Driving
    double vx = -gamepad.getLeftY(); // Forward/backward movement
    double vy = -gamepad.getLeftX(); // Strafing movement
    double w = -gamepad.getRightX(); // Rotational movement
    if (BABYMODE) {
      vx /= 3;
      vy /= 3;
      w /= 2;
    }
    // Convert Joystick Inputs to Swerve Module Instructions (States)
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, w);
    SwerveModuleState[] moduleStates;
    moduleStates = kinematics.toSwerveModuleStates(
      chassisSpeeds
      // ChassisSpeeds.fromFieldRelativeSpeeds(
      //   chassisSpeeds,
      //   Rotation2d.fromDegrees(pigeon.getYaw().refresh().getValue().magnitude())
      // )
    );

    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 0.5);

    // Send Swerve Diagnostics to Logger
    // Logger.recordOutput("RobotRotation", pigeon.getRotation2d());
    Logger.recordOutput("ChassisSpeeds", chassisSpeeds);
    Logger.recordOutput("ModuleStates", moduleStates);
    // Pass Instructions to each Swerve Module
    int i = 0;
    for (SwerveModuleState state : moduleStates)
      modules[i++].updateState(state);
  }

  @Override
  public void autonomousInit() {
    // Zero Out all Module Encoders to Fix Alignment Issues
    for (CustomSwerveModule module : modules)
      module.resetEncoders();
  }

  // @Override
  // public void testInit() {
    // // Zero Out Pigeon to Fix Field-Oriented Drive Alignment
    // pigeon.reset();
  // }
}