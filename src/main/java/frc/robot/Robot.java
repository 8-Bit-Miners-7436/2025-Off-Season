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


/*    //  Todo List  //
* Cleanup/Reformat Code As Needed
* Implement Basic Pre-Season Bot Functionality
* Fix Wheel Alignment Issues & Tune PID
* Add Easy Feature Enable/Disable Options
* Make Limelight-Powered Auto Goal Alignment
* See If Input From AdvantageScope Is Possible
* Figure Out Simulation Software
*/


public class Robot extends LoggedRobot {
  // * Initializes Xbox Controller Object
  private final XboxController gamepad = new XboxController(0);

  // * Initializes Pigeon (Velocity/Gyro Info Provider)
  // private final Pigeon2 pigeon = new Pigeon2(10);
  // private Pose3d llTargetData;
  // private double[] llCameraData;

  public Robot() {
    // * Starts The Connection To The Logger (AdvantageScope)
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // * Starts The Camera Stream
    // UsbCamera camera = CameraServer.startAutomaticCapture();

    // * Configures The Camera
    // camera.setFPS(30);        // Lower FPS To Improve Latency
    // camera.setBrightness(20); // Boost Brightness
  }

  // * Creates Kinematics Object:
  // * Stores Wheel Distances From Robot Center (in meters)
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(0.34, 0.3),   // Front Left
    new Translation2d(0.34, -0.3),  // Front Right
    new Translation2d(-0.34, 0.3),  // Rear Left
    new Translation2d(-0.34, -0.3)  // Rear Right
  );

  // * Places Initialized Swerve Modules In An Array For Easy Access
  private final CustomSwerveModule[] modules = new CustomSwerveModule[] {
    new CustomSwerveModule(1, "Front Left"),
    new CustomSwerveModule(2, "Front Right"),
    new CustomSwerveModule(3, "Rear Left"),
    new CustomSwerveModule(4, "Rear Right")
  };

  @Override
  public void robotPeriodic() {
    // * Records Limelight Diagnostics
    // llTargetData = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    // llCameraData = LimelightHelpers.getCameraPose_TargetSpace("limelight");

    // * Sends Diagnostics To Logger
    // ? Descriptions Here Might Be Inaccurate...
    // Logger.recordOutput("LL Target Data", llTargetData); // Target Rotation & Position In 3D Space
    // Logger.recordOutput("LL Camera Data", llCameraData); // Camera Position In Relation To Target
  }

  @Override
  public void teleopPeriodic() {

    // * Gets Joystick Values
    double vx = -gamepad.getLeftY();  // Forward/Backward movement
    double vy = -gamepad.getLeftX();  // Strafing movement
    double w  = -gamepad.getRightX(); // Rotational movement

    // * Converts Joystick Inputs Into Directional Movement Data
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, w);

    // * Decodes Movement Data Into Swerve Module Instructions (States)
    SwerveModuleState[] moduleStates;
    moduleStates = kinematics.toSwerveModuleStates(

      // * Default Instructions, Drives Like RC Car
      chassisSpeeds // ! Comment Out If Enabling Field-Oriented Controls

      // * Implements Field-Oriented Controls Using Pigeon Rotation Data
      // * Robot Is Controlled From The Driver's Perspective:
      // * Movement Direction Won't Change Depending On Robot's Rotation
      // ChassisSpeeds.fromFieldRelativeSpeeds(
      //   chassisSpeeds,
      //   Rotation2d.fromDegrees(pigeon.getYaw().refresh().getValue().magnitude())
      // )
    );

    // * Limits Wheel Rotation Speed To Limit Battery Strain
    // * Uncomment If Experiencing Brownout Issues
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 0.5);

    // * Sends General Driving Diagnostics To Logger
    // Logger.recordOutput("RobotRotation", pigeon.getRotation2d()); // Pigeon Rotation Data
    Logger.recordOutput("ChassisSpeeds", chassisSpeeds); // Directional Movement Data
    Logger.recordOutput("ModuleStates", moduleStates); // Individual Module States

    // * Passes Instructions To Each Swerve Module
    int i = 0;
    for (SwerveModuleState state : moduleStates) {
      modules[i++].updateState(state);
    }
  }

  @Override
  public void autonomousInit() {
    // * Zeros Out Module Encoders To Fix Alignment Issues
    // * Makes All Modules Believe They Are At 0deg Of Rotation
    // ? This Shouldn't Have To Be Done, Maybe PID Issue?
    for (CustomSwerveModule module : modules) {
      module.resetEncoders();
    }
  }

  @Override
  public void testInit() {
    // * Zeros Out Pigeon To Set Field-Oriented Drive Alignment
    // * Run This When Robot Is Facing Directly Away From Driver
    // pigeon.reset();
  }
}