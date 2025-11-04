package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CustomSwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final PIDController steerPID;
    private final RelativeEncoder driveEncoder;
    private final CANcoder steerEncoder;

    public CustomSwerveModule(int moduleNumber, String modulePosition) {
        // * Initializes Module's Drive And Steer Motors
        // Each Module Has 2 CAN IDs
        // This Allows Us To Simply Offset The Value By 2 For Each Motor
        driveMotor = new SparkMax(moduleNumber * 2, MotorType.kBrushless);
        steerMotor = new SparkMax(moduleNumber * 2 + 1, MotorType.kBrushless);

        // * Initializes The Motor Encoders For Angle And Positioning Feedback
        // Steer Motors Should Use The Built-In Swerve Casing Encoders
        steerEncoder = new CANcoder(moduleNumber + 10);
        driveEncoder = driveMotor.getEncoder();

        // * Initializes PID Controller
        // Todo: Tune PID Values
        // ? Maybe Try Rev PID Generator
        steerPID = new PIDController(0.5, 0, 0.05);
        steerPID.enableContinuousInput(-0.5, 0.5);

        // * Impose Soft Current Limit On Motors To Prevent Brownout
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(30);
        driveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateState(SwerveModuleState newState) {
        // * Get Steer Motor Angle In Rotations
        double steerMotorRot = steerEncoder.getAbsolutePosition().refresh().getValueAsDouble();

        // * Optimizes Rotation Distance And Removes Wraparound
        // For Example: If The Motor Is At 180deg And The Driver
        // Pulls Directly Back On The Stick, Instead Of Turning
        // All The Way To 0deg, It Stays At 180deg
        newState.optimize(Rotation2d.fromRotations(steerMotorRot));

        // * Calculate Speed Based On Error Between The Target And Current Angle
        double steerSpeed = steerPID.calculate(steerMotorRot, newState.angle.getRotations());

        // * Pass Finalized Values/Instructions To Motors
        steerMotor.set(steerSpeed);
        driveMotor.set(newState.speedMetersPerSecond);
    }

    // * Sets Drive Motor Speed
    public void setDriveSpeed(double driveSpeed) {
        driveMotor.set(driveSpeed);
    }

    // * Zeros Out Motor Encoders to Correct Alignment
    public void resetEncoders() {
        steerEncoder.setPosition(0);
        driveEncoder.setPosition(0);
    }

    // * Get Distance Traveled For Auto
    public double getEncoderDistance() {
        return driveEncoder.getPosition();
    }
}