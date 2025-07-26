package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            // Driving motor configuration
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            // Turning motor configuration - using internal encoder for closed loop
            // since we'll handle the analog encoder feedback in software
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            // Configure the turning motor to use its internal encoder for closed loop control
            // We'll use the analog encoder for absolute position feedback in software
            turningConfig.encoder
                    .positionConversionFactor(2 * Math.PI) // radians
                    .velocityConversionFactor(2 * Math.PI / 60.0); // radians per second

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to tune for your robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
