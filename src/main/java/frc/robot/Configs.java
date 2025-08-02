package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {

        //ONLY FOR IF NO MOTORS NEED TO BE INVERTED
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
        //ONLY FOR INVERTING 1 MOTOR
        public static final SparkFlexConfig rearRightDrivingConfig = new SparkFlexConfig();
        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI/ ModuleConstants.kDrivingMotorReduction;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
            rearRightDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(true); // <--- Invert this motor only
        
            rearRightDrivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60.0);
        
            rearRightDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);
        }
    }
}


// determine where the error occurred. ﻿
// ﻿﻿﻿﻿﻿﻿   See https://wpilib.org/stacktrace for more information. ﻿
// ﻿﻿﻿﻿﻿﻿ Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿
// ﻿﻿﻿﻿﻿﻿ ********** Robot program starting ********** ﻿
// ﻿﻿﻿﻿﻿﻿ NT: Listening on NT3 port 1735, NT4 port 5810 ﻿
// ﻿﻿﻿﻿﻿﻿ NT: Got a NT4 connection from 10.63.46.20 port 63866 ﻿
// ﻿﻿﻿﻿﻿﻿ NT: CONNECTED NT4 client 'Dashboard@1' (from 10.63.46.20:63866) ﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ Failed to load RobotConfig from GUI settings: /home/lvuser/deploy/pathplanner/settings.json (No such file or directory) ﻿﻿ frc.robot.RobotContainer.<init>(RobotContainer.java:130) ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿ Error at frc.robot.RobotContainer.<init>(RobotContainer.java:130): Failed to load RobotConfig from GUI settings: /home/lvuser/deploy/pathplanner/settings.json (No such file or directory) ﻿
// ﻿﻿﻿﻿﻿﻿ java.io.FileNotFoundException: /home/lvuser/deploy/pathplanner/settings.json (No such file or directory) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileInputStream.open0(Native Method) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileInputStream.open(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileInputStream.<init>(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileReader.<init>(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at com.pathplanner.lib.config.RobotConfig.fromGUISettings(RobotConfig.java:266) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.RobotContainer.<init>(RobotContainer.java:110) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Robot.robotInit(Robot.java:31) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:107) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:419) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:510) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Main.main(Main.java:23) ﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ Unhandled exception: com.pathplanner.lib.auto.AutoBuilderException: AutoBuilder was not configured before attempting to load a PathPlannerAuto from file ﻿﻿ com.pathplanner.lib.commands.PathPlannerAuto.<init>(PathPlannerAuto.java:78) ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿ Error at com.pathplanner.lib.commands.PathPlannerAuto.<init>(PathPlannerAuto.java:78): Unhandled exception: com.pathplanner.lib.auto.AutoBuilderException: AutoBuilder was not configured before attempting to load a PathPlannerAuto from file ﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 1 ﻿﻿ The robot program quit unexpectedly. This is usually due to a code error.
//   The above stacktrace can help determine where the error occurred.
//   See https://wpilib.org/stacktrace for more information. ﻿﻿ edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:433) ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿ 	at com.pathplanner.lib.commands.PathPlannerAuto.<init>(PathPlannerAuto.java:78) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at com.pathplanner.lib.commands.PathPlannerAuto.<init>(PathPlannerAuto.java:63) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.RobotContainer.<init>(RobotContainer.java:136) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Robot.robotInit(Robot.java:31) ﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ The startCompetition() method (or methods called by it) should have handled the exception above. ﻿﻿ edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440) ﻿﻿﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:107) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:419) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:510) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Main.main(Main.java:23) ﻿
// ﻿﻿﻿﻿﻿﻿  ﻿
// ﻿﻿﻿﻿﻿﻿ ﻿Warning﻿ at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:433): The robot program quit unexpectedly. This is usually due to a code error. ﻿
// ﻿﻿﻿﻿﻿﻿   The above stacktrace can help determine where the error occurred. ﻿
// ﻿﻿﻿﻿﻿﻿   See https://wpilib.org/stacktrace for more information. ﻿
// ﻿﻿﻿﻿﻿﻿ Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:440): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿
// ﻿﻿﻿﻿﻿﻿ ********** Robot program starting ********** ﻿
// ﻿﻿﻿﻿﻿﻿ NT: Listening on NT3 port 1735, NT4 port 5810 ﻿
// ﻿﻿﻿﻿﻿﻿ NT: Got a NT4 connection from 10.63.46.20 port 63881 ﻿
// ﻿﻿﻿﻿﻿﻿ NT: CONNECTED NT4 client 'Dashboard@1' (from 10.63.46.20:63881) ﻿
// ﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ Failed to load RobotConfig from GUI settings: /home/lvuser/deploy/pathplanner/settings.json (No such file or directory) ﻿﻿ frc.robot.RobotContainer.<init>(RobotContaine