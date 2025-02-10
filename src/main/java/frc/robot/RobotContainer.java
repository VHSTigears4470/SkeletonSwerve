// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwervePhysicalConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TestDrivingMotors;
import frc.robot.commands.TestSetPosCommand;
import frc.robot.commands.TestSwerveJoystickCommand;
import frc.robot.commands.TestTurningMotors;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        SwerveSubsystem m_swerveSub = new SwerveSubsystem();

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final CommandXboxController m_driverController = new CommandXboxController(
                        IOConstants.DRIVER_CONTROLLER_PORT);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_swerveSub
                                .setDefaultCommand(new SwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_Y_AXIS),
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_X_AXIS),
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_ROT_AXIS),
                                                () -> true));
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                // m_driverController.x().onTrue(new SwerveJoystickCommand(
                // m_swerveSub,
                // () -> m_driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
                // () -> m_driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
                // () -> m_driverController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
                // // When pressed, changes to robot orientated
                // () ->
                // !m_driverController.button(OIConstants.DRIVER_FIELD_ORIENTED_BUTTON_IDX).getAsBoolean()
                // ));

                int preset = 2;
                switch (preset) {
                        case 0:
                                controllerPresetMain();
                                break;
                        case 1:
                                controllerPresetOne();
                                break;
                        case 2:
                                controllerPresetTwo();
                                break;
                        default:
                                controllerPresetMain();
                                break;
                }
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                /*
                 * // Moves straight one meter -> then straight and left one meter -> then
                 * rotates 180 degrees
                 * // 1. Create trajectory settings
                 * TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                 * AutoConstants.MAX_SPEED_METER_PER_SECOND,
                 * AutoConstants.MAX_ACCELERATION_METER_PER_SECOND_SQUARED)
                 * .setKinematics(SwervePhysicalConstants.DRIVE_KINEMATICS);
                 * 
                 * // 2. Generate trajectory
                 * Trajectory trajectory =
                 * // TrajectoryGenerator.generateTrajectory(
                 * // new Pose2d(0, 0, new Rotation2d(0)),
                 * // List.of(
                 * // new Translation2d(-1, 0)
                 * // ),
                 * // new Pose2d(-1, 0, Rotation2d.fromDegrees(180)),
                 * // trajectoryConfig
                 * // );
                 * TrajectoryGenerator.generateTrajectory(
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * List.of(
                 * new Translation2d(1, 0),
                 * new Translation2d(1, -1)),
                 * new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                 * trajectoryConfig);
                 * 
                 * // 3. Define PID controllers for tracking trajectory
                 * PIDController xController = new PIDController(AutoConstants.P_X_CONTROLLER,
                 * 0, 0);
                 * PIDController yController = new PIDController(AutoConstants.P_Y_CONTROLLER,
                 * 0, 0);
                 * ProfiledPIDController thetaController = new ProfiledPIDController(
                 * AutoConstants.P_THETA_CONTROLLER, 0, 0,
                 * AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
                 * thetaController.enableContinuousInput(-Math.PI, Math.PI);
                 * 
                 * // 4. Construct command to follow trajectory
                 * SwerveControllerCommand swerveControllerCommand = new
                 * SwerveControllerCommand(
                 * trajectory,
                 * m_swerveSub::getPose,
                 * SwervePhysicalConstants.DRIVE_KINEMATICS,
                 * xController,
                 * yController,
                 * thetaController,
                 * m_swerveSub::setModuleStates,
                 * m_swerveSub);
                 * 
                 * // 5. Add some init and wrap-up, and return everything
                 * return new SequentialCommandGroup(
                 * new InstantCommand(() ->
                 * m_swerveSub.resetOdometry(trajectory.getInitialPose())),
                 * swerveControllerCommand,
                 * new InstantCommand(() -> m_swerveSub.stopModules()));
                 */
                return new InstantCommand();
        }

        // Presets
        /**
         * The default / main preset used for comps
         * Does the following:
         * nothing for now :D
         */
        public void controllerPresetMain() {

        }

        /**
         * The default / main preset used for comps
         * Does the following:
         * Button B : Reset Swerve's Odometer
         * Button A : Stops All Swerve Modules
         * Button X : Make Wheels Straight
         * Right Trigger : Hold + Joystick w/o being field orientated
         * Right Bumper : Hold to go forward relative to robot
         * Left Trigger : Rotate Clckwise
         * Left Bumper : Rotate Counter Clockwise
         */
        public void controllerPresetOne() {
                // Reset Odom
                m_driverController.b().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> m_swerveSub.zeroHeading(),
                                                                m_swerveSub),
                                                new InstantCommand(
                                                                () -> m_swerveSub.resetOdometry(new Pose2d()),
                                                                m_swerveSub)));

                // Stop Button
                m_driverController.a().onTrue(
                                new InstantCommand(
                                                () -> m_swerveSub.stopModules(),
                                                m_swerveSub));

                // Straighten Wheels
                m_driverController.x().whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Wheels Straight"));

                // drive with Robot Orientation
                m_driverController.rightTrigger().whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_Y_AXIS),
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_X_AXIS),
                                                () -> m_driverController.getRawAxis(IOConstants.DRIVER_ROT_AXIS),
                                                // When pressed, changes to robot orientated
                                                () -> false,
                                                false,
                                                "Robot Orientated"));

                // drive forward only
                m_driverController.rightBumper().whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.4,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Forward"));

                // rotate clockwise with joystick input
                m_driverController.leftBumper().whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (double) 0.4,
                                                () -> (boolean) false,
                                                false,
                                                "Rotate"));

                // rotate counter clockwise with joystick input
                m_driverController.leftTrigger().whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (double) -0.4,
                                                () -> (boolean) false,
                                                false,
                                                "Rotate"));

        }

        /**
         * The default / main preset used for comps
         * Does the following:
         * Right Trigger : Reset Heading & Odom
         * Right Bumper : Rotate Wheels Clockwise
         * Left Bumper : Rotate Wheels Clockwise
         * 
         * Button B : Set Wheels Right
         * Button X : Set Wheels Left
         * Button Y : Set Wheel Straight
         * Button A : Set Wheels Backward
         * 
         * Left Trigger + Button B : Move Robot Right
         * Left Trigger + Button X : Move Robot Left
         * Left Trigger + Button Y : Move Robot Forward
         * Left Trigger + Button A : Move Robot Backward
         * 
         * Start Button : Move Wheels In Whatever Direction they're facing at 10% max
         * speed
         */
        public void controllerPresetTwo() {
                // Reset odom, reset encoders, go to 0 pos (turn), turn,
                // Reset pos = > zero gyro -> reset odom
                // turn (idk why )
                m_driverController.rightTrigger().onTrue(
                                new SequentialCommandGroup(
                                                new InstantCommand(
                                                                () -> m_swerveSub.zeroHeading(),
                                                                m_swerveSub),
                                                new InstantCommand(
                                                                () -> m_swerveSub.resetOdometry(new Pose2d()),
                                                                m_swerveSub)));

                // right movement
                m_driverController.b().and(m_driverController.leftTrigger()).whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.0,
                                                () -> (double) 0.4,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Right Movement"));

                // right rotation
                m_driverController.b().and(m_driverController.leftTrigger().negate()).whileTrue(
                                new TestSetPosCommand(m_swerveSub, Math.PI * 0.5));

                // left movement
                m_driverController.x().and(m_driverController.leftTrigger()).whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.0,
                                                () -> (double) -0.4,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Left Movement"));

                // left rotation
                m_driverController.x().and(m_driverController.leftTrigger().negate()).whileTrue(
                                new TestSetPosCommand(m_swerveSub, Math.PI * 1.5));

                // forwards movement
                m_driverController.y().and(m_driverController.leftTrigger()).whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) 0.4,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Forward Movement"));

                // forwards rotation
                m_driverController.y().and(m_driverController.leftTrigger().negate()).whileTrue(
                                new TestSetPosCommand(m_swerveSub, 0));

                // backwards movement
                m_driverController.a().and(m_driverController.leftTrigger()).whileTrue(
                                new TestSwerveJoystickCommand(
                                                m_swerveSub,
                                                () -> (double) -0.4,
                                                () -> (double) 0.0,
                                                () -> (double) 0.0,
                                                () -> (boolean) false,
                                                false,
                                                "Backward Movement"));

                // backwards rotation
                m_driverController.a().and(m_driverController.leftTrigger().negate()).whileTrue(
                                new TestSetPosCommand(m_swerveSub, Math.PI));

                // Rotate wheels clockwise
                m_driverController.rightBumper().whileTrue(
                                new TestTurningMotors(m_swerveSub, true));

                // Rotate wheels counterclockwise
                m_driverController.leftBumper().whileTrue(
                                new TestTurningMotors(m_swerveSub, false));

                m_driverController.start().whileTrue(
                                new TestDrivingMotors(m_swerveSub,
                                                0.10 * SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND));
        }
}
