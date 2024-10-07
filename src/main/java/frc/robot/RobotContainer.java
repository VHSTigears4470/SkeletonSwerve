// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TestSetPosCommand;
import frc.robot.commands.TestSwerveJoystickCommand;
import frc.robot.commands.TestTurningMotors;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem m_swerveSub = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveSub
    .setDefaultCommand(new SwerveJoystickCommand(
      m_swerveSub, 
      () -> m_driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
      () -> m_driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
      () -> m_driverController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
      // When pressed, changes to robot orientated
      //() -> !m_driverController.button(OIConstants.DRIVER_FIELD_ORIENTED_BUTTON_IDX).getAsBoolean()
      () -> true
      //() -> true
    ));
// () -> !m_driverController.a().getAsBoolean()
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        // m_driverController.x().onTrue(new SwerveJoystickCommand(
        //         m_swerveSub, 
        //         () -> m_driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
        //         () -> m_driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
        //         () -> m_driverController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
        //         // When pressed, changes to robot orientated
        //         () -> !m_driverController.button(OIConstants.DRIVER_FIELD_ORIENTED_BUTTON_IDX).getAsBoolean()
        // ));
        int preset = 3;

        if(preset == 0) {
                // Reset Odom to center
                m_driverController.b().onTrue(
                        new InstantCommand(
                                () -> m_swerveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))), 
                                m_swerveSub
                        )
                );

                // Stop Button
                m_driverController.a().onTrue(
                        new InstantCommand(
                                () -> m_swerveSub.stopModules(), 
                                m_swerveSub
                        )
                );

                // Straighten Wheels (hopefully)
                m_driverController.x().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Wheels Straight"
                        )
                );


                // drive with Robot Orientation
                m_driverController.rightTrigger().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> m_driverController.getRawAxis(OIConstants.DRIVER_Y_AXIS),
                                () -> m_driverController.getRawAxis(OIConstants.DRIVER_X_AXIS),
                                () -> m_driverController.getRawAxis(OIConstants.DRIVER_ROT_AXIS),
                                // When pressed, changes to robot orientated
                                () -> false,
                                false,
                                "Robot Orientated"
                        )
                );

                // drive forward only
                m_driverController.rightBumper().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.4,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Forward"
                        )
                );

                // rotate only 
                m_driverController.leftBumper().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (double) 0.4,
                                () -> (boolean) false,
                                false,
                                "Rotate"
                        )
                );

                // rotate only 
                m_driverController.leftTrigger().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (double) -0.4,
                                () -> (boolean) false,
                                false,
                                "Rotate"
                        )
                );
        } else if(preset == 1) {
                // Reset odom, reset encoders, go to 0 pos (turn), turn, 
                // Reset pos = > zero gyro -> reset odom
                // turn (idk why )
                m_driverController.rightTrigger().onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        ()-> m_swerveSub.zeroHeading(),
                                        m_swerveSub
                                ),
                                new InstantCommand(
                                        ()-> m_swerveSub.resetOdometry(new Pose2d()),
                                        m_swerveSub
                                )
                        )
                );

                // Make Wheels Straight
                m_driverController.leftTrigger().onTrue(
                        new InstantCommand(
                                () -> m_swerveSub.testTurnMotors(true, 0),
                                m_swerveSub
                        )
                );

                // Rotate wheels forwards only
                m_driverController.rightBumper().whileTrue(
                        new TestTurningMotors(m_swerveSub, true)
                );

                // Rotate wheels backwards only
                m_driverController.leftBumper().whileTrue(
                        new TestTurningMotors(m_swerveSub, false)
                );

                // move in all four directions

                // right? 
                m_driverController.b().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.0,
                                () -> (double) 0.4,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Right"
                        )
                );

                // left? 
                m_driverController.x().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.0,
                                () -> (double) -0.4,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Left"
                        )
                );

                // forwards? 
                m_driverController.y().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) 0.4,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Forward"
                        )
                );

                // backwards? 
                m_driverController.a().whileTrue(
                        new TestSwerveJoystickCommand(
                                m_swerveSub,
                                () -> (double) -0.4,
                                () -> (double) 0.0,
                                () -> (double) 0.0,
                                () -> (boolean) false,
                                false,
                                "Backward"
                        )
                );
        } else if(preset == 2) {
                // Reset odom, reset encoders, go to 0 pos (turn), turn, 
                // Reset pos = > zero gyro -> reset odom
                // turn (idk why )
                m_driverController.a().onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        ()-> m_swerveSub.zeroHeading(),
                                        m_swerveSub
                                ),
                                new InstantCommand(
                                        ()-> m_swerveSub.resetOdometry(new Pose2d()),
                                        m_swerveSub
                                )
                        )
                );

                // Make Wheels Straight
                m_driverController.x().onTrue(
                        new InstantCommand(
                                () -> m_swerveSub.testTurnMotors(true, 0),
                                m_swerveSub
                        )
                );

                // Rotate wheels forwards only
                m_driverController.rightBumper().whileTrue(
                        new TestTurningMotors(m_swerveSub, true)
                );

                // Rotate wheels backwards only
                m_driverController.rightTrigger().whileTrue(
                        new TestTurningMotors(m_swerveSub, false)
                );
        } else if(preset == 3) {
                // Reset odom, reset encoders, go to 0 pos (turn), turn, 
                // Reset pos = > zero gyro -> reset odom
                // turn (idk why )
                m_driverController.rightTrigger().onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        ()-> m_swerveSub.zeroHeading(),
                                        m_swerveSub
                                ),
                                new InstantCommand(
                                        ()-> m_swerveSub.resetOdometry(new Pose2d()),
                                        m_swerveSub
                                )
                        )
                );

                // Make Wheels Straight
                m_driverController.y().whileTrue(
                        new TestSetPosCommand(m_swerveSub, true, 0)
                );
                // Make Wheels Straight
                m_driverController.x().whileTrue(
                        new TestSetPosCommand(m_swerveSub, true, 270)
                );
                // Make Wheels Straight
                m_driverController.b().whileTrue(
                        new TestSetPosCommand(m_swerveSub, true, 90)
                );
                // Make Wheels Straight
                m_driverController.a().whileTrue(
                        new TestSetPosCommand(m_swerveSub, true, 180)
                );

                // Rotate wheels forwards only
                m_driverController.leftTrigger().whileTrue(
                        new TestTurningMotors(m_swerveSub, true)
                );

                // Rotate wheels backwards only
                m_driverController.leftBumper().whileTrue(
                        new TestTurningMotors(m_swerveSub, false)
                );
        }


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 
        // Moves straight one meter -> then straight and left one meter -> then rotates 180 degrees 
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.MAX_SPEED_METER_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METER_PER_SECOND_SQUARED)
                        .setKinematics(DriveConstants.DRIVE_KINEMATICS);

        // 2. Generate trajectory
        Trajectory trajectory = 
                // TrajectoryGenerator.generateTrajectory(
                //         new Pose2d(0, 0, new Rotation2d(0)),
                //         List.of(
                //                 new Translation2d(-1, 0)
                //         ),
                //         new Pose2d(-1, 0, Rotation2d.fromDegrees(180)), 
                //         trajectoryConfig
                // );
                TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_swerveSub::getPose,
                DriveConstants.DRIVE_KINEMATICS,
                xController,
                yController,
                thetaController,
                m_swerveSub::setModuleStates,
                m_swerveSub);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_swerveSub.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_swerveSub.stopModules()));
  }
}
