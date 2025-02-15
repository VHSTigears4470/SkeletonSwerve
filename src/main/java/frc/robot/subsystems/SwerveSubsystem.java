package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwervePhysicalConstants;
import frc.robot.Constants.SwerveTurnConstants;

public class SwerveSubsystem extends SubsystemBase {
    // Swereve Modules
    private final SwerveModule frontLeft = new SwerveModule(
            SwervePhysicalConstants.MotorLocation.FRONT_LEFT,
            SwerveDriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
            SwerveTurnConstants.FRONT_LEFT_TURN_MOTOR_PORT,
            SwerveTurnConstants.FRONT_LEFT_TURN_ABSOLUTE_ENCODER_PORT,
            SwerveDriveConstants.FRONT_LEFT_DRIVE_REVERSED,
            SwerveTurnConstants.FRONT_LEFT_TURN_REVERSED,
            SwerveTurnConstants.FRONT_LEFT_TURN_ABSOLUTE_ENCODER_REVERSED,
            SwerveTurnConstants.FRONT_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION,
            SwerveTurnConstants.P_FRONT_LEFT_TURN,
            SwerveTurnConstants.I_FRONT_LEFT_TURN,
            SwerveTurnConstants.D_FRONT_LEFT_TURN,
            SwerveTurnConstants.STATIC_FRONT_LEFT_TURN);

    private final SwerveModule frontRight = new SwerveModule(
            SwervePhysicalConstants.MotorLocation.FRONT_RIGHT,
            SwerveDriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
            SwerveTurnConstants.FRONT_RIGHT_TURN_MOTOR_PORT,
            SwerveTurnConstants.FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_PORT,
            SwerveDriveConstants.FRONT_RIGHT_DRIVE_REVERSED,
            SwerveTurnConstants.FRONT_RIGHT_TURN_REVERSED,
            SwerveTurnConstants.FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSED,
            SwerveTurnConstants.FRONT_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION,
            SwerveTurnConstants.P_FRONT_RIGHT_TURN,
            SwerveTurnConstants.I_FRONT_RIGHT_TURN,
            SwerveTurnConstants.D_FRONT_RIGHT_TURN,
            SwerveTurnConstants.STATIC_FRONT_RIGHT_TURN);

    private final SwerveModule backLeft = new SwerveModule(
            SwervePhysicalConstants.MotorLocation.BACK_LEFT,
            SwerveDriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
            SwerveTurnConstants.BACK_LEFT_TURN_MOTOR_PORT,
            SwerveTurnConstants.BACK_LEFT_TURN_ABSOLUTE_ENCODER_PORT,
            SwerveDriveConstants.BACK_LEFT_DRIVE_REVERSED,
            SwerveTurnConstants.BACK_LEFT_TURN_REVERSED,
            SwerveTurnConstants.BACK_LEFT_TURN_ABSOLUTE_ENCODER_REVERSED,
            SwerveTurnConstants.BACK_LEFT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION,
            SwerveTurnConstants.P_BACK_LEFT_TURN,
            SwerveTurnConstants.I_BACK_LEFT_TURN,
            SwerveTurnConstants.D_BACK_LEFT_TURN,
            SwerveTurnConstants.STATIC_BACK_LEFT_TURN);

    private final SwerveModule backRight = new SwerveModule(
            SwervePhysicalConstants.MotorLocation.BACK_RIGHT,
            SwerveDriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
            SwerveTurnConstants.BACK_RIGHT_TURN_MOTOR_PORT,
            SwerveTurnConstants.BACK_RIGHT_TURN_ABSOLUTE_ENCODER_PORT,
            SwerveDriveConstants.BACK_RIGHT_DRIVE_REVERSED,
            SwerveTurnConstants.BACK_RIGHT_TURN_REVERSED,
            SwerveTurnConstants.BACK_RIGHT_TURN_ABSOLUTE_ENCODER_REVERSED,
            SwerveTurnConstants.BACK_RIGHT_TURN_ABSOLUTE_ENCODER_OFFSET_ROTATION,
            SwerveTurnConstants.P_BACK_RIGHT_TURN,
            SwerveTurnConstants.I_BACK_RIGHT_TURN,
            SwerveTurnConstants.D_BACK_RIGHT_TURN,
            SwerveTurnConstants.STATIC_BACK_RIGHT_TURN);

    // Gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwervePhysicalConstants.DRIVE_KINEMATICS,
            new Rotation2d(0), getSwerveModulePosistion());

    // List of all desired module states
    private SwerveModuleState[] desiredModuleStates;

    /**
     * Inits SwereveSubsystem
     */
    public SwerveSubsystem() {
        /*
         * AutoBuilder.configureHolonomic(
         * this::getPose,
         * this::resetOdometry,
         * this::getRobotRelativeSpeeds,
         * this::setModuleStates,
         * new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
         * likely live in your Constants class
         * new PIDConstants(AutoConstants.P_X_CONTROLLER, 0.0, 0.0), // Translation PID
         * constants
         * new PIDConstants(AutoConstants.P_THETA_CONTROLLER, 0.0, 0.0), // Rotation PID
         * constants
         * AutoConstants.MAX_SPEED_METER_PER_SECOND, // Max module speed, in m/s
         * DriveConstants.WHEEL_BASE/2, // Drive base radius in meters. Distance from
         * robot center to furthest module.
         * new ReplanningConfig() // Default path replanning config. See the API for the
         * options here
         * ),
         * () -> {
         * // Boolean supplier that controls when the path will be mirrored for the red
         * alliance
         * // This will flip the path being followed to the red side of the field.
         * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
         * 
         * var alliance = DriverStation.getAlliance();
         * if (alliance.isPresent()) {
         * return alliance.get() == DriverStation.Alliance.Red;
         * }
         * return false;
         * },
         * this // Reference to this subsystem to set requirements
         * );
         */
        // Inits module states
        desiredModuleStates = new SwerveModuleState[] { new SwerveModuleState(), new SwerveModuleState(),
                new SwerveModuleState(), new SwerveModuleState() };
        // Waits for gyro to boot up (takes around a second) then resets it heading
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                // Reset to avoid problems since the Gyro was modified
                resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
            } catch (Exception e) {
            }
        }).start();
    }

    /**
     * Converts and returns the heading in the perfered ranges of -180 to 180
     * 
     * @return the heading based on WPILib's preference
     */
    public double getHeading() {
        // Despite what it looks like, it really does convert gyro from -180 to 180
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Returns pose of swereve in meters
     * 
     * @return A Pose2d of the swereve in meters
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Gets a list of Swerve Module Posistion
     * 
     * @return A list of Swerve Module Positions from front left, front right, back
     *         left, and back right
     */
    public SwerveModulePosition[] getSwerveModulePosistion() {
        return new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
    }

    /**
     * Gets a list of Swerve Module States
     * 
     * @return A list of Swerve Module State from front left, front right, back
     *         left, back right
     */
    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwervePhysicalConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleState());
    }

    /**
     * Returns rotation in degrees
     * 
     * @return A Rotation2d of the heading of the swerve in degrees
     */
    public Rotation2d getRotation2d() {
        // Converts gyro's pose into a Rotation2d object
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * A list of the swerve module's latest turn angle
     * 
     * @return A supplier of the wheels latest position
     */
    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[] {
                () -> frontLeft.getTurnPosition(),
                () -> frontRight.getTurnPosition(),
                () -> backLeft.getTurnPosition(),
                () -> backRight.getTurnPosition()
        };
    }

    /**
     * Normalize and sets the desired state / speed of each wheels
     * 
     * @param chassisSpeeds the ChassisSpeeds of the entire swerve
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(SwervePhysicalConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), true);
    }

    /**
     * Normalize and sets the desired state / speed of each wheels with wheel
     * rotating only if there is movement
     * 
     * @param desiredStates the SwereveModuleState list of each motor,
     *                      front left is index 0, front right is index 1, back left
     *                      is index 2,
     *                      back right is index 3
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        desiredModuleStates = desiredStates;
        frontLeft.setDesiredState(desiredStates[0], true);
        frontRight.setDesiredState(desiredStates[1], true);
        backLeft.setDesiredState(desiredStates[2], true);
        backRight.setDesiredState(desiredStates[3], true);
    }

    /**
     * Normalize and sets the desired state / speed of each wheels
     * 
     * @param desiredStates   the SwereveModuleState list of each motor,
     *                        front left is index 0, front right is index 1, back
     *                        left is index 2,
     *                        back right is index 3
     * @param noStillMovement true to prevent wheels from rotating when there is no
     *                        motion to avoid robot shifting
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean noStillMovement) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0], noStillMovement);
        frontRight.setDesiredState(desiredStates[1], noStillMovement);
        backLeft.setDesiredState(desiredStates[2], noStillMovement);
        backRight.setDesiredState(desiredStates[3], noStillMovement);
    }

    /**
     * Stop all motors / Set speed to 0
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * A function designed to only drive the wheels
     * 
     * @param speed Velocity (meters per second) to turn the wheels
     */
    public void testDriveMotors(double speed) {
        // Limits speed to negative max speed to max speed inclusive
        speed = Math.signum(speed)
                * Math.min(Math.abs(speed), SwervePhysicalConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND);

        // Sets speed
        frontLeft.testDriveMotors(speed);
        frontRight.testDriveMotors(speed);
        backLeft.testDriveMotors(speed);
        backRight.testDriveMotors(speed);

        // updates the speed
        desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(speed, new Rotation2d(frontLeft.getTurnPosition())),
                new SwerveModuleState(speed, new Rotation2d(frontRight.getTurnPosition())),
                new SwerveModuleState(speed, new Rotation2d(backLeft.getTurnPosition())),
                new SwerveModuleState(speed, new Rotation2d(backRight.getTurnPosition()))
        };
    }

    /**
     * A function designed to only turn the wheels
     * 
     * @param pos Position to turn wheels to
     */
    public void testTurnMotors(double pos) {
        frontLeft.testTurnMotors(pos);
        frontRight.testTurnMotors(pos);
        backLeft.testTurnMotors(pos);
        backRight.testTurnMotors(pos);

        // updates the rotation
        desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(frontLeft.getDriveVelocity(), new Rotation2d(frontLeft.getTurnPosition() + pos)),
                new SwerveModuleState(frontRight.getDriveVelocity(),
                        new Rotation2d(frontRight.getTurnPosition() + pos)),
                new SwerveModuleState(backLeft.getDriveVelocity(), new Rotation2d(backLeft.getTurnPosition() + pos)),
                new SwerveModuleState(backRight.getDriveVelocity(), new Rotation2d(backRight.getTurnPosition() + pos))
        };
    }

    /**
     * A function designed to continously turn the wheels
     * 
     * @param wheelPos      A supplier of what the newest wheel position is
     * @param turnClockwise Whether the wheel should turn clockwise or not
     */
    public void testTurnMotors(DoubleSupplier[] wheelPos, boolean turnClockwise) {
        double val = 10 * (turnClockwise ? 1 : -1);

        frontLeft.testTurnMotors(wheelPos[0].getAsDouble() + val);
        frontRight.testTurnMotors(wheelPos[1].getAsDouble() + val);
        backLeft.testTurnMotors(wheelPos[2].getAsDouble() + val);
        backRight.testTurnMotors(wheelPos[3].getAsDouble() + val);

        // updates the rotation
        desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(5, new Rotation2d(wheelPos[0].getAsDouble() + val)),
                new SwerveModuleState(5, new Rotation2d(wheelPos[1].getAsDouble() + val)),
                new SwerveModuleState(5, new Rotation2d(wheelPos[2].getAsDouble() + val)),
                new SwerveModuleState(5, new Rotation2d(wheelPos[3].getAsDouble() + val))
        };
    }

    /**
     * Change the pose of the odometery, but keeps the swerve's rotation
     * and wheel's encoder values
     * 
     * @param pose the Pose2d to change the odometery
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePosistion(), pose);
    }

    /**
     * Set the current rotation of the swerve as the new zero
     */
    public void zeroHeading() {
        gyro.reset();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePosistion()); // Update Odom
        updateWheelPositions();

        if (DebuggingConstants.SWERVE_DRIVE_DEBUG) {
            // // Updates general robot data like pos
            updateSmartDashboard();

            // // Updates individual wheel values
            frontRight.updateSmartDashboard();
            frontLeft.updateSmartDashboard();
            backRight.updateSmartDashboard();
            backLeft.updateSmartDashboard();
        }
    }

    /**
     * Updates general robot data to SmartDasboard such as heading or pose
     */
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Heading", gyro.getRoll());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    /**
     * Upddates wheel positions to help advantage scope and other tools to visualize
     * robot
     */
    public void updateWheelPositions() {
        SwerveModuleState[] moudleStates = getSwerveModuleState(); // Current states of wheels

        // Physical / IRL values
        SmartDashboard.putNumberArray(
                "RealState", new double[] {
                        -moudleStates[0].angle.getRadians(),
                        moudleStates[0].speedMetersPerSecond,
                        -moudleStates[1].angle.getRadians(),
                        moudleStates[1].speedMetersPerSecond,
                        -moudleStates[2].angle.getRadians(),
                        moudleStates[2].speedMetersPerSecond,
                        -moudleStates[3].angle.getRadians(),
                        moudleStates[3].speedMetersPerSecond,
                });

        // Controller / Desired values
        SmartDashboard.putNumberArray(
                "DesiredState", new double[] {
                        -desiredModuleStates[0].angle.getRadians(),
                        desiredModuleStates[0].speedMetersPerSecond,
                        -desiredModuleStates[1].angle.getRadians(),
                        desiredModuleStates[1].speedMetersPerSecond,
                        -desiredModuleStates[2].angle.getRadians(),
                        desiredModuleStates[2].speedMetersPerSecond,
                        -desiredModuleStates[3].angle.getRadians(),
                        desiredModuleStates[3].speedMetersPerSecond,
                });

        // Phsyical / IRL Values but with Fake Speeds to help align and see deviation
        SmartDashboard.putNumberArray(
                "RealStateFakeSpeed", new double[] {
                        -moudleStates[0].angle.getRadians(),
                        5,
                        -moudleStates[1].angle.getRadians(),
                        5,
                        -moudleStates[2].angle.getRadians(),
                        5,
                        -moudleStates[3].angle.getRadians(),
                        5,
                });

        // Controller / Desired Values but with Fake Speeds to help align and see
        // deviation
        SmartDashboard.putNumberArray(
                "DesiredStateFakeSpeed", new double[] {
                        -desiredModuleStates[0].angle.getRadians(),
                        4,
                        -desiredModuleStates[1].angle.getRadians(),
                        4,
                        -desiredModuleStates[2].angle.getRadians(),
                        4,
                        -desiredModuleStates[3].angle.getRadians(),
                        4,
                });
    }
}
