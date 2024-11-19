package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    // Swereve Modules
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
            DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
            DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
            DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED,
            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS,
            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            DriveConstants.P_FRONT_LEFT_TURNING,
            DriveConstants.I_FRONT_LEFT_TURNING,
            DriveConstants.D_FRONT_LEFT_TURNING,
            DriveConstants.STATIC_FRONT_LEFT_TURNING);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
            DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
            DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
            DriveConstants.FRONT_RIGHT_TURNING_ENCODER_REVERSED,
            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS,
            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            DriveConstants.P_FRONT_RIGHT_TURNING,
            DriveConstants.I_FRONT_RIGHT_TURNING,
            DriveConstants.D_FRONT_RIGHT_TURNING,
            DriveConstants.STATIC_FRONT_RIGHT_TURNING);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
            DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT,
            DriveConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
            DriveConstants.BACK_LEFT_TURNING_ENCODER_REVERSED,
            DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
            DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS,
            DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            DriveConstants.P_BACK_LEFT_TURNING,
            DriveConstants.I_BACK_LEFT_TURNING,
            DriveConstants.D_BACK_LEFT_TURNING,
            DriveConstants.STATIC_BACK_LEFT_TURNING);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
            DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT,
            DriveConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
            DriveConstants.BACK_RIGHT_TURNING_ENCODER_REVERSED,
            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RADIANS,
            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED,
            DriveConstants.P_BACK_RIGHT_TURNING,
            DriveConstants.I_BACK_RIGHT_TURNING,
            DriveConstants.D_BACK_RIGHT_TURNING,
            DriveConstants.STATIC_BACK_RIGHT_TURNING);
    
    // Gyro
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d(0), getSwerveModulePosistion());
    // SwerveModuleState Publisher for AdvantageScope
    private final StructArrayPublisher<SwerveModuleState> realStatesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("RealStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> dummyPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("DummyStates", SwerveModuleState.struct).publish();
    private SwerveModuleState[] desiredModuleStates; 

    /**
     * Inits SwereveSubsystem
     */
    public SwerveSubsystem() {
        /*
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotRelativeSpeeds,
            this::setModuleStates,
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(AutoConstants.P_X_CONTROLLER, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AutoConstants.P_THETA_CONTROLLER, 0.0, 0.0), // Rotation PID constants
                    AutoConstants.MAX_SPEED_METER_PER_SECOND, // Max module speed, in m/s
                    DriveConstants.WHEEL_BASE/2, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
        */
        desiredModuleStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
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
     * Set the current rotation of the swerve as the new zero
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Converts and returns the heading in the perfered ranges of -180 to 180 
     * @return the heading based on WPILib's preference
     */
    public double getHeading() {
        // Despite what it looks like, it really does convert gyro from -180 to 180
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Returns rotation in degrees
     * @return A Rotation2d of the heading of the swerve in degrees
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Returns pose of swereve in meters
     * @return A Pose2d of the swereve in meters
     */
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Change the pose of the odometery, but keeps the swerve's rotation 
     * and wheel's encoder values
     * @param pose the Pose2d to change the odometery 
     */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePosistion(), pose);
    }

    public SwerveModulePosition[] getSwerveModulePosistion() {
        // return new SwerveModulePosition[]{frontRight.getPosition(), frontLeft.getPosition(), backRight.getPosition(), backLeft.getPosition()};
        return new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }
    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            frontLeft.getState(), 
            frontRight.getState(), 
            backLeft.getState(), 
            backRight.getState()
        };
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePosistion());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Heading", gyro.getRoll());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());

        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SwerveModuleState[] moudleStates = getSwerveModuleState();
        realStatesPublisher.set(moudleStates);
        desiredStatesPublisher.set(desiredModuleStates);
            dummyPublisher.set(
                new SwerveModuleState[]{
                    new SwerveModuleState(0.0, new Rotation2d(0)),
                    new SwerveModuleState(0.0, new Rotation2d(0)),
                    new SwerveModuleState(0.0, new Rotation2d(0)),
                    new SwerveModuleState(0.0, new Rotation2d(0))
                }
            );
            SmartDashboard.putNumberArray(
                "RealStateSmartDashboard"
                , new double[]{
                    moudleStates[0].angle.getRadians(),
                    moudleStates[0].speedMetersPerSecond,
                    moudleStates[1].angle.getRadians(),
                    moudleStates[1].speedMetersPerSecond,
                    moudleStates[2].angle.getRadians(),
                    moudleStates[2].speedMetersPerSecond,
                    moudleStates[3].angle.getRadians(),
                    moudleStates[3].speedMetersPerSecond,
                }
            );

            SmartDashboard.putNumberArray(
                "DesiredStateSmartDashboard"
                , new double[]{
                    desiredModuleStates[0].angle.getRadians(),
                    desiredModuleStates[0].speedMetersPerSecond,
                    desiredModuleStates[1].angle.getRadians(),
                    desiredModuleStates[1].speedMetersPerSecond,
                    desiredModuleStates[2].angle.getRadians(),
                    desiredModuleStates[2].speedMetersPerSecond,
                    desiredModuleStates[3].angle.getRadians(),
                    desiredModuleStates[3].speedMetersPerSecond,
                }
            );

            SmartDashboard.putNumberArray(
                "StateDummy",
                new double[]{
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0
                }
            );
            SmartDashboard.putNumberArray(
                "DebuggingRealStateSmartDashboard"
                , new double[]{
                    moudleStates[0].angle.getRadians(),
                    5,
                    moudleStates[1].angle.getRadians(),
                    5,
                    moudleStates[2].angle.getRadians(),
                    5,
                    moudleStates[3].angle.getRadians(),
                    5,
                }
            );

            SmartDashboard.putNumberArray(
                "DebuggingDesiredStateSmartDashboard"
                , new double[]{
                    desiredModuleStates[0].angle.getRadians(),
                    4,
                    desiredModuleStates[1].angle.getRadians(),
                    4,
                    desiredModuleStates[2].angle.getRadians(),
                    4,
                    desiredModuleStates[3].angle.getRadians(),
                    4,
                }
            );
        
        // SmartDashboard.putNumberArray(
        //     "StatesCopy", 
        // );
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
     * Normalize and sets the desired state / speed of each wheels
     * @param desiredStates the SwereveModuleState list of each motor, 
     * front left is index 0, front right is index 1, back left is index 2, 
     * back right is index 3
     * @param noStillMovement true to prevent wheels from rotating when there is no motion to avoid robot shifting
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean noStillMovement) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0], noStillMovement);
        frontRight.setDesiredState(desiredStates[1], noStillMovement);
        backLeft.setDesiredState(desiredStates[2], noStillMovement);
        backRight.setDesiredState(desiredStates[3], noStillMovement);
    }

    /**
     * Normalize and sets the desired state / speed of each wheels with wheel rotating only if there is movement
     * @param desiredStates the SwereveModuleState list of each motor, 
     * front left is index 0, front right is index 1, back left is index 2, 
     * back right is index 3
     * @param noStillMovement true to prevent wheels from rotating when there is no motion to avoid robot shifting
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        desiredModuleStates = desiredStates;
        frontLeft.setDesiredState(desiredStates[0], true);
        frontRight.setDesiredState(desiredStates[1], true);
        backLeft.setDesiredState(desiredStates[2], true);
        backRight.setDesiredState(desiredStates[3], true);
    }

    public void testTurnMotors(DoubleSupplier[] wheelPos, boolean forward) {
        double pos;
        double val = 10;
        if(forward)
            val *= 1;
        else
            val *= -1;
        frontLeft.testTurnMotors(wheelPos[0].getAsDouble() + val);
        frontRight.testTurnMotors(wheelPos[1].getAsDouble() + val);
        backLeft.testTurnMotors(wheelPos[2].getAsDouble() + val);
        backRight.testTurnMotors(wheelPos[3].getAsDouble() + val);
        // desiredModuleStates
        desiredModuleStates = new SwerveModuleState[]{
            new SwerveModuleState(5, new Rotation2d(wheelPos[0].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[1].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[2].getAsDouble() + val)),
            new SwerveModuleState(5, new Rotation2d(wheelPos[3].getAsDouble() + val))
        };
                
    }

    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[]{
            () -> frontLeft.getTurnPosition(),
            () -> frontRight.getTurnPosition(),
            () -> backLeft.getTurnPosition(),
            () -> backRight.getTurnPosition()
        };
    }

    public void testTurnMotors(boolean forward, double pos) {
        frontLeft.testTurnMotors(pos);
        frontRight.testTurnMotors(pos);
        backLeft.testTurnMotors(pos);
        backRight.testTurnMotors(pos);
    }

    /**
     * Normalize and sets the desired state / speed of each wheels
     * @param chassisSpeeds the ChassisSpeeds of the entire swerve
     */
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), true);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleState());
    }
}
