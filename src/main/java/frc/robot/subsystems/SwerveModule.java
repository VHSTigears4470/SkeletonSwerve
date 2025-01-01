package frc.robot.subsystems;

// Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwervePhysicalConstants;
import frc.robot.Constants.SwervePhysicalConstants.MotorLocation;



public class SwerveModule {
    // Motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    // Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    // Feedforward
    private final SimpleMotorFeedforward driveFeedforward;

    // PID
    private final PIDController turnPidController;
    private final PIDController drivePidController;
    private double staticTurn; // Static voltage for the turning wheel

    // Absolute Encoder and their Settings
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    // Motor Ids (used mostly for debugging)
    private final int absoluteEncoderId;
    private final int turnMotorId;
    private final int driveMotorId;
    private final MotorLocation motorLocation;

    // Desired state of wheel (used mostly for debugging)
    private SwerveModuleState desiredState;

    /**
     * Initializes a SwerveModule for the SwerveSubsystem
     * @param driveMotorID Drive Motor's ID
     * @param turnMotorId Turn Motor's ID
     * @param absoluteEncoderId Absolute Motor's ID
     * @param driveMotorReversed If the drive motor is reversed
     * @param turnMotorReversed If the turn motor is reversed
     * @param absoluteEncoderReversed If the encoder is reversed
     * @param absoluteEncoderOffset Offset of the absolute encoder to make the wheels "straight"
     * @param pTurn P value for PID of the turn wheel
     * @param iTurn I value for PID of the turn wheel
     * @param dTurn D value for PID of the turn wheel
     * @param staticTurn Static voltage constantly applied to the turn wheel (helps overcome friction)
     */
    public SwerveModule(
        MotorLocation motorLocation, int driveMotorId, int turnMotorId, int absoluteEncoderId,
        boolean driveMotorReversed, boolean turnMotorReversed, boolean absoluteEncoderReversed,
        double absoluteEncoderOffset, double pTurn, double iTurn, double dTurn, double staticTurn) {
        
        // Init variables
        this.motorLocation = motorLocation;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwervePhysicalConstants.DRIVE_ENCODER_ROTATION_TO_METER);
        driveEncoder.setVelocityConversionFactor(SwervePhysicalConstants.DRIVE_ENCODER_RPM_TO_METER_PER_SECOND);
        
        turnEncoder.setPositionConversionFactor(SwervePhysicalConstants.TURN_ENCODER_ROTATION_TO_RADIANS);
        turnEncoder.setVelocityConversionFactor(SwervePhysicalConstants.TURN_ENCODER_RPM_TO_METER_PER_SECOND);

        driveFeedforward = new SimpleMotorFeedforward(SwerveDriveConstants.FEEDFORWARD_S_DRIVE, SwerveDriveConstants.FEEDFORWARD_V_DRIVE, SwerveDriveConstants.FEEDFORWARD_A_DRIVE);
        drivePidController = new PIDController(SwerveDriveConstants.P_DRIVE, SwerveDriveConstants.I_DRIVE, SwerveDriveConstants.D_DRIVE);

        turnPidController = new PIDController(pTurn, iTurn, dTurn);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        this.absoluteEncoderId = absoluteEncoderId;
        this.turnMotorId = turnMotorId;
        this.driveMotorId = driveMotorId;
        
        resetEncoders();
        
        // Init Smartdashboard values used to modify PID controllers
        SmartDashboard.putData(motorLocation + " turn PID", turnPidController);
        SmartDashboard.putData(motorLocation + " drive PID", drivePidController);

        // Init Smartdashboard values used to modify Static Turn
        this.staticTurn = staticTurn;
        SmartDashboard.putNumber(motorLocation + " STATIC", this.staticTurn);
    }

    /**
     * Gets the absolute encoder value in radians with offset included
     * @return double of the absolute encoder with offset in radians
     */
    public double getAbsoluteEncoderRad() {
        // Gets raw value of absolute encoder (%)
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        
        // angle = absoluteEncoder.getVelocity().getValue() / RobotController.getCurrent5V();
        // Converts to radians
        angle *= 2.0 * Math.PI;
        
        // Apply Offset
        angle -= absoluteEncoderOffsetRad;
        
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    /**
     * Gets this module's desired swerve module state
     * @return last inputted desired swerve module state
     */
    public SwerveModuleState getDesiredSwerveModuleState() {
        return desiredState;
    }

    /**
     * Gets the encoder position of the drive motor in meters
     * @return double of the drive motor's encoder value convereted
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets this module's drive velocity in meters / second
     * @return double of velocity of the drive module converted
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets encoder values of drive and turn motors
     * @return SwereveModulePosition of this module's drive and turn motor's position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
    }

    /**
     * Gets the speed and angle of this module
     * @return SwerveModuleState of this moduel's drive and turn motor's position
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    /**
     * Gets the encoder position of the turn motor in radians
     * @return double of turn motor's encoder value converted
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**
     * Gets this module's turn velocity in meters / second
     * @return double of velocity of the turn module converted
     */
    public double getTurnVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Moves the module's turn and drive motors to the inputted state
     * @param state the SwereveModuleState this module should aim for
     * @param bypassStillMovement toggle on whether to prevent robot moving with very small inputs
     */
    public void setDesiredState(SwerveModuleState state, boolean bypassStillMovement) {
        // Prevents robot from shifting when still (mainly used to prevent robot from turning when the drive wheels are not moving)
        if(bypassStillMovement && Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // Finds optimal direction to rotate module (90 degrees turns at max)
        state = SwerveModuleState.optimize(state, getState().angle);
        
        // Sets drive motor speeds
        if(SwerveDriveConstants.IS_USING_PID_DRIVE) {
            // Calculates speed using PID
            driveMotor.set(driveFeedforward.calculate(state.speedMetersPerSecond) + drivePidController.calculate(state.speedMetersPerSecond));
        } else {
            // Calculates speed using max speed
            driveMotor.set(state.speedMetersPerSecond / SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        }

        // Sets turn motor speeds with PID and StaticTurn
        turnMotor.setVoltage(turnPidController.calculate(getTurnPosition(), state.angle.getRadians()) + staticTurn);
        
        // Updates desired state
        desiredState = state;
    }

    /**
     * Stops the drive and turn motor by setting their speed to 0
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Sets the speed of the motor
     * @param speed of motor in meters per second (m/s)
     */
    public void testDriveMotors(double speed) {
        turnMotor.set(SwervePhysicalConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
    }

    /**
     * Turns only the motor to a set angle
     * @param position is where the turn wheel should be rotated (in radians)
     */
    public void testTurnMotors(double position) {
        turnMotor.setVoltage(turnPidController.calculate(getTurnPosition(), position) + staticTurn);
    }

    /**
     * Sets the drive encoder to zero 
     * Syncs turn encoder with the absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Puts new SmartDashboards values onto the board and allows for modification of certain values
     */
    public void updateSmartDashboard() {
        // Position of Drive and Turn Motors
        SmartDashboard.putNumber(motorLocation + " driver encoder", driveEncoder.getPosition());
        SmartDashboard.putNumber(motorLocation + " turn encoder", turnEncoder.getPosition());
        
        // To change static voltage applied to the turn motor
        staticTurn = SmartDashboard.getNumber(motorLocation + " STATIC", 0);
        SmartDashboard.putNumber(motorLocation + " STATIC", staticTurn);
        staticTurn = SmartDashboard.getNumber(motorLocation + " STATIC", 0);

        // Prints both current and desired speeds of the drive wheels
        System.out.printf("21%s", (motorLocation + " : curr = "));
        System.out.printf("0.5%d", getDriveVelocity());
        System.out.print(" : desired = ");
        System.out.printf("0.5%d", getDesiredSwerveModuleState().speedMetersPerSecond);
        System.out.println();
    }
}
