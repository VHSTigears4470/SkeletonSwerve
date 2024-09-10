package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    // Motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    // Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    // PID
    private final PIDController turnPidController;
    // Absolute Encoder with Settings
    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final int absoluteEncoderId;

    /**
     * Initializes a SwerveModule for the SwerveSubsystem
     * @param driveMotorID Drive Motor's ID
     * @param turnMotorId Turn Motor's ID
     * @param driveMotorReversed If the drive motor is reversed
     * @param turnMotorReversed If the turn motor is reversed
     * @param absoluteEncoderId Absolute Motor's ID
     * @param absoluteEncoderOffset Offset of the absolute encoder to make the wheels "straight"
     * @param absoluteEncoderReversed If the encoder is reversed
     */
    public SwerveModule(int driveMotorID, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATION_TO_METER);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_RPM_TO_METER_PER_SECOND);
        turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_ROTATION_TO_RADIANS);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_RPM_TO_METER_PER_SECOND);

        turnPidController = new PIDController(ModuleConstants.P_TURN, ModuleConstants.I_TURN, ModuleConstants.D_TURN);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        this.absoluteEncoderId = absoluteEncoderId;
        // To Modify Values on Smartdashboard for PID, use the go to Test instead of TeleOperated
        SmartDashboard.putData("Swerve[" + absoluteEncoderId + "] PID", turnPidController);
    }

    /**
     * Gets the encoder position of the drive motor in meters
     * @return double of the drive motor's encoder value convereted
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the encoder position of the turn motor in radians
     * @return double of turn motor's encoder value converted
     */
    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    /**
     * Gets this module's drive velocity in meters / second
     * @return double of velocity of the drive module converted
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets this module's turn velocity in meters / second
     * @return double of velocity of the turn module converted
     */
    public double getTurnVelocity() {
        return driveEncoder.getVelocity();
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
     * Sets the drive encoder to zero and syncs turn encoder to 
     * the adjusted reading of the absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Gets the speed and angle of this module
     * @return SwerveModuleState of this moduel's drive and turn motor's position
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    /**
     * Gets encoder values of drive and turn motors
     * @return SwereveModulePosition of this module's drive and turn motor's position
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurnPosition()));
    }

    /**
     * Moves the module's turn and drive motors to the inputted state
     * @param state the SwereveModuleState this module should aim for
     */
    public void setDesiredState(SwerveModuleState state) {
        // Prevents only turning motion without movement (helps prevents robot from shifting when still)
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // Finds best route to rotate module (90 degrees turns at max)
        state = SwerveModuleState.optimize(state, getState().angle);
        // Sets motor speeds
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.PHYSICAL_MAX_SPEED_METER_PER_SECOND);
        turnMotor.set(turnPidController.calculate(getTurnPosition(), state.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoderId + "] state", state.toString());
        // SmartDashboard.putNumber("Swerve[" + absoluteEncoderId + "] absolute encoder", getAbsoluteEncoderRad());
        // SmartDashboard.putNumber("Swerve[" + absoluteEncoderId + "] driver encoder", driveEncoder.getPosition());
        // SmartDashboard.putNumber("Swerve[" + absoluteEncoderId + "] turn encoder", turnEncoder.getPosition());
    }

    /**
     * Stops the drive and turn motor by setting their speed to 0
     */
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}