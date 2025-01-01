package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwervePhysicalConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TestSwerveJoystickCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final boolean noStillMovement;
    private final String statusName;

    /**
     * Contructs a command to control the swerve via joystick, has more features to better debug movement
     * @param swerveSubsystem subsystem that controls the swerve
     * @param xSpdFunction x speed (left and right)
     * @param ySpdFunction y speed (forward and backwards)
     * @param turningSpdFunction turning speed (rotation) not angle control
     * @param fieldOrientedFunction field orientation (true for field orientated, false for robot orientated)
     * @param noStilMovement whether the robot can turn despite no directional movement (usually true to keep robot from shifting)
     * @param statusName the name of which command the robot is running
     */
    public TestSwerveJoystickCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, boolean noStillMovement, String statusName) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ACCELERATION_UNIT_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(SwervePhysicalConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNIT_PER_SECOND);
        this.noStillMovement = noStillMovement;
        this.statusName = statusName;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", statusName); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = IOConstants.DRIVER_X_AXIS_INVERTED * xSpdFunction.get();
        double ySpeed = IOConstants.DRIVER_Y_AXIS_INVERTED * ySpdFunction.get();
        double turningSpeed = IOConstants.DRIVER_ROT_AXIS_INVERTED * turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > IOConstants.DRIVER_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > IOConstants.DRIVER_DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > IOConstants.DRIVER_DEADBAND ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * SwervePhysicalConstants.TELE_DRIVE_MAX_SPEED_METER_PER_SECOND;
        ySpeed = yLimiter.calculate(ySpeed) * SwervePhysicalConstants.TELE_DRIVE_MAX_SPEED_METER_PER_SECOND;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * SwervePhysicalConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIAN_PER_SECOND;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwervePhysicalConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates, noStillMovement);

        // Debuging
        if(DebuggingConstants.SWERVE_DRIVE_DEBUG) {
            SmartDashboard.putString("Joystick", "X : " + xSpdFunction.get() + "Y : " + ySpdFunction.get() + " Theta : " + turningSpdFunction.get());
            SmartDashboard.putString("ChassisSpeeds", "X : " + chassisSpeeds.vxMetersPerSecond + "Y : " + chassisSpeeds.vyMetersPerSecond + " Theta : " + chassisSpeeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
