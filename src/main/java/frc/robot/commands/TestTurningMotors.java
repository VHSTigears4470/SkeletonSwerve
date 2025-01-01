package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TestTurningMotors extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final boolean moveClockwise;
    private final DoubleSupplier[] wheelRotationSupplier;

    /**
     * Contructs a Command to continously rotate only the wheels
     * @param swerveSubsystem subsystem that controls the swerve
     * @param moveClockwise whether or not the robot wheels should move clockwise
     */
    public TestTurningMotors(SwerveSubsystem swerveSubsystem, boolean moveClockwise) {
        this.swerveSubsystem = swerveSubsystem;
        this.moveClockwise = moveClockwise;
        wheelRotationSupplier = swerveSubsystem.getWheelRotationSupplier();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Mode", "Turning Motors"); // Helps understand which command swerve drive is using
    }

    @Override
    public void execute() {
        swerveSubsystem.testTurnMotors(wheelRotationSupplier, moveClockwise);
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
