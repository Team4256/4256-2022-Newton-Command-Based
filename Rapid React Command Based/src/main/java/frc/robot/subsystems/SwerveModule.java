package frc.robot.subsystems;

import java.time.chrono.ThaiBuddhistChronology;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Parameters;

public final class SwerveModule {
	public static final double ROTATOR_GEAR_RATIO = 1.0;
	public static final double TRACTION_GEAR_RATIO = 52.0 / 9.0;// updated 2019
	public static final double TRACTION_WHEEL_CIRCUMFERENCE = 4.0 * Math.PI;// inches
	private final TractionControl driveMotor;
	private final RotationControl turningMotor;
	private double decapitated = 1.0;
	private double tractionDeltaPathLength = 0.0;
	private double tractionPreviousPathLength = 0.0;
	public final PIDController turningPidController;
	public String moduleName;
	

	// This constructor is intended for use with the module which has an encoder on
	// the traction motor.

	public SwerveModule(int driveMotorId, int turningMotorId, int absoluteEncoderId, String name) {

		moduleName = name;
        driveMotor = new TractionControl(driveMotorId);
        turningMotor = new RotationControl(turningMotorId, absoluteEncoderId);
		turningPidController = new PIDController(2, 0, 0);
        turningPidController.enableContinuousInput(-180, 180);

        driveMotor.resetEncoder();
		turningMotor.resetEncoder();
    }

	public double getMPS() {
		return driveMotor.getRPS() * Parameters.RPS_TO_METERS_PER_SECOND;
	}

	public double getAngle() {
		return turningMotor.getCurrentAngle();
	}

	public RotationControl getTurningMotor() {
		return turningMotor;
	}

	public TractionControl getDriveMotor() {
		return driveMotor;
	}

	public SwerveModuleState getState() {
        return new SwerveModuleState(getMPS(), new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < .5
		) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
		
        driveMotor.set(state.speedMetersPerSecond / 3.83);
        turningMotor.SetAngle(turningPidController.calculate(getAngle(), state.angle.getRadians()));
		SmartDashboard.putNumber("Swerve[" + moduleName + "] angle", getAngle());
		SmartDashboard.putString("Swerve[" + moduleName + "] state", state.toString());
		
    }
	
	public void driveToDirection(double direction) {
		driveMotor.set(.5);
		turningMotor.SetAngle(direction);
	}

    public void stop() {
        driveMotor.set(0);
		turningMotor.SetAngle(0);
    }
   
}