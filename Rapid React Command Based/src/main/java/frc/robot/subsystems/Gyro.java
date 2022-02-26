package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Parameters;
public class Gyro extends AHRS {
	public final Compass compass;
	public static Gyro instance = null;
	
	private Gyro(final byte updateHz) {
	//		super(I2C.Port.kOnboard, updateHz);
		super(SPI.Port.kMXP, Parameters.GYRO_UPDATE_HZ);
		compass = new Compass();
	}

	public static synchronized Gyro getInstance() {
		if (instance == null) {
			instance = new Gyro(Parameters.GYRO_UPDATE_HZ);
		} 
		return instance;
	}
	
	/**
	 * Tares the gyro's compass
	 * @param tareAngle the new tare angle; can be positive or negative
	 * @param relativeReference if true, tares relative to the current tare rather than 0
	 * @see Compass#setTareAngle(double)
	 */
	// public void setTareAngle(double tareAngle, final boolean relativeReference) {
	// 	if (relativeReference) {tareAngle += compass.getTareAngle();}
	// 	compass.setTareAngle(tareAngle);
	// }
	
	/**
	 * Cleans up and returns gyro input, accounting for the tare angle
	 * There is a negative because the gyro returns clockwise positive.
	 * @return gyro heading in the range [0, 360)
	 */
	public double getCurrentAngle() {return -Compass.validate((double)getAngle());}
	
	public void setOffset(double offset) {
		this.setAngleAdjustment(offset);
	}

	public double getOffset() {
		return this.getAngleAdjustment();
	}

	public void resetWithOffset() {
		double currentHeading = getCurrentAngle();
		setOffset(0);
		setOffset(currentHeading);
		reset();
	}

	public double getAngleWithOffset() {
		return getCurrentAngle() + getOffset();
	}
	/**
	 * Uses <code>compass.legalPath(start, end)</code> to find the most efficient arc from <code>getCurrentAngle()</code> to target
	 * @param target angle, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of current, negative otherwise)
	 * @see Compass#legalPath(current, target)
	 */
	public double pathTo(final double target) {return compass.legalPath(getCurrentAngle(), target);}
	
	/**
	 * @return magnitude of the acceleration vector
	 */
	public double netAcceleration() {
		final double xx_yy = (double)(getWorldLinearAccelX()*getWorldLinearAccelX() + getWorldLinearAccelY()*getWorldLinearAccelY());
		return Math.sqrt(xx_yy + (double)(getWorldLinearAccelZ()*getWorldLinearAccelZ()));
	}
}
