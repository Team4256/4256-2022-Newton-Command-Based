package frc.robot.subsystems.Utility;//COMPLETE 2016

public class Compass {
		
	/**
	 * Wraps values around a circle
	 * 
	 * @param angle degrees; can be positive, negative, huge, or tiny
	 * @return the corresponding angle in the range [0, 360)
	 */
	public static double validate(final double angle) {
		final double temp = 360 - (Math.abs(angle) % 360);
		if (angle < 0)
			return (temp < 360) ? temp : 0;
		else
			return (angle % 360 < 360) ? angle % 360 : 0;
	}

	/**
	 * Finds the measure of the minor arc between two points on a circle
	 * 
	 * @param start the first point, designated in degrees
	 * @param end   the second point, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of start,
	 *         negative if it's counterclockwise of start)
	 */
	public static double path(final double start, final double end) {
		return Math.IEEEremainder(end - start, 360.0);
	}

	/**
	 * Calls <code>validate(angle)</code>. If the result is inside the protected
	 * zone, this method pushes it out to the nearest zone border.
	 * 
	 * @param angle degrees; can be positive, negative, huge, or tiny
	 * @return the corresponding angle in the range [0, 360) \ (zone start, zone
	 *         end)<br>
	 *         this denotes angles in [0, 360) but outside (zone start, zone end)
	 * @see #validate(angle)
	 */
	public  static double legalize(double angle) {
	
		return validate(angle);
	}
	
	
	
	/**
	 * This function finds the shortest legal path from the start angle to the end
	 * angle and returns the size of that path in degrees. Positive means clockwise
	 * and negative means counter-clockwise.
	 **/
	/**
	 * Uses <code>path(start, end)</code> to find the smallest arc between start and
	 * <code>legalize(end)</code> that doesn't intersect the protected zone. If
	 * start was in that zone, <code>borderPath(start)</code> is added to the result
	 * 
	 * @param start the first point, designated in degrees
	 * @param end   the second point, designated in degrees
	 * @return arc measure in degrees (positive if the arc is clockwise of start,
	 *         negative if it's counterclockwise of start)
	 * @see #path(start, end)
	 * @see #legalize(angle)
	 * @see #borderPath(angle)
	 */
	public static double legalPath(final double start, final double end) {
		final double start_legal = legalize(start);
		final double path_escape = 0;
		double path_main = path(start_legal, legalize(end));
		
		return path_main + path_escape;
	}

	/**
	 * Calculates the standard deviation of an array of angles (can handle 360->0
	 * boundary condition)
	 * 
	 * @param angles an array of angles in degrees
	 * @return the standard deviation in degrees
	 */
	public static double stdd(final double[] angles) {
		double sin = 0.0, cos = 0.0;
		for (double angle : angles) {
			sin += Math.sin(Math.toRadians(angle));
			cos += Math.cos(Math.toRadians(angle));
		}

		sin /= angles.length;
		cos /= angles.length;
		final double stdd = Math.sqrt(-Math.log(sin * sin + cos * cos));

		return Math.toDegrees(stdd);
	}

	/**
	 * @param x x coordinate
	 * @param y y coordinate
	 * @return the angle between the Y axis and (x, y) measured in degrees
	 */
	public static double convertToAngle(final double x, final double y) {
		return Math.toDegrees(Math.atan2(x, -y));
	}
}