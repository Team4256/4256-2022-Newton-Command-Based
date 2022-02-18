package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 * @author mwtidd
 */
public class LeftTrigger extends Button {
	private final Joystick joystick;

	public LeftTrigger(Joystick joystick) {
		this.joystick = joystick;

	}

	public boolean get() {
		return joystick.getRawAxis(2) > .25;
	}
}