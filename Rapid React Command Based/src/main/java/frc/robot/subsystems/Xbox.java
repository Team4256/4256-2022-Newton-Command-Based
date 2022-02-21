package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DPad.Direction;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Joystick;

public class Xbox extends Joystick{
    public Xbox(int port) {
		super(port);
	}

    public JoystickButton xButton = new JoystickButton(this, 3);
	public JoystickButton yButton = new JoystickButton(this, 4);
	public JoystickButton aButton = new JoystickButton(this, 1);
	public JoystickButton bButton = new JoystickButton(this, 2);
	public JoystickButton rightBumper = new JoystickButton(this, 6);
	public JoystickButton leftBumper = new JoystickButton(this, 5);
	public JoystickButton startButton = new JoystickButton(this, 8);
	public JoystickButton selectButton = new JoystickButton(this, 7);
	public JoystickButton leftStickButton = new JoystickButton(this, 9);
	public JoystickButton rightStickButton = new JoystickButton(this, 10);
	public JoystickButton DPadUpButton = new JoystickButton(this, 11);
	public Button leftTriggerButton = new LeftTrigger(this);
	public Button rightTriggerButton = new RightTrigger(this);
    
	private int m_outputs;
	private short m_leftRumble;
	private short m_rightRumble;

	public DPad dPadUp = new DPad(this, Direction.UP);
	public DPad dPadDown = new DPad(this, Direction.DOWN);
	public DPad dPadLeft = new DPad(this, Direction.LEFT);
	public DPad dPadRight = new DPad(this, Direction.RIGHT);

	public double getTriggerTwist() {
		double leftTriggerValue = this.getRawAxis(2);
		double rightTriggerValue = -1 * this.getRawAxis(3);

		return leftTriggerValue + rightTriggerValue;

	}

	public double getLeftStickX() {
		return this.getRawAxis(0);
	}

	public double getLeftStickY() {
		return -this.getRawAxis(1);
	}

	public double getRightStickX() {
		return this.getRawAxis(4); // 4
	}

	public double getRightStickY() {
		return -this.getRawAxis(5);
	}

	public void setRumble(double leftValue, double rightValue) {
		setRumble(RumbleType.kLeftRumble, leftValue);
		setRumble(RumbleType.kRightRumble, rightValue);
	}

	public void setRumble(RumbleType type, double value) {
		if (value < 0) {
			value = 0;
		} else if (value > 1) {
			value = 1;
		}
		if (type == RumbleType.kLeftRumble) {
			m_leftRumble = (short) (value * 65535);
		} else {
			m_rightRumble = (short) (value * 65535);
		}
		HAL.setJoystickOutputs((byte) getPort(), m_outputs, m_leftRumble, m_rightRumble);
	}

}