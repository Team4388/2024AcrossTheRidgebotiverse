package frc4388.utility;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * A button binding for two controllers, preferably an {@link frc4388.utility.controller.DeadbandedXboxController Xbox Controller} and {@link frc4388.utility.controller.VirtualController Virtual Xbox Controller}
 * @author Zachary Wilke
 */
public class DualJoystickButton extends Trigger {

    /**
     * Creates an Button binding on two controllers
     * @param joystickA A controller
     * @param joystickB A controller
     * @param buttonNumber The button to bind to
     */
    public DualJoystickButton(GenericHID joystickA, GenericHID joystickB, int buttonNumber) {
        super(() -> (joystickA.getRawButton(buttonNumber) || joystickB.getRawButton(buttonNumber)));
    }
}
