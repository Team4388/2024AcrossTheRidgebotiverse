package frc4388.utility.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class VirtualController extends GenericHID {
    private short m_buttonStates = 0; 
    private short m_buttonStatesLastFrame = 0; 
    private double[] m_axes = new double[6];
    private short[] m_pov = new short[1];

    public VirtualController(int port) {
        super(port);
    }

    public void setFrame(double[] axes, short buttonFlags, short[] pov) {
        m_axes = axes;
        setOutputs(buttonFlags);
        m_pov = pov;
    }

    public void zeroControls() {
        m_axes = new double[6];
        m_buttonStates = 0;
        m_buttonStatesLastFrame = 0;
        m_pov = new short[1];
    }

    public static boolean getFlag(int value, int index) {
        return ((value & 1 << index) != 0);
    }
    
    @Override
    public boolean getRawButton(int button) { // man why are buttons indexed at 1.
        return getFlag(m_buttonStates, button - 1);
    }

    @Override
    public boolean getRawButtonPressed(int button) {  
        return (!getFlag(m_buttonStatesLastFrame, button - 1) && getRawButton(button));
    }
    
    @Override
    public boolean getRawButtonReleased(int button) {
        return (getFlag(m_buttonStatesLastFrame, button - 1) && !getRawButton(button));
    }

    @Override
    public double getRawAxis(int axis) {
        return m_axes[axis];
    }
    
    @Override
    public int getPOV(int pov) {
        return m_pov[pov];
    }

    @Override
    public int getAxisCount() {
        return m_axes.length;
    }

    @Override
    public int getPOVCount() {
        return m_pov.length;
    }

    @Override
    public int getButtonCount() {
        return 10;
    }

    @Override
    public boolean isConnected() {
        return true;
    }
    
    @Override
    public HIDType getType() {
        return HIDType.kXInputGamepad;
    }

    @Override
    public String getName() {
        return "Virtual Controller";
    }

    @Override
    public int getAxisType(int axis) {
        return 1; /* ! Warning, does not return accurate data.
		     Hopefully this isn't a problem            */
    }

    @Override
    public void setOutput(int outputNumber, boolean value) {
        // do not use
        //m_buttonStatesLastFrame[outputNumber - 1] = m_buttonStates[outputNumber - 1];
        //m_buttonStates[outputNumber - 1] = value;
    }

    @Override
    public void setOutputs(int value) {
        m_buttonStatesLastFrame = m_buttonStates;
        m_buttonStates = (short) value;
    }

    @Override
    public void setRumble(RumbleType type, double value) {
        System.out.println("Why are you Setting rumble on an virtual controller?");
    }
}
