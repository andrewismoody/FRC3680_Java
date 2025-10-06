package frc.robot.misc;

import java.util.Hashtable;
import java.util.Map;
import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class GameController {
    PS4Controller ps4Controller;
    XboxController xboxController;
    Joystick fsController;
    double thumbstickDeadZone = 0.2;

    public enum ControllerType {
        Xbox,
        PS4,
        FlightStick,
    }

    public enum ButtonName {
        TopButton,
        BottomButton,
        LeftButton,
        RightButton,
        LeftShoulderButton,
        RightShoulderButton,
        LeftTrigger,
        RightTrigger,
        RightThumbstickY,
        RightThumbstickX,
        LeftThumbstickY,
        LeftThumbstickX,
        POVAngle,
        POVUp,
        POVDown,
        POVLeft,
        POVRight,
        Start,
        Select,
        Logo,
        Any,
    }

    Hashtable<ButtonName, Supplier<Boolean>> BinaryButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Boolean>>();
    Hashtable<ButtonName, Supplier<Double>> ValueButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Double>>();
    Hashtable<ButtonName, Supplier<Integer>> POVButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Integer>>();

    Hashtable<ButtonName, ArrayList<Consumer<Boolean>>> BinaryButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Boolean>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Double>>> ValueButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Double>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Integer>>> POVButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Integer>>>();

    Hashtable<ButtonName, Boolean> ValueButtonInversion = new Hashtable<>();

    NetworkTable myTable;

    void RegisterBinaryButtonSupplier(ButtonName button, Supplier<Boolean> func) {
        BinaryButtonSuppliers.put(button, func);
    }

    void RegisterValueButtonSupplier(ButtonName button, Supplier<Double> func) {
        ValueButtonSuppliers.put(button, func);
    }

    void RegisterPOVButtonSupplier(ButtonName button, Supplier<Integer> func) {
        POVButtonSuppliers.put(button, func);
    }

    public void SetValueButtonInversion(ButtonName button, Boolean invert) {
        ValueButtonInversion.put(button, invert);
    }

    public void RegisterBinaryButtonConsumer(ButtonName button, Consumer<Boolean> func) {
        if (BinaryButtonConsumers.get(button) == null) {
            BinaryButtonConsumers.put(button, new ArrayList<Consumer<Boolean>>());
        }
        BinaryButtonConsumers.get(button).add(func);
    }

    public void RegisterValueButtonConsumer(ButtonName button, Consumer<Double> func) {
        if (ValueButtonConsumers.get(button) == null) {
            ValueButtonConsumers.put(button, new ArrayList<Consumer<Double>>());
        }
        ValueButtonConsumers.get(button).add(func);
    }

    public void RegisterPOVButtonConsumer(ButtonName button, Consumer<Integer> func) {
        if (POVButtonConsumers.get(button) == null) {
            POVButtonConsumers.put(button, new ArrayList<Consumer<Integer>>());
        }
        POVButtonConsumers.get(button).add(func);
    }

    public void GetBinaryButtonState(ButtonName button) {
        for (Consumer<Boolean> consumer : BinaryButtonConsumers.get(button))
            consumer.accept(GetBinaryButtonSupplier(button));
    }

    public void GetValueButtonValue(ButtonName button) {
        var value = GetValueButtonSupplier(button);
        if (ValueButtonInversion.get(button) != null && ValueButtonInversion.get(button))
            value *= -1;

        for (Consumer<Double> consumer : ValueButtonConsumers.get(button))
            consumer.accept(value);
    }

    public void GetPOVButtonValue(ButtonName button) {
        for (Consumer<Integer> consumer : POVButtonConsumers.get(button))
            consumer.accept(GetPOVButtonSupplier(button));
    }

    public void ProcessButtons() {
        for (ButtonName button : BinaryButtonConsumers.keySet()) {
            GetBinaryButtonState(button);
        }

        for (ButtonName button : ValueButtonConsumers.keySet()) {
            GetValueButtonValue(button);
        }

        for (ButtonName button : POVButtonConsumers.keySet()) {
            GetPOVButtonValue(button);
        }
    }

    public Boolean GetBinaryButtonSupplier(ButtonName button) {
        return BinaryButtonSuppliers.get(button).get();
    }

    public Double GetValueButtonSupplier(ButtonName button) {
        return ValueButtonSuppliers.get(button).get();
    }

    public Integer GetPOVButtonSupplier(ButtonName button) {
        return POVButtonSuppliers.get(button).get();
    }

    ControllerType Type;

    int Index;

    public GameController(int index, ControllerType type) {
        Index = index;
        Type = type;
        switch (Type) {
            case Xbox:
                xboxController = new XboxController(index);
                break;
            case PS4:
                ps4Controller = new PS4Controller(index);
                break;
            case FlightStick:
                fsController = new Joystick(index);
                break;
        }

        RegisterBinaryButtonSupplier(ButtonName.TopButton, this::getTopButton);
        RegisterBinaryButtonSupplier(ButtonName.BottomButton, this::getBottomButton);
        RegisterBinaryButtonSupplier(ButtonName.LeftButton, this::getLeftButton);
        RegisterBinaryButtonSupplier(ButtonName.RightButton, this::getRightButton);
        RegisterBinaryButtonSupplier(ButtonName.LeftShoulderButton, this::getLeftShoulderButton);
        RegisterBinaryButtonSupplier(ButtonName.RightShoulderButton, this::getRightShoulderButton);
        RegisterBinaryButtonSupplier(ButtonName.Start, this::getStartButton);
        RegisterBinaryButtonSupplier(ButtonName.Select, this::getSelectButton);
        RegisterBinaryButtonSupplier(ButtonName.Logo, this::getLogoButton);
        RegisterBinaryButtonSupplier(ButtonName.POVUp, this::getPOVUp);
        RegisterBinaryButtonSupplier(ButtonName.POVDown, this::getPOVDown);
        RegisterBinaryButtonSupplier(ButtonName.POVLeft, this::getPOVLeft);
        RegisterBinaryButtonSupplier(ButtonName.POVRight, this::getPOVRight);

        RegisterValueButtonSupplier(ButtonName.LeftTrigger, this::getLeftTriggerValue);
        RegisterValueButtonSupplier(ButtonName.RightTrigger, this::getRightTriggerValue);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickX, this::getLeftX);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickY, this::getLeftY);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickX, this::getRightX);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickY, this::getRightY);

        RegisterPOVButtonSupplier(ButtonName.POVAngle, this::getPOV);
        
        RegisterBinaryButtonSupplier(ButtonName.Any, this::getAnyButton);

        myTable = NetworkTableInstance.getDefault().getTable("GameController");
    }

   public boolean getAnyButton() {
       // any binary pressed
       for (Map.Entry<ButtonName, java.util.function.Supplier<Boolean>> e : BinaryButtonSuppliers.entrySet()) {
           if (e.getValue().get())
            return true;
       }
       // any axis beyond deadzone
       for (Map.Entry<ButtonName, java.util.function.Supplier<Double>> e : ValueButtonSuppliers.entrySet()) {
           double v = e.getValue().get();
           if (Math.abs(v) > thumbstickDeadZone)
            return true;
       }
       // any POV pressed (angle != -1)
       for (Map.Entry<ButtonName, java.util.function.Supplier<Integer>> e : POVButtonSuppliers.entrySet()) {
           if (e.getValue().get() != -1)
            return true;
       }

       return false;
   }

    boolean getLeftButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getXButton();
                break;
            case PS4:
                value = ps4Controller.getSquareButton();
                break;
            case FlightStick:
                value = fsController.getRawButton(0);
                break;
        }

        myTable.getEntry(ButtonName.LeftButton.toString()).setBoolean(value);

        return value;
    }

    boolean getTopButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getYButton();
                break;
            case PS4:
                value = ps4Controller.getTriangleButton();
                break;
            case FlightStick:
                value = fsController.getRawButton(1);
                break;
        }

        myTable.getEntry(ButtonName.TopButton.toString()).setBoolean(value);

        return value;
    }

    boolean getRightButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getBButton();
                break;
            case PS4:
                value = ps4Controller.getCircleButton();
                break;
            case FlightStick:
                value = fsController.getRawButton(2);
                break;
        }

        myTable.getEntry(ButtonName.RightButton.toString()).setBoolean(value);

        return value;
    }
 
    boolean getBottomButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getAButton();
                break;
            case PS4:
                value = ps4Controller.getCrossButton();
                break;
            case FlightStick:
                value = fsController.getRawButton(3);
                break;
        }

        myTable.getEntry(ButtonName.BottomButton.toString()).setBoolean(value);

        return value;
    }
 
    boolean getLeftShoulderButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getLeftBumper();
                break;
            case PS4:
                value = ps4Controller.getL1Button();
                break;
            case FlightStick:
                value = fsController.getRawButton(4);
                break;
        }

        myTable.getEntry(ButtonName.LeftShoulderButton.toString()).setBoolean(value);

        return value;
    }

    double getLeftTriggerValue() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getLeftTriggerAxis();
                break;
            case PS4:
                value = ps4Controller.getL2Button() ? 1.0 : 0.0;
                break;
            case FlightStick:
                value = fsController.getRawAxis(2);
                break;
        }

        myTable.getEntry(ButtonName.LeftTrigger.toString()).setDouble(value);

        return value;
    }
 
    boolean getRightShoulderButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getRightBumper();
                break;
            case PS4:
                value = ps4Controller.getR1Button();
                break;
            case FlightStick:
                value = fsController.getRawButton(5);
                break;
        }

        myTable.getEntry(ButtonName.RightShoulderButton.toString()).setBoolean(value);

        return value;
    }

    double getRightTriggerValue() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getRightTriggerAxis();
                break;
            case PS4:
                value = ps4Controller.getR2Button() ? 1.0 : 0.0;
                break;
            case FlightStick:
                value = fsController.getRawAxis(3);
                break;
        }

        myTable.getEntry(ButtonName.RightTrigger.toString()).setDouble(value);

        return value;
    }

    double getRightY() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getRightY();
                break;
            case PS4:
                value = ps4Controller.getRightY();
                break;
            case FlightStick:
                value = fsController.getRawAxis(0);
                break;
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        myTable.getEntry(ButtonName.RightThumbstickY.toString()).setDouble(value);

        return value;
    }

    double getRightX() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getRightX();
                break;
            case PS4:
                value = ps4Controller.getRightX();
                break;
            case FlightStick:
                value = fsController.getRawAxis(1);
                break;
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        myTable.getEntry(ButtonName.RightThumbstickX.toString()).setDouble(value);

        return value;
    }

    double getLeftY() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getLeftY();
                break;
            case PS4:
                value = ps4Controller.getLeftY();
                break;
            case FlightStick:
                value = fsController.getRawAxis(0);
                break;
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        myTable.getEntry(ButtonName.LeftThumbstickY.toString()).setDouble(value);

        return value;
    }

    double getLeftX() {
        var value = 0.0;

        switch (Type) {
            case Xbox:
                value = xboxController.getLeftX();
                 break;
           case PS4:
                value = ps4Controller.getLeftX();
                break;
            case FlightStick:
                value = fsController.getRawAxis(1);
                break;
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        myTable.getEntry(ButtonName.LeftThumbstickX.toString()).setDouble(value);

        return value;
    }

    int getPOV() {
        var value = 0;

        switch (Type) {
            case Xbox:
                value = xboxController.getPOV();
                break;
            case PS4:
                value = ps4Controller.getPOV();
                break;
            case FlightStick:
                value = fsController.getPOV();
                break;
        }

        myTable.getEntry(ButtonName.POVAngle.toString()).setInteger(value);

        return value;
    }

    boolean getPOVUp() {
        var value = false;

        int angle = getPOV();
        if (angle == 0) { // -1 means not pressed
            value = true;
        }

        myTable.getEntry(ButtonName.POVUp.toString()).setBoolean(value);

        return value;
    }
 
    boolean getPOVDown() {
        var value = false;
        int angle = getPOV();
        if (angle == 180) {  // -1 means not pressed
            value = true;
        }

        myTable.getEntry(ButtonName.POVDown.toString()).setBoolean(value);

        return value;
    }
 
    boolean getPOVRight() {
        var value = false;
        int angle = getPOV();
        if (angle == 90) {  // -1 means not pressed
            value = true;
        }

        myTable.getEntry(ButtonName.POVRight.toString()).setBoolean(value);

        return value;
    }
 
    boolean getPOVLeft() {
        var value = false;
        int angle = getPOV();
        if (angle == 270) {  // -1 means not pressed
            value = true;
        }

        myTable.getEntry(ButtonName.POVLeft.toString()).setBoolean(value);

        return value;
    }

    boolean getStartButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getStartButton();
                break;
            case PS4:
                value = ps4Controller.getShareButton();
                break;
             case FlightStick:
                value = fsController.getTop();
                break;
       }

        myTable.getEntry(ButtonName.Start.toString()).setBoolean(value);

        return value;
    }
 
    boolean getSelectButton() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getBackButton();
                break;
            case PS4:
                value = ps4Controller.getOptionsButton();
                break;
             case FlightStick:
                value = fsController.getTrigger();
                break;
        }

        myTable.getEntry(ButtonName.Select.toString()).setBoolean(value);

        return value;
    }
 
    boolean getLogoButton() {
        var value = false;
        
        switch (Type) {
            case Xbox:
                break;
            case PS4:
                value = ps4Controller.getPSButton();
                break;
             case FlightStick:
                break;
        }

        myTable.getEntry(ButtonName.Logo.toString()).setBoolean(value);

        return value;
    }
    
    public static GameController Initialize() {
        GameController m_controller = null;

        JoystickIndexLoop: for (int j = 0; j < 6; j++) {
            System.out.printf("Checking for joystick on port %d\n", j);

            if (DriverStation.isJoystickConnected(j)) {
                var jtype = DriverStation.getJoystickType(j);
                System.out.printf("Joystick is connected on port %d; found type %d\n", j, jtype);
                switch (GenericHID.HIDType.of(jtype)) {
                case kXInputFlightStick, kHIDFlight:
                    System.out.printf("Joystick on port %d is a FlightStick\n", j);
                    m_controller = new GameController(j, ControllerType.FlightStick);
                    break JoystickIndexLoop;
                default:
                case kXInputGamepad, kHIDGamepad:
                    if (DriverStation.getJoystickIsXbox(j)) {
                    System.out.printf("Joystick on port %d is an Xbox controller\n", j);
                    m_controller = new GameController(j, ControllerType.Xbox);
                    break JoystickIndexLoop;
                    } else {
                    System.out.printf("Joystick on port %d is not an Xbox controller, assuming PS4\n", j);
                    m_controller = new GameController(j, ControllerType.PS4);
                    break JoystickIndexLoop;
                    }
                }
            } else {
                System.out.printf("Joystick is not connected on port %d\n", j);
            }
        }

        if (m_controller == null) {
            System.out.println("no joysticks detected!  Assuming XBox Controller on port 0");
            m_controller = new GameController(0, ControllerType.Xbox);
        }

        return m_controller;
    }
}
