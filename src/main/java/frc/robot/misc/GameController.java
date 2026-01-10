package frc.robot.misc;

import java.util.Hashtable;
import java.util.Map;
import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class GameController {
    PS4Controller ps4Controller;
    XboxController xboxController;
    Joystick fsController;
    Joystick switch2ProController;
    double thumbstickDeadZone = 0.2;

    public enum ControllerType {
        Xbox,
        PS4,
        FlightStick,
        Switch2Pro,
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
        Capture,
        Chat,
        GripLeft,
        GripRight,
    }

    Hashtable<ButtonName, Supplier<Boolean>> BinaryButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Boolean>>();
    Hashtable<ButtonName, Supplier<Double>> ValueButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Double>>();
    Hashtable<ButtonName, Supplier<Integer>> POVButtonSuppliers = new Hashtable<GameController.ButtonName,Supplier<Integer>>();

    Hashtable<ButtonName, ArrayList<Consumer<Boolean>>> BinaryButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Boolean>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Double>>> ValueButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Double>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Integer>>> POVButtonConsumers = new Hashtable<GameController.ButtonName,ArrayList<Consumer<Integer>>>();

    Hashtable<ButtonName, Boolean> ValueButtonInversion = new Hashtable<>();

    NetworkTable myTable;
    NetworkTableEntry LeftButtonNTEntry;
    NetworkTableEntry TopButtonNTEntry;
    NetworkTableEntry RightButtonNTEntry;
    NetworkTableEntry BottomButtonNTEntry;
    NetworkTableEntry LeftShoulderButtonNTEntry;
    NetworkTableEntry RightShoulderButtonNTEntry;
    NetworkTableEntry LeftTriggerNTEntry;
    NetworkTableEntry RightTriggerNTEntry;
    NetworkTableEntry LeftThumbstickYNTEntry;
    NetworkTableEntry LeftThumbstickXNTEntry;
    NetworkTableEntry RightThumbstickYNTEntry;
    NetworkTableEntry RightThumbstickXNTEntry;
    NetworkTableEntry POVAngleNTEntry;
    NetworkTableEntry POVUpNTEntry;
    NetworkTableEntry POVDownNTEntry;
    NetworkTableEntry POVLeftNTEntry;
    NetworkTableEntry POVRightNTEntry;
    NetworkTableEntry StartNTEntry;
    NetworkTableEntry SelectNTEntry;
    NetworkTableEntry LogoNTEntry;
    NetworkTableEntry AnyButtonNTEntry;
    NetworkTableEntry CaptureNTEntry;
    NetworkTableEntry ChatNTEntry;
    NetworkTableEntry GripLeftNTEntry;
    NetworkTableEntry GripRightNTEntry;
    NetworkTableEntry JoystickTypeNTEntry;

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
        if (ValueButtonInversion.getOrDefault(button, false))
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
            case Switch2Pro:
                switch2ProController = new Joystick(index);
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
        RegisterBinaryButtonSupplier(ButtonName.LeftTrigger, this::getLeftTriggerBinaryValue);
        RegisterBinaryButtonSupplier(ButtonName.RightTrigger, this::getRightTriggerBinaryValue);
        RegisterBinaryButtonSupplier(ButtonName.Capture, this::getCaptureButton);
        RegisterBinaryButtonSupplier(ButtonName.Chat, this::getChatButton);
        RegisterBinaryButtonSupplier(ButtonName.GripLeft, this::getGripLeftButton);
        RegisterBinaryButtonSupplier(ButtonName.GripRight, this::getGripRightButton); 

        RegisterValueButtonSupplier(ButtonName.LeftTrigger, this::getLeftTriggerValue);
        RegisterValueButtonSupplier(ButtonName.RightTrigger, this::getRightTriggerValue);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickX, this::getLeftX);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickY, this::getLeftY);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickX, this::getRightX);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickY, this::getRightY);

        RegisterPOVButtonSupplier(ButtonName.POVAngle, this::getPOV);
        
        RegisterBinaryButtonSupplier(ButtonName.Any, this::getAnyButton);

        myTable = NetworkTableInstance.getDefault().getTable("GameController");
        LeftButtonNTEntry = myTable.getEntry(ButtonName.LeftButton.toString());
        TopButtonNTEntry = myTable.getEntry(ButtonName.TopButton.toString());
        RightButtonNTEntry = myTable.getEntry(ButtonName.RightButton.toString());
        BottomButtonNTEntry = myTable.getEntry(ButtonName.BottomButton.toString());
        LeftShoulderButtonNTEntry = myTable.getEntry(ButtonName.LeftShoulderButton.toString());
        RightShoulderButtonNTEntry = myTable.getEntry(ButtonName.RightShoulderButton.toString());
        LeftTriggerNTEntry = myTable.getEntry(ButtonName.LeftTrigger.toString());
        RightTriggerNTEntry = myTable.getEntry(ButtonName.RightTrigger.toString());
        LeftThumbstickYNTEntry = myTable.getEntry(ButtonName.LeftThumbstickY.toString());
        LeftThumbstickXNTEntry = myTable.getEntry(ButtonName.LeftThumbstickX.toString());
        RightThumbstickYNTEntry = myTable.getEntry(ButtonName.RightThumbstickY.toString());
        RightThumbstickXNTEntry = myTable.getEntry(ButtonName.RightThumbstickX.toString());
        POVAngleNTEntry = myTable.getEntry(ButtonName.POVAngle.toString());
        POVUpNTEntry = myTable.getEntry(ButtonName.POVUp.toString());
        POVDownNTEntry = myTable.getEntry(ButtonName.POVDown.toString());
        POVLeftNTEntry = myTable.getEntry(ButtonName.POVLeft.toString());
        POVRightNTEntry = myTable.getEntry(ButtonName.POVRight.toString());
        StartNTEntry = myTable.getEntry(ButtonName.Start.toString());
        SelectNTEntry = myTable.getEntry(ButtonName.Select.toString());
        LogoNTEntry = myTable.getEntry(ButtonName.Logo.toString());
        AnyButtonNTEntry = myTable.getEntry(ButtonName.Any.toString());
        CaptureNTEntry = myTable.getEntry(ButtonName.Capture.toString());
        ChatNTEntry = myTable.getEntry(ButtonName.Chat.toString());
        GripLeftNTEntry = myTable.getEntry(ButtonName.GripLeft.toString());
        GripRightNTEntry = myTable.getEntry(ButtonName.GripRight.toString());
        JoystickTypeNTEntry = myTable.getEntry("JoystickType");

        JoystickTypeNTEntry.setString(Type.toString());
    }

    // TODO 1: this doesn't seem to interrupt motion (especially seektag) -make sure this doesn't break teleop after auto
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

    public void ClearAllRegistrations() {
        BinaryButtonConsumers.clear();
        ValueButtonConsumers.clear();
        POVButtonConsumers.clear();
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
            case Switch2Pro:
                value = switch2ProController.getRawButton(3);
                break;
        }

        LeftButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(4);
                break;

        }

        TopButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(2);
                break;
        }

        RightButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(1);
                break;
        }

        BottomButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(13);
                break;
        }

        LeftShoulderButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(14) ? 1.0 : 0.0;
                break;
        }

        LeftTriggerNTEntry.setDouble(value);

        return value;
    }
 
    boolean getLeftTriggerBinaryValue() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getLeftTriggerAxis() > 0.1 ? true : false;
                break;
            case PS4:
                value = ps4Controller.getL2Button();
                break;
            case FlightStick:
                value = fsController.getRawAxis(2)> 0.1 ? true : false;
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(14);
                break;
        }

        LeftTriggerNTEntry.setDouble(value == true ? 1.0 : 0.0);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(5);
                break;
        }

        RightShoulderButtonNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(6) ? 1.0 : 0.0;
                break;
        }

        RightTriggerNTEntry.setDouble(value);

        return value;
    }
 
    boolean getRightTriggerBinaryValue() {
        var value = false;

        switch (Type) {
            case Xbox:
                value = xboxController.getRightTriggerAxis() > 0.1 ? true : false;
                break;
            case PS4:
                value = ps4Controller.getR2Button();
                break;
            case FlightStick:
                value = fsController.getRawAxis(3)> 0.1 ? true : false;
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(6);
                break;
        }

        RightTriggerNTEntry.setDouble(value == true ? 1.0 : 0.0);

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
            case Switch2Pro:
                value = -switch2ProController.getRawAxis(3);
                break;
            }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        RightThumbstickYNTEntry.setDouble(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawAxis(2);
                break;
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        RightThumbstickXNTEntry.setDouble(value);

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
            case Switch2Pro:
                value = -switch2ProController.getRawAxis(1);
            }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        LeftThumbstickYNTEntry.setDouble(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawAxis(0);
        }

        value = Math.abs(value) > thumbstickDeadZone ? value : 0.0;

        LeftThumbstickXNTEntry.setDouble(value);

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
            case Switch2Pro:
                value = mapDirectionToPOV(switch2ProController, 11, 10, 12, 9);
                break;
        }

        POVAngleNTEntry.setInteger(value);

        return value;
    }

    boolean getPOVUp() {
        var value = false;

        int angle = getPOV();
        if (angle == 0) { // -1 means not pressed
            value = true;
        }

        POVUpNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getPOVDown() {
        var value = false;
        int angle = getPOV();
        if (angle == 180) {  // -1 means not pressed
            value = true;
        }

        POVDownNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getPOVRight() {
        var value = false;
        int angle = getPOV();
        if (angle == 90) {  // -1 means not pressed
            value = true;
        }

        POVRightNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getPOVLeft() {
        var value = false;
        int angle = getPOV();
        if (angle == 270) {  // -1 means not pressed
            value = true;
        }

        POVLeftNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(7);
                break;
       }

        StartNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(15);
                break;
            }

        SelectNTEntry.setBoolean(value);

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
            case Switch2Pro:
                value = switch2ProController.getRawButton(17);
                break;
        }

        LogoNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getCaptureButton() {
        var value = false;
        
        switch (Type) {
            case Xbox:
                break;
            case PS4:
                break;
             case FlightStick:
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(18);
                break;
        }

        CaptureNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getChatButton() {
        var value = false;
        
        switch (Type) {
            case Xbox:
                break;
            case PS4:
                break;
             case FlightStick:
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(21);
                break;
        }

        ChatNTEntry.setBoolean(value);

        return value;
    }
 
    boolean getGripLeftButton() {
        var value = false;
        
        switch (Type) {
            case Xbox:
                break;
            case PS4:
                break;
             case FlightStick:
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(20);
                break;
        }

        GripLeftNTEntry.setBoolean(value);
        return value;
    }
 
    boolean getGripRightButton() {
        var value = false;
        
        switch (Type) {
            case Xbox:
                break;
            case PS4:
                break;
             case FlightStick:
                break;
            case Switch2Pro:
                value = switch2ProController.getRawButton(19);
                break;
        }

        GripRightNTEntry.setBoolean(value);
        return value;
    }

    int mapDirectionToPOV(Joystick joystick, int POVleft, int POVright, int POVup, int POVdown) {
        if (joystick.getRawButton(POVleft) && !joystick.getRawButton(POVright) &&
            !joystick.getRawButton(POVup) && !joystick.getRawButton(POVdown)) {
            return 270; // 270 degrees
        } else if (!joystick.getRawButton(POVleft) && joystick.getRawButton(POVright) &&
                   !joystick.getRawButton(POVup) && !joystick.getRawButton(POVdown)) {
            return 90; // 90 degrees
        } else if (!joystick.getRawButton(POVleft) && !joystick.getRawButton(POVright) &&
                   joystick.getRawButton(POVup) && !joystick.getRawButton(POVdown)) {
            return 0; // 0 degrees
        } else if (!joystick.getRawButton(POVleft) && !joystick.getRawButton(POVright) &&
                   !joystick.getRawButton(POVup) && joystick.getRawButton(POVdown)) {
            return 180; // 180 degrees
        } else if (joystick.getRawButton(POVleft) && !joystick.getRawButton(POVright) &&
            joystick.getRawButton(POVup) && !joystick.getRawButton(POVdown)) {
            return 315; // 315 degrees
        } else if (!joystick.getRawButton(POVleft) && joystick.getRawButton(POVright) &&
                   joystick.getRawButton(POVup) && !joystick.getRawButton(POVdown)) {
            return 45; // 45 degrees
        } else if (joystick.getRawButton(POVleft) && !joystick.getRawButton(POVright) &&
                   !joystick.getRawButton(POVup) && joystick.getRawButton(POVdown)) {
            return 225; // 225 degrees
        } else if (!joystick.getRawButton(POVleft) && joystick.getRawButton(POVright) &&
                   !joystick.getRawButton(POVup) && joystick.getRawButton(POVdown)) {
            return 135; // 135 degrees
        } else {
            return -1; // No direction or multiple directions pressed
        }
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
                case kHIDJoystick:
                    System.out.printf("Joystick on port %d is a generic controller, assuming Switch2Pro controller\n", j);
                    m_controller = new GameController(j, ControllerType.Switch2Pro);
                    break JoystickIndexLoop;
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
