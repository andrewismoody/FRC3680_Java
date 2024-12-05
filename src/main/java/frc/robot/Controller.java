package frc.robot;

import java.util.Hashtable;
import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class Controller {
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
        POV,
        Start,
        Select,
        Logo,
    }

    Hashtable<ButtonName, Supplier<Boolean>> BinaryButtonSuppliers = new Hashtable<Controller.ButtonName,Supplier<Boolean>>();
    Hashtable<ButtonName, Supplier<Double>> ValueButtonSuppliers = new Hashtable<Controller.ButtonName,Supplier<Double>>();
    Hashtable<ButtonName, Supplier<Integer>> POVButtonSuppliers = new Hashtable<Controller.ButtonName,Supplier<Integer>>();

    Hashtable<ButtonName, ArrayList<Consumer<Boolean>>> BinaryButtonConsumers = new Hashtable<Controller.ButtonName,ArrayList<Consumer<Boolean>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Double>>> ValueButtonConsumers = new Hashtable<Controller.ButtonName,ArrayList<Consumer<Double>>>();
    Hashtable<ButtonName, ArrayList<Consumer<Integer>>> POVButtonConsumers = new Hashtable<Controller.ButtonName,ArrayList<Consumer<Integer>>>();

    void RegisterBinaryButtonSupplier(ButtonName button, Supplier<Boolean> func) {
        BinaryButtonSuppliers.put(button, func);
    }

    void RegisterValueButtonSupplier(ButtonName button, Supplier<Double> func) {
        ValueButtonSuppliers.put(button, func);
    }

    void RegisterPOVButtonSupplier(ButtonName button, Supplier<Integer> func) {
        POVButtonSuppliers.put(button, func);
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
        for (Consumer<Double> consumer : ValueButtonConsumers.get(button))
            consumer.accept(GetValueButtonSupplier(button));
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
            GetValueButtonValue(button);;
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

    public Controller(int index, ControllerType type) {
        Index = index;
        Type = type;
        switch (Type) {
            case Xbox:
                xboxController = new XboxController(index);
            case PS4:
                ps4Controller = new PS4Controller(index);
            case FlightStick:
                fsController = new Joystick(index);
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

        RegisterValueButtonSupplier(ButtonName.LeftTrigger, this::getLeftTriggerValue);
        RegisterValueButtonSupplier(ButtonName.RightTrigger, this::getRightTriggerValue);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickX, this::getLeftX);
        RegisterValueButtonSupplier(ButtonName.LeftThumbstickY, this::getLeftY);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickX, this::getRightX);
        RegisterValueButtonSupplier(ButtonName.RightThumbstickY, this::getRightY);

        RegisterPOVButtonSupplier(ButtonName.POV, this::getPOV);
    }

    boolean getLeftButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getXButton();
            case PS4:
                return ps4Controller.getSquareButton();
            case FlightStick:
                return fsController.getRawButton(0);
        }

        return false;
    }

    boolean getTopButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getYButton();
            case PS4:
                return ps4Controller.getTriangleButton();
            case FlightStick:
                return fsController.getRawButton(1);
        }

        return false;
    }

    boolean getRightButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getBButton();
            case PS4:
                return ps4Controller.getCircleButton();
            case FlightStick:
                return fsController.getRawButton(2);
        }

        return false;
    }
 
    boolean getBottomButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getAButton();
            case PS4:
                return ps4Controller.getCrossButton();
            case FlightStick:
                return fsController.getRawButton(3);
        }

        return false;
    }
 
    boolean getLeftShoulderButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftBumper();
            case PS4:
                return ps4Controller.getL1Button();
            case FlightStick:
                return fsController.getRawButton(4);
        }

        return false;
    }

    double getLeftTriggerValue() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftTriggerAxis();
            case PS4:
                return ps4Controller.getL2Button() ? 1.0 : 0.0;
            case FlightStick:
                return fsController.getRawAxis(2);
        }

        return 0.0;
    }
 
    boolean getRightShoulderButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightBumper();
            case PS4:
                return ps4Controller.getR1Button();
            case FlightStick:
                return fsController.getRawButton(5);
        }

        return false;
    }

    double getRightTriggerValue() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightTriggerAxis();
            case PS4:
                return ps4Controller.getR2Button() ? 1.0 : 0.0;
            case FlightStick:
                return fsController.getRawAxis(3);
        }

        return 0.0;
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
                return fsController.getRawAxis(0);
        }

        return Math.abs(value) > thumbstickDeadZone ? value : 0.0;
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
                return fsController.getRawAxis(1);
        }

        return Math.abs(value) > thumbstickDeadZone ? value : 0.0;
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
                return fsController.getRawAxis(0);
        }

        return Math.abs(value) > thumbstickDeadZone ? value : 0.0;
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
                return fsController.getRawAxis(1);
        }

        return Math.abs(value) > thumbstickDeadZone ? value : 0.0;
    }

    int getPOV() {
        switch (Type) {
            case Xbox:
                return xboxController.getPOV();
            case PS4:
                return ps4Controller.getPOV();
            case FlightStick:
                return fsController.getPOV();
        }

        return 0;
    }
 
    boolean getStartButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getStartButton();
            case PS4:
                return ps4Controller.getShareButton();
             case FlightStick:
                return fsController.getTop();
       }

        return false;
    }
 
    boolean getSelectButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getBackButton();
            case PS4:
                return ps4Controller.getOptionsButton();
             case FlightStick:
                return fsController.getTrigger();
        }

        return false;
    }
 
    boolean getLogoButton() {
        switch (Type) {
            case Xbox:
                return false;
            case PS4:
                return ps4Controller.getPSButton();
             case FlightStick:
                return false;
        }

        return false;
    }
}
