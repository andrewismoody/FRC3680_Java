package frc.robot;

import java.util.Hashtable;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.Consumer;

public class Controller {
    PS4Controller ps4Controller;
    XboxController xboxController;

    public enum ControllerType {
        Xbox,
        PS4
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

    Hashtable<ButtonName, Consumer<Boolean>> BinaryButtonConsumers = new Hashtable<Controller.ButtonName,Consumer<Boolean>>();
    Hashtable<ButtonName, Consumer<Double>> ValueButtonConsumers = new Hashtable<Controller.ButtonName,Consumer<Double>>();
    Hashtable<ButtonName, Consumer<Integer>> POVButtonConsumers = new Hashtable<Controller.ButtonName,Consumer<Integer>>();

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
        BinaryButtonConsumers.put(button, func);
    }

    public void RegisterValueButton(ButtonName button, Consumer<Double> func) {
        ValueButtonConsumers.put(button, func);
    }

    public void RegisterPOVButton(ButtonName button, Consumer<Integer> func) {
        POVButtonConsumers.put(button, func);
    }

    public void GetBinaryButtonState(ButtonName button) {
        BinaryButtonConsumers.get(button).accept(GetBinaryButtonSupplier(button));
    }

    public void GetValueButtonValue(ButtonName button) {
        ValueButtonConsumers.get(button).accept(GetValueButtonSupplier(button));
    }

    public void GetPOVButtonValue(ButtonName button) {
        ValueButtonConsumers.get(button).accept(GetValueButtonSupplier(button));
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
        }

        return false;
    }

    boolean getTopButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getYButton();
            case PS4:
                return ps4Controller.getTriangleButton();
        }

        return false;
    }

    boolean getRightButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getBButton();
            case PS4:
                return ps4Controller.getCircleButton();
        }

        return false;
    }
 
    boolean getBottomButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getAButton();
            case PS4:
                return ps4Controller.getCrossButton();
        }

        return false;
    }
 
    boolean getLeftShoulderButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftBumper();
            case PS4:
                return ps4Controller.getL1Button();
        }

        return false;
    }

    double getLeftTriggerValue() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftTriggerAxis();
            case PS4:
                return ps4Controller.getL2Button() ? 1.0 : 0.0;
        }

        return 0.0;
    }
 
    boolean getRightShoulderButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightBumper();
            case PS4:
                return ps4Controller.getR1Button();
        }

        return false;
    }

    double getRightTriggerValue() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightTriggerAxis();
            case PS4:
                return ps4Controller.getR2Button() ? 1.0 : 0.0;
        }

        return 0.0;
    }

    double getRightY() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightY();
            case PS4:
                return ps4Controller.getRightY();
        }

        return 0.0;
    }

    double getRightX() {
        switch (Type) {
            case Xbox:
                return xboxController.getRightX();
            case PS4:
                return ps4Controller.getRightX();
        }

        return 0.0;
    }

    double getLeftY() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftY();
            case PS4:
                return ps4Controller.getLeftY();
        }

        return 0.0;
    }

    double getLeftX() {
        switch (Type) {
            case Xbox:
                return xboxController.getLeftX();
            case PS4:
                return ps4Controller.getLeftX();
        }

        return 0.0;
    }

    int getPOV() {
        switch (Type) {
            case Xbox:
                return xboxController.getPOV();
            case PS4:
                return ps4Controller.getPOV();
        }

        return 0;
    }
 
    boolean getStartButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getStartButton();
            case PS4:
                return ps4Controller.getShareButton();
        }

        return false;
    }
 
    boolean getSelectButton() {
        switch (Type) {
            case Xbox:
                return xboxController.getBackButton();
            case PS4:
                return ps4Controller.getOptionsButton();
        }

        return false;
    }
 
    boolean getLogoButton() {
        switch (Type) {
            case Xbox:
                return false;
            case PS4:
                return ps4Controller.getPSButton();
        }

        return false;
    }
}
