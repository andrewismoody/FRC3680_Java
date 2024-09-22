package frc.robot;

import java.util.Hashtable;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

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

    Hashtable<ButtonName, Supplier<Boolean>> BinaryButtonFunctions = new Hashtable<Controller.ButtonName,Supplier<Boolean>>();
    Hashtable<ButtonName, Supplier<Double>> ValueButtonFunctions = new Hashtable<Controller.ButtonName,Supplier<Double>>();
    Hashtable<ButtonName, Supplier<Integer>> POVButtonFunctions = new Hashtable<Controller.ButtonName,Supplier<Integer>>();

    void RegisterBinaryButton(ButtonName button, Supplier<Boolean> func) {
        BinaryButtonFunctions.put(button, func);
    }

    void RegisterValueButton(ButtonName button, Supplier<Double> func) {
        ValueButtonFunctions.put(button, func);
    }

    void RegisterPOVButton(ButtonName button, Supplier<Integer> func) {
        POVButtonFunctions.put(button, func);
    }

    public Boolean GetBinaryButtonState(ButtonName button) {
        return BinaryButtonFunctions.get(button).get();
    }

    public Double GetValueButtonValue(ButtonName button) {
        return ValueButtonFunctions.get(button).get();
    }

    public Integer GetPOVButtonValue(ButtonName button) {
        return POVButtonFunctions.get(button).get();
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

        RegisterBinaryButton(ButtonName.TopButton, this::getTopButton);
        RegisterBinaryButton(ButtonName.BottomButton, this::getBottomButton);
        RegisterBinaryButton(ButtonName.LeftButton, this::getLeftButton);
        RegisterBinaryButton(ButtonName.RightButton, this::getRightButton);
        RegisterBinaryButton(ButtonName.LeftShoulderButton, this::getLeftShoulderButton);
        RegisterBinaryButton(ButtonName.RightShoulderButton, this::getRightShoulderButton);
        RegisterBinaryButton(ButtonName.Start, this::getStartButton);
        RegisterBinaryButton(ButtonName.Select, this::getSelectButton);
        RegisterBinaryButton(ButtonName.Logo, this::getLogoButton);

        RegisterValueButton(ButtonName.LeftTrigger, this::getLeftTriggerValue);
        RegisterValueButton(ButtonName.RightTrigger, this::getRightTriggerValue);
        RegisterValueButton(ButtonName.LeftThumbstickX, this::getLeftX);
        RegisterValueButton(ButtonName.LeftThumbstickY, this::getLeftY);
        RegisterValueButton(ButtonName.RightThumbstickX, this::getRightX);
        RegisterValueButton(ButtonName.RightThumbstickY, this::getRightY);

        RegisterPOVButton(ButtonName.POV, this::getPOV);
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
