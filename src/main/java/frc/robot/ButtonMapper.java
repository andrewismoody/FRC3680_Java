package frc.robot;

import java.util.Hashtable;
import java.util.function.Consumer;

import frc.robot.Controller.ButtonName;

public class ButtonMapper {
    Controller myController;

    Hashtable<ButtonName, Consumer<Boolean>> BinaryButtonFunctions = new Hashtable<Controller.ButtonName,Consumer<Boolean>>();
    Hashtable<ButtonName, Consumer<Double>> ValueButtonFunctions = new Hashtable<Controller.ButtonName,Consumer<Double>>();
    Hashtable<ButtonName, Consumer<Integer>> POVButtonFunctions = new Hashtable<Controller.ButtonName,Consumer<Integer>>();
    
    public ButtonMapper(Controller controller) {
        myController = controller;
    }

    public void RegisterBinaryButton(ButtonName button, Consumer<Boolean> func) {
        BinaryButtonFunctions.put(button, func);
    }

    public void RegisterValueButton(ButtonName button, Consumer<Double> func) {
        ValueButtonFunctions.put(button, func);
    }

    public void RegisterPOVButton(ButtonName button, Consumer<Integer> func) {
        POVButtonFunctions.put(button, func);
    }

    public void GetBinaryButtonState(ButtonName button) {
        BinaryButtonFunctions.get(button).accept(myController.GetBinaryButtonState(button));
    }

    public void GetValueButtonValue(ButtonName button) {
        ValueButtonFunctions.get(button).accept(myController.GetValueButtonValue(button));
    }

    public void GetPOVButtonValue(ButtonName button) {
        ValueButtonFunctions.get(button).accept(myController.GetValueButtonValue(button));
    }

    public void ProcessButtons() {
        for (ButtonName button : BinaryButtonFunctions.keySet()) {
            GetBinaryButtonState(button);
        }

        for (ButtonName button : ValueButtonFunctions.keySet()) {
            GetValueButtonValue(button);;
        }

        for (ButtonName button : POVButtonFunctions.keySet()) {
            GetPOVButtonValue(button);
        }
    }
}
