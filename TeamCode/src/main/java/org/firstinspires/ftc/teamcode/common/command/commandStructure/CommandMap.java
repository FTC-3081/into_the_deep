package org.firstinspires.ftc.teamcode.common.command.commandStructure;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class CommandMap {

    private final ArrayList<BooleanSupplier> commandTriggers = new ArrayList<>();
    private final ArrayList<Command> commands = new ArrayList<>();

    private final ArrayList<BooleanSupplier> toggleTriggers = new ArrayList<>();
    private final ArrayList<CommandToggler> toggles = new ArrayList<>();

    private static CommandMap instance;

    private CommandMap(){}
    public static synchronized CommandMap getInstance(){
        if (instance == null)
            instance = new CommandMap();

        return instance;
    }

    public void mapCommand(BooleanSupplier trigger, Command command){
        commandTriggers.add(trigger);
        commands.add(command);
    }

    public void mapCommand(BooleanSupplier trigger, CommandToggler toggler){
        commandTriggers.add(trigger);
        commands.add(toggler.getCurrentCommand());
        toggler.setMapIndex(commands.size() - 1);
        toggleTriggers.add(trigger);
        toggles.add(toggler);
    }

    public void loop(){
        for(int i = 0; i < toggles.size(); i++){
            if(toggleTriggers.get(i).getAsBoolean()){
                toggles.get(i).toggle();
                commands.set(toggles.get(i).getMapIndex(), toggles.get(i).getCurrentCommand());
            }
        }
        for(int i = 0; i < commands.size(); i++){
            if(commandTriggers.get(i).getAsBoolean() || !commands.get(i).isFinished()){
                commands.get(i).execute();
            }
        }
    }
}