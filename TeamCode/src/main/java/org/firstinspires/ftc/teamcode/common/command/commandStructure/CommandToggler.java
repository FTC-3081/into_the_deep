package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public class CommandToggler {

    private Command currentCommand;
    private Command altCommand;
    private int mapIndex = 0;

    /**
     * @param initCommand the command that will be run first
     * @param altCommand the command that will be run second
     */
    public CommandToggler(Command initCommand, Command altCommand){
        currentCommand = initCommand;
        this.altCommand = altCommand;
        toggle();
    }

    public void setMapIndex(int mapIndex){
        this.mapIndex = mapIndex;
    }

    public int getMapIndex(){
        return mapIndex;
    }

    void toggle(){
        Command tempCommand = currentCommand;
        currentCommand = altCommand;
        altCommand = tempCommand;
    }

    Command getCurrentCommand(){
        return currentCommand;
    }
}