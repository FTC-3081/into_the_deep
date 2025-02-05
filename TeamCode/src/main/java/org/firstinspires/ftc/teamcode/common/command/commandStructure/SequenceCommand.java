package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public class SequenceCommand extends Command{

    Command[] commands;
    int activeCommand = 0;

    /**
     * Intanitates a command that runs subcommands sequentially
     * @param commands the commands in the sequence
     */
    public SequenceCommand(Command... commands){
        this.commands = commands;
    }

    public SequenceCommand(boolean isAuto, Command... commands){
        this.commands = commands;
        finished = false;
    }

    public void execute(){
        commands[activeCommand].execute();
        if(commands[activeCommand].isFinished()) activeCommand++;
        finished = activeCommand == commands.length;
        if(finished) activeCommand = 0;
    }
}
