package org.firstinspires.ftc.teamcode.common.command.commandStructure;

import java.util.function.BooleanSupplier;

public class PauseUntilCommand extends Command{

    private final BooleanSupplier endCondition;

    /**
     * Intantiates a command that pauses the sequence until some condition is true
     * @param endCondition the condition that must be met
     */
    public PauseUntilCommand(BooleanSupplier endCondition){
        this.endCondition = endCondition;
    }

    public void execute(){
        finished = endCondition.getAsBoolean();
    }
}
