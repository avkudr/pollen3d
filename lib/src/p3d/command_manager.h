#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "p3d/commands.h"

namespace p3d
{
class CommandManager
{
public:
    static CommandManager *get() {
        static CommandManager *instance = nullptr;
        if (!instance) instance = new CommandManager();
        return instance;
    }

    void executeCommand(Command *cmd)
    {
        if (cmd->isValid()) {
            cmd->execute();

            if (m_mergeNextCommand) {
                CommandGroup *newGroup = new CommandGroup();
                newGroup->add(m_commandStack.back());
                newGroup->add(cmd);
                m_commandStack.pop_back();
                m_commandStack.push_back(newGroup);
                m_mergeNextCommand = false;
            } else
                m_commandStack.push_back(cmd);
        }
    }
    void undoCommand()
    {
        if (m_commandStack.size() > 0) {
            auto command = m_commandStack[m_commandStack.size() - 1];
            if (command) {
                LOG_INFO("Undo last command");
                command->undo();
                m_commandStack.pop_back();
                delete command;
            }
        }
    }

    void mergeNextCommand() { m_mergeNextCommand = true; }

private:
    CommandManager() {}
    ~CommandManager()
    {
        m_commandStack.clear();
    }

    std::vector<Command *> m_commandStack;

    bool m_mergeNextCommand{false};
};

}  // namespace p3d
