#include "command_manager.h"

P3D_API std::shared_ptr<p3d::CommandManager> p3d::_cmdManager =
    std::make_shared<p3d::LibCommandManager>();

void p3d::LibCommandManager::executeCommand(p3d::Command *cmd)
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

void p3d::LibCommandManager::undoCommand()
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
