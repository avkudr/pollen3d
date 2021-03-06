#include "command_manager.h"

namespace p3d::cmder
{

std::shared_ptr<CommandManager> _cmdManager =
        std::make_shared<p3d::cmder::LibCommandManager>();

void LibCommandManager::executeCommand(p3d::Command *cmd)
{
    if (!cmd->isValid()) return;
    cmd->execute();

    if (!m_withUndo) return;

    if (m_commandStack.empty()) m_mergeNextCommand = false;
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

void LibCommandManager::undoCommand()
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

std::shared_ptr<CommandManager> get()
{
    return p3d::cmder::_cmdManager;
}

void set(std::shared_ptr<CommandManager> cmdManager)
{
    if (cmdManager) p3d::cmder::_cmdManager = cmdManager;
}

void executeCommand(Command *cmd)
{
    if (p3d::cmder::_cmdManager) p3d::cmder::_cmdManager->executeCommand(cmd);
}

void undoOn()
{
    if (p3d::cmder::_cmdManager) p3d::cmder::_cmdManager->setUndoOn();
}

void undoOff()
{
    if (p3d::cmder::_cmdManager) p3d::cmder::_cmdManager->setUndoOff();
}

} // namespace p3d::cmder
