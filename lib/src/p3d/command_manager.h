#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "p3d/commands.h"

namespace p3d
{
class P3D_API CommandManager
{
public:
    CommandManager() {}
    virtual ~CommandManager() { m_commandStack.clear(); }

    virtual void executeCommand(Command *cmd) = 0;
    virtual void undoCommand() = 0;
    void mergeNextCommand() { m_mergeNextCommand = true; }

protected:
    std::vector<Command *> m_commandStack;
    bool m_mergeNextCommand{false};
};

class P3D_HIDDEN LibCommandManager : public CommandManager
{
public:
    LibCommandManager() : CommandManager() {}
    virtual ~LibCommandManager() {}

    void executeCommand(Command *cmd) override;
    void undoCommand() override;
};

extern P3D_API std::shared_ptr<CommandManager> _cmdManager;

}  // namespace p3d
