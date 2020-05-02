#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "p3d/commands.h"

namespace p3d::cmder
{
class P3D_API CommandManager
{
public:
    CommandManager() {}
    virtual ~CommandManager() { m_commandStack.clear(); }

    virtual void executeCommand(Command *cmd) = 0;
    virtual void undoCommand() = 0;
    void mergeNextCommand() { m_mergeNextCommand = true; }

    void setUndoOn() { m_withUndo = true; }
    void setUndoOff() { m_withUndo = false; }

protected:
    bool m_withUndo{true};
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

void P3D_API executeCommand(Command *cmd);
std::shared_ptr<CommandManager> P3D_API get();
void P3D_API set(std::shared_ptr<CommandManager> cmdManager);
void P3D_API undoOn();
void P3D_API undoOff();

// void P3D_API on(); // TODO
// void P3D_API off(); // TODO

}  // namespace p3d::cmder
