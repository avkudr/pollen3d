#include "commands.h"

using namespace p3d;

Command::~Command() {}

CommandManager *CommandManager::get()
{
    static CommandManager *instance = nullptr;
    if (!instance) instance = new CommandManager();
    return instance;
}
