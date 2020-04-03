#include "commands.h"

using namespace p3d;

CommandManager *CommandManager::m_instance = nullptr;

Command::~Command() {}

