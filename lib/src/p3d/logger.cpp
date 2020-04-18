#include "logger.h"

P3D_API std::shared_ptr<p3d::Logger> p3d::_logger = nullptr;

namespace p3d
{
void loggerStd() { _logger = std::make_shared<StdLogger>(); }
void loggerOn()
{
    if (_logger) _logger->setOn();
}
void loggerOff()
{
    if (_logger) _logger->setOff();
}
}
