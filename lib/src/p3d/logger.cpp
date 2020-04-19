#include "logger.h"


namespace p3d::logger
{
std::shared_ptr<p3d::logger::Logger> _logger = std::make_shared<StdLogger>();

std::shared_ptr<Logger> get()
{
    return p3d::logger::_logger;
}

void on()
{
    if (p3d::logger::_logger)
        p3d::logger::_logger->setOn();
}

void off()
{
    if (p3d::logger::_logger)
        p3d::logger::_logger->setOff();
}

void set(std::shared_ptr<Logger> l)
{
    if (l) p3d::logger::_logger = l;
}

void setStd()
{
    p3d::logger::_logger = std::make_shared<StdLogger>();
}

} // namespace p3d
