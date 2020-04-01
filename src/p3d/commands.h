#pragma once

#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

#include "p3d/console_logger.h"
#include "p3d/core/core.h"

#include <opencv2/core.hpp>

namespace impl
{
inline bool isMetaEqual(const entt::meta_any &lhs, const entt::meta_any &rhs)
{
    // We have to do this because Eigen operator== throws an exception if
    // matrices are of the different size...
    try {
        return (lhs == rhs);
    } catch (...) {
        return false;
    }
}
}  // namespace impl

class Command
{
public:
    Command() {}
    virtual ~Command();
    virtual void undo()
    {
        //LOG_INFO("No undo command for the last operation");
    }
    virtual void execute() = 0;

    bool isValid() const { return m_isValid; }

protected:
    bool m_isValid = false;
};

/*
entt::meta<A>.
    .type(hash("A"))
    .data<&A::setValue,&A::getValue>(hash("value"))
    .data<&A::setF,&A::getF>(hash("F"));
*/

template <typename T>
class CommandSetProperty : public Command
{
public:
    CommandSetProperty(T *instance, const P3D_ID_TYPE &propId, const entt::meta_any &v, bool force = false) : m_propId(propId), m_to(v), m_instance(instance)
    {
        if (instance == nullptr) return;
        if (v.data() == nullptr) return;

        m_data = entt::resolve<T>().data(P3D_ID_TYPE(m_propId));
        if (!m_data) {
            LOG_WARN("Property <%i> doesn't exist", propId);
            return;
        }

        m_from = entt::meta_any{m_data.get(*instance)};

        if (force)
            m_isValid = true;
        else if (m_to.type() == m_data.type() && !impl::isMetaEqual(m_to, m_from)) {
            m_isValid = true;
        }
    }

    ~CommandSetProperty() {}
    void execute()
    {
        if (!m_instance) return;
        if (!m_data) return;

        if (m_to.type() == m_data.type()) {
            // next line is here to ensure that the *from* value is taken right before
            // setting new value. It is important for group command when the previous
            // state of the property should be taken here and not on the creation of the
            // command
            m_from = entt::meta_any{m_data.get(*m_instance)};

            // setting new value
            m_data.set(*m_instance, m_to);
        }
        LOG_DBG("SetProperty: %i", m_propId);
        //if (m_to.try_cast<int>()) printf("SetProperty: %i (%i)\n", m_propId, m_to.cast<int>());
    }
    void undo()
    {
        if (!m_instance) return;
        if (!m_data) return;
        m_data.set(*m_instance, m_from);
        //if (m_from.try_cast<int>()) printf("Undo SetProperty: %i (%i)\n", m_propId, m_from.cast<int>());
    }

private:
    T *m_instance;
    int m_propId;
    entt::meta_any m_to{};
    entt::meta_any m_from{};
    entt::meta_data m_data;
};

template <typename T, typename Setter, typename Getter>
class CommandSetPropertyCV : public Command
{
public:
    CommandSetPropertyCV(
        T *instance,
        Setter &&__setter,
        Getter &&__getter,
        const cv::Mat &value)
    {
        if (!instance) return;
        setter = std::bind(__setter, instance, std::placeholders::_1);
        getter = std::bind(__getter, instance);
        m_from = getter().clone();
        m_to = value.clone();
        m_isValid = true;
    }

    ~CommandSetPropertyCV() {}
    void execute()
    {
        LOG_DBG("SetProperty: cv::Mat");
        setter(m_to);
    }
    void undo()
    {
        setter(m_from);
    }

private:
    std::function<void(const cv::Mat &)> setter;
    std::function<const cv::Mat &()> getter;
    cv::Mat m_to{};
    cv::Mat m_from{};
};

class CommandGroup : public Command
{
public:
    CommandGroup() {}
    virtual ~CommandGroup() {}
    virtual void add(Command *cmd)
    {
        if (cmd->isValid()) {
            m_commandStack.push_back(cmd);
            m_isValid = true;  // at least one valid command
        }
    }

    virtual void undo()
    {
        for (int i = m_commandStack.size() - 1; i >= 0; --i) {
            auto &command = m_commandStack[i];
            if (command) {
                command->undo();
                m_commandStack.pop_back();
                delete command;
            }
        }
    }

    virtual void execute()
    {
        for (auto &cmd : m_commandStack) {
            cmd->execute();
        }
    }

    bool empty() { return m_commandStack.size() == 0; }
    bool isValid() const { return m_isValid; }

protected:
    std::vector<Command *> m_commandStack;
};

class CommandManager
{
public:
    static CommandManager *get()
    {
        if (!m_instance) m_instance = new CommandManager();
        return m_instance;
    }

    void executeCommand(Command *cmd)
    {
        if (cmd->isValid()) {
            cmd->execute();
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

private:
    CommandManager() {}
    ~CommandManager()
    {
        m_commandStack.clear();
    }

    static CommandManager *m_instance;
    std::vector<Command *> m_commandStack;
};
