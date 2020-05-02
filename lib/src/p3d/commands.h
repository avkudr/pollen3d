#pragma once

#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

#include <opencv2/core.hpp>

#include "p3d/core.h"
#include "p3d/logger.h"

namespace p3d
{
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

class P3D_API Command
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

/**
 * @warning when one of the propeties is an Eigen::Matrix make sure that the
 * actual type of "const entt::meta_any &v" is the exact type of the property
 *
 * Example: there is a property of type Mat3Xf
 *
 * GOOD:
 *     Mat3Xf result = ...;
 *     CommandSetProperty(&smth,id,result);
 *
 * BAD
 *     Mat3Xf a = ...;
 *     Mat3Xf b = ...;
 *     CommandSetProperty(&smth,id,a+b);
 *
 * the type of a+b is not Mat3Xf!
 */

class P3D_API CommandSetProperty : public Command
{
public:
    CommandSetProperty(PObject *instance, const P3D_ID_TYPE &propId,
                       const entt::meta_any &v, bool force = false)
        : m_propId(propId), m_to(v), m_instance(instance)
    {
        if (instance == nullptr) return;
        if (v.data() == nullptr) return;

        const auto &alias = instance->getAlias();
        m_data = entt::resolve(alias).data(P3D_ID_TYPE(m_propId));
        if (!m_data) {
            LOG_WARN("%s, property <%i> doesn't exist", instance->className(), propId);
            return;
        }

        m_from = entt::meta_any{m_data.get(*instance)};

        if (force) {
            m_isValid = true;
            return;
        }

        if (m_to.type() != m_data.type()) {
            LOG_DBG("%s, set property: types are different", instance->className());
            return;
        }

        if (!impl::isMetaEqual(m_to, m_from)) {
            m_isValid = true;
        } else {
            LOG_DBG("set property: new equals old");
        }
    }

    ~CommandSetProperty() {}
    void execute()
    {
        if (!m_instance) return;
        if (!m_data) return;

        // if (m_to.type() == m_data.type()) {
        // next line is here to ensure that the *from* value is taken right
        // before setting new value. It is important for group command when
        // the previous state of the property should be taken here and not
        // on the creation of the command
        m_from = m_instance->getData(m_data);

        // setting new value
        m_instance->setData(m_data, m_to);
        //}
        LOG_DBG("%s, set prop: %i (%p)", m_instance->className(), m_propId, m_instance);
    }
    void undo()
    {
        if (!m_instance) return;
        if (!m_data) return;
        LOG_DBG("%s, undo set prop: %i (%p)", m_instance->className(), m_propId,
                m_instance);
        m_instance->setData(m_data, m_from);
    }

private:
    PObject *m_instance;
    P3D_ID_TYPE m_propId;
    entt::meta_any m_to{};
    entt::meta_any m_from{};
    entt::meta_data m_data;
};

/** @brief special set property for cv::Mat objects
 *
 * @example
 * auto cmd = new CommandSetPropertyCV{imPair, &ImagePair::setDisparityMap,
 *                                   &ImagePair::getDisparityMap, mFiltered};
 */
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
        for (int i = int(m_commandStack.size()) - 1; i >= 0; --i) {
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

}  // namespace p3d
