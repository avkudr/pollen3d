#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include <string_view>

#include "p3d/core/core.h"
#include "p3d/console_logger.h"

class Command
{
public:
    Command() {}
    virtual ~Command();
    virtual void undo(){
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
    CommandSetProperty(T * instance, const int &propId, entt::meta_any v) :
        m_propId(propId),m_to(v),m_instance(instance)
    {
        if (instance == nullptr) return;
        if (v.data() == nullptr) return;

        m_data = entt::resolve<T>().data(P3D_ID_TYPE(m_propId));
        if (!m_data) {
            LOG_WARN("Property <%i> doesn't exist", propId);
            return;
        }

        m_from = m_data.get(*m_instance);
        if (m_to.type() == m_data.type() && m_to != m_from){
            m_isValid = true;
        }
    }

    ~CommandSetProperty(){}
    void execute(){
        if (!m_instance) return;
        if (!m_data) return;
        if (m_to.type() == m_data.type()){
            m_data.set(*m_instance,m_to);
        }
        LOG_DBG("SetProperty: %i", m_propId);
    }
    void undo(){
        if (!m_instance) return;
        if (!m_data) return;
        m_data.set(*m_instance,m_from);
    }

private:
    T * m_instance;
    int m_propId;
    entt::meta_any m_to;
    entt::meta_any m_from;
    entt::meta_data m_data;
};

class CommandManager{

public:
    static CommandManager * get() {
        if (!m_instance) m_instance = new CommandManager();
        return m_instance;
    }


    void executeCommand(Command * cmd){
        if (cmd->isValid()) {
            cmd->execute();
            m_commandStack.push_back(cmd);
        }
    }
    void undoCommand(){
        if ( m_commandStack.size() > 0){
            auto command = m_commandStack[m_commandStack.size()-1];
            if (command) {
                LOG_INFO("Undo last command");
                command->undo();
                m_commandStack.pop_back();
                delete command;
            }
        }
    }

private:
    CommandManager(){}
    ~CommandManager(){
        m_commandStack.clear();
    }

    static CommandManager * m_instance;
    std::vector<Command*> m_commandStack;
};
