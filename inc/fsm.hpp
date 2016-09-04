#ifndef _FSM_HPP_
#define _FSM_HPP_

#include <cstdint>
#include <iostream>
#include <vector>
#include <cassert>

typedef int32_t state_id_t;
typedef int32_t event_id_t;
typedef int32_t priority_t;

#define FSM_PRI_LOW 0
#define FSM_PRI_NORMAL 1
#define FSM_PRI_HIGH 2

#define FSM_INIT_STATE_ID -1
#define FSM_INVALID -1

/**
 *  @brief  finite state machine model
 **/
template <typename T>
class fsm
{
public:
    fsm();
    ~fsm();

    /*
     *  @brief  add a new state to this fsm
     *  @param[In]  custom type of state
     *  @return     state id of new state
     */
    state_id_t set_state(T state);
    
    /*
     *  @brief  set initial state id
     *  @param[In]  state id
     */
    void set_init_state(state_id_t sid);
    
    /*
     *  @brief  add a new transition event to this fsm
     *  @param[In]  src source state id
     *  @param[In]  dst destination state id
     *  @param[In]  pri priority of this event (e.g. FSM_PRI_NORMAL)
     */
    event_id_t set_event(state_id_t src, state_id_t dst, priority_t pri=FSM_PRI_NORMAL);
    
    /*
     *  @brief  trigger the event which id is eid
     *  @param[In]  eid event id
     */
    void fire_event(event_id_t eid);
    
    /*
     *  @brief  handle event and state translation 
     */
    void update();
    
    /*
     *  @brief  get current state
     */
    const T & peek();

private:
    struct event_t
    {
        priority_t priority;        
        state_id_t src;
        state_id_t dst;
    };
#define event_reset(e) (((e).priority = FSM_PRI_LOW, (e).src = FSM_INVALID, (e).dst = FSM_INVALID)) 
  
    state_id_t m_cur_state;
    event_id_t m_cur_event;

    std::vector<event_t> m_event_table;
    std::vector<T> m_state_table;
};

template <typename T>
fsm<T>::fsm(): 
    m_cur_state(FSM_INVALID),
    m_cur_event(FSM_INVALID)
{
}

template <typename T>
fsm<T>::~fsm()
{
}

template <typename T>
state_id_t fsm<T>::set_state(T state)
{
    m_state_table.push_back(state);
    return m_state_table.size() - 1;
}

template <typename T>
void fsm<T>::set_init_state(state_id_t sid)
{
    m_cur_state = sid;
#ifdef DEBUG_FSM
    std::cout << "FSM Initial state : " << sid << std::endl;
#endif
}

template <typename T>
event_id_t fsm<T>::set_event(state_id_t src, state_id_t dst, priority_t pri)
{
    m_event_table.push_back(event_t{pri, src, dst});
    return m_event_table.size() - 1;
}

template <typename T>
void fsm<T>::fire_event(event_id_t eid)
{
    assert(eid < m_event_table.size());
    event_t & event = m_event_table[eid];

    /*  Check src */
    if (event.src != m_cur_state) {
        return;
    }
    
    /*  Preempt */
    if (event.priority > m_event_table[m_cur_event].priority) {
        m_cur_event = eid;
    }
}

template <typename T>
void fsm<T>::update()
{
#ifdef DEBUG_FSM
    state_id_t pre_state;
    pre_state = m_cur_state;
#endif
    if (m_cur_event != FSM_INVALID) {
        event_t & event = m_event_table[m_cur_event];
    
        /* Check src */
        assert(event.src == m_cur_state);

        m_cur_state = event.dst;
    }
#ifdef DEBUG_FSM
    std::cout << "FSM : " << pre_state << " -> " << m_cur_state << std::endl;
#endif
}

template <typename T>
const T & fsm<T>::peek()
{
    assert(m_cur_state < m_state_table.size());
    return m_state_table[m_cur_state];
}

#endif
