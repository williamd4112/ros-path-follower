#ifndef _FSM_HPP_
#define _FSM_HPP_

#include <cstdint>

typedef int32_t state_id_t;

typedef state_id_t (*fsm_func)()

class fsm
{
public:
    fsm();
    ~fsm();
private:  
    
};

#endif
