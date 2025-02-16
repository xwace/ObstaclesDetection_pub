#pragma once

#ifdef __cplusplus
extern "C"{
#endif



#ifdef __cplusplus
}
#endif

#include <mutex>

namespace NS_FIFO_C
{

    template<class Type>
    class Fifo_c
    {
        public:
            Fifo_c(int ssize)
            {
                data = new Type[ssize];
                memset(data,0,ssize*sizeof(Type));
                size = ssize;
            }
            void push(Type i);
            Type get(int i);
            int size = 0;

        private:
            Type* data;
            int index = 0;
            
            std::mutex Gmutex;
    };

    template <class Type>
    void Fifo_c<Type>::push(Type i)
    {
        std::lock_guard<std::mutex> lock(Gmutex);
        data[index] = i;
        index = ++index%size;
    }

    template <class Type>
    Type Fifo_c<Type>::get(int i)
    {
        std::lock_guard<std::mutex> lock(Gmutex);
        int t_index = ((index-1)>=0)?(index-1):(size-1);
        int t = ((t_index - i)>=0)?(t_index - i):(size+(t_index - i));
        return data[t];
    }



}
